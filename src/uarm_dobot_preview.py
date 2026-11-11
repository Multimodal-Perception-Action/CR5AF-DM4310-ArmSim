"""Receive-only U-Arm to Dobot mapping preview for Windows.

The program reads Feetech STS positions from COM5 and Dobot V4 feedback from
TCP/30004.  It never writes a servo register and never sends data to the Dobot.
The calculated joint target is a preview relative to the poses seen at startup.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

from feedback_monitor import FRAME_BYTES, decode_v4, recv_exact
from feetech_port import resolve_uarm_port


SERVO_IDS = tuple(range(1, 8))
MODEL_STS3215 = 777
TICKS_PER_TURN = 4096
DEGREES_PER_TICK = 360.0 / TICKS_PER_TURN
# Upstream servo2Dobot.py maps IDs 1..6 in order and reverses joint 2.
JOINT_SIGNS = (1.0, -1.0, 1.0, 1.0, 1.0, 1.0)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def shortest_tick_step(previous: int, current: int) -> int:
    """Return the signed one-turn step without a discontinuity at 0/4095."""
    return ((current - previous + TICKS_PER_TURN // 2) % TICKS_PER_TURN
            - TICKS_PER_TURN // 2)


def read_positions(protocol: sms_sts) -> tuple[list[int], list[dict[str, int]]]:
    positions: list[int] = []
    transport: list[dict[str, int]] = []
    for servo_id in SERVO_IDS:
        position, result, packet_error = protocol.ReadPos(servo_id)
        transport.append({"id": servo_id, "comm_result": result,
                          "packet_error": packet_error})
        if result != COMM_SUCCESS:
            raise RuntimeError(f"servo {servo_id} read failed: comm_result={result}")
        positions.append(position % TICKS_PER_TURN)
    return positions, transport


def read_servo_health(protocol: sms_sts) -> list[dict[str, object]]:
    health: list[dict[str, object]] = []
    for servo_id in SERVO_IDS:
        model, model_result, model_error = protocol.ping(servo_id)
        voltage_raw, voltage_result, voltage_error = protocol.read1ByteTxRx(servo_id, 62)
        temperature, temp_result, temp_error = protocol.read1ByteTxRx(servo_id, 63)
        status, status_result, status_error = protocol.read1ByteTxRx(servo_id, 65)
        baud_register, baud_result, baud_error = protocol.read1ByteTxRx(servo_id, 6)
        mode, mode_result, mode_error = protocol.read1ByteTxRx(servo_id, 33)
        torque_enable, torque_result, torque_error = protocol.read1ByteTxRx(servo_id, 40)
        homing_raw, homing_result, homing_error = protocol.read2ByteTxRx(servo_id, 31)
        homing_offset = -(homing_raw & 0x07FF) if homing_raw & 0x0800 else homing_raw & 0x07FF
        health.append({
            "id": servo_id,
            "model_number": model,
            "model_is_sts3215": model == MODEL_STS3215,
            "voltage_v": voltage_raw / 10.0,
            "temperature_c": temperature,
            "status_register": status,
            "voltage_alarm": bool(status & 0x01),
            "baud_register": baud_register,
            "operating_mode": mode,
            "torque_enable": torque_enable,
            "homing_offset_raw": homing_raw,
            "homing_offset_ticks": homing_offset,
            "comm_results": [model_result, voltage_result, temp_result, status_result,
                             baud_result, mode_result, torque_result, homing_result],
            "packet_errors": [model_error, voltage_error, temp_error, status_error,
                              baud_error, mode_error, torque_error, homing_error],
        })
    return health


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto",
                        help="COM port, or 'auto' for the known CH343 FE-URT-2")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--log-dir", default=str(Path(__file__).parents[1] / "logs"))
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0:
        parser.error("--duration and --rate must be positive")

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"uarm_dobot_preview_{stamp}.jsonl"

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    robot_socket: socket.socket | None = None
    opened = False
    records = 0
    try:
        opened = bool(port.openPort())
        if not opened:
            raise RuntimeError(f"could not open {port_name}")
        if not port.setBaudRate(args.baud):
            raise RuntimeError(f"could not set {port_name} to {args.baud} baud")

        robot_socket = socket.create_connection((args.host, 30004), timeout=args.timeout)
        robot_socket.settimeout(args.timeout)
        first_feedback = decode_v4(recv_exact(robot_socket, FRAME_BYTES))
        robot_baseline = [float(value) for value in first_feedback["q_actual_deg"]]
        servo_baseline, baseline_transport = read_positions(protocol)
        previous = servo_baseline[:]
        accumulated_ticks = [0] * len(SERVO_IDS)
        health = read_servo_health(protocol)
        voltage_alarm = any(bool(item["voltage_alarm"]) for item in health)
        all_models_match = all(bool(item["model_is_sts3215"]) for item in health)

        header = {
            "timestamp_utc": utc_now(),
            "kind": "session",
            "operation": "receive_only_mapping_preview",
            "writes_to_uarm": 0,
            "bytes_sent_to_dobot": 0,
            "uarm_port_request": args.port,
            "uarm_port": port_name,
            "uarm_baud": args.baud,
            "servo_ids": list(SERVO_IDS),
            "servo_health": health,
            "all_models_are_sts3215": all_models_match,
            "voltage_alarm": voltage_alarm,
            "uarm_bus_healthy": all_models_match and not voltage_alarm,
            "servo_baseline_ticks": servo_baseline,
            "servo_baseline_transport": baseline_transport,
            "dobot_host": args.host,
            "dobot_baseline_deg": robot_baseline,
            "dobot_initial_feedback": first_feedback,
            "mapping": {"servo_ids_to_joints": [1, 2, 3, 4, 5, 6],
                        "joint_signs": list(JOINT_SIGNS), "servo_id_7": "gripper_preview_only"},
            "motion_output_available": False,
        }

        period = 1.0 / args.rate
        deadline = time.monotonic() + args.duration
        next_tick = time.monotonic()
        with log_path.open("x", encoding="utf-8") as log:
            log.write(json.dumps(header, ensure_ascii=False, allow_nan=False) + "\n")
            print(json.dumps(header, ensure_ascii=False, allow_nan=False))
            while time.monotonic() < deadline:
                positions, transport = read_positions(protocol)
                for index, current in enumerate(positions):
                    accumulated_ticks[index] += shortest_tick_step(previous[index], current)
                previous = positions
                offsets_deg = [value * DEGREES_PER_TICK for value in accumulated_ticks]
                target_preview = [
                    robot_baseline[index] + JOINT_SIGNS[index] * offsets_deg[index]
                    for index in range(6)
                ]
                record = {
                    "timestamp_utc": utc_now(),
                    "kind": "sample",
                    "read_only": True,
                    "servo_positions_ticks": positions,
                    "servo_offsets_deg": offsets_deg,
                    "servo_transport": transport,
                    "dobot_target_preview_deg": target_preview,
                    "gripper_offset_deg": offsets_deg[6],
                }
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                records += 1
                if records == 1 or records % max(1, int(args.rate)) == 0:
                    print(json.dumps(record, ensure_ascii=False, allow_nan=False))
                next_tick += period
                sleep_for = next_tick - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)

        print(json.dumps({"log": str(log_path), "samples": records,
                          "voltage_alarm": voltage_alarm,
                          "motion_commands_sent": 0}, ensure_ascii=False))
        if not all_models_match:
            return 2
        return 3 if voltage_alarm else 0
    finally:
        if robot_socket is not None:
            robot_socket.close()
        if opened and port.ser is not None:
            port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
