"""Network-only CR5AF -> RS485 -> T-CAN485 -> DM-J4310 gripper CLI.

Runtime control uses only TCP port 60000. COM6/USB is not opened or required.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "dm4310_gripper_calibration.json"
GATEWAY_HEADER = b"\xD5\x4D\x47\x50"
CAN_RESPONSE_HEADER = b"\xD5\x4D\x52\x50"


def xor_checksum(data: bytes) -> int:
    result = 0
    for value in data:
        result ^= value
    return result


def command_packet(header: bytes, guard: int) -> bytes:
    body = header + b"\x01\x00" + bytes([guard])
    return body + bytes([xor_checksum(body)])


PACKETS = {
    "identity": command_packet(b"\xD5\x4D\x49\x51", 0x60),
    "open": command_packet(b"\xD5\x4D\x4F\x51", 0xC1),
    "close": command_packet(b"\xD5\x4D\x43\x51", 0xC2),
    "hold_heartbeat": command_packet(b"\xD5\x4D\x48\x51", 0xC3),
    "stop": command_packet(b"\xD5\x4D\x44\x51", 0x5A),
    "status": command_packet(b"\xD5\x4D\x53\x51", 0x00),
}
EVENTS = {"identity": 0x01, "open": 0x10, "close": 0x11, "stop": 0x12}
ERROR_STATES = {
    0x0: "disabled/no fault",
    0x1: "enabled",
    0x3: "output-shaft calibration fault",
    0x4: "sensor output fault",
    0x5: "motor encoder calibration fault",
    0x8: "over-voltage",
    0x9: "under-voltage",
    0xA: "over-current",
    0xB: "MOS over-temperature",
    0xC: "motor winding over-temperature",
    0xD: "communication loss",
    0xE: "overload",
}


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def decode_feedback(
    packet: bytes, pmax: float, vmax: float, tmax: float
) -> dict[str, object]:
    data = packet[8:16]
    error = (data[0] >> 4) & 0x0F
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    return {
        "data_hex": data.hex(" ").upper(),
        "motor_id": data[0] & 0x0F,
        "error_code": error,
        "state": ERROR_STATES.get(error, "unknown"),
        "position_rad": uint_to_float(position_raw, -pmax, pmax, 16),
        "velocity_rad_s": uint_to_float(velocity_raw, -vmax, vmax, 12),
        "torque_nm": uint_to_float(torque_raw, -tmax, tmax, 12),
        "mos_temperature_c": data[6],
        "rotor_temperature_c": data[7],
    }


def load_config() -> dict[str, object]:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def extract_packet(buffer: bytearray, header: bytes) -> bytes | None:
    while True:
        start = buffer.find(header)
        if start < 0:
            if len(buffer) > 3:
                del buffer[:-3]
            return None
        if len(buffer) - start < 17:
            if start:
                del buffer[:start]
            return None
        packet = bytes(buffer[start : start + 17])
        del buffer[: start + 17]
        if xor_checksum(packet[:16]) == packet[16]:
            return packet


def receive_packet(
    sock: socket.socket, buffer: bytearray, header: bytes, timeout: float
) -> bytes:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        packet = extract_packet(buffer, header)
        if packet is not None:
            return packet
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        if not data:
            raise ConnectionError("TCP60000 closed before response")
        buffer.extend(data)
    raise TimeoutError(f"no {header.hex(' ')} response within {timeout}s")


def decode_gateway_event(packet: bytes) -> dict[str, object]:
    position_rad, target_rad = struct.unpack("<ff", packet[8:16])
    return {
        "event": packet[4],
        "result": packet[5],
        "motor_error": packet[6],
        "flags": packet[7],
        "verified_disabled": bool(packet[7] & 0x01),
        "holding_active": bool(packet[7] & 0x02),
        "contact_detected": packet[5] == 0x04,
        "position_rad": position_rad,
        "target_rad": target_rad,
        "packet_hex": packet.hex(" ").upper(),
    }


def gateway_action_passed(action: str, event: dict[str, object]) -> bool:
    """Accept disabled completion or a watchdog-protected active CLOSE hold."""
    result = int(event["result"])
    return bool(
        int(event["event"]) == EVENTS[action]
        and int(event["motor_error"]) == 0
        and (
            (result == 1 and bool(event["verified_disabled"]))
            or (
                (
                    (action == "close" and result == 4)
                    or result == 5
                )
                and (
                    bool(event.get("holding_active"))
                    or bool(event["verified_disabled"])
                )
            )
        )
    )


def normalize_gripper_position(
    position_rad: float, open_rad: float, closed_rad: float
) -> tuple[float, float]:
    """Return calibrated-branch position and detected 2*pi branch offset."""
    midpoint = (open_rad + closed_rad) / 2.0
    branch_turns = round((position_rad - midpoint) / (2.0 * math.pi))
    branch_offset = branch_turns * 2.0 * math.pi
    return position_rad - branch_offset, branch_offset


def dry_run(action: str, config: dict[str, object]) -> int:
    mapping = config["mapping"]
    target = mapping.get(f"{action}_command_rad")
    result = {
        "mode": "dry-run",
        "network_opened": False,
        "action": action,
        "target_rad": target,
        "packet_hex": PACKETS[action].hex(" ").upper(),
        "calibrated_open_rad": config["open"]["position_rad"],
        "calibrated_closed_rad": config["closed"]["position_rad"],
        "endpoint_margin_rad": mapping["endpoint_margin_rad"],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def run_status(host: str, timeout: float, config: dict[str, object]) -> int:
    sock = None
    buffer = bytearray()
    try:
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(0.05)
        sock.sendall(PACKETS["status"])
        packet = receive_packet(sock, buffer, CAN_RESPONSE_HEADER, timeout)
        feedback = decode_feedback(
            packet,
            float(config["pmax_rad"]),
            float(config["vmax_rad_s"]),
            float(config["tmax_nm"]),
        )
        open_rad = float(config["open"]["position_rad"])
        closed_rad = float(config["closed"]["position_rad"])
        normalized_position, branch_offset = normalize_gripper_position(
            float(feedback["position_rad"]), open_rad, closed_rad
        )
        fraction_closed = (normalized_position - open_rad) / (
            closed_rad - open_rad
        )
        feedback["calibration_branch_position_rad"] = normalized_position
        feedback["coordinate_branch_offset_rad"] = branch_offset
        feedback["fraction_closed"] = max(0.0, min(1.0, fraction_closed))
        print(json.dumps(feedback, ensure_ascii=False, indent=2))
        return 0
    finally:
        if sock is not None:
            sock.close()


def run_action(host: str, action: str, timeout: float) -> int:
    sock = None
    buffer = bytearray()
    action_sent = False
    passed = False
    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "transport": "TCP60000 only; COM6/USB unused",
        "action": action,
        "request_hex": PACKETS[action].hex(" ").upper(),
    }
    try:
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(0.05)
        if action != "stop":
            sock.sendall(PACKETS["identity"])
            identity_packet = receive_packet(
                sock, buffer, GATEWAY_HEADER, min(timeout, 3.0)
            )
            identity = decode_gateway_event(identity_packet)
            report["identity"] = identity
            if not (
                identity["event"] == EVENTS["identity"]
                and identity["result"] == 1
                and identity["flags"] == 0x10
            ):
                raise RuntimeError("guarded gateway build 1.0 identity mismatch")

        sock.sendall(PACKETS[action])
        action_sent = action != "stop"
        result_packet = receive_packet(sock, buffer, GATEWAY_HEADER, timeout)
        result = decode_gateway_event(result_packet)
        report["result"] = result
        passed = gateway_action_passed(action, result)
        report["passed"] = passed
    except Exception as exc:
        report["passed"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if action_sent and not passed and sock is not None:
            try:
                sock.sendall(PACKETS["stop"])
                report["fallback_stop_sent"] = True
            except Exception as stop_exc:
                report["fallback_stop_error"] = (
                    f"{type(stop_exc).__name__}: {stop_exc}"
                )
        if sock is not None:
            sock.close()

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"gripper_{action}_{datetime.now():%Y%m%d_%H%M%S}.json"
    log_file.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if passed else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="DM-J4310 gripper controller")
    parser.add_argument("action", choices=("status", "open", "close", "stop"))
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    config = load_config()
    if args.dry_run:
        if args.action not in ("open", "close"):
            raise SystemExit("--dry-run is supported for open or close")
        return dry_run(args.action, config)
    if args.action == "status":
        return run_status(args.host, args.timeout, config)
    return run_action(args.host, args.action, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
