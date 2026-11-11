"""Read-only voltage monitor for a Feetech STS U-Arm bus.

Only register-read requests are sent.  The program never enables torque,
writes a register, or sends a position command.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

from feetech_port import resolve_uarm_port


PRESENT_VOLTAGE = 62
PRESENT_TEMPERATURE = 63
STATUS = 65
TORQUE_ENABLE = 40


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_one(protocol: sms_sts, servo_id: int) -> dict[str, object]:
    voltage_raw, voltage_result, voltage_error = protocol.read1ByteTxRx(
        servo_id, PRESENT_VOLTAGE
    )
    temperature, temperature_result, temperature_error = protocol.read1ByteTxRx(
        servo_id, PRESENT_TEMPERATURE
    )
    status, status_result, status_error = protocol.read1ByteTxRx(servo_id, STATUS)
    torque, torque_result, torque_error = protocol.read1ByteTxRx(
        servo_id, TORQUE_ENABLE
    )
    results = [voltage_result, temperature_result, status_result, torque_result]
    return {
        "id": servo_id,
        "voltage_v": voltage_raw / 10.0,
        "temperature_c": temperature,
        "status_register": status,
        "voltage_alarm": bool(status & 0x01),
        "torque_enable": torque,
        "comm_success": all(result == COMM_SUCCESS for result in results),
        "comm_results": results,
        "packet_errors": [
            voltage_error,
            temperature_error,
            status_error,
            torque_error,
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", default="1,2,3,4,5,6,7")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args()
    ids = [int(item.strip()) for item in args.ids.split(",") if item.strip()]
    if not ids or args.samples < 1 or args.interval < 0:
        parser.error("IDs and samples must be non-empty; interval must be non-negative")

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    records: list[dict[str, object]] = []
    try:
        opened = bool(port.openPort())
        if not opened:
            raise RuntimeError(f"could not open {port_name}")
        if not port.setBaudRate(args.baud):
            raise RuntimeError(f"could not set {port_name} to {args.baud} baud")
        for sample_number in range(1, args.samples + 1):
            record = {
                "timestamp_utc": utc_now(),
                "sample": sample_number,
                "read_only": True,
                "port": port_name,
                "baud": args.baud,
                "servos": [read_one(protocol, servo_id) for servo_id in ids],
            }
            records.append(record)
            print(json.dumps(record, ensure_ascii=False, allow_nan=False))
            if sample_number < args.samples:
                time.sleep(args.interval)
    finally:
        if opened and port.ser is not None:
            port.closePort()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"uarm_power_monitor_{stamp}.jsonl"
    with log_path.open("x", encoding="utf-8") as log_file:
        for record in records:
            log_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    servos = [servo for record in records for servo in record["servos"]]
    summary = {
        "log": str(log_path.resolve()),
        "samples": len(records),
        "minimum_voltage_v": min(float(servo["voltage_v"]) for servo in servos),
        "maximum_voltage_v": max(float(servo["voltage_v"]) for servo in servos),
        "all_torque_disabled": all(int(servo["torque_enable"]) == 0 for servo in servos),
        "any_voltage_alarm": any(bool(servo["voltage_alarm"]) for servo in servos),
        "write_commands_sent": 0,
        "motion_commands_sent": 0,
    }
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
    return 3 if summary["any_voltage_alarm"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
