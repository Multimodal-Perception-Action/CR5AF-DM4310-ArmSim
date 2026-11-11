"""Read-only inventory and protection audit for Feetech STS servos."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

from feetech_port import resolve_uarm_port


def read_u8(protocol: sms_sts, servo_id: int, address: int) -> tuple[int, int, int]:
    return protocol.read1ByteTxRx(servo_id, address)


def read_u16(protocol: sms_sts, servo_id: int, address: int) -> tuple[int, int, int]:
    return protocol.read2ByteTxRx(servo_id, address)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", default="1,2,3,4,5,6,7")
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args()
    ids = [int(item.strip()) for item in args.ids.split(",") if item.strip()]

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    servos: list[dict[str, object]] = []
    try:
        opened = bool(port.openPort())
        if not opened:
            raise RuntimeError(f"could not open {port_name}")
        if not port.setBaudRate(args.baud):
            raise RuntimeError(f"could not set {port_name} to {args.baud} baud")

        for servo_id in ids:
            reads: dict[str, tuple[int, int, int]] = {
                "firmware_major": read_u8(protocol, servo_id, 0),
                "firmware_minor": read_u8(protocol, servo_id, 1),
                "endianness": read_u8(protocol, servo_id, 2),
                "model_number": read_u16(protocol, servo_id, 3),
                "configured_id": read_u8(protocol, servo_id, 5),
                "baud_register": read_u8(protocol, servo_id, 6),
                "response_level": read_u8(protocol, servo_id, 8),
                "maximum_temperature_c": read_u8(protocol, servo_id, 13),
                "maximum_input_voltage_raw": read_u8(protocol, servo_id, 14),
                "minimum_input_voltage_raw": read_u8(protocol, servo_id, 15),
                "unload_conditions": read_u8(protocol, servo_id, 19),
                "led_alarm_conditions": read_u8(protocol, servo_id, 20),
                "operating_mode": read_u8(protocol, servo_id, 33),
                "torque_enable": read_u8(protocol, servo_id, 40),
                "present_position_raw": read_u16(protocol, servo_id, 56),
                "present_speed_raw": read_u16(protocol, servo_id, 58),
                "present_load_raw": read_u16(protocol, servo_id, 60),
                "present_voltage_raw": read_u8(protocol, servo_id, 62),
                "present_temperature_c": read_u8(protocol, servo_id, 63),
                "status_register": read_u8(protocol, servo_id, 65),
                "moving": read_u8(protocol, servo_id, 66),
                "present_current_raw": read_u16(protocol, servo_id, 69),
            }
            failed = [
                name for name, (_, result, _) in reads.items()
                if result != COMM_SUCCESS
            ]
            values = {name: value for name, (value, _, _) in reads.items()}
            current_raw = values["present_current_raw"]
            servos.append({
                "id": servo_id,
                **values,
                "maximum_input_voltage_v": values["maximum_input_voltage_raw"] / 10.0,
                "minimum_input_voltage_v": values["minimum_input_voltage_raw"] / 10.0,
                "present_voltage_v": values["present_voltage_raw"] / 10.0,
                "present_current_magnitude_ma": (current_raw & 0x7FFF) * 6.5,
                "present_current_reverse": bool(current_raw & 0x8000),
                "voltage_status": bool(values["status_register"] & 0x01),
                "read_failures": failed,
                "packet_error_union": sum(error for _, _, error in reads.values()),
            })
    finally:
        if opened and port.ser is not None:
            port.closePort()

    record = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "read_only_sts_inventory",
        "port": port_name,
        "baud": args.baud,
        "write_commands_sent": 0,
        "motion_commands_sent": 0,
        "servos": servos,
    }
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"uarm_servo_inventory_{stamp}.json"
    with log_path.open("x", encoding="utf-8") as log_file:
        json.dump(record, log_file, ensure_ascii=False, indent=2)
    print(json.dumps(record, ensure_ascii=False, indent=2))
    print(json.dumps({"log": str(log_path.resolve())}, ensure_ascii=False))
    return 0 if all(not servo["read_failures"] for servo in servos) else 2


if __name__ == "__main__":
    raise SystemExit(main())
