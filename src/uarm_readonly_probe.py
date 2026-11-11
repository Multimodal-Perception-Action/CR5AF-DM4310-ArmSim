"""Read-only Feetech STS/SMS bus probe for a Windows FE-URT-2 adapter.

This utility sends only protocol Ping requests. It does not write registers,
set position, enable torque, or invoke a motion command.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

from scservo_sdk import PortHandler, sms_sts

from feetech_port import resolve_uarm_port


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto",
                        help="COM port, or 'auto' for the known CH343 FE-URT-2")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", default="1", help="Comma-separated servo IDs")
    parser.add_argument("--log", default="logs/uarm_readonly_probe.jsonl")
    args = parser.parse_args()

    ids = [int(value.strip(), 0) for value in args.ids.split(",") if value.strip()]
    if not ids or any(servo_id < 0 or servo_id > 253 for servo_id in ids):
        parser.error("IDs must be one or more values from 0 through 253")

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    result: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "port_request": args.port,
        "port": port_name,
        "baud": args.baud,
        "ids": ids,
        "operation": "protocol_ping_only",
        "sent_control_or_write_commands": False,
        "port_opened": False,
        "baud_set": False,
        "responses": [],
        "error": None,
    }

    exit_code = 0
    try:
        result["port_opened"] = bool(port.openPort())
        if not result["port_opened"]:
            result["error"] = "Could not open serial port"
            exit_code = 2
        else:
            result["baud_set"] = bool(port.setBaudRate(args.baud))
        result["baud_set"] = bool(result["port_opened"] and port.getBaudRate() == args.baud)
        if result["port_opened"] and not result["baud_set"]:
            result["baud_set"] = bool(port.setBaudRate(args.baud))
        if result["port_opened"] and not result["baud_set"]:
            result["error"] = "Could not set requested baud rate"
            exit_code = 3
        if result["port_opened"] and result["baud_set"]:
            responses = []
            for servo_id in ids:
                try:
                    # Official SDK Ping request: identity/status query only.
                    responses.append({"id": servo_id, "response": protocol.ping(servo_id)})
                except Exception as exc:  # record per-device failure and continue
                    responses.append({"id": servo_id, "error": f"{type(exc).__name__}: {exc}"})
            result["responses"] = responses
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        exit_code = 1
    finally:
        if port.ser is not None:
            port.closePort()
        result["port_closed"] = port.ser is None or not port.is_open
        rendered = json.dumps(result, indent=2)
        print(rendered)
        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(json.dumps(result, ensure_ascii=False) + "\n")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
