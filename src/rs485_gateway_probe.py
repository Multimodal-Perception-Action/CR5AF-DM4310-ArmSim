"""Opt-in CR5AF terminal-RS485 to T-CAN485 dry-run probe.

The T-CAN485 must be running the passive audit firmware. That firmware only
prints received RS485 bytes and has CAN transmission disabled at compile time.
"""
from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

DEFAULT_HOST = "192.168.201.1"
DEFAULT_PORT = 60000
DEFAULT_COM = "COM6"
PROBE = bytes.fromhex("D5 4D 01 00 54 45 53 54")


def main() -> int:
    parser = argparse.ArgumentParser(description="Dry-run terminal RS485 gateway probe")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--com", default=DEFAULT_COM)
    parser.add_argument("--serial-baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "robot_host": args.host,
        "robot_port": args.port,
        "serial_port": args.com,
        "probe_hex": PROBE.hex(" ").upper(),
        "can_transmit_expected": False,
    }
    serial_port: serial.Serial | None = None
    tcp_socket: socket.socket | None = None
    received = bytearray()
    try:
        serial_port = serial.Serial(args.com, args.serial_baud, timeout=0.1)
        serial_port.reset_input_buffer()
        tcp_socket = socket.create_connection((args.host, args.port), timeout=3.0)
        tcp_socket.settimeout(1.0)
        tcp_socket.sendall(PROBE)
        report["tcp_sent_bytes"] = len(PROBE)

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            received.extend(serial_port.read(4096))
            if b"D5 4D 01 00 54 45 53 54" in received:
                break
        decoded = received.decode("utf-8", errors="replace")
        report["serial_received_bytes"] = len(received)
        report["serial_text"] = decoded
        report["probe_observed"] = "D5 4D 01 00 54 45 53 54" in decoded
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["probe_observed"] = False
    finally:
        if tcp_socket is not None:
            tcp_socket.close()
        if serial_port is not None:
            serial_port.close()

    logs = Path(__file__).resolve().parents[1] / "logs"
    logs.mkdir(exist_ok=True)
    filename = logs / f"rs485_gateway_probe_{datetime.now():%Y%m%d_%H%M%S}.json"
    filename.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["log_file"] = str(filename)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("probe_observed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
