"""Send one read-only DM4310 register request through CR5AF port 60000."""
from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


def xor_checksum(data: bytes) -> int:
    result = 0
    for value in data:
        result ^= value
    return result


def request_packet(motor_id: int, rid: int) -> bytes:
    body = b"\xD5\x4D\x52\x51" + motor_id.to_bytes(2, "little") + bytes([rid])
    return body + bytes([xor_checksum(body)])


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only DM4310 register probe")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--motor-id", type=lambda x: int(x, 0), default=1)
    parser.add_argument("--rid", type=lambda x: int(x, 0), default=0x0E,
                        help="read-only register, default sw_ver 0x0E")
    parser.add_argument("--com", default="COM6")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    packet = request_packet(args.motor_id, args.rid)
    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "motor_id": args.motor_id,
        "rid": args.rid,
        "request_hex": packet.hex(" ").upper(),
        "operation": "DM parameter read (CAN 0x7FF/opcode 0x33)",
    }
    serial_port = None
    tcp_socket = None
    usb_log = bytearray()
    tcp_reply = bytearray()
    try:
        serial_port = serial.Serial(args.com, 115200, timeout=0.05)
        serial_port.reset_input_buffer()
        # Opening CH9102 may reset the ESP32. Wait until the gateway has fully
        # initialized before asking the robot to emit the RS485 request.
        ready_deadline = time.monotonic() + 4.0
        while time.monotonic() < ready_deadline:
            usb_log.extend(serial_port.read(4096))
            if b"ALLOWLIST:" in usb_log:
                break
        report["gateway_ready_marker"] = b"ALLOWLIST:" in usb_log
        # If opening COM6 did not reset the already-running board, no boot
        # marker will be printed. The four-second quiet period still guarantees
        # that any reset caused by CH9102 has finished before transmission.
        report["gateway_startup_wait_completed"] = True
        tcp_socket = socket.create_connection((args.host, 60000), timeout=3)
        tcp_socket.settimeout(0.05)
        tcp_socket.sendall(packet)
        report["tcp_sent_bytes"] = len(packet)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            usb_log.extend(serial_port.read(4096))
            try:
                part = tcp_socket.recv(4096)
                if part:
                    tcp_reply.extend(part)
            except socket.timeout:
                pass
        report["usb_log"] = usb_log.decode("utf-8", errors="replace")
        report["tcp_reply_hex"] = tcp_reply.hex(" ").upper()
        report["can_response_observed"] = b"CAN_RX" in usb_log
        report["can_ack_observed"] = b"TX_SUCCESS" in usb_log
        report["can_tx_failed_observed"] = b"TX_FAILED" in usb_log
        report["can_bus_error_observed"] = b"BUS_ERROR" in usb_log
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["can_response_observed"] = False
    finally:
        if tcp_socket is not None:
            tcp_socket.close()
        if serial_port is not None:
            serial_port.close()

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"dm4310_read_probe_{datetime.now():%Y%m%d_%H%M%S}.json"
    log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("can_response_observed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
