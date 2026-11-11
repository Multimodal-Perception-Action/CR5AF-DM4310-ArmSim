"""Run one guarded 250 ms zero-torque enable test on DM-J4310 motor ID 1.

The T-CAN485 firmware performs all preflight checks and always enters its
disable path locally. This host sends no position target and records USB logs.
"""
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


def guarded_packet(header: bytes, guard: int) -> bytes:
    body = header + b"\x01\x00" + bytes([guard])
    return body + bytes([xor_checksum(body)])


ENABLE_TEST_PACKET = guarded_packet(b"\xD5\x4D\x45\x51", 0xA5)
DISABLE_PACKET = guarded_packet(b"\xD5\x4D\x44\x51", 0x5A)
IDENT_PACKET = guarded_packet(b"\xD5\x4D\x49\x51", 0x60)
TINY_MOVE_PACKET = guarded_packet(b"\xD5\x4D\x4D\x51", 0xB6)


def main() -> int:
    parser = argparse.ArgumentParser(description="Guarded DM-J4310 active test")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--com", default="COM6")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--execute-zero-torque-enable-pulse",
        action="store_true",
        help="required arming flag; enables motor ID 1 for at most 250 ms",
    )
    parser.add_argument(
        "--execute-tiny-move",
        action="store_true",
        help="fixed +0.02 rad ramp and return on motor ID 1",
    )
    args = parser.parse_args()
    if args.execute_zero_torque_enable_pulse == args.execute_tiny_move:
        raise SystemExit(
            "Select exactly one active test flag after clearing the mechanism."
        )

    tiny_move = args.execute_tiny_move
    active_packet = TINY_MOVE_PACKET if tiny_move else ENABLE_TEST_PACKET
    result_prefix = "TINY_MOVE_RESULT " if tiny_move else "SAFE_TEST_RESULT "
    operation_name = "tiny_move" if tiny_move else "zero_torque_enable"

    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": f"DM-J4310 guarded {operation_name}",
        "motor_id": 1,
        "enable_duration_limit_ms": None if tiny_move else 250,
        "commanded_kp": 10.0 if tiny_move else 0.0,
        "commanded_kd": 0.2 if tiny_move else 0.0,
        "commanded_velocity_rad_s": 0.0,
        "commanded_torque_nm": 0.0,
        "position_target_change_rad": 0.02 if tiny_move else None,
        "returns_to_start": tiny_move,
        "request_hex": active_packet.hex(" ").upper(),
        "emergency_disable_hex": DISABLE_PACKET.hex(" ").upper(),
        "firmware_ident_request_hex": IDENT_PACKET.hex(" ").upper(),
    }
    serial_port = None
    tcp_socket = None
    usb_log = bytearray()
    tcp_log = bytearray()
    active_request_sent = False
    result_line = None
    emergency_disable_sent = False
    try:
        serial_port = serial.Serial(args.com, 115200, timeout=0.02)
        serial_port.reset_input_buffer()
        # Opening CH9102 can reset ESP32. Wait before the identity request.
        startup_deadline = time.monotonic() + 4.0
        while time.monotonic() < startup_deadline:
            usb_log.extend(serial_port.read(4096))
        tcp_socket = socket.create_connection((args.host, 60000), timeout=3.0)
        tcp_socket.settimeout(0.02)
        # Query firmware identity over RS485. The gateway prints a USB-only
        # marker and does not touch CAN for this request.
        tcp_socket.sendall(IDENT_PACKET)
        ident_deadline = time.monotonic() + 2.0
        ident_marker = (
            "FIRMWARE_ID guarded_gateway_build_0.8 safe_enable_and_tiny_move"
        )
        while time.monotonic() < ident_deadline:
            usb_log.extend(serial_port.read(4096))
            if ident_marker in usb_log.decode("utf-8", errors="replace"):
                break
        report["guarded_firmware_observed"] = ident_marker in usb_log.decode(
            "utf-8", errors="replace"
        )
        if not report["guarded_firmware_observed"]:
            raise RuntimeError("guarded gateway build 0.8 identity not observed")

        tcp_socket.sendall(active_packet)
        active_request_sent = True
        deadline = time.monotonic() + args.timeout
        pending_text = ""
        while time.monotonic() < deadline:
            serial_bytes = serial_port.read(4096)
            if serial_bytes:
                usb_log.extend(serial_bytes)
                pending_text += serial_bytes.decode("utf-8", errors="replace")
                for line in pending_text.splitlines():
                    if line.startswith(result_prefix):
                        result_line = line.strip()
                if result_line is not None:
                    break
                if "\n" in pending_text:
                    pending_text = pending_text.rsplit("\n", 1)[-1]
            try:
                tcp_bytes = tcp_socket.recv(4096)
                if tcp_bytes:
                    tcp_log.extend(tcp_bytes)
            except socket.timeout:
                pass
        report["result_line"] = result_line
        report["passed"] = bool(
            result_line
            and result_line.startswith(result_prefix + "PASS ")
            and "verified_disabled=1" in result_line
        )
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["passed"] = False
    finally:
        # If the host did not observe the board's completed disable verification,
        # request the separately allowlisted safety disable while TCP is alive.
        if active_request_sent and not report.get("passed") and tcp_socket is not None:
            try:
                tcp_socket.sendall(DISABLE_PACKET)
                emergency_disable_sent = True
                time.sleep(0.25)
                if serial_port is not None:
                    usb_log.extend(serial_port.read(4096))
            except Exception as disable_exc:
                report["emergency_disable_error"] = (
                    f"{type(disable_exc).__name__}: {disable_exc}"
                )
        report["emergency_disable_sent"] = emergency_disable_sent
        report["usb_log"] = usb_log.decode("utf-8", errors="replace")
        report["tcp_response_hex"] = tcp_log.hex(" ").upper()
        if tcp_socket is not None:
            tcp_socket.close()
        if serial_port is not None:
            serial_port.close()

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / (
        f"dm4310_{operation_name}_test_{datetime.now():%Y%m%d_%H%M%S}.json"
    )
    log_file.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
