"""Request one read-only DM-J4310 status feedback frame via CR5AF RS485."""
from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


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


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def request_packet(motor_id: int) -> bytes:
    body = b"\xD5\x4D\x53\x51" + motor_id.to_bytes(2, "little") + b"\x00"
    return body + bytes([xor_checksum(body)])


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def extract_packet(buffer: bytearray):
    header = b"\xD5\x4D\x52\x50"
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
        packet = bytes(buffer[start:start + 17])
        del buffer[:start + 17]
        if xor_checksum(packet[:16]) == packet[16]:
            return packet


def decode_feedback(packet: bytes, pmax: float, vmax: float, tmax: float):
    data = packet[8:16]
    error = (data[0] >> 4) & 0x0F
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    return {
        "response_can_id": int.from_bytes(packet[4:6], "little"),
        "data_hex": data.hex(" ").upper(),
        "motor_id_low_nibble": data[0] & 0x0F,
        "error_code": error,
        "state": ERROR_STATES.get(error, "unknown"),
        "position_raw": position_raw,
        "position_rad": uint_to_float(position_raw, -pmax, pmax, 16),
        "velocity_raw": velocity_raw,
        "velocity_rad_s": uint_to_float(velocity_raw, -vmax, vmax, 12),
        "torque_raw": torque_raw,
        "torque_nm": uint_to_float(torque_raw, -tmax, tmax, 12),
        "mos_temperature_c": data[6],
        "rotor_temperature_c": data[7],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only DM4310 status probe")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--com", default="COM6")
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--pmax", type=float, default=12.5)
    parser.add_argument("--vmax", type=float, default=30.0)
    parser.add_argument("--tmax", type=float, default=10.0)
    args = parser.parse_args()

    request = request_packet(args.motor_id)
    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "DM status refresh (official opcode 0xCC, read-only)",
        "request_hex": request.hex(" ").upper(),
        "scales": {"PMAX": args.pmax, "VMAX": args.vmax, "TMAX": args.tmax},
    }
    serial_port = None
    tcp_socket = None
    usb_log = bytearray()
    tcp_buffer = bytearray()
    try:
        serial_port = serial.Serial(args.com, 115200, timeout=0.02)
        serial_port.reset_input_buffer()
        startup_deadline = time.monotonic() + 4.0
        while time.monotonic() < startup_deadline:
            usb_log.extend(serial_port.read(4096))

        tcp_socket = socket.create_connection((args.host, 60000), timeout=3.0)
        tcp_socket.settimeout(0.05)
        tcp_socket.sendall(request)
        report["tcp_sent_bytes"] = len(request)
        deadline = time.monotonic() + args.timeout
        response = None
        while time.monotonic() < deadline:
            usb_log.extend(serial_port.read(4096))
            try:
                data = tcp_socket.recv(4096)
                if data:
                    tcp_buffer.extend(data)
            except socket.timeout:
                pass
            response = extract_packet(tcp_buffer)
            if response is not None:
                break
        report["usb_log"] = usb_log.decode("utf-8", errors="replace")
        report["response_observed"] = response is not None
        if response is not None:
            report["response_packet_hex"] = response.hex(" ").upper()
            report["feedback"] = decode_feedback(
                response, args.pmax, args.vmax, args.tmax
            )
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["response_observed"] = False
    finally:
        if tcp_socket is not None:
            tcp_socket.close()
        if serial_port is not None:
            serial_port.close()

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"dm4310_status_probe_{datetime.now():%Y%m%d_%H%M%S}.json"
    log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("response_observed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
