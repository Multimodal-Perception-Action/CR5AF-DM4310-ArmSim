"""Read a fixed allowlist of DM-J4310 identity/configuration registers."""
from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


REGISTERS = {
    0x07: "MST_ID",
    0x08: "ESC_ID",
    0x09: "TIMEOUT",
    0x0A: "CTRL_MODE",
    0x0D: "hw_ver",
    0x0E: "sw_ver",
    0x15: "PMAX",
    0x16: "VMAX",
    0x17: "TMAX",
    0x23: "can_br",
    0x24: "sub_ver",
    0x25: "Boot_ver",
}
FLOAT_REGISTERS = {0x15, 0x16, 0x17}
CONTROL_MODES = {
    1: "MIT",
    2: "position-velocity",
    3: "velocity",
    4: "torque-position mixed",
}
CAN_BITRATES = {
    0: "125 kbit/s Classic CAN",
    1: "200 kbit/s Classic CAN",
    2: "250 kbit/s Classic CAN",
    3: "500 kbit/s Classic CAN",
    4: "1 Mbit/s Classic CAN",
    5: "2 Mbit/s CAN FD",
    6: "2.5 Mbit/s CAN FD",
    7: "3.2 Mbit/s CAN FD",
    8: "4 Mbit/s CAN FD",
    9: "5 Mbit/s CAN FD",
}


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def request_packet(motor_id: int, rid: int) -> bytes:
    body = b"\xD5\x4D\x52\x51" + motor_id.to_bytes(2, "little") + bytes([rid])
    return body + bytes([xor_checksum(body)])


def extract_response(buffer: bytearray, expected_motor_id: int, expected_rid: int):
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
        if xor_checksum(packet[:16]) != packet[16]:
            continue
        frame_data = packet[8:16]
        motor_id = int.from_bytes(frame_data[0:2], "little")
        if (motor_id == expected_motor_id and frame_data[2] == 0x33
                and frame_data[3] == expected_rid):
            return packet, frame_data


def decode_register(rid: int, value_bytes: bytes) -> dict[str, object]:
    result: dict[str, object] = {
        "value_hex": value_bytes.hex(" ").upper(),
    }
    if rid in FLOAT_REGISTERS:
        result["float32_le"] = struct.unpack("<f", value_bytes)[0]
        return result
    result["uint32_le"] = int.from_bytes(value_bytes, "little")
    if all(0x20 <= byte <= 0x7E for byte in value_bytes):
        result["ascii"] = value_bytes.decode("ascii")
    if rid == 0x23:
        result["can_bitrate"] = CAN_BITRATES.get(result["uint32_le"], "unknown")
    if rid == 0x0A:
        result["control_mode"] = CONTROL_MODES.get(result["uint32_le"], "unknown/reserved")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only DM4310 register audit")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--com", default="COM6")
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "fixed allowlist of DM parameter reads only",
        "motor_id": args.motor_id,
        "registers": {},
    }
    serial_port = None
    tcp_socket = None
    tcp_buffer = bytearray()
    try:
        serial_port = serial.Serial(args.com, 115200, timeout=0.02)
        serial_port.reset_input_buffer()
        # CH9102 can reset the ESP32 when COM6 opens. Wait for any reset to end.
        startup_deadline = time.monotonic() + 4.0
        while time.monotonic() < startup_deadline:
            serial_port.read(4096)

        tcp_socket = socket.create_connection((args.host, 60000), timeout=3.0)
        tcp_socket.settimeout(0.05)
        for rid, name in REGISTERS.items():
            packet = request_packet(args.motor_id, rid)
            tcp_socket.sendall(packet)
            entry: dict[str, object] = {
                "rid": f"0x{rid:02X}",
                "request_hex": packet.hex(" ").upper(),
            }
            usb_log = bytearray()
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
                response = extract_response(tcp_buffer, args.motor_id, rid)
                if response is not None:
                    break
            entry["usb_log"] = usb_log.decode("utf-8", errors="replace")
            if response is None:
                entry["response_observed"] = False
            else:
                response_packet, frame_data = response
                entry["response_observed"] = True
                entry["response_packet_hex"] = response_packet.hex(" ").upper()
                entry["response_can_id"] = int.from_bytes(response_packet[4:6], "little")
                entry.update(decode_register(rid, frame_data[4:8]))
            report["registers"][name] = entry
            time.sleep(0.05)
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if tcp_socket is not None:
            tcp_socket.close()
        if serial_port is not None:
            serial_port.close()

    register_results = report["registers"]
    report["all_responses_observed"] = (
        len(register_results) == len(REGISTERS)
        and all(entry.get("response_observed") for entry in register_results.values())
    )
    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"dm4310_register_audit_{datetime.now():%Y%m%d_%H%M%S}.json"
    log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["all_responses_observed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
