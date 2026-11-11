"""Receive-only parser/logger for documented 1440-byte Dobot V4 feedback on TCP/30004."""
from __future__ import annotations

import argparse
import json
import socket
import struct
from datetime import datetime, timezone
from pathlib import Path

HOST = "192.168.201.1"
PORT = 30004
# Contract: Dobot TCP/IP Remote Control Interface Guide V4.5.1, pp. 100-103.
# The feedback frame carries no independent firmware/protocol-version field.  This
# parser therefore validates the V4.5.1 layout contract, not a guessed firmware.
PROTOCOL_LAYOUT = "Dobot TCP/IP V4.5.1 / feedback layout 1440 bytes"
FRAME_BYTES = 1440
TEST_VALUE = 0x0123456789ABCDEF
OFFSETS = {
    "message_size": 0, "digital_inputs": 8, "digital_outputs": 16,
    "robot_mode": 24, "timestamp_ms": 32, "runtime_ms": 40,
    "test_value": 48, "speed_scaling": 64, "q_actual_deg": 432,
    "tool_vector_actual": 624, "enable_status": 1026,
    "running_status": 1028, "error_status": 1029,
}


def recv_exact(sock: socket.socket, size: int) -> bytes:
    """Read exactly size bytes or raise; performs no send/retry."""
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        data = sock.recv(remaining)
        if not data:
            raise ConnectionError(f"peer closed with {remaining} bytes still required")
        chunks.append(data)
        remaining -= len(data)
    return b"".join(chunks)


def decode_v4(frame: bytes) -> dict[str, object]:
    """Decode only documented V4.5.1 fields; no inferred offsets are used."""
    if len(frame) != FRAME_BYTES:
        raise ValueError(f"expected {FRAME_BYTES} bytes, received {len(frame)}")
    u16 = lambda offset: struct.unpack_from("<H", frame, offset)[0]
    u64 = lambda offset: struct.unpack_from("<Q", frame, offset)[0]
    f64x6 = lambda offset: list(struct.unpack_from("<6d", frame, offset))
    message_size = u16(OFFSETS["message_size"])
    test_value = u64(OFFSETS["test_value"])
    if message_size != FRAME_BYTES:
        raise ValueError(f"V4 MessageSize is {message_size}, expected {FRAME_BYTES}")
    if test_value != TEST_VALUE:
        raise ValueError(f"V4 TestValue mismatch: 0x{test_value:016X}")
    return {
        "protocol_layout": PROTOCOL_LAYOUT,
        "firmware_version_from_packet": None,
        "firmware_version_note": "feedback payload has no version field; verify firmware in DobotStudio Pro",
        "message_size": message_size,
        "digital_inputs": u64(OFFSETS["digital_inputs"]),
        "digital_outputs": u64(OFFSETS["digital_outputs"]),
        "robot_mode": u64(OFFSETS["robot_mode"]),
        "timestamp_ms": u64(OFFSETS["timestamp_ms"]),
        "runtime_ms": u64(OFFSETS["runtime_ms"]),
        "test_value": test_value,
        "test_value_valid": True,
        "speed_scaling": struct.unpack_from("<d", frame, OFFSETS["speed_scaling"])[0],
        "q_actual_deg": f64x6(OFFSETS["q_actual_deg"]),
        "tool_vector_actual": f64x6(OFFSETS["tool_vector_actual"]),
        "enable_status": frame[OFFSETS["enable_status"]],
        "running_status": frame[OFFSETS["running_status"]],
        "error_status": frame[OFFSETS["error_status"]],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Passive Dobot TCP/30004 feedback capture")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--packets", type=int, default=3)
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--receive-timeout", type=float, default=3.0)
    parser.add_argument("--log-dir", default=str(Path(__file__).parents[1] / "logs"))
    args = parser.parse_args()
    if args.packets < 1:
        parser.error("--packets must be positive")
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"feedback_30004_{stamp}.jsonl"
    sock: socket.socket | None = None
    try:
        sock = socket.create_connection((args.host, PORT), timeout=args.connect_timeout)
        sock.settimeout(args.receive_timeout)
        with path.open("x", encoding="utf-8") as log:
            for sequence in range(args.packets):
                frame = recv_exact(sock, FRAME_BYTES)
                record = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "sequence": sequence,
                          "read_only": True, "sent_bytes": 0, "received_bytes": len(frame),
                          "raw_hex": frame.hex(), "decoded": decode_v4(frame)}
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                print(json.dumps({k: v for k, v in record.items() if k != "raw_hex"}, ensure_ascii=False))
    finally:
        if sock is not None:
            sock.close()
    print(f"raw capture written: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
