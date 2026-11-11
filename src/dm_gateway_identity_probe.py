"""Read-only DM gateway identity query over TCP/60000; sends no motor/CAN command."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import sys

from gripper import EVENTS, GATEWAY_HEADER, PACKETS, decode_gateway_event, receive_packet


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.201.1"
    sock: socket.socket | None = None
    result: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "probe": "gateway identity only",
        "host": host,
        "port": 60000,
        "request_hex": PACKETS["identity"].hex(" ").upper(),
        "motor_command_sent": False,
        "can_status_request_sent": False,
    }
    try:
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(0.05)
        sock.sendall(PACKETS["identity"])
        packet = receive_packet(sock, bytearray(), GATEWAY_HEADER, 3.0)
        decoded = decode_gateway_event(packet)
        result["response_hex"] = packet.hex(" ").upper()
        result["decoded"] = decoded
        result["passed"] = (
            decoded["event"] == EVENTS["identity"] and decoded["result"] == 1
        )
    except Exception as exc:
        result["passed"] = False
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if sock is not None:
            sock.close()

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"dm_gateway_identity_{datetime.now():%Y%m%d_%H%M%S}.json"
    result["log_file"] = str(log_path)
    log_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
