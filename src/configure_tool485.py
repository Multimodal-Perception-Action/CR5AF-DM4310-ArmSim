"""Apply the explicitly selected CR5AF tool-RS485 serial format."""
from __future__ import annotations

import argparse
import json
import socket
from datetime import datetime, timezone
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Configure tool RS485 as 115200/N/1")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    command = b'SetTool485(115200,"N",1)'
    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "host": args.host,
        "port": 29999,
        "command": command.decode("ascii"),
        "scope": "tool RS485 serial format only; no enable/motion/project command",
    }
    sock = None
    try:
        sock = socket.create_connection((args.host, 29999), timeout=args.timeout)
        sock.settimeout(args.timeout)
        sock.sendall(command)
        report["response"] = sock.recv(4096).decode("ascii", errors="replace")
        report["success"] = str(report["response"]).startswith("0,")
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["success"] = False
    finally:
        if sock is not None:
            sock.close()

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"configure_tool485_{datetime.now():%Y%m%d_%H%M%S}.json"
    log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
