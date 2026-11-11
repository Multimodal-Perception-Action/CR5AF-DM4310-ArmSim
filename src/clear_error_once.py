"""Send exactly one Dobot ClearError() command and record the raw reply."""

from __future__ import annotations

import argparse
import json
import socket
from datetime import datetime, timezone
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--log", default=str(Path(__file__).parents[1] / "logs" / "clear_error.jsonl"))
    args = parser.parse_args()

    sock: socket.socket | None = None
    record: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "host": args.host,
        "port": 29999,
        "command": "ClearError()",
        "attempts": 1,
        "enable_sent": False,
        "motion_sent": False,
    }
    try:
        sock = socket.create_connection((args.host, 29999), timeout=args.timeout)
        sock.settimeout(args.timeout)
        sock.sendall(b"ClearError()")
        reply = sock.recv(4096).decode("utf-8", errors="replace").strip()
        record["reply"] = reply
        try:
            record["error_id"] = int(reply.split(",", 1)[0])
        except ValueError:
            record["error_id"] = None
        return_code = 0 if record["error_id"] == 0 else 1
    except Exception as exc:
        record["exception"] = f"{type(exc).__name__}: {exc}"
        return_code = 2
    finally:
        if sock is not None:
            sock.close()
        record["socket_closed"] = True
        path = Path(args.log)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as output:
            output.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(json.dumps(record, ensure_ascii=False, indent=2))
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
