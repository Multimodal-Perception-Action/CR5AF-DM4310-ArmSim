"""Passive TCP reachability test for the Dobot controller. Sends no application data."""
from __future__ import annotations

import argparse
import json
import socket
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_HOST = "192.168.201.1"
PORTS = (29999, 30004)


def test_port(host: str, port: int, timeout: float) -> dict[str, object]:
    result: dict[str, object] = {"host": host, "port": port, "sent_bytes": 0}
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            result["connected"] = True
            result["peer"] = "%s:%s" % sock.getpeername()
    except OSError as exc:
        result["connected"] = False
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Dobot passive TCP connectivity test")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    report = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "read_only": True,
              "results": [test_port(args.host, p, args.timeout) for p in PORTS]}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if all(r["connected"] for r in report["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
