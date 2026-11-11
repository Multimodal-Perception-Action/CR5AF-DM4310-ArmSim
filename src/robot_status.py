"""Explicit opt-in dashboard read-query client. Never imports or exposes write commands."""
from __future__ import annotations

import argparse
import socket

HOST = "192.168.201.1"
PORT = 29999
# These are documented immediate status/calculation queries only.  No command is sent by default.
READ_ONLY_COMMANDS = {"RobotMode": "RobotMode()", "GetErrorID": "GetErrorID()",
                      "GetPose": "GetPose()", "GetAngle": "GetAngle()"}


def query(host: str, name: str, timeout: float) -> str:
    """Send exactly one allow-listed read query and return its response."""
    if name not in READ_ONLY_COMMANDS:
        raise ValueError("command is not in the read-only allow-list")
    with socket.create_connection((host, PORT), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(READ_ONLY_COMMANDS[name].encode("ascii"))
        return sock.recv(4096).decode("utf-8", errors="replace")


def main() -> int:
    parser = argparse.ArgumentParser(description="Opt-in Dobot dashboard status query")
    parser.add_argument("command", choices=sorted(READ_ONLY_COMMANDS))
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    print(query(args.host, args.command, args.timeout))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
