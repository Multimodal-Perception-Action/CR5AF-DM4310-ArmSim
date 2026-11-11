"""Portable deployment settings. Loading these settings never opens hardware."""
import ipaddress
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def load_connection_settings(root: Path = ROOT) -> dict[str, str]:
    path = root / "config/device_connection.json"
    settings = {"robot_host": "192.168.201.1", "leader_port": "auto"}
    if path.exists():
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
        if not isinstance(payload, dict) or set(payload) - set(settings):
            raise ValueError("device_connection.json requires robot_host and leader_port only")
        settings.update(payload)
    host, port = settings["robot_host"], settings["leader_port"]
    if not isinstance(host, str) or not isinstance(port, str):
        raise ValueError("robot_host and leader_port must be strings")
    ipaddress.IPv4Address(host)
    if port.casefold() != "auto" and not re.fullmatch(r"COM[1-9][0-9]*", port, re.IGNORECASE):
        raise ValueError("leader_port must be auto or an explicit Windows COM port")
    return {"robot_host": host, "leader_port": "auto" if port.casefold() == "auto" else port.upper()}
