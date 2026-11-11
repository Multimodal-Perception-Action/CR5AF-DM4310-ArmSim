"""Identify the U-Arm FE-URT-2 serial port without guessing a COM number."""

from __future__ import annotations

from serial.tools import list_ports
from device_connection import load_connection_settings


WCH_VID = 0x1A86
CH343_PID = 0x55D3


def resolve_uarm_port(requested: str) -> str:
    if requested.casefold() == "auto":
        requested = load_connection_settings()["leader_port"]
    if requested.casefold() != "auto":
        return requested
    matches = [
        info.device
        for info in list_ports.comports()
        if info.vid == WCH_VID
        and info.pid == CH343_PID
    ]
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one U-Arm CH343 adapter (VID:PID 1A86:55D3); "
            f"found {matches}. Set leader_port explicitly in "
            "config/device_connection.json when multiple adapters are connected."
        )
    return matches[0]
