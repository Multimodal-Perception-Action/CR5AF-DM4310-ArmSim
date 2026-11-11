"""Identify the U-Arm FE-URT-2 serial port without guessing a COM number."""

from __future__ import annotations

from serial.tools import list_ports


WCH_VID = 0x1A86
CH343_PID = 0x55D3
KNOWN_URT2_SERIALS = {
    "5C84288247",  # original FE-URT-2 (COM5)
    "5C84288524",  # replacement FE-URT-2 (COM7)
}


def resolve_uarm_port(requested: str) -> str:
    if requested.casefold() != "auto":
        return requested
    matches = [
        info.device
        for info in list_ports.comports()
        if info.vid == WCH_VID
        and info.pid == CH343_PID
        and (not info.serial_number or info.serial_number in KNOWN_URT2_SERIALS)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one U-Arm CH343 adapter "
            f"(VID:PID 1A86:55D3, serial in {sorted(KNOWN_URT2_SERIALS)}); "
            f"found {matches}"
        )
    return matches[0]
