"""Read-only U-Arm ID7 to DM4310 gripper mapping preview."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import time

from scservo_sdk import PortHandler, sms_sts

from feetech_port import resolve_uarm_port


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "config" / "uarm_gripper_calibration.json"
TICKS_PER_TURN = 4096


def shortest_delta(origin: int, current: int) -> int:
    return ((current - origin + TICKS_PER_TURN // 2) % TICKS_PER_TURN
            - TICKS_PER_TURN // 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.03)
    args = parser.parse_args()
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    open_ticks = int(config["endpoints"]["maximum_open"]["ticks"])
    closed_ticks = int(config["endpoints"]["fully_closed"]["ticks"])
    span = shortest_delta(open_ticks, closed_ticks)
    if span == 0:
        raise RuntimeError("gripper endpoint span is zero")

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    values: list[int] = []
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        for index in range(args.samples):
            value, result, packet_error = protocol.read2ByteTxRx(7, 56)
            if result != 0 or packet_error != 0:
                raise RuntimeError(
                    f"ID7 read failed result={result} packet_error={packet_error}"
                )
            values.append(value % TICKS_PER_TURN)
            if index + 1 < args.samples:
                time.sleep(args.interval)
    finally:
        if opened and port.ser is not None:
            port.closePort()

    current = int(statistics.median(values))
    raw_fraction = shortest_delta(open_ticks, current) / span
    closed_fraction = max(0.0, min(1.0, raw_fraction))
    dm = config["mapping"]["dm4310_target_range"]
    open_rad = float(dm["safe_open_command_rad"])
    close_rad = float(dm["safe_close_command_rad"])
    target_rad = open_rad + closed_fraction * (close_rad - open_rad)
    physical_opening_ticks = max(
        0,
        min(abs(span), -shortest_delta(closed_ticks, current)),
    )
    output = {
        "mode": "read_only_mapping_preview",
        "port": port_name,
        "servo_id": 7,
        "samples": args.samples,
        "current_ticks": current,
        "open_ticks": open_ticks,
        "closed_ticks": closed_ticks,
        "span_ticks": span,
        "physical_opening_ticks_from_closed": physical_opening_ticks,
        "physical_opening_deg_from_closed": physical_opening_ticks * 360.0 / TICKS_PER_TURN,
        "direction_note": (
            "physical angle increases toward open while raw ID7 ticks decrease"
        ),
        "raw_closed_fraction": raw_fraction,
        "closed_fraction": closed_fraction,
        "open_fraction": 1.0 - closed_fraction,
        "dm4310_preview_target_rad": target_rad,
        "hardware_output_sent": False,
    }
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
