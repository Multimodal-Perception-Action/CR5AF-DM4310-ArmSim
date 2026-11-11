"""Read-only capture of U-Arm ID1-6 travel about a confirmed center.

The operator manually sweeps the leader joints. This program only reads servo
positions and health; it never writes a register and never connects to the CR5.
Wrapped 0..4095 encoder values are unwrapped about the configured center.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import time
from typing import Any

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

from feetech_port import resolve_uarm_port
from uarm_dobot_preview import (
    DEGREES_PER_TICK,
    read_positions,
    read_servo_health,
    shortest_tick_step,
)
from uarm_dobot_teleop import require_uarm_health, uarm_health_warnings


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = ROOT / "config" / "uarm_cr5_full_calibration.json"
HEALTH_CONFIG_PATH = ROOT / "config" / "uarm_dobot_teleop.json"


def percentile(values: list[int], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot calculate percentile of empty values")
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--axis", default="all", choices=["all", "1", "2", "3", "4", "5", "6"],
        help="capture one leader axis at a time, or all six",
    )
    parser.add_argument("--center-tolerance-ticks", type=int, default=30)
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 180:
        parser.error("--duration must be within (0, 180] seconds")
    if args.rate < 10 or args.rate > 100:
        parser.error("--rate must be within 10..100 Hz")
    if args.center_tolerance_ticks < 1 or args.center_tolerance_ticks > 100:
        parser.error("--center-tolerance-ticks must be within 1..100")

    calibration = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    health_config = json.loads(HEALTH_CONFIG_PATH.read_text(encoding="utf-8"))
    center = [int(value) for value in calibration["leader"]["center_candidate_ticks"]]
    if len(center) != 6:
        raise ValueError("calibration must contain six leader center ticks")
    selected_indices = list(range(6)) if args.axis == "all" else [int(args.axis) - 1]

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    samples: list[dict[str, Any]] = []
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        health = read_servo_health(protocol)
        require_uarm_health(health, health_config)
        first, transport = read_positions(protocol)
        if any(
            int(item["comm_result"]) != COMM_SUCCESS
            or int(item["packet_error"]) != 0
            for item in transport
        ):
            raise RuntimeError(f"initial position read failed: {transport}")
        initial_error = [shortest_tick_step(center[i], first[i]) for i in range(6)]
        if any(abs(value) > args.center_tolerance_ticks for value in initial_error):
            raise RuntimeError(
                "leader is not at the configured center candidate; "
                f"relative ticks={initial_error}, tolerance={args.center_tolerance_ticks}"
            )

        unwrapped = initial_error[:]
        previous = first[:]
        deadline = time.monotonic() + args.duration
        period = 1.0 / args.rate
        next_tick = time.monotonic()
        sample_number = 0
        while time.monotonic() < deadline:
            current, transport = read_positions(protocol)
            if any(
                int(item["comm_result"]) != COMM_SUCCESS
                or int(item["packet_error"]) != 0
                for item in transport
            ):
                raise RuntimeError(f"runtime position read failed: {transport}")
            for index in range(6):
                unwrapped[index] += shortest_tick_step(previous[index], current[index])
            previous = current
            samples.append({
                "sample": sample_number,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "positions_ticks": current,
                "relative_unwrapped_ticks": unwrapped[:],
            })
            sample_number += 1
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        if opened and port.ser is not None:
            port.closePort()

    summary = []
    axes_with_both_sides = 0
    for index in range(6):
        values = [int(sample["relative_unwrapped_ticks"][index]) for sample in samples]
        minimum = min(values)
        maximum = max(values)
        robust_minimum = percentile(values, 0.01)
        robust_maximum = percentile(values, 0.99)
        both_sides = minimum < -10 and maximum > 10
        if index in selected_indices:
            axes_with_both_sides += int(both_sides)
        summary.append({
            "servo_id": index + 1,
            "selected_for_this_capture": index in selected_indices,
            "minimum_relative_ticks": minimum,
            "maximum_relative_ticks": maximum,
            "robust_p01_relative_ticks": robust_minimum,
            "robust_p99_relative_ticks": robust_maximum,
            "minimum_relative_deg": minimum * DEGREES_PER_TICK,
            "maximum_relative_deg": maximum * DEGREES_PER_TICK,
            "both_sides_observed": both_sides,
        })

    record = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "read_only_uarm_manual_range_capture",
        "port": port_name,
        "duration_s": args.duration,
        "rate_hz": args.rate,
        "selected_axis": args.axis,
        "center_ticks": center,
        "health": health,
        "health_warnings": uarm_health_warnings(health, health_config),
        "write_commands_sent": 0,
        "motion_commands_sent": 0,
        "cr5_connections_opened": 0,
        "selected_axes_with_both_sides": axes_with_both_sides,
        "complete_selected_sweep": axes_with_both_sides == len(selected_indices),
        "summary": summary,
        "samples": samples,
    }
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"uarm_range_capture_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    print(json.dumps({key: value for key, value in record.items() if key != "samples"},
                     ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps({"log": str(path)}, ensure_ascii=False))
    return 0 if record["complete_selected_sweep"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
