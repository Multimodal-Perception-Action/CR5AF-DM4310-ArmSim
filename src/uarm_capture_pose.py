"""Capture a labeled, read-only U-Arm pose for endpoint calibration."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

from feetech_port import resolve_uarm_port
from uarm_dobot_preview import DEGREES_PER_TICK, SERVO_IDS, read_positions


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args()
    if args.samples < 1 or args.interval < 0:
        parser.error("samples must be positive and interval non-negative")

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    samples: list[list[int]] = []
    transport_samples: list[list[dict[str, int]]] = []
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        for sample_number in range(args.samples):
            positions, transport = read_positions(protocol)
            if any(int(item["comm_result"]) != COMM_SUCCESS for item in transport):
                raise RuntimeError(f"read failure: {transport}")
            samples.append(positions)
            transport_samples.append(transport)
            if sample_number + 1 < args.samples:
                time.sleep(args.interval)
    finally:
        if opened and port.ser is not None:
            port.closePort()

    summary = []
    for index, servo_id in enumerate(SERVO_IDS):
        values = [sample[index] for sample in samples]
        median_ticks = int(statistics.median(values))
        summary.append({
            "id": servo_id,
            "median_ticks": median_ticks,
            "median_single_turn_deg": median_ticks * DEGREES_PER_TICK,
            "minimum_ticks": min(values),
            "maximum_ticks": max(values),
            "peak_to_peak_ticks": max(values) - min(values),
        })

    record = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "read_only_labeled_pose_capture",
        "label": args.label,
        "port": port_name,
        "baud": 1_000_000,
        "samples": args.samples,
        "interval_s": args.interval,
        "write_commands_sent": 0,
        "motion_commands_sent": 0,
        "summary": summary,
        "raw_positions_ticks": samples,
        "transport": transport_samples,
    }
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"uarm_pose_{args.label}_{stamp}.json"
    with log_path.open("x", encoding="utf-8") as log_file:
        json.dump(record, log_file, ensure_ascii=False, indent=2)
    print(json.dumps(record, ensure_ascii=False, indent=2))
    print(json.dumps({"log": str(log_path.resolve())}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
