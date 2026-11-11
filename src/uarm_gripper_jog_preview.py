"""Offline/read-only preview of thresholded U-Arm ID7 gripper jog control.

This program never writes a servo register and never sends a DM4310 command.
It can replay a captured U-Arm JSON log or read ID7 live while only simulating
the resulting DM4310 target.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import statistics
import time

from scservo_sdk import PortHandler, sms_sts

from feetech_port import resolve_uarm_port


ROOT = Path(__file__).parents[1]
CONFIG_PATH = ROOT / "config" / "uarm_gripper_calibration.json"


@dataclass
class JogState:
    mode: str = "hold"
    target_rad: float = 0.0


def update_jog(
    state: JogState,
    ticks: int,
    dt: float,
    config: dict,
) -> JogState:
    jog = config["mapping"]["inactive_rate_control_experiment"]
    dm = config["mapping"]["dm4310_target_range"]

    if state.mode == "opening":
        if ticks >= int(jog["open_exit_ticks"]):
            state.mode = "hold"
    elif state.mode == "closing":
        if ticks <= int(jog["close_exit_ticks"]):
            state.mode = "hold"
    else:
        if ticks <= int(jog["open_enter_ticks"]):
            state.mode = "opening"
        elif ticks >= int(jog["close_enter_ticks"]):
            state.mode = "closing"

    safe_open = float(dm["safe_open_command_rad"])
    safe_close = float(dm["safe_close_command_rad"])
    if state.mode == "opening":
        rate = float(jog["open_target_rate_rad_per_s"])
        state.target_rad = max(safe_open, state.target_rad - rate * dt)
    elif state.mode == "closing":
        rate = float(jog["close_target_rate_rad_per_s"])
        state.target_rad = min(safe_close, state.target_rad + rate * dt)
    state.target_rad = min(safe_close, max(safe_open, state.target_rad))
    return state


def replay_values(path: Path) -> tuple[list[int], float]:
    record = json.loads(path.read_text(encoding="utf-8"))
    values = [int(row[6]) for row in record["raw_positions_ticks"]]
    return values, float(record["interval_s"])


def live_values(port_arg: str, samples: int, interval: float) -> list[int]:
    port_name = resolve_uarm_port(port_arg)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    values: list[int] = []
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        for index in range(samples):
            value, result, packet_error = protocol.read2ByteTxRx(7, 56)
            if result != 0 or packet_error != 0:
                raise RuntimeError(
                    f"ID7 read failed result={result} packet_error={packet_error}"
                )
            values.append(int(value) % 4096)
            if index + 1 < samples:
                time.sleep(interval)
    finally:
        if opened and port.ser is not None:
            port.closePort()
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-log", type=Path)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument(
        "--initial-target",
        choices=("open", "mid", "closed"),
        default="mid",
        help="Simulation-only initial target; live control must use DM feedback.",
    )
    args = parser.parse_args()
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    dm = config["mapping"]["dm4310_target_range"]
    safe_open = float(dm["safe_open_command_rad"])
    safe_close = float(dm["safe_close_command_rad"])
    initial = {
        "open": safe_open,
        "mid": (safe_open + safe_close) / 2.0,
        "closed": safe_close,
    }[args.initial_target]

    if args.replay_log:
        values, interval = replay_values(args.replay_log)
        source = str(args.replay_log.resolve())
    else:
        values = live_values(args.port, args.samples, args.interval)
        interval = args.interval
        source = "live_read_only_id7"

    state = JogState(target_rad=initial)
    transitions: list[dict[str, int | float | str]] = []
    previous_mode = state.mode
    minimum_target = state.target_rad
    maximum_target = state.target_rad
    mode_counts = {"opening": 0, "closing": 0, "hold": 0}
    for index, ticks in enumerate(values):
        update_jog(state, ticks, interval, config)
        mode_counts[state.mode] += 1
        minimum_target = min(minimum_target, state.target_rad)
        maximum_target = max(maximum_target, state.target_rad)
        if state.mode != previous_mode:
            transitions.append({
                "sample": index,
                "ticks": ticks,
                "from": previous_mode,
                "to": state.mode,
                "simulated_target_rad": state.target_rad,
            })
            previous_mode = state.mode

    output = {
        "mode": "offline_or_read_only_jog_preview",
        "source": source,
        "samples": len(values),
        "interval_s": interval,
        "input_min_ticks": min(values),
        "input_median_ticks": int(statistics.median(values)),
        "input_max_ticks": max(values),
        "initial_target_rad": initial,
        "final_simulated_target_rad": state.target_rad,
        "minimum_simulated_target_rad": minimum_target,
        "maximum_simulated_target_rad": maximum_target,
        "mode_counts": mode_counts,
        "transitions": transitions,
        "endpoint_saturation_verified": (
            minimum_target >= safe_open and maximum_target <= safe_close
        ),
        "servo_write_commands_sent": 0,
        "dm4310_commands_sent": 0,
        "hardware_output_sent": False,
    }
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
