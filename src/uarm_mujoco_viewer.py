"""Read-only live MuJoCo viewer for the seven-servo U-Arm leader.

IDs 1-6 drive a six-axis SO-101 wrist model. ID7 is shown with a symmetric
gripper placeholder because the exact custom printed trigger is not present in
the available six-axis MJCF. The program only reads COM7 and sends no writes.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any

import mujoco
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
MODEL_PATH = (
    ROOT / "official" / "PathOnRoboticsOSS" / "hardware" / "robots"
    / "so101_6dof_wrist" / "software" / "robot_assets"
    / "so101_PathonAI_6dof_symmetric_gripper.xml"
)
CALIBRATION_PATH = ROOT / "config" / "uarm_cr5_full_calibration.json"
HEALTH_CONFIG_PATH = ROOT / "config" / "uarm_dobot_teleop.json"
GRIPPER_CONFIG_PATH = ROOT / "config" / "uarm_gripper_calibration.json"
JOINT_NAMES = [f"Revolute_{index}" for index in range(1, 7)]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--duration", type=float, default=900.0)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate < 5 or args.rate > 100:
        parser.error("duration must be positive and rate must be within 5..100 Hz")

    calibration = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    health_config = json.loads(HEALTH_CONFIG_PATH.read_text(encoding="utf-8"))
    gripper_config = json.loads(GRIPPER_CONFIG_PATH.read_text(encoding="utf-8"))
    center = [int(value) for value in calibration["leader"]["center_candidate_ticks"]]
    signs = [float(value) for value in calibration["mapping"]["joint_signs"]]
    if len(center) != 6 or len(signs) != 6:
        raise ValueError("calibration must contain six center ticks and signs")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                 for name in JOINT_NAMES]
    if any(value < 0 for value in joint_ids):
        raise RuntimeError("six-axis model joint lookup failed")
    qpos_addresses = [int(model.jnt_qposadr[value]) for value in joint_ids]
    jaw_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_jaw_slider"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_jaw_slider"),
    ]
    jaw_qpos = [int(model.jnt_qposadr[value]) for value in jaw_ids]

    endpoints = gripper_config["endpoints"]
    open_ticks = int(endpoints["maximum_open"]["ticks"])
    close_ticks = int(endpoints["fully_closed"]["ticks"])
    gripper_span = close_ticks - open_ticks
    if gripper_span <= 0:
        raise ValueError("invalid gripper endpoint order")

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    viewer = None
    samples: list[dict[str, Any]] = []
    clipped_counts = [0] * 6
    started = time.monotonic()
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
            raise RuntimeError(f"initial U-Arm read failed: {transport}")
        unwrapped = [shortest_tick_step(center[i], first[i]) for i in range(6)]
        previous = first[:]

        if args.gui:
            from mujoco import viewer as mujoco_viewer

            viewer = mujoco_viewer.launch_passive(model, data)
            viewer.cam.lookat[:] = [0.08, 0.0, 0.16]
            viewer.cam.distance = 0.65
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22

        deadline = time.monotonic() + args.duration
        period = 1.0 / args.rate
        next_tick = time.monotonic()
        sample_number = 0
        while time.monotonic() < deadline and (viewer is None or viewer.is_running()):
            positions, transport = read_positions(protocol)
            if any(
                int(item["comm_result"]) != COMM_SUCCESS
                or int(item["packet_error"]) != 0
                for item in transport
            ):
                raise RuntimeError(f"U-Arm runtime read failed: {transport}")
            commanded: list[float] = []
            displayed: list[float] = []
            for index in range(6):
                unwrapped[index] += shortest_tick_step(previous[index], positions[index])
                angle = signs[index] * unwrapped[index] * DEGREES_PER_TICK * 3.141592653589793 / 180.0
                joint_id = joint_ids[index]
                low, high = map(float, model.jnt_range[joint_id])
                shown = clamp(angle, low, high)
                clipped_counts[index] += int(shown != angle)
                data.qpos[qpos_addresses[index]] = shown
                commanded.append(angle)
                displayed.append(shown)
            previous = positions

            open_fraction = clamp(
                (close_ticks - int(positions[6])) / gripper_span, 0.0, 1.0
            )
            left = -0.013 + 0.026 * open_fraction
            data.qpos[jaw_qpos[0]] = left
            data.qpos[jaw_qpos[1]] = -left
            mujoco.mj_forward(model, data)
            if viewer is not None:
                viewer.sync()

            samples.append({
                "sample": sample_number,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "servo_ticks": positions,
                "unwrapped_relative_ticks": unwrapped[:],
                "requested_joint_rad": commanded,
                "displayed_joint_rad": displayed,
                "gripper_open_fraction": open_fraction,
            })
            sample_number += 1
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        if viewer is not None:
            viewer.close()
        if opened and port.ser is not None:
            port.closePort()

    record = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "read_only_uarm_six_axis_mujoco_viewer",
        "model_path": str(MODEL_PATH),
        "model_provenance": "PathOn Robotics SO-101 6DoF wrist; gripper is a visual placeholder",
        "port": port_name,
        "center_ticks": center,
        "joint_signs": signs,
        "health_warnings": uarm_health_warnings(health, health_config),
        "sample_count": len(samples),
        "clipped_sample_counts": clipped_counts,
        "writes_to_uarm": 0,
        "connections_to_cr5": 0,
        "duration_actual_s": time.monotonic() - started,
        "samples": samples,
    }
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"uarm_mujoco_viewer_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    print(json.dumps({key: value for key, value in record.items() if key != "samples"},
                     ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps({"log": str(path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
