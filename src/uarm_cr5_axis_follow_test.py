"""Guarded single-axis CR5 mapping and U-Arm follow smoke test.

Default mode is read-only and never opens TCP/29999.  Live motion requires all
three explicit interlocks.  Only one selected U-Arm ID / CR5 joint pair is
used; the other five CR5 joints are held at the session baseline.  The selected
target is hard-clamped to +/-1 degree and command slew is limited to 3 deg/s.
Optional model-target mode tests the virtual-to-hardware offset inverse with a
single +/-1 degree model-space step and automatic guarded return.

This tool never requests control mode, clears errors, resets the robot, or runs
a project.  Live cleanup attempts a guarded return to the baseline when robot
feedback is healthy, then always attempts Stop and DisableRobot and verifies
the disabled feedback state.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
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
from uarm_dobot_teleop import (
    FeedbackStream,
    load_config,
    require_uarm_health,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "uarm_dobot_teleop.json"
DEFAULT_MODEL_MAPPING = ROOT / "config" / "uarm_cr5_simulation.json"
LIVE_CONFIRM = "I_UNDERSTAND_UARM_CR5_AXIS_WILL_MOVE"
JOINT_LIMITS_DEG = (
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-160.0, 160.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
)
LIMIT_MARGIN_DEG = 5.0
MAX_RELATIVE_DEG = 1.0
MAX_SPEED_DEG_S = 3.0
MINIMUM_LIVE_INPUT_EXCURSION_DEG = 0.5
MINIMUM_LIVE_ROBOT_EXCURSION_DEG = 0.3
RATE_HZ = 20.0
PERIOD_S = 1.0 / RATE_HZ


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class Dashboard:
    """Strict command allow-list for this single-axis test."""

    ALLOWED_EXACT = {
        "RobotMode()",
        "EnableRobot()",
        "Stop()",
        "DisableRobot()",
    }

    def __init__(self, host: str) -> None:
        self.socket = socket.create_connection((host, 29999), timeout=3.0)
        self.socket.settimeout(3.0)

    def command(self, command: str) -> str:
        if command not in self.ALLOWED_EXACT and not command.startswith("ServoJ("):
            raise ValueError(f"command outside axis-follow allow-list: {command}")
        self.socket.sendall((command.rstrip() + "\n").encode("ascii"))
        response = bytearray()
        while len(response) < 4096 and b";" not in response:
            chunk = self.socket.recv(1024)
            if not chunk:
                raise ConnectionError("TCP/29999 closed before complete reply")
            response.extend(chunk)
        reply = response.decode("utf-8", errors="replace").strip()
        try:
            error_id = int(reply.split(",", 1)[0])
        except (ValueError, IndexError) as exc:
            raise RuntimeError(f"unparseable Dobot reply: {reply}") from exc
        if error_id != 0:
            raise RuntimeError(f"Dobot rejected {command}: {reply}")
        return reply

    def close(self) -> None:
        self.socket.close()


def require_idle_preflight(state: dict[str, Any]) -> None:
    failures: list[str] = []
    mode = int(state["robot_mode"])
    enabled = int(state["enable_status"])
    if (mode, enabled) not in ((4, 0), (5, 1)):
        failures.append(f"mode/enable={mode}/{enabled}, expected idle 4/0 or 5/1")
    if int(state["running_status"]) != 0:
        failures.append("RunningStatus is not 0")
    if int(state["error_status"]) != 0:
        failures.append("ErrorStatus is nonzero")
    if failures:
        raise RuntimeError("idle preflight rejected: " + "; ".join(failures))


def require_disabled_preflight(state: dict[str, Any]) -> None:
    """Strict final-state check used by the continuous controller cleanup."""
    failures: list[str] = []
    if int(state["robot_mode"]) == 11:
        failures.append("RobotMode=11 indicates collision detection; clear only after the area is safe")
    elif int(state["robot_mode"]) != 4:
        failures.append(f"RobotMode={state['robot_mode']}, expected 4")
    if int(state["enable_status"]) != 0:
        failures.append("EnableStatus is not 0")
    if int(state["running_status"]) != 0:
        failures.append("RunningStatus is not 0")
    if int(state["error_status"]) != 0:
        failures.append("ErrorStatus is nonzero")
    if failures:
        raise RuntimeError("disabled preflight rejected: " + "; ".join(failures))


def require_enabled_healthy(state: dict[str, Any]) -> None:
    failures: list[str] = []
    if int(state["robot_mode"]) == 11:
        failures.append("RobotMode=11 indicates collision detection")
    elif int(state["robot_mode"]) not in (5, 7):
        failures.append(f"RobotMode={state['robot_mode']}, expected 5 or 7")
    if int(state["enable_status"]) != 1:
        failures.append("EnableStatus is not 1")
    if int(state["error_status"]) != 0:
        failures.append("ErrorStatus is nonzero")
    if failures:
        raise RuntimeError("enabled-state rejected: " + "; ".join(failures))


def wait_enable(
    feedback: FeedbackStream, enabled: bool, timeout_s: float = 8.0
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    last: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        last = feedback.latest()
        if int(last["error_status"]) != 0:
            raise RuntimeError("CR5 ErrorStatus became nonzero")
        if int(last["enable_status"]) == int(enabled):
            if not enabled or int(last["robot_mode"]) in (5, 7):
                return last
        time.sleep(0.02)
    raise TimeoutError(
        f"timed out waiting for EnableStatus={int(enabled)}; last={last}"
    )


def validate_pose(values: list[float]) -> None:
    if len(values) != 6:
        raise ValueError("expected six CR5 joint values")
    failures: list[str] = []
    for index, (value, limits) in enumerate(zip(values, JOINT_LIMITS_DEG), start=1):
        low, high = limits
        if not low + LIMIT_MARGIN_DEG <= value <= high - LIMIT_MARGIN_DEG:
            failures.append(f"J{index}={value:.4f} outside guarded limits")
    if failures:
        raise RuntimeError("; ".join(failures))


def servo_command(joints: list[float]) -> str:
    validate_pose(joints)
    return (
        "ServoJ(" + ",".join(f"{value:.6f}" for value in joints)
        + f",t={PERIOD_S:.3f},aheadtime=50,gain=250)"
    )


def slew(previous: list[float], target: list[float]) -> list[float]:
    maximum_step = MAX_SPEED_DEG_S * PERIOD_S
    result: list[float] = []
    for old, new in zip(previous, target):
        delta = max(-maximum_step, min(maximum_step, new - old))
        result.append(old + delta)
    return result


def compact_state(state: dict[str, Any]) -> dict[str, Any]:
    return {
        "robot_mode": state["robot_mode"],
        "enable_status": state["enable_status"],
        "running_status": state["running_status"],
        "error_status": state["error_status"],
        "q_actual_deg": state["q_actual_deg"],
        "timestamp_ms": state["timestamp_ms"],
    }


def safe_return_to_baseline(
    dashboard: Dashboard,
    feedback: FeedbackStream,
    baseline: list[float],
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    """Return at <=3 deg/s only while fresh feedback remains healthy."""

    state = feedback.latest()
    require_enabled_healthy(state)
    previous = [float(value) for value in state["q_actual_deg"]]
    validate_pose(baseline)
    deadline = time.monotonic() + timeout_s
    commands = 0
    best_error = max(abs(a - b) for a, b in zip(previous, baseline))
    while time.monotonic() < deadline:
        state = feedback.latest()
        require_enabled_healthy(state)
        actual = [float(value) for value in state["q_actual_deg"]]
        error = max(abs(a - b) for a, b in zip(actual, baseline))
        best_error = min(best_error, error)
        if error <= 0.20 and int(state["running_status"]) == 0:
            return {
                "passed": True,
                "commands": commands,
                "maximum_error_deg": error,
                "state": compact_state(state),
            }
        previous = slew(previous, baseline)
        dashboard.command(servo_command(previous))
        commands += 1
        time.sleep(PERIOD_S)
    raise TimeoutError(
        f"baseline return timed out; best maximum error={best_error:.4f} deg"
    )


def write_report(report: dict[str, Any]) -> Path:
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"uarm_cr5_axis_follow_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", type=int, required=True, choices=range(1, 7))
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--live-motion", action="store_true")
    parser.add_argument("--enable-robot", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--model-step-deg", type=float, choices=(-1.0, 1.0),
        help="live-only +/-1 degree model-space target step; inverse offsets map it to CR5 joint targets",
    )
    parser.add_argument("--model-mapping", type=Path, default=DEFAULT_MODEL_MAPPING)
    parser.add_argument(
        "--use-simulation-mapping", action="store_true",
        help="use the six-axis signs/scales from the explicitly simulation-only mapping for this guarded live axis test",
    )
    parser.add_argument(
        "--continuous-ui-mapping", type=Path,
        help="use the saved real teleoperation-panel signs/scales for this guarded live axis test",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 10.0:
        parser.error("--duration must be within (0, 10] seconds")
    if args.live_motion and (
        not args.enable_robot or args.confirm != LIVE_CONFIRM
    ):
        parser.error(
            "live motion requires --enable-robot and "
            f"--confirm {LIVE_CONFIRM}"
        )
    if not args.live_motion and (args.enable_robot or args.confirm):
        parser.error("enable/confirmation options require --live-motion")
    if args.model_step_deg is not None and not args.live_motion:
        parser.error("--model-step-deg requires --live-motion")

    config = load_config(args.config)
    joint_index = args.joint - 1
    sign = float(config["joint_signs"][joint_index])
    scale = 1.0
    model_offsets = [0.0] * 6
    if args.use_simulation_mapping:
        mapping = json.loads(args.model_mapping.read_text(encoding="utf-8"))
        if mapping.get("purpose") != "simulation_only":
            parser.error("simulation mapping must be explicitly marked simulation_only")
        signs = mapping.get("joint_signs", [])
        scales = mapping.get("joint_scales", [1.0] * 6)
        if (len(signs) != 6 or len(scales) != 6
                or any(float(value) not in (-1.0, 1.0) for value in signs)
                or any(not math.isfinite(float(value)) or not 0.1 <= float(value) <= 2.0 for value in scales)):
            parser.error("simulation mapping must contain six valid signs and scales")
        sign = float(signs[joint_index])
        scale = float(scales[joint_index])
    mapping_source = "simulation_only" if args.use_simulation_mapping else "teleop_config"
    if args.use_simulation_mapping and args.continuous_ui_mapping:
        parser.error("choose either simulation mapping or continuous UI mapping")
    if args.continuous_ui_mapping:
        mapping = json.loads(args.continuous_ui_mapping.read_text(encoding="utf-8"))
        signs = mapping.get("joint_signs", [])
        scales = mapping.get("joint_scales", [1.0] * 6)
        if (len(signs) != 6 or len(scales) != 6
                or any(float(value) not in (-1.0, 1.0) for value in signs)
                or any(not math.isfinite(float(value)) or not 0.1 <= float(value) <= 2.0 for value in scales)):
            parser.error("continuous UI mapping must contain six valid signs and scales")
        sign = float(signs[joint_index])
        scale = float(scales[joint_index])
        mapping_source = "continuous_teleop_ui"
    if args.model_step_deg is not None:
        mapping = json.loads(args.model_mapping.read_text(encoding="utf-8"))
        if mapping.get("purpose") != "simulation_only":
            parser.error("model mapping must be explicitly marked simulation_only")
        model_offsets = [float(value) for value in mapping.get("joint_offsets_deg", [])]
        if len(model_offsets) != 6 or not all(math.isfinite(value) for value in model_offsets):
            parser.error("model mapping must define six finite joint_offsets_deg")
        sign = float(mapping["joint_signs"][joint_index])
        scale = float(mapping.get("joint_scales", [1.0] * 6)[joint_index])
    report: dict[str, Any] = {
        "timestamp_utc": utc_now(),
        "operation": (
            "CR5 single-axis model-to-hardware target mapping test"
            if args.model_step_deg is not None
            else "U-Arm to CR5 guarded single-axis follow smoke test"
        ),
        "live_motion": bool(args.live_motion),
        "selected_servo_id": args.joint,
        "selected_cr5_joint": args.joint,
        "target_coordinate_mode": (
            "model_target_inverse_offset_to_hardware" if args.model_step_deg is not None
            else "session_relative_hardware_joint"
        ),
        "model_target_step_deg": args.model_step_deg,
        "model_to_hardware_offsets_deg": model_offsets,
        "model_to_hardware_formula": "q_hardware_deg = q_model_deg - offset_deg",
        "mapping_sign": sign,
        "mapping_scale": scale,
        "mapping_source": mapping_source,
        "maximum_relative_deg": MAX_RELATIVE_DEG,
        "maximum_speed_deg_s": MAX_SPEED_DEG_S,
        "rate_hz": RATE_HZ,
        "commands_never_used": [
            "RequestControl", "ClearError", "ResetRobot", "RunScript"
        ],
        "samples": [],
        "passed": False,
    }
    port_name = None if args.model_step_deg is not None else resolve_uarm_port(args.port)
    port = PortHandler(port_name) if port_name is not None else None
    protocol = sms_sts(port) if port is not None else None
    feedback: FeedbackStream | None = None
    dashboard: Dashboard | None = None
    opened = False
    enable_requested = False
    baseline: list[float] | None = None
    caught: BaseException | None = None
    return_code = 1
    maximum_input_excursion = 0.0
    maximum_robot_excursion = 0.0
    try:
        if port is not None:
            opened = bool(port.openPort())
            if not opened or not port.setBaudRate(1_000_000):
                raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        feedback = FeedbackStream(args.host, timeout=3.0)
        initial = feedback.wait_first(3.0)
        require_idle_preflight(initial)
        baseline = [float(value) for value in initial["q_actual_deg"]]
        validate_pose(baseline)
        if port is not None and protocol is not None:
            health = read_servo_health(protocol)
            require_uarm_health(health, config)
            positions, transport = read_positions(protocol)
            if any(
                int(item["comm_result"]) != COMM_SUCCESS
                or int(item["packet_error"]) != 0
                for item in transport
            ):
                raise RuntimeError(f"initial U-Arm position read failed: {transport}")
            previous_tick = int(positions[joint_index])
        else:
            health = None
            positions = None
            previous_tick = None
        accumulated_ticks = 0
        previous_command = baseline[:]
        report["uarm_port"] = port_name
        report["initial"] = compact_state(initial)
        report["uarm_health"] = health
        report["uarm_baseline_ticks"] = positions
        model_baseline = [baseline[i] + model_offsets[i] for i in range(6)]
        report["model_baseline_deg"] = model_baseline
        report["planned_target_range_deg"] = [
            baseline[joint_index] - MAX_RELATIVE_DEG,
            baseline[joint_index] + MAX_RELATIVE_DEG,
        ]

        if args.live_motion:
            dashboard = Dashboard(args.host)
            report["robot_mode_reply"] = dashboard.command("RobotMode()")
            if int(initial["enable_status"]) == 0:
                enable_requested = True
                report["enable_reply"] = dashboard.command("EnableRobot()")
                enabled_state = wait_enable(feedback, True)
            else:
                enabled_state = feedback.latest()
                report["enable_skipped_already_enabled"] = True
                enable_requested = True
            require_enabled_healthy(enabled_state)
            report["enabled_state"] = compact_state(enabled_state)

        started = time.monotonic()
        deadline = started + args.duration
        next_tick = time.monotonic()
        sample_number = 0
        while time.monotonic() < deadline:
            if args.model_step_deg is None:
                current, transport = read_positions(protocol)
                if any(
                    int(item["comm_result"]) != COMM_SUCCESS
                    or int(item["packet_error"]) != 0
                    for item in transport
                ):
                    raise RuntimeError(f"U-Arm runtime read failed: {transport}")
                current_tick = int(current[joint_index])
                accumulated_ticks += shortest_tick_step(previous_tick, current_tick)
                previous_tick = current_tick
                raw_relative = sign * scale * accumulated_ticks * DEGREES_PER_TICK
                relative = max(-MAX_RELATIVE_DEG, min(MAX_RELATIVE_DEG, raw_relative))
            else:
                current_tick = None
                elapsed = time.monotonic() - started
                raw_relative = args.model_step_deg if 0.5 <= elapsed < 1.5 else 0.0
                relative = raw_relative
            maximum_input_excursion = max(maximum_input_excursion, abs(raw_relative))
            model_target = model_baseline[:]
            model_target[joint_index] += relative
            target = [model_target[i] - model_offsets[i] for i in range(6)]
            validate_pose(target)

            robot_state = feedback.latest()
            actual_selected = float(robot_state["q_actual_deg"][joint_index])
            maximum_robot_excursion = max(
                maximum_robot_excursion,
                abs(actual_selected - baseline[joint_index]),
            )
            if int(robot_state["error_status"]) != 0:
                raise RuntimeError("CR5 ErrorStatus became nonzero")
            reply = None
            output = target
            if args.live_motion:
                require_enabled_healthy(robot_state)
                output = slew(previous_command, target)
                reply = dashboard.command(servo_command(output))
                previous_command = output

            report["samples"].append({
                "sample": sample_number,
                "timestamp_utc": utc_now(),
                "servo_ticks": current_tick,
                "accumulated_ticks": accumulated_ticks,
                "raw_relative_deg": raw_relative,
                "clamped_relative_deg": relative,
                "target_deg": target,
                "model_target_deg": model_target,
                "output_deg": output,
                "dobot_reply": reply,
                "dobot": compact_state(robot_state),
            })
            sample_number += 1
            next_tick += PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        report["maximum_input_excursion_deg"] = maximum_input_excursion
        report["maximum_robot_excursion_deg"] = maximum_robot_excursion
        if args.live_motion and (
            maximum_input_excursion < MINIMUM_LIVE_INPUT_EXCURSION_DEG
            or maximum_robot_excursion < MINIMUM_LIVE_ROBOT_EXCURSION_DEG
        ):
            raise RuntimeError(
                "no meaningful end-to-end movement: "
                f"input={maximum_input_excursion:.3f} deg, "
                f"robot={maximum_robot_excursion:.3f} deg"
            )
        report["passed"] = True
        return_code = 0
    except BaseException as exc:
        caught = exc
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["passed"] = False
        return_code = 1
    finally:
        if dashboard is not None and enable_requested:
            if feedback is not None and baseline is not None:
                try:
                    report["baseline_return"] = safe_return_to_baseline(
                        dashboard, feedback, baseline
                    )
                except Exception as exc:
                    report["baseline_return"] = {
                        "passed": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                    report["passed"] = False
                    return_code = 1
            try:
                report["finally_stop_reply"] = dashboard.command("Stop()")
            except Exception as exc:
                report["finally_stop_error"] = f"{type(exc).__name__}: {exc}"
                report["passed"] = False
                return_code = 1
            try:
                report["finally_disable_reply"] = dashboard.command("DisableRobot()")
            except Exception as exc:
                report["finally_disable_error"] = f"{type(exc).__name__}: {exc}"
                report["passed"] = False
                return_code = 1
            if feedback is not None:
                try:
                    disabled = wait_enable(feedback, False)
                    report["final_disabled_state"] = compact_state(disabled)
                    report["final_disable_verified"] = True
                except Exception as exc:
                    report["final_disable_verified"] = False
                    report["final_disable_error"] = (
                        f"{type(exc).__name__}: {exc}"
                    )
                    report["passed"] = False
                    return_code = 1
        if dashboard is not None:
            dashboard.close()
        if feedback is not None:
            feedback.close()
        if opened and port is not None and port.ser is not None:
            port.closePort()
        report["completed_at_utc"] = utc_now()
        path = write_report(report)
        report["log_file"] = str(path)
        summary = {key: value for key, value in report.items() if key != "samples"}
        summary["sample_count"] = len(report["samples"])
        print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))

    if caught is not None:
        return return_code
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
