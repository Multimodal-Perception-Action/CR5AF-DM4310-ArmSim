"""Guarded single-process U-Arm -> CR5AF + DM4310 teleoperation.

One COM port is opened exactly once.  IDs 1-6 drive CR5 J1-J6 relative to a
session baseline; ID7 independently selects the guarded DM4310 OPEN/CLOSE
actions.  Preview is the default and opens no motion socket.

Default live profile clamps each CR5 joint to baseline +/-2 degrees; an
explicit staged-test argument may request up to the separately unlocked hard
maximum of +/-5 degrees. Command slew is limited to 3 deg/s and output is
20 Hz. Real output requires explicit live, enable, and confirmation interlocks
plus validated config locks.
No control-mode request, error clear, reset, project execution, raw CAN frame,
or U-Arm register write exists in this program.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
from typing import Any, Iterator, TextIO

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
    uarm_health_warnings,
)
from uarm_cr5_axis_follow_test import (
    Dashboard,
    compact_state,
    require_disabled_preflight,
    require_enabled_healthy,
    safe_return_to_baseline,
    validate_pose,
    wait_enable,
)
from uarm_gripper_binary_follow import (
    action_worker,
    desired_state,
    read_dm_preflight,
    send_best_effort_stop,
)


ROOT = Path(__file__).resolve().parents[1]
TELEOP_CONFIG = ROOT / "config" / "uarm_dobot_teleop.json"
GRIPPER_CONFIG = ROOT / "config" / "uarm_gripper_calibration.json"
DM_CONFIG = ROOT / "config" / "dm4310_gripper_calibration.json"
LIVE_CONFIRM = "I_UNDERSTAND_FULL_UARM_CR5_WILL_MOVE"
RATE_HZ = 20.0
PERIOD_S = 1.0 / RATE_HZ
DEFAULT_MAX_RELATIVE_DEG = 2.0
HARD_MAX_RELATIVE_DEG = 5.0
MAX_SPEED_DEG_S = 3.0
ACTUAL_ENVELOPE_TOLERANCE_DEG = 0.25


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def arm_servo_command(joints: list[float]) -> str:
    validate_pose(joints)
    return (
        "ServoJ(" + ",".join(f"{value:.6f}" for value in joints)
        + f",t={PERIOD_S:.3f},aheadtime=50,gain=250)"
    )


def arm_slew(previous: list[float], target: list[float]) -> list[float]:
    maximum_step = MAX_SPEED_DEG_S * PERIOD_S
    output: list[float] = []
    for old, new in zip(previous, target):
        delta = max(-maximum_step, min(maximum_step, new - old))
        output.append(old + delta)
    return output


def write_report(report: dict[str, Any]) -> Path:
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"uarm_cr5_full_teleop_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    return path


@contextmanager
def telemetry_log(path: Path | None, report: dict[str, Any]) -> Iterator[TextIO | None]:
    """Optional GUI mirror; never opens hardware and never replaces an old log.

    This context encloses the existing session cleanup so that even an error
    during cleanup still attempts a terminal record and closes the stream.
    """
    if path is None:
        yield None
        return
    escaped_error: str | None = None
    with path.open("x", encoding="utf-8") as stream:
        try:
            stream.write(json.dumps({
                "kind": "session",
                "schema_version": 1,
                "timestamp_utc": utc_now(),
                "live_output": report["live_output"],
                "operation": report["operation"],
                "duration_requested_s": report["duration_requested_s"],
                "rate_hz": report["rate_hz"],
                "arm_max_relative_deg": report["arm_max_relative_deg"],
                "arm_max_speed_deg_s": report["arm_max_speed_deg_s"],
                "arm_mapping_signs": report["arm_mapping_signs"],
            }, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            yield stream
        except BaseException as exc:
            escaped_error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            cleanup_keys = (
                "arm_baseline_return", "finally_stop_reply", "finally_stop_error",
                "finally_disable_reply", "finally_disable_error", "cr5_final",
                "cr5_final_disable_verified", "cr5_final_disable_error",
                "gripper_worker_stop_timeout", "gripper_final_best_effort_stop_sent",
                "uarm_shutdown_health_error", "dm_final", "dm_final_error",
                "dm_final_disabled_no_fault",
            )
            stream.write(json.dumps({
                "kind": "terminal",
                "schema_version": 1,
                "timestamp_utc": utc_now(),
                "live_output": report["live_output"],
                "passed": bool(report.get("passed")) and escaped_error is None,
                "error": report.get("error"),
                "escaped_error": escaped_error,
                "arm_sample_count": len(report["arm_samples"]),
                "gripper_action_count": len(report["gripper_actions"]),
                "cleanup": {key: report[key] for key in cleanup_keys if key in report},
                "report_file": report.get("log_file"),
            }, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--max-relative-deg", type=float, default=DEFAULT_MAX_RELATIVE_DEG,
        help="per-joint session-relative clamp; live hard maximum is 5 degrees",
    )
    parser.add_argument("--gripper-hysteresis-ticks", type=int, default=20)
    parser.add_argument("--gripper-stable-samples", type=int, default=4)
    parser.add_argument("--live-output", action="store_true")
    parser.add_argument("--enable-robot", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--telemetry-log", type=Path, default=None,
        help="optional new JSONL path for a read-only GUI mirror; existing files are refused",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 30.0:
        parser.error("--duration must be within (0, 30] seconds")
    if args.max_relative_deg < 0.5 or args.max_relative_deg > HARD_MAX_RELATIVE_DEG:
        parser.error("--max-relative-deg must be within 0.5..5.0 degrees")
    if args.gripper_hysteresis_ticks < 1 or args.gripper_hysteresis_ticks > 100:
        parser.error("--gripper-hysteresis-ticks must be within 1..100")
    if args.gripper_stable_samples < 1 or args.gripper_stable_samples > 20:
        parser.error("--gripper-stable-samples must be within 1..20")
    if args.live_output and (
        not args.enable_robot or args.confirm != LIVE_CONFIRM
    ):
        parser.error(
            "live output requires --enable-robot and "
            f"--confirm {LIVE_CONFIRM}"
        )
    if not args.live_output and (args.enable_robot or args.confirm):
        parser.error("enable/confirmation options require --live-output")

    teleop = load_config(TELEOP_CONFIG)
    gripper = json.loads(GRIPPER_CONFIG.read_text(encoding="utf-8"))
    dm_config = json.loads(DM_CONFIG.read_text(encoding="utf-8"))
    input_calibration = teleop.get("input_calibration", {})
    deadband_ticks = input_calibration.get(
        "joint_input_deadband_ticks", [2, 2, 2, 2, 2, 2]
    )
    if (
        len(deadband_ticks) != 6
        or any(int(value) < 0 or int(value) > 20 for value in deadband_ticks)
    ):
        raise ValueError("joint_input_deadband_ticks must contain six values in 0..20")
    deadband_ticks = [int(value) for value in deadband_ticks]
    if args.live_output:
        unlocked_relative = float(
            teleop.get("full_teleop_unlock_profile", {}).get(
                "maximum_relative_deg_per_joint", DEFAULT_MAX_RELATIVE_DEG
            )
        )
        required_locks = {
            "power_validated": bool(teleop.get("power_validated")),
            "mapping_validated": bool(teleop.get("mapping_validated")),
            # Deliberately independent from motion_unlocked: enabling this
            # tightly guarded staged controller must not unlock older,
            # wider-range teleoperation programs.
            "full_teleop_unlocked": bool(teleop.get("full_teleop_unlocked")),
            "gripper_endpoints_confirmed": bool(
                gripper["endpoints"]["maximum_open"].get("confirmed")
                and gripper["endpoints"]["fully_closed"].get("confirmed")
            ),
            "requested_range_within_unlocked_profile": bool(
                args.max_relative_deg <= unlocked_relative
            ),
        }
        if not all(required_locks.values()):
            raise RuntimeError(f"live output locks are not all true: {required_locks}")

    report: dict[str, Any] = {
        "timestamp_utc": utc_now(),
        "operation": "guarded full U-Arm CR5 + DM4310 teleoperation",
        "live_output": bool(args.live_output),
        "duration_requested_s": args.duration,
        "rate_hz": RATE_HZ,
        "arm_max_relative_deg": args.max_relative_deg,
        "arm_max_speed_deg_s": MAX_SPEED_DEG_S,
        "arm_actual_envelope_tolerance_deg": ACTUAL_ENVELOPE_TOLERANCE_DEG,
        "arm_mapping_signs": teleop["joint_signs"],
        "arm_input_deadband_ticks": deadband_ticks,
        "input_calibration_evidence": input_calibration.get("evidence_log"),
        "commands_never_used": [
            "RequestControl", "ClearError", "ResetRobot", "RunScript",
            "raw CAN forwarding", "U-Arm register write",
        ],
        "arm_samples": [],
        "gripper_actions": [],
        "passed": False,
    }

    # Open and flush the optional telemetry session before any hardware access.
    # Exclusive creation failures therefore cannot start a robot session.
    with telemetry_log(args.telemetry_log, report) as telemetry:
        return run_session(args, teleop, gripper, dm_config, deadband_ticks, report, telemetry)


def run_session(
    args: argparse.Namespace,
    teleop: dict[str, Any],
    gripper: dict[str, Any],
    dm_config: dict[str, Any],
    deadband_ticks: list[int],
    report: dict[str, Any],
    telemetry: TextIO | None,
) -> int:
    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    feedback: FeedbackStream | None = None
    dashboard: Dashboard | None = None
    opened = False
    enable_requested = False
    baseline: list[float] | None = None
    gripper_worker: threading.Thread | None = None
    gripper_stop: threading.Event | None = None
    gripper_result: dict[str, Any] | None = None
    active_gripper_action: str | None = None
    last_completed_gripper: str | None = None
    caught: BaseException | None = None
    return_code = 1
    started = time.monotonic()
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        feedback = FeedbackStream(args.host, timeout=3.0)
        initial = feedback.wait_first(3.0)
        require_disabled_preflight(initial)
        baseline = [float(value) for value in initial["q_actual_deg"]]
        validate_pose(baseline)

        health = read_servo_health(protocol)
        require_uarm_health(health, teleop)
        startup_health_warnings = uarm_health_warnings(health, teleop)
        positions, transport = read_positions(protocol)
        if any(
            int(item["comm_result"]) != COMM_SUCCESS
            or int(item["packet_error"]) != 0
            for item in transport
        ):
            raise RuntimeError(f"initial U-Arm read failed: {transport}")
        dm_initial = read_dm_preflight(args.host, dm_config)

        report["uarm_port"] = port_name
        report["uarm_health"] = health
        report["uarm_health_warnings"] = startup_health_warnings
        report["uarm_baseline_ticks"] = positions
        report["cr5_initial"] = compact_state(initial)
        report["dm_initial"] = dm_initial

        previous_positions = positions[:]
        accumulated_ticks = [0] * 6
        previous_arm_command = baseline[:]
        midpoint = float(gripper["endpoints"]["midpoint_ticks"])
        current_gripper_desired = (
            "open" if positions[6] <= midpoint else "close"
        )
        gripper_candidate: str | None = None
        gripper_candidate_count = 0
        gripper_input_ready = False

        if args.live_output:
            dashboard = Dashboard(args.host)
            report["robot_mode_reply"] = dashboard.command("RobotMode()")
            enable_requested = True
            report["enable_reply"] = dashboard.command("EnableRobot()")
            enabled = wait_enable(feedback, True)
            require_enabled_healthy(enabled)
            report["cr5_enabled"] = compact_state(enabled)

        deadline = time.monotonic() + args.duration
        next_tick = time.monotonic()
        sample_number = 0
        while time.monotonic() < deadline:
            current, transport = read_positions(protocol)
            if any(
                int(item["comm_result"]) != COMM_SUCCESS
                or int(item["packet_error"]) != 0
                for item in transport
            ):
                raise RuntimeError(f"U-Arm runtime read failed: {transport}")

            relative: list[float] = []
            unfiltered_relative: list[float] = []
            raw_relative: list[float] = []
            for index in range(6):
                accumulated_ticks[index] += shortest_tick_step(
                    previous_positions[index], current[index]
                )
                signed_ticks = (
                    float(teleop["joint_signs"][index])
                    * accumulated_ticks[index]
                )
                unfiltered = signed_ticks * DEGREES_PER_TICK
                magnitude = max(0.0, abs(signed_ticks) - deadband_ticks[index])
                filtered_ticks = magnitude if signed_ticks >= 0 else -magnitude
                raw = filtered_ticks * DEGREES_PER_TICK
                unfiltered_relative.append(unfiltered)
                raw_relative.append(raw)
                relative.append(
                    max(-args.max_relative_deg, min(args.max_relative_deg, raw))
                )
            previous_positions = current
            target = [baseline[i] + relative[i] for i in range(6)]
            validate_pose(target)

            robot_state = feedback.latest()
            if int(robot_state["error_status"]) != 0:
                raise RuntimeError("CR5 ErrorStatus became nonzero")
            validate_pose(
                [float(value) for value in robot_state["q_actual_deg"]]
            )
            actual_relative = [
                float(robot_state["q_actual_deg"][i]) - baseline[i]
                for i in range(6)
            ]
            actual_limit = args.max_relative_deg + ACTUAL_ENVELOPE_TOLERANCE_DEG
            if any(abs(value) > actual_limit for value in actual_relative):
                raise RuntimeError(
                    "CR5 actual position exceeded guarded session envelope: "
                    f"relative={actual_relative}, limit={actual_limit} deg"
                )
            arm_reply = None
            arm_output = target
            if args.live_output:
                require_enabled_healthy(robot_state)
                arm_output = arm_slew(previous_arm_command, target)
                arm_reply = dashboard.command(arm_servo_command(arm_output))
                previous_arm_command = arm_output

            proposed = desired_state(
                int(current[6]), midpoint, args.gripper_hysteresis_ticks,
                current_gripper_desired,
            )
            if proposed == gripper_candidate:
                gripper_candidate_count += 1
            else:
                gripper_candidate = proposed
                gripper_candidate_count = 1
            gripper_input_stable = (
                gripper_candidate_count >= args.gripper_stable_samples
            )
            if gripper_input_stable:
                current_gripper_desired = proposed
                gripper_input_ready = True

            now_s = time.monotonic() - started
            if gripper_worker is not None and not gripper_worker.is_alive():
                gripper_worker.join()
                assert gripper_result is not None
                gripper_result["completed_t_s"] = now_s
                report["gripper_actions"].append(gripper_result)
                if not gripper_result.get("passed"):
                    raise RuntimeError(
                        f"gripper action failed: {gripper_result}"
                    )
                # action_worker marks success only after the guarded gateway
                # reports verified_disabled. Keep a redundant assertion here
                # so later worker changes cannot weaken this controller.
                terminal_events = gripper_result.get("events", [])
                if not any(
                    bool(event.get("verified_disabled"))
                    for event in terminal_events
                ):
                    raise RuntimeError(
                        "gripper action ended without verified_disabled"
                    )
                if not gripper_result.get("stopped_for_reversal"):
                    last_completed_gripper = active_gripper_action
                else:
                    last_completed_gripper = None
                gripper_worker = None
                gripper_stop = None
                gripper_result = None
                active_gripper_action = None

            if (
                gripper_worker is not None
                and current_gripper_desired != active_gripper_action
            ):
                assert gripper_stop is not None
                gripper_stop.set()

            if (
                args.live_output
                and gripper_input_ready
                and gripper_input_stable
                and gripper_worker is None
                and current_gripper_desired != last_completed_gripper
            ):
                active_gripper_action = current_gripper_desired
                gripper_stop = threading.Event()
                gripper_result = {
                    "action": active_gripper_action,
                    "started_t_s": now_s,
                    "trigger_ticks": int(current[6]),
                    "action_sent": False,
                }
                gripper_worker = threading.Thread(
                    target=action_worker,
                    args=(args.host, active_gripper_action,
                          gripper_stop, gripper_result),
                    daemon=True,
                )
                gripper_worker.start()

            report["arm_samples"].append({
                "sample": sample_number,
                "timestamp_utc": utc_now(),
                "servo_ticks": current,
                "unfiltered_relative_deg": unfiltered_relative,
                "raw_relative_deg": raw_relative,
                "clamped_relative_deg": relative,
                "arm_target_deg": target,
                "arm_output_deg": arm_output,
                "arm_actual_relative_deg": actual_relative,
                "arm_reply": arm_reply,
                "cr5": compact_state(robot_state),
                "gripper_desired": current_gripper_desired,
                "gripper_active_action": active_gripper_action,
            })
            if telemetry is not None:
                telemetry.write(json.dumps(
                    {"kind": "sample", "live_output": bool(args.live_output),
                     **report["arm_samples"][-1]},
                    ensure_ascii=False, allow_nan=False,
                ) + "\n")
                telemetry.flush()
            sample_number += 1
            next_tick += PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        report["passed"] = True
        return_code = 0
    except BaseException as exc:
        caught = exc
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["passed"] = False
        return_code = 1
    finally:
        if gripper_worker is not None and gripper_worker.is_alive():
            assert gripper_stop is not None
            gripper_stop.set()
            gripper_worker.join(timeout=6.0)
        if gripper_result is not None:
            gripper_result["completed_t_s"] = time.monotonic() - started
            report["gripper_actions"].append(gripper_result)
            if not gripper_result.get("passed"):
                report["passed"] = False
                return_code = 1
        if gripper_worker is not None and gripper_worker.is_alive():
            send_best_effort_stop(args.host)
            report["gripper_worker_stop_timeout"] = True
            report["passed"] = False
            return_code = 1

        if args.live_output:
            # STOP is an output command, so it is intentionally omitted from
            # default preview. In live mode it is sent best-effort on every
            # exit path, including a normal one.
            send_best_effort_stop(args.host)
            report["gripper_final_best_effort_stop_sent"] = True

        if dashboard is not None and enable_requested:
            if feedback is not None and baseline is not None:
                try:
                    report["arm_baseline_return"] = safe_return_to_baseline(
                        dashboard, feedback, baseline
                    )
                except Exception as exc:
                    report["arm_baseline_return"] = {
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
                    report["cr5_final"] = compact_state(disabled)
                    report["cr5_final_disable_verified"] = True
                except Exception as exc:
                    report["cr5_final_disable_verified"] = False
                    report["cr5_final_disable_error"] = (
                        f"{type(exc).__name__}: {exc}"
                    )
                    report["passed"] = False
                    return_code = 1

        if opened:
            try:
                shutdown_health = read_servo_health(protocol)
                require_uarm_health(shutdown_health, teleop)
                report["uarm_shutdown_health"] = shutdown_health
                report["uarm_shutdown_health_warnings"] = uarm_health_warnings(
                    shutdown_health, teleop
                )
            except Exception as exc:
                report["uarm_shutdown_health_error"] = (
                    f"{type(exc).__name__}: {exc}"
                )
                if args.live_output:
                    report["passed"] = False
                    return_code = 1

        if dashboard is not None:
            dashboard.close()
        if feedback is not None:
            feedback.close()
        if opened and port.ser is not None:
            port.closePort()

        try:
            report["dm_final"] = read_dm_preflight(args.host, dm_config)
            report["dm_final_disabled_no_fault"] = (
                int(report["dm_final"]["error_code"]) == 0
            )
        except Exception as exc:
            report["dm_final_disabled_no_fault"] = False
            report["dm_final_error"] = f"{type(exc).__name__}: {exc}"
            if args.live_output:
                report["passed"] = False
                return_code = 1

        report["duration_actual_s"] = time.monotonic() - started
        report["completed_at_utc"] = utc_now()
        path = write_report(report)
        report["log_file"] = str(path)
        summary = {key: value for key, value in report.items()
                   if key != "arm_samples"}
        summary["arm_sample_count"] = len(report["arm_samples"])
        print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))

    if caught is not None:
        return return_code
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
