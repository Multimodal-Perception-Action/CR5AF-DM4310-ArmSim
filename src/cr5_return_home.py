"""Guarded one-shot return to the operator-confirmed CR5 operational home."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from uarm_dobot_teleop import FeedbackStream
from uarm_cr5_axis_follow_test import (
    Dashboard,
    compact_state,
    require_disabled_preflight,
    require_enabled_healthy,
    wait_enable,
)
from uarm_cr5_continuous_teleop import (
    PERIOD_S,
    RETURN_HOME_SPEED_DEG_S,
    RETURN_HOME_TOLERANCE_DEG,
    TRACKING_ERROR_DEG,
    servo_command,
    step_pose_toward,
    validate_pose,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIRM = "I_UNDERSTAND_CR5_WILL_RETURN_HOME"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def send_guarded_movj(dashboard: Dashboard, home: list[float]) -> tuple[str, str]:
    values = ",".join(f"{value:.6f}" for value in validate_pose(home))
    command = f"MovJ(joint={{{values}}},a=5,v=5,cp=0)"
    dashboard.socket.sendall((command + "\n").encode("ascii"))
    response = bytearray()
    while len(response) < 4096 and b";" not in response:
        chunk = dashboard.socket.recv(1024)
        if not chunk:
            raise ConnectionError("TCP/29999 closed before complete MovJ reply")
        response.extend(chunk)
    reply = response.decode("utf-8", errors="replace").strip()
    try:
        error_id = int(reply.split(",", 1)[0])
    except (ValueError, IndexError) as exc:
        raise RuntimeError(f"unparseable MovJ reply: {reply}") from exc
    if error_id != 0:
        raise RuntimeError(f"Dobot rejected guarded return-home command: {reply}")
    return command, reply


def send_clear_collision(dashboard: Dashboard) -> str:
    command = "ClearError()"
    dashboard.socket.sendall((command + "\n").encode("ascii"))
    response = bytearray()
    while len(response) < 4096 and b";" not in response:
        chunk = dashboard.socket.recv(1024)
        if not chunk:
            raise ConnectionError("TCP/29999 closed before complete ClearError reply")
        response.extend(chunk)
    reply = response.decode("utf-8", errors="replace").strip()
    try:
        error_id = int(reply.split(",", 1)[0])
    except (ValueError, IndexError) as exc:
        raise RuntimeError(f"unparseable ClearError reply: {reply}") from exc
    if error_id != 0:
        raise RuntimeError(f"Dobot rejected collision clear: {reply}")
    return reply


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--method", choices=("movj", "servoj"), default="movj")
    parser.add_argument(
        "--clear-collision",
        action="store_true",
        help="clear RobotMode 11 only after the operator has confirmed the area is safe",
    )
    args = parser.parse_args()
    if not args.execute or args.confirm != CONFIRM:
        parser.error("real return-home requires --execute and the exact confirmation token")

    calibration = json.loads(
        (ROOT / "config/uarm_cr5_full_calibration.json").read_text(encoding="utf-8")
    )
    home_record = calibration["robot"]["operational_home"]
    if not home_record.get("operator_designated_as_final_home"):
        raise RuntimeError("operational home is not operator-confirmed")
    home = validate_pose(home_record["joint_deg"])

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = ROOT / "logs" / f"cr5_return_home_{stamp}.json"
    report = {
        "timestamp_utc": utc_now(),
        "operation": "guarded CR5 return to operator-confirmed operational home",
        "host": args.host,
        "home_deg": home,
        "speed_deg_s": RETURN_HOME_SPEED_DEG_S,
        "tolerance_deg": RETURN_HOME_TOLERANCE_DEG,
        "tracking_error_limit_deg": TRACKING_ERROR_DEG,
        "method": args.method,
        "clear_error_sent": False,
        "clear_collision_authorized": args.clear_collision,
        "gripper_command_sent": False,
        "samples": [],
        "passed": False,
    }
    feedback = None
    dashboard = None
    enable_responsibility = False
    reached = False
    started = time.monotonic()
    try:
        feedback = FeedbackStream(args.host, timeout=3.0)
        initial = feedback.wait_first(3.0)
        report["initial"] = compact_state(initial)
        if int(initial["error_status"]) != 0:
            raise RuntimeError("return-home refused: CR5 ErrorStatus is nonzero")
        if int(initial["running_status"]) != 0:
            raise RuntimeError("return-home refused: CR5 is already running")
        if int(initial["robot_mode"]) == 11:
            if not args.clear_collision:
                raise RuntimeError(
                    "return-home refused: RobotMode 11 collision requires an explicit clear-collision authorization"
                )
            if int(initial["enable_status"]) != 0:
                raise RuntimeError("collision clear requires CR5 to be disabled")
            dashboard = Dashboard(args.host)
            report["clear_collision_reply"] = send_clear_collision(dashboard)
            report["clear_error_sent"] = True
            clear_deadline = time.monotonic() + 5.0
            while True:
                initial = feedback.latest(maximum_age=0.25)
                if int(initial["robot_mode"]) == 4 and int(initial["enable_status"]) == 0:
                    break
                if int(initial["error_status"]) != 0:
                    raise RuntimeError("collision clear produced a nonzero ErrorStatus")
                if time.monotonic() > clear_deadline:
                    raise TimeoutError(
                        f"collision state did not clear to mode 4: {compact_state(initial)}"
                    )
                time.sleep(0.05)
            report["after_collision_clear"] = compact_state(initial)
        if int(initial["robot_mode"]) not in (4, 5):
            raise RuntimeError(
                f"return-home refused: unsupported RobotMode={initial['robot_mode']}"
            )
        current = validate_pose(initial["q_actual_deg"])
        maximum_distance = max(abs(goal-value) for goal, value in zip(home, current))
        report["maximum_initial_distance_deg"] = maximum_distance

        if dashboard is None:
            dashboard = Dashboard(args.host)
        enable_responsibility = True
        if int(initial["enable_status"]) == 0:
            report["enable_reply"] = dashboard.command("EnableRobot()")
            state = wait_enable(feedback, True)
        else:
            state = initial
        require_enabled_healthy(state)
        commanded = validate_pose(state["q_actual_deg"])
        if args.method == "movj":
            report["movj_command"], report["movj_reply"] = send_guarded_movj(
                dashboard, home
            )
            deadline = time.monotonic() + 120.0
        else:
            # A zero-delta preload normally enters a fresh streaming session.
            # Some stale controller JOG sessions reject even this frame; use
            # the guarded MovJ method for recovery in that situation.
            report["servo_preload_reply"] = dashboard.command(servo_command(commanded))
            time.sleep(0.1)
            preload_state = feedback.latest(maximum_age=0.25)
            require_enabled_healthy(preload_state)
            deadline = time.monotonic() + maximum_distance / RETURN_HOME_SPEED_DEG_S + 15.0
        stable_samples = 0
        sample_index = 0

        while True:
            state = feedback.latest(maximum_age=0.25)
            require_enabled_healthy(state)
            actual = validate_pose(state["q_actual_deg"])
            remaining = [abs(goal-value) for goal, value in zip(home, actual)]
            tracking = [abs(value-target) for value, target in zip(actual, commanded)]
            if args.method == "servoj" and max(tracking) > TRACKING_ERROR_DEG:
                raise RuntimeError(
                    f"return-home tracking error exceeded {TRACKING_ERROR_DEG} degrees: {tracking}"
                )
            if max(remaining) <= RETURN_HOME_TOLERANCE_DEG:
                stable_samples += 1
                if stable_samples >= 5:
                    reached = True
                    break
            else:
                stable_samples = 0
            if time.monotonic() > deadline:
                raise TimeoutError(f"return-home timed out; remaining errors: {remaining}")
            if args.method == "servoj":
                commanded = step_pose_toward(
                    commanded, home, RETURN_HOME_SPEED_DEG_S * PERIOD_S
                )
                reply = dashboard.command(servo_command(commanded))
            else:
                commanded = home
                reply = None
            report["samples"].append(
                {
                    "index": sample_index,
                    "elapsed_s": time.monotonic() - started,
                    "actual_deg": actual,
                    "commanded_deg": commanded,
                    "remaining_error_deg": remaining,
                    "robot_mode": int(state["robot_mode"]),
                    "reply": reply,
                }
            )
            sample_index += 1
            time.sleep(PERIOD_S)

        report["reached_home"] = True
        report["home_state_before_disable"] = compact_state(state)
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if dashboard is not None and enable_responsibility:
            try:
                report["stop_reply"] = dashboard.command("Stop()")
            except BaseException as exc:
                report["stop_error"] = repr(exc)
            time.sleep(0.2)
            try:
                report["disable_reply"] = dashboard.command("DisableRobot()")
            except BaseException as exc:
                report["disable_error"] = repr(exc)
            if feedback is not None:
                try:
                    final = wait_enable(feedback, False)
                    require_disabled_preflight(final)
                    report["final"] = compact_state(final)
                    report["final_disable_verified"] = True
                except BaseException as exc:
                    report["final_disable_verified"] = False
                    report["final_disable_error"] = repr(exc)
        if dashboard is not None:
            dashboard.close()
        if feedback is not None:
            feedback.close()
        report["passed"] = bool(reached and report.get("final_disable_verified"))
        report["completed_at_utc"] = utc_now()
        report["duration_s"] = time.monotonic() - started
        report["log_file"] = str(log_path)
        log_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "error": report.get("error"),
                "reached_home": report.get("reached_home", False),
                "final_disable_verified": report.get("final_disable_verified", False),
                "final": report.get("final"),
                "log_file": str(log_path),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
