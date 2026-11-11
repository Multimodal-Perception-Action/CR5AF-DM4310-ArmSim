"""Guarded CR5AF J1-J6 one-at-a-time motion smoke test.

Default mode is read-only: one documented V4.5.1 feedback frame is decoded and
the six relative moves are planned, but TCP/29999 is never opened. Real motion
requires all three explicit interlocks: ``--live-motion``, ``--enable-robot``
and the exact confirmation string.

Live sequence for each joint is a low-speed relative move of at most 1 degree
followed by an equal return move. Every step is verified against TCP/30004.
The script never requests TCP control mode, clears errors, runs a project, or
changes robot parameters. If enable was requested, ``Stop()`` and
``DisableRobot()`` are attempted from ``finally`` on every exit path.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import threading
import time
from typing import Any

from feedback_monitor import FRAME_BYTES, decode_v4, recv_exact


ROOT = Path(__file__).resolve().parents[1]
LIVE_CONFIRM = "I_UNDERSTAND_CR5_JOINTS_WILL_MOVE"
JOINT_LIMITS_DEG = (
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-160.0, 160.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
)
LIMIT_MARGIN_DEG = 5.0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class FeedbackStream:
    """Continuously receive the documented 1440-byte V4.5.1 feedback frame."""

    def __init__(self, host: str) -> None:
        self.socket = socket.create_connection((host, 30004), timeout=3.0)
        self.socket.settimeout(1.0)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: dict[str, Any] | None = None
        self._latest_time = 0.0
        self._error: str | None = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                state = decode_v4(recv_exact(self.socket, FRAME_BYTES))
                with self._lock:
                    self._latest = state
                    self._latest_time = time.monotonic()
            except socket.timeout:
                continue
            except Exception as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._error = f"{type(exc).__name__}: {exc}"
                return

    def latest(self, maximum_age_s: float = 0.25) -> dict[str, Any]:
        with self._lock:
            if self._error:
                raise RuntimeError(f"feedback stream failed: {self._error}")
            if self._latest is None:
                raise RuntimeError("no feedback frame received")
            if time.monotonic() - self._latest_time > maximum_age_s:
                raise RuntimeError("feedback frame is stale")
            return dict(self._latest)

    def wait_first(self, timeout_s: float = 3.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                return self.latest(maximum_age_s=timeout_s)
            except RuntimeError as exc:
                if "failed" in str(exc):
                    raise
                time.sleep(0.01)
        raise TimeoutError("timed out waiting for first feedback frame")

    def close(self) -> None:
        self._stop.set()
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        self._thread.join(timeout=2.0)


class Dashboard:
    """Strict command allow-list for this test only."""

    ALLOWED_EXACT = {"RobotMode()", "EnableRobot()", "Stop()", "DisableRobot()"}
    ALLOWED_PREFIX = "RelJointMovJ("

    def __init__(self, host: str) -> None:
        self.socket = socket.create_connection((host, 29999), timeout=3.0)
        self.socket.settimeout(3.0)

    def command(self, command: str) -> str:
        if command not in self.ALLOWED_EXACT and not command.startswith(
            self.ALLOWED_PREFIX
        ):
            raise ValueError(f"command outside smoke-test allow-list: {command}")
        self.socket.sendall((command + "\n").encode("ascii"))
        response = bytearray()
        while len(response) < 4096 and b";" not in response:
            chunk = self.socket.recv(1024)
            if not chunk:
                raise ConnectionError("TCP/29999 closed before a complete reply")
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


def require_disabled_preflight(state: dict[str, Any]) -> None:
    failures: list[str] = []
    if int(state["enable_status"]) != 0:
        failures.append("EnableStatus is not 0")
    if int(state["running_status"]) != 0:
        failures.append("RunningStatus is not 0")
    if int(state["error_status"]) != 0:
        failures.append("ErrorStatus is nonzero")
    if int(state["robot_mode"]) != 4:
        failures.append(f"RobotMode={state['robot_mode']}, expected disabled mode 4")
    if failures:
        raise RuntimeError("disabled preflight rejected: " + "; ".join(failures))


def require_enabled_healthy(state: dict[str, Any]) -> None:
    failures: list[str] = []
    if int(state["enable_status"]) != 1:
        failures.append("EnableStatus is not 1")
    if int(state["error_status"]) != 0:
        failures.append("ErrorStatus is nonzero")
    if int(state["robot_mode"]) not in (5, 7):
        failures.append(f"RobotMode={state['robot_mode']}, expected 5 or 7")
    if failures:
        raise RuntimeError("enabled-state check rejected: " + "; ".join(failures))


def wait_enabled_state(
    feedback: FeedbackStream, enabled: bool, timeout_s: float
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    last: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        last = feedback.latest()
        if int(last["error_status"]) != 0:
            raise RuntimeError("robot reported ErrorStatus during state transition")
        if int(last["enable_status"]) == int(enabled):
            if not enabled or int(last["robot_mode"]) in (5, 7):
                return last
        time.sleep(0.02)
    raise TimeoutError(
        f"timed out waiting for EnableStatus={int(enabled)}; last={last}"
    )


def select_delta(current_deg: float, joint_index: int, magnitude_deg: float) -> float:
    low, high = JOINT_LIMITS_DEG[joint_index]
    if current_deg + magnitude_deg <= high - LIMIT_MARGIN_DEG:
        return magnitude_deg
    if current_deg - magnitude_deg >= low + LIMIT_MARGIN_DEG:
        return -magnitude_deg
    raise RuntimeError(f"J{joint_index + 1} has no safe {magnitude_deg} degree direction")


def relative_command(offsets: list[float], acceleration: int, velocity: int) -> str:
    values = ",".join(f"{value:.6f}" for value in offsets)
    return f"RelJointMovJ({values},a={acceleration},v={velocity},cp=0)"


def wait_target(
    feedback: FeedbackStream,
    target: list[float],
    timeout_s: float,
    tolerance_deg: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    stable = 0
    last: dict[str, Any] | None = None
    best_error = float("inf")
    while time.monotonic() < deadline:
        last = feedback.latest()
        require_enabled_healthy(last)
        actual = [float(value) for value in last["q_actual_deg"]]
        maximum_error = max(abs(a - b) for a, b in zip(actual, target))
        best_error = min(best_error, maximum_error)
        if maximum_error <= tolerance_deg and int(last["running_status"]) == 0:
            stable += 1
            if stable >= 5:
                result = dict(last)
                result["maximum_target_error_deg"] = maximum_error
                return result
        else:
            stable = 0
        time.sleep(0.02)
    raise TimeoutError(
        f"motion target timeout; best maximum error={best_error:.4f} deg; last={last}"
    )


def compact_state(state: dict[str, Any]) -> dict[str, Any]:
    return {
        "robot_mode": state["robot_mode"],
        "enable_status": state["enable_status"],
        "running_status": state["running_status"],
        "error_status": state["error_status"],
        "q_actual_deg": state["q_actual_deg"],
        "timestamp_ms": state["timestamp_ms"],
    }


def write_report(report: dict[str, Any]) -> Path:
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"cr5_joint_smoke_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--delta-deg", type=float, default=1.0)
    parser.add_argument("--velocity-percent", type=int, default=5)
    parser.add_argument("--acceleration-percent", type=int, default=5)
    parser.add_argument("--motion-timeout", type=float, default=15.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.20)
    parser.add_argument("--live-motion", action="store_true")
    parser.add_argument("--enable-robot", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if not 0.1 <= args.delta_deg <= 1.0:
        parser.error("--delta-deg must be within 0.1..1.0")
    if not 1 <= args.velocity_percent <= 10:
        parser.error("--velocity-percent must be within 1..10")
    if not 1 <= args.acceleration_percent <= 10:
        parser.error("--acceleration-percent must be within 1..10")
    if args.motion_timeout < 3.0 or args.motion_timeout > 30.0:
        parser.error("--motion-timeout must be within 3..30 seconds")
    if args.live_motion and (
        not args.enable_robot or args.confirm != LIVE_CONFIRM
    ):
        parser.error(
            "live motion requires --enable-robot and "
            f"--confirm {LIVE_CONFIRM}"
        )
    if not args.live_motion and (args.enable_robot or args.confirm):
        parser.error("enable/confirmation options require --live-motion")

    report: dict[str, Any] = {
        "timestamp_utc": utc_now(),
        "operation": "CR5AF J1-J6 guarded one-at-a-time smoke test",
        "live_motion": bool(args.live_motion),
        "requested_delta_deg": args.delta_deg,
        "velocity_percent": args.velocity_percent,
        "acceleration_percent": args.acceleration_percent,
        "commands_never_used": [
            "RequestControl", "ClearError", "ResetRobot", "RunScript"
        ],
        "steps": [],
        "passed": False,
    }
    feedback: FeedbackStream | None = None
    dashboard: Dashboard | None = None
    enable_requested = False
    caught: BaseException | None = None
    try:
        feedback = FeedbackStream(args.host)
        initial = feedback.wait_first()
        require_disabled_preflight(initial)
        report["initial"] = compact_state(initial)
        baseline = [float(value) for value in initial["q_actual_deg"]]
        plan: list[dict[str, Any]] = []
        for index, value in enumerate(baseline):
            delta = select_delta(value, index, args.delta_deg)
            target = baseline[:]
            target[index] += delta
            plan.append({
                "joint": index + 1,
                "outward_delta_deg": delta,
                "planned_target_deg": target,
                "planned_return_deg": baseline,
            })
        report["plan"] = plan

        if not args.live_motion:
            report["passed"] = True
            report["note"] = "read-only plan completed; TCP/29999 was not opened"
            return_code = 0
        else:
            dashboard = Dashboard(args.host)
            report["robot_mode_reply_before_enable"] = dashboard.command("RobotMode()")
            enable_requested = True
            report["enable_reply"] = dashboard.command("EnableRobot()")
            enabled = wait_enabled_state(feedback, True, 8.0)
            require_enabled_healthy(enabled)
            report["enabled_state"] = compact_state(enabled)

            for item in plan:
                index = int(item["joint"]) - 1
                before_state = feedback.latest()
                require_enabled_healthy(before_state)
                before = [float(value) for value in before_state["q_actual_deg"]]
                delta = select_delta(before[index], index, args.delta_deg)
                outward_target = before[:]
                outward_target[index] += delta
                offsets = [0.0] * 6
                offsets[index] = delta
                outward_command = relative_command(
                    offsets, args.acceleration_percent, args.velocity_percent
                )
                step: dict[str, Any] = {
                    "joint": index + 1,
                    "before": compact_state(before_state),
                    "outward_command": outward_command,
                    "outward_target_deg": outward_target,
                }
                report["steps"].append(step)
                step["outward_reply"] = dashboard.command(outward_command)
                outward_done = wait_target(
                    feedback,
                    outward_target,
                    args.motion_timeout,
                    args.tolerance_deg,
                )
                step["outward_verified"] = compact_state(outward_done)
                step["outward_max_error_deg"] = outward_done[
                    "maximum_target_error_deg"
                ]

                return_offsets = [0.0] * 6
                return_offsets[index] = -delta
                return_command = relative_command(
                    return_offsets,
                    args.acceleration_percent,
                    args.velocity_percent,
                )
                step["return_command"] = return_command
                step["return_reply"] = dashboard.command(return_command)
                return_done = wait_target(
                    feedback,
                    before,
                    args.motion_timeout,
                    args.tolerance_deg,
                )
                step["return_verified"] = compact_state(return_done)
                step["return_max_error_deg"] = return_done[
                    "maximum_target_error_deg"
                ]
                step["passed"] = True

            report["passed"] = True
            return_code = 0
    except BaseException as exc:
        caught = exc
        report["error"] = f"{type(exc).__name__}: {exc}"
        return_code = 1
    finally:
        if dashboard is not None and enable_requested:
            try:
                report["finally_stop_reply"] = dashboard.command("Stop()")
            except Exception as exc:
                report["finally_stop_error"] = f"{type(exc).__name__}: {exc}"
            try:
                report["finally_disable_reply"] = dashboard.command("DisableRobot()")
            except Exception as exc:
                report["finally_disable_error"] = f"{type(exc).__name__}: {exc}"
            if feedback is not None:
                try:
                    disabled = wait_enabled_state(feedback, False, 8.0)
                    report["final_disabled_state"] = compact_state(disabled)
                    report["final_disable_verified"] = True
                except Exception as exc:
                    report["final_disable_verified"] = False
                    report["final_disable_check_error"] = (
                        f"{type(exc).__name__}: {exc}"
                    )
                    report["passed"] = False
                    return_code = 1
        if dashboard is not None:
            dashboard.close()
        if feedback is not None:
            feedback.close()
        report["completed_at_utc"] = utc_now()
        path = write_report(report)
        report["log_file"] = str(path)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))

    if caught is not None:
        return return_code
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
