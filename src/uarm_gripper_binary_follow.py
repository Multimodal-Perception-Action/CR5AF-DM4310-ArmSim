"""Guarded, time-limited U-Arm ID7 -> DM4310 binary gripper follower.

The installed guarded gateway exposes only fixed OPEN, CLOSE and STOP actions.
This controller therefore switches at the calibrated U-Arm midpoint; it does
not claim proportional position control. Real output requires two explicit CLI
interlocks. The CR5 arm itself is never enabled or commanded.
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

from scservo_sdk import PortHandler, sms_sts

from feedback_monitor import FRAME_BYTES, decode_v4, recv_exact
from feetech_port import resolve_uarm_port
from gripper import (
    CAN_RESPONSE_HEADER,
    EVENTS,
    GATEWAY_HEADER,
    PACKETS,
    decode_feedback,
    decode_gateway_event,
    extract_packet,
    gateway_action_passed,
    receive_packet,
)


ROOT = Path(__file__).parents[1]
UARM_CONFIG_PATH = ROOT / "config" / "uarm_gripper_calibration.json"
DM_CONFIG_PATH = ROOT / "config" / "dm4310_gripper_calibration.json"
LIVE_CONFIRM = "I_UNDERSTAND_GRIPPER_WILL_MOVE"


def read_cr5_preflight(host: str) -> dict[str, Any]:
    sock: socket.socket | None = None
    try:
        sock = socket.create_connection((host, 30004), timeout=3.0)
        sock.settimeout(3.0)
        state = decode_v4(recv_exact(sock, FRAME_BYTES))
    finally:
        if sock is not None:
            sock.close()
    if (
        int(state["enable_status"]) != 0
        or int(state["running_status"]) != 0
        or int(state["error_status"]) != 0
    ):
        raise RuntimeError(
            "CR5 preflight rejected: expected enable=0 running=0 error=0"
        )
    return state


def read_dm_preflight(host: str, dm_config: dict[str, Any]) -> dict[str, Any]:
    sock: socket.socket | None = None
    buffer = bytearray()
    try:
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(0.05)
        sock.sendall(PACKETS["status"])
        packet = receive_packet(sock, buffer, CAN_RESPONSE_HEADER, 5.0)
        feedback = decode_feedback(
            packet,
            float(dm_config["pmax_rad"]),
            float(dm_config["vmax_rad_s"]),
            float(dm_config["tmax_nm"]),
        )
    finally:
        if sock is not None:
            sock.close()
    if int(feedback["error_code"]) != 0:
        raise RuntimeError(f"DM4310 preflight rejected: {feedback['state']}")
    if abs(float(feedback["velocity_rad_s"])) > 0.20:
        raise RuntimeError("DM4310 preflight rejected: motor is not stationary")
    if (
        int(feedback["mos_temperature_c"]) >= 60
        or int(feedback["rotor_temperature_c"]) >= 60
    ):
        raise RuntimeError("DM4310 preflight rejected: temperature >= 60 C")
    return feedback


def send_best_effort_stop(host: str) -> None:
    sock: socket.socket | None = None
    try:
        sock = socket.create_connection((host, 60000), timeout=1.0)
        sock.settimeout(0.2)
        sock.sendall(PACKETS["stop"])
    except OSError:
        pass
    finally:
        if sock is not None:
            sock.close()


def action_worker(
    host: str,
    action: str,
    stop_event: threading.Event,
    result: dict[str, Any],
) -> None:
    sock: socket.socket | None = None
    buffer = bytearray()
    stop_sent = False
    holding = False
    releasing = False
    events: list[dict[str, Any]] = []
    try:
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(0.05)
        sock.sendall(PACKETS["identity"])
        identity = decode_gateway_event(
            receive_packet(sock, buffer, GATEWAY_HEADER, 3.0)
        )
        if not (
            identity["event"] == EVENTS["identity"]
            and identity["result"] == 1
            and identity["flags"] == 0x10
        ):
            raise RuntimeError("guarded gateway build 1.0 identity mismatch")
        result["identity"] = identity
        sock.sendall(PACKETS[action])
        result["action_sent"] = True
        deadline = time.monotonic() + 15.0
        next_heartbeat = float("inf")
        while True:
            now = time.monotonic()
            if (not holding or stop_sent) and now >= deadline:
                raise TimeoutError(f"no terminal gateway event for {action}")
            # Continue watchdog supervision throughout controlled opening.
            if holding and (not stop_sent or releasing) and now >= next_heartbeat:
                sock.sendall(PACKETS["hold_heartbeat"])
                next_heartbeat = now + 0.20
            if stop_event.is_set() and not stop_sent:
                requested_release = bool(
                    holding and action == "close"
                    and result.get("requested_reversal_action") == "open"
                )
                sock.sendall(
                    PACKETS["open"] if requested_release else PACKETS["stop"]
                )
                stop_sent = True
                releasing = requested_release
                result["controlled_release_requested"] = requested_release
                result["stop_sent"] = not requested_release
                deadline = time.monotonic() + 5.0
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("TCP60000 closed during action")
                buffer.extend(chunk)
            except socket.timeout:
                pass
            while True:
                packet = extract_packet(buffer, GATEWAY_HEADER)
                if packet is None:
                    break
                event = decode_gateway_event(packet)
                events.append(event)
                if (
                    action == "close" and event["event"] == EVENTS["close"]
                    and event.get("contact_detected")
                    and event.get("holding_active")
                ):
                    if not gateway_action_passed(action, event):
                        raise RuntimeError("gripper contact hold was not confirmed")
                    holding = True
                    result.update(contact_detected=True, holding_active=True)
                    next_heartbeat = time.monotonic()
                    continue
                if releasing and event["event"] == EVENTS["open"]:
                    passed = gateway_action_passed("open", event)
                    result.update(
                        passed=passed, holding_active=False,
                        released_to_action="open", stopped_for_reversal=True,
                    )
                    if not passed:
                        raise RuntimeError("controlled release did not open safely")
                    return
                if stop_sent and event["event"] == EVENTS["stop"]:
                    passed = bool(
                        event["result"] == 1 and event["verified_disabled"]
                    )
                    result.update(
                        passed=passed, holding_active=False,
                        stopped_for_reversal=True, safe_stop_confirmed=passed,
                    )
                    if not passed:
                        raise RuntimeError("gripper stop was not safely disabled")
                    return
                if not stop_sent and event["event"] == EVENTS[action]:
                    passed = gateway_action_passed(action, event)
                    result.update(
                        passed=passed,
                        contact_detected=bool(event.get("contact_detected")),
                        holding_active=bool(event.get("holding_active")),
                        stopped_for_reversal=False,
                    )
                    if not passed:
                        raise RuntimeError("gripper action did not complete safely")
                    return
    except Exception as exc:
        result["passed"] = False
        result["error"] = f"{type(exc).__name__}: {exc}"
        if result.get("action_sent") and sock is not None and not stop_sent:
            try:
                sock.sendall(PACKETS["stop"])
                result["fallback_stop_sent"] = True
            except OSError as stop_exc:
                result["fallback_stop_error"] = (
                    f"{type(stop_exc).__name__}: {stop_exc}"
                )
    finally:
        result["events"] = events
        if sock is not None:
            sock.close()


def desired_state(ticks: int, midpoint: float, hysteresis: int, previous: str) -> str:
    if ticks <= midpoint - hysteresis:
        return "open"
    if ticks >= midpoint + hysteresis:
        return "close"
    return previous


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--hysteresis-ticks", type=int, default=20)
    parser.add_argument("--stable-samples", type=int, default=4)
    parser.add_argument("--live-output", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 120:
        parser.error("duration must be in (0, 120] seconds")
    if args.rate < 5 or args.rate > 50:
        parser.error("rate must be between 5 and 50 Hz")
    if args.live_output and args.confirm != LIVE_CONFIRM:
        parser.error(f"live output requires --confirm {LIVE_CONFIRM}")

    uarm_config = json.loads(UARM_CONFIG_PATH.read_text(encoding="utf-8"))
    dm_config = json.loads(DM_CONFIG_PATH.read_text(encoding="utf-8"))
    endpoints = uarm_config["endpoints"]
    if not (
        endpoints["maximum_open"].get("confirmed")
        and endpoints["fully_closed"].get("confirmed")
    ):
        raise RuntimeError("both U-Arm gripper endpoints must be confirmed")
    midpoint = float(endpoints["midpoint_ticks"])

    cr5 = read_cr5_preflight(args.host)
    dm = read_dm_preflight(args.host, dm_config)
    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    opened = False
    samples: list[dict[str, Any]] = []
    actions: list[dict[str, Any]] = []
    worker: threading.Thread | None = None
    worker_stop: threading.Event | None = None
    worker_result: dict[str, Any] | None = None
    active_action: str | None = None
    last_completed: str | None = None
    candidate: str | None = None
    candidate_count = 0
    input_ready = False
    current_desired = "close" if float(dm["position_rad"]) > (
        float(dm_config["open"]["position_rad"])
        + float(dm_config["closed"]["position_rad"])
    ) / 2.0 else "open"
    period = 1.0 / args.rate
    started = time.monotonic()
    exit_reason = "duration_complete"
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        while time.monotonic() - started < args.duration:
            loop_started = time.monotonic()
            ticks, result, packet_error = protocol.read2ByteTxRx(7, 56)
            if result != 0 or packet_error != 0:
                exit_reason = "uarm_read_failure"
                raise RuntimeError(
                    f"ID7 read failed result={result} packet_error={packet_error}"
                )
            ticks = int(ticks) % 4096
            proposed = desired_state(
                ticks, midpoint, args.hysteresis_ticks, current_desired
            )
            if proposed == candidate:
                candidate_count += 1
            else:
                candidate = proposed
                candidate_count = 1
            if candidate_count >= args.stable_samples:
                current_desired = proposed
                input_ready = True
            input_stable = candidate_count >= args.stable_samples

            now_s = time.monotonic() - started
            samples.append({
                "t_s": now_s,
                "ticks": ticks,
                "desired": current_desired,
                "active_action": active_action,
            })

            if worker is not None and not worker.is_alive():
                worker.join()
                assert worker_result is not None
                worker_result["completed_t_s"] = now_s
                actions.append(worker_result)
                if worker_result.get("released_to_action"):
                    last_completed = worker_result["released_to_action"]
                elif worker_result.get("passed") and not worker_result.get(
                    "stopped_for_reversal"
                ):
                    last_completed = active_action
                else:
                    last_completed = None
                worker = None
                worker_stop = None
                worker_result = None
                active_action = None

            if worker is not None and current_desired != active_action:
                assert worker_stop is not None
                assert worker_result is not None
                worker_result["requested_reversal_action"] = current_desired
                worker_stop.set()

            if (
                args.live_output
                and input_ready
                and input_stable
                and worker is None
                and current_desired != last_completed
            ):
                active_action = current_desired
                worker_stop = threading.Event()
                worker_result = {
                    "action": active_action,
                    "started_t_s": now_s,
                    "trigger_ticks": ticks,
                    "action_sent": False,
                }
                worker = threading.Thread(
                    target=action_worker,
                    args=(args.host, active_action, worker_stop, worker_result),
                    daemon=True,
                )
                worker.start()

            remaining = period - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
    except Exception:
        if exit_reason == "duration_complete":
            exit_reason = "exception"
        raise
    finally:
        if worker is not None and worker.is_alive():
            assert worker_stop is not None
            worker_stop.set()
            worker.join(timeout=6.0)
        if worker_result is not None:
            worker_result["completed_t_s"] = time.monotonic() - started
            actions.append(worker_result)
        if worker is not None and worker.is_alive():
            send_best_effort_stop(args.host)
            exit_reason = "worker_stop_timeout"
        if opened and port.ser is not None:
            port.closePort()

    final_dm = read_dm_preflight(args.host, dm_config)
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "guarded_binary_live" if args.live_output else "read_only_preview",
        "duration_requested_s": args.duration,
        "duration_actual_s": time.monotonic() - started,
        "exit_reason": exit_reason,
        "uarm_port": port_name,
        "midpoint_ticks": midpoint,
        "hysteresis_ticks": args.hysteresis_ticks,
        "cr5_preflight": {
            "robot_mode": cr5["robot_mode"],
            "enable_status": cr5["enable_status"],
            "running_status": cr5["running_status"],
            "error_status": cr5["error_status"],
        },
        "dm_preflight": dm,
        "actions": actions,
        "final_dm": final_dm,
        "final_dm_disabled_no_fault": int(final_dm["error_code"]) == 0,
        "samples": samples,
    }
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"uarm_gripper_follow_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {key: value for key, value in report.items() if key != "samples"}
    summary["log_file"] = str(path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if report["final_dm_disabled_no_fault"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
