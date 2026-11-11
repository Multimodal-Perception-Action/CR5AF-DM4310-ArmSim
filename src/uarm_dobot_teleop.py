"""Guarded FEETECH U-Arm to Dobot CR5AF joint teleoperation.

Preview is the default and sends no Dobot command. Live ServoJ output requires
all three configuration locks plus two explicit command-line arguments. The
program never powers on, enables, clears errors, or changes control mode.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from scservo_sdk import PortHandler, sms_sts

from feedback_monitor import FRAME_BYTES, decode_v4, recv_exact
from feetech_port import resolve_uarm_port
from uarm_dobot_preview import (
    DEGREES_PER_TICK,
    JOINT_SIGNS,
    SERVO_IDS,
    read_positions,
    read_servo_health,
    shortest_tick_step,
)


LIVE_ACK = "I_ACCEPT_LIVE_SERVOJ"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class FeedbackStream:
    def __init__(self, host: str, timeout: float) -> None:
        self.socket = socket.create_connection((host, 30004), timeout=timeout)
        self.socket.settimeout(1.0)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: dict[str, Any] | None = None
        self._latest_monotonic = 0.0
        self._error: str | None = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                decoded = decode_v4(recv_exact(self.socket, FRAME_BYTES))
                with self._lock:
                    self._latest = decoded
                    self._latest_monotonic = time.monotonic()
            except socket.timeout:
                continue
            except Exception as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._error = f"{type(exc).__name__}: {exc}"
                return

    def latest(self, maximum_age: float = 0.25) -> dict[str, Any]:
        with self._lock:
            if self._error:
                raise RuntimeError(f"feedback stream failed: {self._error}")
            if self._latest is None:
                raise RuntimeError("no Dobot feedback received")
            if time.monotonic() - self._latest_monotonic > maximum_age:
                raise RuntimeError("Dobot feedback is stale")
            return dict(self._latest)

    def wait_first(self, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                return self.latest(maximum_age=timeout)
            except RuntimeError as exc:
                if "failed" in str(exc):
                    raise
                time.sleep(0.01)
        raise TimeoutError("timed out waiting for Dobot feedback")

    def close(self) -> None:
        self._stop.set()
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        self._thread.join(timeout=2.0)


class DashboardClient:
    """Small allow-list client used only after every live-motion lock passes."""

    ALLOWED_PREFIXES = ("RobotMode(", "ServoJ(", "Stop(")

    def __init__(self, host: str, timeout: float) -> None:
        self.socket = socket.create_connection((host, 29999), timeout=timeout)
        self.socket.settimeout(timeout)

    def command(self, command: str) -> str:
        if not command.startswith(self.ALLOWED_PREFIXES):
            raise ValueError(f"command is outside teleop allow-list: {command}")
        self.socket.sendall((command.rstrip() + "\n").encode("ascii"))
        chunks: list[bytes] = []
        while sum(map(len, chunks)) < 4096:
            chunk = self.socket.recv(1024)
            if not chunk:
                raise ConnectionError("Dobot closed dashboard socket")
            chunks.append(chunk)
            if b";" in chunk:
                break
        reply = b"".join(chunks).decode("utf-8", errors="replace").strip()
        try:
            error_id = int(reply.split(",", 1)[0])
        except ValueError as exc:
            raise RuntimeError(f"unparseable Dobot reply: {reply}") from exc
        if error_id != 0:
            raise RuntimeError(f"Dobot rejected {command}: {reply}")
        return reply

    def close(self) -> None:
        self.socket.close()


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as source:
        config = json.load(source)
    if len(config["joint_signs"]) != 6 or len(config["joint_limits_deg"]) != 6:
        raise ValueError("teleop config must define six joint signs and limits")
    if len(config["max_relative_offset_deg"]) != 6 or len(config["max_speed_deg_s"]) != 6:
        raise ValueError("teleop config must define six offset and speed limits")
    return config


def require_uarm_health(
    health: list[dict[str, object]], config: dict[str, Any]
) -> None:
    power = config.get("power_validation", {})
    voltage_range = power.get("accepted_voltage_range_v", [5.0, 8.4])
    minimum_voltage, maximum_voltage = map(float, voltage_range)
    maximum_temperature = float(power.get("maximum_temperature_c", 60.0))
    failures: list[str] = []
    for item in health:
        servo_id = int(item["id"])
        if not item["model_is_sts3215"]:
            failures.append(f"ID {servo_id}: model {item['model_number']} is not STS3215")
        voltage = float(item["voltage_v"])
        if not minimum_voltage <= voltage <= maximum_voltage:
            failures.append(
                f"ID {servo_id}: voltage {voltage:.1f} V outside configured "
                f"{minimum_voltage:.1f}-{maximum_voltage:.1f} V"
            )
        if float(item["temperature_c"]) >= maximum_temperature:
            failures.append(
                f"ID {servo_id}: temperature {item['temperature_c']} C >= "
                f"{maximum_temperature:.1f} C"
            )
        if int(item["status_register"]) != 0:
            failures.append(f"ID {servo_id}: status register={item['status_register']}")
        if bool(item["voltage_alarm"]):
            failures.append(f"ID {servo_id}: voltage alarm is set")
        if int(item["operating_mode"]) != 0:
            failures.append(f"ID {servo_id}: operating mode={item['operating_mode']}, expected 0")
        if int(item["torque_enable"]) != 0:
            failures.append(f"ID {servo_id}: leader torque is enabled")
        if any(int(value) != 0 for value in item["comm_results"]):
            failures.append(f"ID {servo_id}: register communication failure")
        if any(int(value) != 0 for value in item["packet_errors"]):
            failures.append(f"ID {servo_id}: register packet error")
    if failures:
        raise RuntimeError("U-Arm preflight failed: " + "; ".join(failures))


def uarm_health_warnings(
    health: list[dict[str, object]], config: dict[str, Any]
) -> list[str]:
    power = config.get("power_validation", {})
    documented = power.get("documented_c046_range_v", [5.0, 8.4])
    documented_minimum, documented_maximum = map(float, documented)
    warnings: list[str] = []
    voltage_advisories: list[tuple[int, float]] = []
    for item in health:
        voltage = float(item["voltage_v"])
        if not documented_minimum <= voltage <= documented_maximum:
            voltage_advisories.append((int(item["id"]), voltage))
    if voltage_advisories:
        ids = ",".join(str(item[0]) for item in voltage_advisories)
        values = [item[1] for item in voltage_advisories]
        warnings.append(
            f"advisory UARM_VOLTAGE_READBACK: IDs {ids} report "
            f"{min(values):.1f}-{max(values):.1f} V outside documented C046 "
            f"{documented_minimum:.1f}-{documented_maximum:.1f} V; accepted by "
            "operator while every status register and voltage-alarm bit is clear"
        )
    return warnings


def require_robot_health(feedback: dict[str, Any], *, allow_running: bool = False) -> None:
    failures: list[str] = []
    allowed_modes = {5, 7} if allow_running else {5}
    if int(feedback["robot_mode"]) not in allowed_modes:
        failures.append(f"RobotMode={feedback['robot_mode']}, expected one of {sorted(allowed_modes)}")
    if int(feedback["enable_status"]) != 1:
        failures.append("EnableStatus is not 1")
    if not allow_running and int(feedback["running_status"]) != 0:
        failures.append("RunningStatus is not idle")
    if int(feedback["error_status"]) != 0:
        failures.append("ErrorStatus is nonzero")
    if failures:
        raise RuntimeError("Dobot preflight failed: " + "; ".join(failures))


def validate_target(target: list[float], offset: list[float], config: dict[str, Any]) -> None:
    margin = float(config["joint_limit_margin_deg"])
    failures: list[str] = []
    for index, (value, relative, limits, maximum) in enumerate(zip(
            target, offset, config["joint_limits_deg"], config["max_relative_offset_deg"]), start=1):
        low, high = map(float, limits)
        if not low + margin <= value <= high - margin:
            failures.append(f"J{index} target {value:.3f} outside guarded limits")
        if abs(relative) > float(maximum):
            failures.append(f"J{index} relative offset {relative:.3f} exceeds {maximum} deg")
    if failures:
        raise RuntimeError("; ".join(failures))


def rate_limit(previous: list[float], target: list[float], speed: list[float], period: float) -> list[float]:
    limited: list[float] = []
    for old, new, maximum_speed in zip(previous, target, speed):
        maximum_step = float(maximum_speed) * period
        delta = max(-maximum_step, min(maximum_step, new - old))
        limited.append(old + delta)
    return limited


def main() -> int:
    root = Path(__file__).parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto",
                        help="COM port, or 'auto' for the known CH343 FE-URT-2")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--config", type=Path, default=root / "config" / "uarm_dobot_teleop.json")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--live-motion", action="store_true")
    parser.add_argument("--ack", default="")
    args = parser.parse_args()
    config = load_config(args.config)
    live = bool(args.live_motion)
    if live:
        locks = (config.get("motion_unlocked"), config.get("mapping_validated"),
                 config.get("power_validated"))
        if not all(locks):
            raise RuntimeError("live motion is locked in config")
        if args.ack != LIVE_ACK:
            raise RuntimeError(f"live motion requires --ack {LIVE_ACK}")
        if args.duration > float(config["maximum_live_duration_s"]):
            raise RuntimeError("requested duration exceeds configured live maximum")

    rate = float(config["rate_hz"])
    period = 1.0 / rate
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"uarm_dobot_teleop_{'live' if live else 'preview'}_{stamp}.jsonl"

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    feedback: FeedbackStream | None = None
    dashboard: DashboardClient | None = None
    opened = False
    active_motion_started = False
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        feedback = FeedbackStream(args.host, timeout=3.0)
        initial_feedback = feedback.wait_first(3.0)
        health = read_servo_health(protocol)
        positions, transport = read_positions(protocol)
        health_error: str | None = None
        health_warnings = uarm_health_warnings(health, config)
        try:
            require_uarm_health(health, config)
            if live:
                require_robot_health(initial_feedback)
            elif int(initial_feedback["error_status"]) != 0:
                raise RuntimeError("Dobot preview preflight failed: ErrorStatus is nonzero")
        except RuntimeError as exc:
            health_error = str(exc)
            if live:
                raise

        robot_baseline = [float(value) for value in initial_feedback["q_actual_deg"]]
        previous_positions = positions[:]
        accumulated = [0] * len(SERVO_IDS)
        previous_command = robot_baseline[:]

        if live:
            dashboard = DashboardClient(args.host, timeout=2.0)
            dashboard.command("RobotMode()")
            active_motion_started = True

        session = {
            "timestamp_utc": utc_now(), "kind": "session", "live_motion": live,
            "uarm_port_request": args.port, "uarm_port": port_name,
            "uarm_writes": 0, "automatic_enable": False, "automatic_clear_error": False,
            "automatic_control_mode_change": False, "health_error": health_error,
            "health_warnings": health_warnings,
            "power_validation": config.get("power_validation"),
            "health": health, "servo_baseline_ticks": positions,
            "servo_transport": transport, "dobot_baseline_deg": robot_baseline,
            "config": config,
        }
        deadline = time.monotonic() + args.duration
        next_tick = time.monotonic()
        sample = 0
        with log_path.open("x", encoding="utf-8") as log:
            log.write(json.dumps(session, ensure_ascii=False, allow_nan=False) + "\n")
            print(json.dumps(session, ensure_ascii=False, allow_nan=False))
            while time.monotonic() < deadline:
                current, transport = read_positions(protocol)
                if live and any(int(item["packet_error"]) != 0 for item in transport):
                    raise RuntimeError(f"U-Arm reported a runtime alarm: {transport}")
                for index, value in enumerate(current):
                    accumulated[index] += shortest_tick_step(previous_positions[index], value)
                previous_positions = current
                servo_deg = [ticks * DEGREES_PER_TICK for ticks in accumulated]
                relative = [float(config["joint_signs"][i]) * servo_deg[i] for i in range(6)]
                target = [robot_baseline[i] + relative[i] for i in range(6)]
                validate_target(target, relative, config)

                robot_state = feedback.latest()
                if live:
                    require_robot_health(robot_state, allow_running=True)
                    command = rate_limit(previous_command, target,
                                         config["max_speed_deg_s"], period)
                    servo_command = ("ServoJ(" + ",".join(f"{value:.4f}" for value in command)
                                     + f",t={period:.3f},aheadtime=50,gain=250)")
                    reply = dashboard.command(servo_command)
                    previous_command = command
                else:
                    command = target
                    reply = None

                record = {
                    "timestamp_utc": utc_now(), "kind": "sample", "sample": sample,
                    "live_motion": live, "servo_positions_ticks": current,
                    "servo_offsets_deg": servo_deg, "relative_joint_deg": relative,
                    "target_deg": target, "output_deg": command,
                    "gripper_offset_deg": servo_deg[6], "dobot_reply": reply,
                    "dobot_mode": robot_state["robot_mode"],
                    "dobot_error_status": robot_state["error_status"],
                }
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                if sample == 0 or sample % max(1, int(rate)) == 0:
                    print(json.dumps(record, ensure_ascii=False, allow_nan=False))
                sample += 1
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        print(json.dumps({"log": str(log_path), "samples": sample,
                          "live_motion": live}, ensure_ascii=False))
        return 0 if health_error is None else 3
    finally:
        if dashboard is not None:
            if active_motion_started:
                try:
                    dashboard.command("Stop()")
                except Exception:
                    pass
            dashboard.close()
        if feedback is not None:
            feedback.close()
        if opened and port.ser is not None:
            port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
