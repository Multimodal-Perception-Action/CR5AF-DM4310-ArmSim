"""Continuous, explicit-start U-Arm/CR5 teleoperation with a GUI watchdog.

No duration or session-relative travel cap. Manufacturer joint limits retain
a five-degree margin; speed is operator-selectable within 1..20 deg/s. A
normal return-home request moves to the operator-recorded six-axis operational
home before disabling. Fault/emergency stops always stop at the current pose.
Default is read-only.
"""
from __future__ import annotations

import argparse
import ctypes
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import threading
import time
from typing import Any

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
from feetech_port import resolve_uarm_port
from uarm_dobot_preview import read_positions, read_servo_health, shortest_tick_step
from uarm_dobot_teleop import FeedbackStream, load_config, require_uarm_health, uarm_health_warnings
from uarm_cr5_axis_follow_test import (
    Dashboard, JOINT_LIMITS_DEG, LIMIT_MARGIN_DEG, compact_state,
    require_disabled_preflight, require_enabled_healthy, wait_enable,
)
from uarm_gripper_binary_follow import read_dm_preflight, desired_state, send_best_effort_stop
from gripper import (
    PACKETS, EVENTS, GATEWAY_HEADER, receive_packet, decode_gateway_event,
    extract_packet, gateway_action_passed,
)

ROOT = Path(__file__).resolve().parents[1]
LIVE_CONFIRM = "I_UNDERSTAND_CONTINUOUS_CR5_WILL_MOVE"
PERIOD_S = 0.05
DEFAULT_MAX_SPEED_DEG_S = 12.0
MIN_MAX_SPEED_DEG_S = 1.0
HARD_MAX_SPEED_DEG_S = 20.0
TRACKING_ERROR_DEG = 3.0
DASHBOARD_TIMEOUT_S = 3.0
RETURN_HOME_SPEED_DEG_S = 6.0
RETURN_HOME_TOLERANCE_DEG = 0.35
GRIPPER_MIN_HYSTERESIS_TICKS = 20.0
GRIPPER_ENDPOINT_DEADBAND_FRACTION = 0.25
GRIPPER_STABLE_SAMPLES = 5
GUARDED_LIMITS = tuple((lo + LIMIT_MARGIN_DEG, hi - LIMIT_MARGIN_DEG) for lo, hi in JOINT_LIMITS_DEG)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def finite_six(values):
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("Expected six finite joint values")
    return [float(v) for v in values]


def validate_pose(values, tolerance=0.0):
    values = finite_six(values)
    for i, (value, (lo, hi)) in enumerate(zip(values, GUARDED_LIMITS)):
        if not lo-tolerance <= value <= hi+tolerance:
            raise RuntimeError(f"J{i+1} actual/target {value:.3f} exceeds guarded limits {lo}..{hi}")
    return values


def stop_request_kind(path):
    if path is None or not path.exists():
        return None
    try:
        text = path.read_text(encoding='ascii').strip().lower()
    except OSError:
        return 'emergency_stop'
    return 'return_home' if text == 'return_home' else 'emergency_stop'


def step_pose_toward(current, target, maximum_step):
    result = []
    for value, goal in zip(finite_six(current), finite_six(target)):
        delta = goal-value
        result.append(goal if abs(delta) <= maximum_step else value+math.copysign(maximum_step, delta))
    return result


def ticks_seven(values):
    if len(values) != 7 or any(isinstance(v, bool) or int(v) != v or not 0 <= v < 4096 for v in values):
        raise ValueError("Expected seven encoder ticks within 0..4095")
    return [int(v) for v in values]


def gripper_trigger_hysteresis_ticks(gripper_config):
    """Keep the middle half of calibrated ID7 travel as a hold-last zone."""
    endpoints = gripper_config.get('endpoints', {})
    try:
        opened = float(endpoints['maximum_open']['ticks'])
        closed = float(endpoints['fully_closed']['ticks'])
    except (KeyError, TypeError, ValueError):
        return GRIPPER_MIN_HYSTERESIS_TICKS
    travel = abs(closed-opened)
    if not math.isfinite(travel) or travel <= 0:
        return GRIPPER_MIN_HYSTERESIS_TICKS
    return max(
        GRIPPER_MIN_HYSTERESIS_TICKS,
        travel*GRIPPER_ENDPOINT_DEADBAND_FRACTION,
    )


class ContinuousMapping:
    """Pure position mapping; clipping rebases the integrator to avoid windup."""
    def __init__(self, baseline, ticks, signs, selected, deadband,
                 sources=None, scales=None, max_speed_deg_s=3.0,
                 max_follow_lag_deg=0.0, offsets_deg=None):
        self.baseline = validate_pose(baseline)
        self.previous = ticks_seven(ticks)
        self.signs = finite_six(signs)
        if any(s not in (-1, 1) for s in self.signs):
            raise ValueError("Signs must be -1 or 1")
        self.selected = set(selected)
        if not self.selected or not self.selected <= set(range(1, 7)):
            raise ValueError("Select joints 1..6")
        self.sources = list(sources if sources is not None else range(1, 7))
        if sorted(self.sources) != list(range(1, 7)):
            raise ValueError("Joint sources must be a permutation of U-Arm IDs 1..6")
        self.scales = finite_six(scales if scales is not None else [1.0]*6)
        if any(not 0.1 <= value <= 2.0 for value in self.scales):
            raise ValueError("Joint scales must be within 0.1..2.0")
        self.max_speed_deg_s = float(max_speed_deg_s)
        if not MIN_MAX_SPEED_DEG_S <= self.max_speed_deg_s <= HARD_MAX_SPEED_DEG_S:
            raise ValueError("Maximum speed must be within 1..20 deg/s")
        self.max_follow_lag_deg = float(max_follow_lag_deg)
        if not 0.0 <= self.max_follow_lag_deg <= 180.0:
            raise ValueError("Maximum follow lag must be within 0..180 degrees")
        self.offsets_deg = finite_six(offsets_deg if offsets_deg is not None else [0.0]*6)
        if any(not -180.0 <= value <= 180.0 for value in self.offsets_deg):
            raise ValueError("Joint offsets must be within -180..180 degrees")
        if len(deadband) != 6 or any(int(d) != d or not 0 <= d <= 20 for d in deadband):
            raise ValueError("Six deadbands in 0..20 required")
        self.deadband = list(deadband)
        self.accumulated = [0.0]*6
        self.offset_progress_deg = [0.0]*6
        self.output = self.baseline[:]

    def update(self, ticks):
        ticks = ticks_seven(ticks)
        target = self.baseline[:]
        raw = [0.0]*6
        clipped = []
        lag_clipped = []
        max_step_deg = self.max_speed_deg_s*PERIOD_S
        for i in range(6):
            source = self.sources[i]-1
            delta = shortest_tick_step(self.previous[source], ticks[source])
            if abs(delta) > 512:
                raise RuntimeError(
                    f"U-Arm ID{source+1} for CR5 J{i+1} encoder discontinuity "
                    ">45 degrees per sample"
                )
            if i+1 not in self.selected:
                continue
            offset_remaining = self.offsets_deg[i]-self.offset_progress_deg[i]
            self.offset_progress_deg[i] += max(
                -max_step_deg, min(max_step_deg, offset_remaining)
            )
            self.accumulated[i] += self.signs[i]*delta*self.scales[i]
            signed = self.accumulated[i]
            filtered = math.copysign(max(0.0, abs(signed)-self.deadband[i]), signed)
            raw[i] = filtered*360/4096
            requested = self.baseline[i]+self.offset_progress_deg[i]+raw[i]
            lo, hi = GUARDED_LIMITS[i]
            target[i] = max(lo, min(hi, requested))
            if target[i] != requested:
                clipped.append(i+1)
                bounded_ticks = (
                    target[i]-self.baseline[i]-self.offset_progress_deg[i]
                )*4096/360
                self.accumulated[i] = bounded_ticks + math.copysign(self.deadband[i], bounded_ticks)
            if self.max_follow_lag_deg > 0:
                lower = max(lo, self.output[i]-self.max_follow_lag_deg)
                upper = min(hi, self.output[i]+self.max_follow_lag_deg)
                bounded = max(lower, min(upper, target[i]))
                if bounded != target[i]:
                    target[i] = bounded
                    lag_clipped.append(i+1)
                    bounded_ticks = (
                        bounded-self.baseline[i]-self.offset_progress_deg[i]
                    )*4096/360
                    self.accumulated[i] = (
                        bounded_ticks + math.copysign(self.deadband[i], bounded_ticks)
                    )
        self.previous = ticks
        step = max_step_deg
        self.output = [old+max(-step, min(step, new-old)) for old,new in zip(self.output,target)]
        return dict(target_deg=target, output_deg=self.output[:], raw_relative_deg=raw,
                    joint_offsets_deg=self.offsets_deg[:],
                    joint_offset_progress_deg=self.offset_progress_deg[:],
                    limit_clipped_axes=clipped,
                    follow_lag_clipped_axes=lag_clipped)


class Heartbeat:
    """Use file changes and a monotonic clock after the initial wall-time check."""
    def __init__(self, path):
        self.path = Path(path)
        stat = self.path.stat()
        age = time.time()-stat.st_mtime
        if not -2 <= age <= 1.0:
            raise RuntimeError("GUI heartbeat is missing or stale at startup")
        self.signature = (stat.st_mtime_ns, stat.st_size)
        self.changed_at = time.monotonic()

    def check(self):
        try:
            stat = self.path.stat()
        except OSError as exc:
            raise RuntimeError("GUI heartbeat file disappeared") from exc
        if not -2 <= time.time()-stat.st_mtime <= 1.5:
            raise RuntimeError("GUI heartbeat wall-clock age is stale")
        signature = (stat.st_mtime_ns, stat.st_size)
        now = time.monotonic()
        if signature != self.signature:
            self.signature, self.changed_at = signature, now
        if now-self.changed_at > 1.5:
            raise RuntimeError("GUI heartbeat stopped for more than 1.5 seconds")


class FeedbackGuard:
    def __init__(self, feedback):
        self.feedback = feedback
        self.timestamp = None
        self.advanced_at = time.monotonic()

    def latest(self):
        state = self.feedback.latest(maximum_age=.25)
        now = time.monotonic()
        stamp = int(state['timestamp_ms'])
        if self.timestamp is not None and stamp < self.timestamp:
            raise RuntimeError("Controller timestamp moved backwards")
        if stamp != self.timestamp:
            self.timestamp, self.advanced_at = stamp, now
        if now-self.advanced_at > .25:
            raise RuntimeError("Controller timestamp stopped advancing")
        validate_pose(state['q_actual_deg'], tolerance=.25)
        if int(state['error_status']):
            raise RuntimeError("CR5 ErrorStatus is nonzero")
        return state


class ProcessMutex:
    def __enter__(self):
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.kernel.CreateMutexW.argtypes = (ctypes.c_void_p, ctypes.c_int, ctypes.c_wchar_p)
        self.kernel.CreateMutexW.restype = ctypes.c_void_p
        self.kernel.CloseHandle.argtypes = (ctypes.c_void_p,)
        ctypes.set_last_error(0)
        self.handle = self.kernel.CreateMutexW(None, False, 'Local\\CR5_UArm_Continuous_Controller_v1')
        error = ctypes.get_last_error()
        if not self.handle:
            raise OSError(error, "Could not create controller lock")
        if error == 183:
            self.kernel.CloseHandle(self.handle)
            self.handle = None
            raise RuntimeError("Another continuous controller is already running")
        return self

    def __exit__(self, *args):
        if self.handle:
            self.kernel.CloseHandle(self.handle)


def servo_command(values):
    values = validate_pose(values)
    return 'ServoJ('+','.join(f'{v:.6f}' for v in values)+',t=0.050,aheadtime=50,gain=250)'


def check_transport(transport):
    if len(transport) != 7 or any(int(x['comm_result']) != COMM_SUCCESS or int(x['packet_error']) for x in transport):
        raise RuntimeError("U-Arm communication error")


def require_idle_preflight(state):
    """Only idle disabled/idle enabled states; never adopt drag or motion."""
    mode, enabled = int(state['robot_mode']), int(state['enable_status'])
    if mode == 11:
        raise RuntimeError("Preflight rejected: RobotMode=11 indicates collision detection")
    if (mode, enabled) not in ((4, 0), (5, 1)) or int(state['running_status']) or int(state['error_status']):
        raise RuntimeError("Preflight requires idle mode 4/disabled or mode 5/enabled, without errors")
    validate_pose(state['q_actual_deg'])


def check_cancellation(args, heartbeat):
    if args.stop_file and args.stop_file.exists():
        raise RuntimeError("Operator stop requested")
    if heartbeat:
        heartbeat.check()


def wait_enabled_cancellable(feedback, args, heartbeat):
    deadline = time.monotonic()+8
    while time.monotonic() < deadline:
        check_cancellation(args, heartbeat)
        state = feedback.latest(maximum_age=.25)
        if int(state['error_status']):
            raise RuntimeError("CR5 ErrorStatus became nonzero while enabling")
        if int(state['enable_status']) == 1 and int(state['robot_mode']) in (5,7):
            require_enabled_healthy(state)
            return state
        time.sleep(.02)
    raise TimeoutError("CR5 enable feedback was not confirmed")


class GripperTargetMailbox:
    """Latest target only. STOP is carried by a separate, never-cleared Event."""
    def __init__(self, action):
        self._lock = threading.Lock()
        self._action = action

    def request(self, action):
        if action not in ('open', 'close'):
            raise ValueError('Gripper target must be open or close')
        with self._lock:
            self._action = action

    def latest(self):
        with self._lock:
            return self._action


def cancellable_gripper_worker(host, action, stop_event, result, targets=None):
    """One supervised session with independent target and latched STOP channels."""
    sock = None
    events = []
    commands = []
    sent = stopped = holding = releasing = False
    commanded_action = action
    targets = targets if targets is not None else GripperTargetMailbox(action)
    started = time.monotonic()
    last_heartbeat = None
    heartbeat_count = 0
    max_heartbeat_gap = 0.0

    def record_command(name):
        commands.append(dict(command=name, timestamp_utc=utc_now(),
                             elapsed_s=time.monotonic()-started))

    try:
        if stop_event.is_set():
            result.update(passed=True, cancelled_before_action=True)
            return
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(.05)
        if stop_event.is_set():
            result.update(passed=True, cancelled_before_action=True)
            return
        buffer = bytearray()
        sock.sendall(PACKETS['identity'])
        identity = decode_gateway_event(receive_packet(sock, buffer, GATEWAY_HEADER, 3.0))
        result['identity'] = identity
        if not (identity['event'] == EVENTS['identity'] and identity['result'] == 1 and identity['flags'] == 0x10):
            raise RuntimeError("Incompatible guarded gateway identity")
        if stop_event.is_set():
            result.update(passed=True, cancelled_before_action=True)
            return
        startup_reversal_supported = float(identity['position_rad']) >= 4.49
        result['startup_reversal_supported'] = startup_reversal_supported
        commanded_action = targets.latest()
        # sendall can fail after delivering a prefix (or a full packet). Take
        # cleanup responsibility before the attempt, not after its success.
        sent = True
        result['action_attempted'] = True
        sock.sendall(PACKETS[commanded_action])
        record_command(commanded_action)
        result.update(
            action_sent=True,
            transition_target=commanded_action,
            transition_pending=True,
        )
        deadline = time.monotonic()+15
        # The gateway may enter active control before its first hold event.
        # Heartbeats must cover startup, ramps, contact settle and all holds.
        next_heartbeat = time.monotonic()
        while True:
            now = time.monotonic()
            if (not holding or stopped or releasing) and now >= deadline:
                raise TimeoutError("Gripper action/stop did not produce a terminal event")
            # STOP has its own permanently latched channel. No reversal can
            # clear it, including a new target posted while sendall is blocked.
            if stop_event.is_set() and not stopped:
                sock.sendall(PACKETS['stop'])
                record_command('stop')
                stopped = True
                result['stop_sent'] = True
                deadline = time.monotonic()+3
            if not stopped and not stop_event.is_set():
                requested_action = targets.latest()
                result['requested_target'] = requested_action
                if (requested_action != commanded_action
                        and (holding or startup_reversal_supported)):
                    sock.sendall(PACKETS[requested_action])
                    record_command(requested_action)
                    commanded_action = requested_action
                    # A packet was sent even when the latest target returns to
                    # the last confirmed state; the gateway may still be
                    # physically reversing an in-flight transition.
                    releasing = True
                    result.update(
                        transition_target=commanded_action,
                        transition_pending=True,
                        controlled_transition_requested=commanded_action,
                    )
                    deadline = time.monotonic()+8
                # Legacy gateways ignore startup reversals: the latest target
                # stays pending until the first hold rather than becoming STOP.
            if not stopped and not stop_event.is_set() and now >= next_heartbeat:
                sock.sendall(PACKETS['hold_heartbeat'])
                heartbeat_at = time.monotonic()
                if last_heartbeat is not None:
                    max_heartbeat_gap = max(
                        max_heartbeat_gap, heartbeat_at-last_heartbeat
                    )
                last_heartbeat = heartbeat_at
                heartbeat_count += 1
                result.update(
                    heartbeat_count=heartbeat_count,
                    last_heartbeat_at_utc=utc_now(),
                    max_heartbeat_gap_s=max_heartbeat_gap,
                )
                next_heartbeat = heartbeat_at + .20
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("Gateway closed during action")
                buffer.extend(chunk)
            except socket.timeout:
                pass
            while True:
                packet = extract_packet(buffer, GATEWAY_HEADER)
                if packet is None:
                    break
                event = decode_gateway_event(packet)
                event.update(
                    received_at_utc=utc_now(),
                    received_elapsed_s=time.monotonic()-started,
                    since_last_heartbeat_s=(time.monotonic()-last_heartbeat
                                            if last_heartbeat is not None else None),
                )
                events.append(event)
                if event.get('diagnostic') == 'can_exchange':
                    result['last_can_diagnostic'] = event
                    if event.get('recovered'):
                        result['can_recovery_count'] = result.get('can_recovery_count', 0)+1
                    continue
                held_action = (
                    'open' if event['event'] == EVENTS['open'] else
                    'close' if event['event'] == EVENTS['close'] else None
                )
                if not stopped and held_action and event.get('holding_active'):
                    if not gateway_action_passed(held_action, event):
                        raise RuntimeError("Gripper active hold was not confirmed")
                    holding = True
                    releasing = held_action != commanded_action
                    result.update(
                        current_action=held_action,
                        contact_detected=bool(event.get('contact_detected')),
                        holding_active=True,
                        transition_pending=held_action != commanded_action,
                    )
                    continue
                if stopped and event['event'] == EVENTS['stop']:
                    passed = bool(
                        event['result'] == 1 and event['verified_disabled']
                    )
                    result.update(
                        passed=passed,
                        holding_active=False,
                        stopped_for_reversal=False,
                        stopped_by_operator=True,
                        transition_pending=False,
                    )
                    if not passed:
                        raise RuntimeError("Gripper stop did not confirm safe disable")
                    return
                if not stopped and event['event'] == EVENTS['stop']:
                    # A watchdog, feedback-limit, or other gateway-side abort is
                    # terminal even though the host did not request STOP.  Do
                    # not hide the actual failure behind a later host timeout.
                    result.update(
                        passed=False,
                        holding_active=False,
                        transition_pending=False,
                        gateway_abort_event=event,
                    )
                    raise RuntimeError(
                        "Gateway aborted active gripper session: "
                        f"result={event['result']} motor_error={event['motor_error']} "
                        f"verified_disabled={event['verified_disabled']} "
                        f"abort_reason={event.get('abort_reason', 'unknown')}"
                    )
                if not stopped and held_action:
                    passed = gateway_action_passed(held_action, event)
                    result.update(
                        passed=passed,
                        stopped_for_reversal=False,
                        contact_detected=bool(event.get('contact_detected')),
                        holding_active=bool(event.get('holding_active')),
                    )
                    if not passed:
                        raise RuntimeError("Gripper action did not complete safely")
                    return
    except Exception as exc:
        result.update(passed=False, error=f'{type(exc).__name__}: {exc}')
        if sent and sock is not None:
            try:
                sock.sendall(PACKETS['stop'])
                result['fallback_stop_sent'] = True
            except OSError:
                pass
    finally:
        result['events'] = events
        result['command_history'] = commands
        if sock is not None:
            sock.close()


def write_row(stream, row):
    stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    stream.flush()


def open_dashboard(host):
    """Open a clean command stream with enough time for CR5 enable replies."""
    dashboard = Dashboard(host)
    dashboard.socket.settimeout(DASHBOARD_TIMEOUT_S)
    return dashboard


def run_session(args, config, gripper_config, dm_config, report, stream):
    port = feedback = dashboard = None
    dashboard_io_healthy = True
    opened = enable_requested = False
    worker = worker_stop = worker_result = active_action = worker_targets = None
    worker_recorded = False
    started = time.monotonic()
    return_home_requested = False
    joint_offsets_deg = getattr(args, 'joint_offsets_deg', [0.0]*6)
    report.update(passed=False, arm_sample_count=0, gripper_action_count=0,
                  max_actual_excursion_deg=[0.0]*6, automatic_return=False)

    def live_command(command):
        nonlocal dashboard_io_healthy
        if dashboard is None:
            raise RuntimeError("Dashboard is not connected")
        try:
            return dashboard.command(command)
        except BaseException:
            # A timed-out TCP stream may still receive that command's late
            # response. It must not be reused because replies would be shifted.
            dashboard_io_healthy = False
            raise
    try:
        heartbeat = Heartbeat(args.heartbeat_file) if args.heartbeat_file else None
        if args.stop_file and args.stop_file.exists():
            raise RuntimeError("Stop file already exists; use a new session")
        port_name = resolve_uarm_port(args.port)
        port = PortHandler(port_name)
        protocol = sms_sts(port)
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"Could not open {port_name} at 1 Mbps")
        feedback = FeedbackStream(args.host, timeout=3.0)
        initial = feedback.wait_first(3.0)
        require_idle_preflight(initial)
        report['cr5_entry_state'] = compact_state(initial)
        baseline = validate_pose(initial['q_actual_deg'])
        health = read_servo_health(protocol)
        require_uarm_health(health, config)
        positions, transport = read_positions(protocol)
        check_transport(transport)
        deadband = config.get('input_calibration', {}).get('joint_input_deadband_ticks', [2]*6)
        mapping = ContinuousMapping(
            baseline, positions, args.joint_signs, args.joints, deadband,
            args.joint_sources, args.joint_scales, args.max_speed_deg_s,
            args.max_follow_lag_deg, joint_offsets_deg,
        )
        dm_initial = (
            {"skipped": True, "reason": "arm_only mode; no gripper or DM gateway access"}
            if args.arm_only else read_dm_preflight(args.host, dm_config)
        )
        # Pair only after the potentially slow hardware checks. A hand movement
        # during preflight must not become an initial motion command.
        initial = feedback.latest(maximum_age=.25)
        require_idle_preflight(initial)
        if args.live_output and int(initial['enable_status']) == 1:
            # Explicit live Start authorizes normalization of an already enabled
            # idle robot. Drag/moving/error states are never adopted.
            check_cancellation(args, heartbeat)
            dashboard = open_dashboard(args.host)
            mode_reply = live_command('RobotMode()')
            if mode_reply.replace(' ', '') != '0,{5},RobotMode();':
                raise RuntimeError(f"Robot left enabled idle before normalization: {mode_reply}")
            checked = feedback.latest(maximum_age=.25)
            require_idle_preflight(checked)
            if int(checked['robot_mode']) != 5:
                raise RuntimeError("Robot left enabled idle before normalization")
            check_cancellation(args, heartbeat)
            enable_requested = True  # Cleanup responsibility begins before output.
            report['initial_normalization'] = dict(from_state=compact_state(checked),
                                                  mode_reply=mode_reply)
            report['initial_normalization']['disable_reply'] = live_command('DisableRobot()')
            initial = wait_enable(feedback, False)
            require_disabled_preflight(initial)
            report['initial_normalization']['disabled_state'] = compact_state(initial)
        if args.live_output:
            require_disabled_preflight(initial)
        baseline = validate_pose(initial['q_actual_deg'])
        positions, transport = read_positions(protocol)
        check_transport(transport)
        mapping = ContinuousMapping(
            baseline, positions, args.joint_signs, args.joints, deadband,
            args.joint_sources, args.joint_scales, args.max_speed_deg_s,
            args.max_follow_lag_deg, joint_offsets_deg,
        )
        report.update(uarm_port=port_name, uarm_health=health,
                      uarm_health_warnings=uarm_health_warnings(health, config),
                      uarm_baseline_ticks=positions, cr5_initial=compact_state(initial), dm_initial=dm_initial)
        midpoint = float(gripper_config['endpoints']['midpoint_ticks'])
        trigger_hysteresis = gripper_trigger_hysteresis_ticks(gripper_config)
        report['gripper_trigger'] = dict(
            midpoint_ticks=midpoint,
            hysteresis_ticks=trigger_hysteresis,
            open_enter_ticks=midpoint-trigger_hysteresis,
            close_enter_ticks=midpoint+trigger_hysteresis,
            stable_samples=GRIPPER_STABLE_SAMPLES,
            neutral_behavior='hold_last_confirmed_state',
        )
        desired = 'open' if positions[6] <= midpoint else 'close'
        candidate, count = desired, 0
        last_requested = desired  # No initial action, regardless of current gripper position.
        guard = FeedbackGuard(feedback)
        if heartbeat:
            heartbeat.check()
        if args.stop_file and args.stop_file.exists():
            raise RuntimeError("Stopped during preflight")
        if args.live_output:
            if dashboard is None:
                dashboard = open_dashboard(args.host)
            report['robot_mode_reply'] = live_command('RobotMode()')
            check_cancellation(args, heartbeat)
            enable_requested = True
            report['enable_reply'] = live_command('EnableRobot()')
            wait_enabled_cancellable(feedback, args, heartbeat)
            # Enabling may take seconds. Pair again at the actual start so
            # moving the leader while waiting cannot queue a catch-up motion.
            positions, transport = read_positions(protocol)
            check_transport(transport)
            start_state = guard.latest()
            require_enabled_healthy(start_state)
            baseline = validate_pose(start_state['q_actual_deg'])
            mapping = ContinuousMapping(
                baseline, positions, args.joint_signs, args.joints, deadband,
                args.joint_sources, args.joint_scales, args.max_speed_deg_s,
                args.max_follow_lag_deg, joint_offsets_deg,
            )
            desired = 'open' if positions[6] <= midpoint else 'close'
            candidate, count, last_requested = desired, 0, desired
            report.update(uarm_baseline_ticks=positions, cr5_control_start=compact_state(start_state))
            check_cancellation(args, heartbeat)
        write_row(stream, dict(kind='ready', timestamp_utc=utc_now(), live_output=args.live_output,
                               baseline_deg=baseline, uarm_baseline_ticks=positions))
        last_loop = time.monotonic()
        next_health = last_loop+3.0
        while True:
            now = time.monotonic()
            request = stop_request_kind(args.stop_file)
            if request:
                return_home_requested = request == 'return_home'
                report.update(passed=True, stop_reason=('return_home_requested' if return_home_requested else 'operator_stop'))
                break
            if heartbeat:
                heartbeat.check()
            if now-last_loop > .25:
                raise RuntimeError("Control loop stalled for more than 250 ms")
            last_loop = now
            if now >= next_health:
                current_health = read_servo_health(protocol)
                require_uarm_health(current_health, config)
                report['last_uarm_health'] = current_health
                write_row(stream, dict(kind='uarm_health', timestamp_utc=utc_now(),
                                       health=current_health,
                                       warnings=uarm_health_warnings(current_health,config)))
                next_health = time.monotonic()+3.0
            current, transport = read_positions(protocol)
            check_transport(transport)
            state = guard.latest()
            if args.live_output:
                require_enabled_healthy(state)
                errors = [abs(float(a)-b) for a,b in zip(state['q_actual_deg'], mapping.output)]
                if max(errors) > TRACKING_ERROR_DEG:
                    raise RuntimeError(f"CR5 tracking error exceeded {TRACKING_ERROR_DEG} degrees: {errors}")
                for i in range(6):
                    if i+1 not in args.joints and abs(float(state['q_actual_deg'][i])-baseline[i]) > .3:
                        raise RuntimeError(f"Unselected J{i+1} moved outside the hold tolerance")
            mapped = mapping.update(current)
            if time.monotonic()-last_loop > .25:
                raise RuntimeError("U-Arm/control read stalled for more than 250 ms")
            # Recheck cancellation immediately before any new arm output.
            request = stop_request_kind(args.stop_file)
            if request:
                return_home_requested = request == 'return_home'
                report.update(passed=True, stop_reason=('return_home_requested' if return_home_requested else 'operator_stop'))
                break
            if heartbeat:
                heartbeat.check()
            reply = live_command(servo_command(mapped['output_deg'])) if args.live_output else None
            # A dashboard reply may block while STOP is requested. Check again
            # before publishing any gripper target, not only when starting its
            # worker, so an existing grasp cannot reverse after cancellation.
            request = stop_request_kind(args.stop_file)
            if request:
                return_home_requested = request == 'return_home'
                report.update(passed=True, stop_reason=(
                    'return_home_requested' if return_home_requested else 'operator_stop'
                ))
                break
            if heartbeat:
                heartbeat.check()
            proposed = desired_state(
                current[6], midpoint, trigger_hysteresis, desired
            )
            candidate, count = (candidate, count+1) if proposed == candidate else (proposed, 1)
            if count >= GRIPPER_STABLE_SAMPLES:
                desired = proposed
            if worker is not None and not worker.is_alive():
                worker.join()
                write_row(stream, dict(kind='gripper_action', timestamp_utc=utc_now(), **worker_result))
                report['gripper_action_count'] += 1
                report['last_gripper_action'] = worker_result
                worker_recorded = True
                if not worker_result.get('passed'):
                    raise RuntimeError(f"Gripper action failed: {worker_result}")
                if worker_result.get('released_to_action'):
                    last_requested = worker_result['released_to_action']
                elif worker_result.get('stopped_for_reversal') or worker_result.get('cancelled_before_action'):
                    last_requested = None
                worker = worker_stop = worker_result = active_action = worker_targets = None
            if (
                worker is not None and worker.is_alive()
                and worker_result.get('current_action')
                and worker_result['current_action'] != active_action
            ):
                active_action = worker_result['current_action']
                last_requested = active_action
            if worker is not None and worker.is_alive():
                # Always publish the latest stable intention, even if it
                # returns to the current target before a prior request is read.
                worker_targets.request(desired)
            if (args.live_output and not args.arm_only and worker is None
                    and count >= GRIPPER_STABLE_SAMPLES
                    and desired != last_requested):
                # A dashboard reply may have blocked while the operator stopped.
                check_cancellation(args, heartbeat)
                active_action = desired
                last_requested = desired
                worker_stop = threading.Event()
                worker_targets = GripperTargetMailbox(desired)
                worker_result = dict(
                    action=desired,
                    action_sent=False,
                    trigger_ticks=current[6],
                    transition_target=desired,
                    transition_pending=True,
                )
                worker_recorded = False
                worker = threading.Thread(target=cancellable_gripper_worker,
                    args=(args.host, desired, worker_stop, worker_result, worker_targets), daemon=True)
                worker.start()
            actual_relative = [float(v)-b for v,b in zip(state['q_actual_deg'], baseline)]
            report['max_actual_excursion_deg'] = [max(old, abs(v)) for old,v in zip(report['max_actual_excursion_deg'],actual_relative)]
            gripper_snapshot = dict(worker_result) if worker_result is not None else {}
            write_row(stream, dict(kind='sample', continuous=True, live_output=args.live_output,
                arm_only=args.arm_only,
                timestamp_utc=utc_now(), sample=report['arm_sample_count'], servo_ticks=current,
                arm_target_deg=mapped['target_deg'], arm_output_deg=mapped['output_deg'],
                raw_relative_deg=mapped['raw_relative_deg'], arm_actual_relative_deg=actual_relative,
                arm_max_relative_deg=None, arm_max_speed_deg_s=args.max_speed_deg_s,
                joint_sources=args.joint_sources, joint_scales=args.joint_scales,
                joint_offsets_deg=joint_offsets_deg,
                joint_offset_progress_deg=mapped['joint_offset_progress_deg'],
                max_follow_lag_deg=args.max_follow_lag_deg,
                joint_limits_deg=GUARDED_LIMITS, selected_joints=args.joints,
                limit_clipped_axes=mapped['limit_clipped_axes'],
                follow_lag_clipped_axes=mapped['follow_lag_clipped_axes'], arm_reply=reply,
                cr5=compact_state(state), gripper_desired=desired, gripper_active_action=active_action,
                gripper_transition_target=gripper_snapshot.get('transition_target'),
                gripper_contact_detected=bool(gripper_snapshot.get('contact_detected', False)),
                gripper_holding_active=bool(gripper_snapshot.get('holding_active', False)),
                gripper_transition_pending=bool(gripper_snapshot.get('transition_pending', False)),
                gripper_heartbeat_count=gripper_snapshot.get('heartbeat_count', 0),
                gripper_max_heartbeat_gap_s=gripper_snapshot.get('max_heartbeat_gap_s', 0.0),
                gripper_command_issued=report['gripper_action_count'] > 0 or worker is not None))
            report['arm_sample_count'] += 1
            # A full period after processing prevents any catch-up burst and
            # guarantees <=3 deg/s even when reads/commands run late.
            time.sleep(PERIOD_S)

        if return_home_requested:
            if not args.live_output:
                raise RuntimeError('Return-home requires live output')
            home = validate_pose(args.operational_home)
            if worker_stop is not None:
                worker_stop.set()
            if worker is not None and worker.is_alive():
                worker.join(timeout=7.0)
            if worker is not None and worker.is_alive():
                raise RuntimeError('Gripper did not stop before return-home motion')
            if worker_result is not None and not worker_result.get('passed'):
                raise RuntimeError(f'Gripper stop failed before return-home: {worker_result}')
            state = guard.latest()
            require_enabled_healthy(state)
            commanded = validate_pose(state['q_actual_deg'])
            maximum_distance = max(abs(goal-value) for goal,value in zip(home, commanded))
            deadline = time.monotonic()+maximum_distance/RETURN_HOME_SPEED_DEG_S+12.0
            stable_samples = 0
            return_sample = 0
            while True:
                if stop_request_kind(args.stop_file) != 'return_home':
                    raise RuntimeError('Immediate stop requested during return-home motion')
                if heartbeat:
                    heartbeat.check()
                state = guard.latest()
                require_enabled_healthy(state)
                actual = validate_pose(state['q_actual_deg'])
                differences = [abs(goal-value) for goal,value in zip(home, actual)]
                if max(differences) <= RETURN_HOME_TOLERANCE_DEG:
                    stable_samples += 1
                    if stable_samples >= 5:
                        break
                else:
                    stable_samples = 0
                if time.monotonic() > deadline:
                    raise TimeoutError(f'Return-home timed out; remaining joint errors: {differences}')
                commanded = step_pose_toward(
                    commanded, home, RETURN_HOME_SPEED_DEG_S*PERIOD_S
                )
                tracking = [abs(value-target) for value,target in zip(actual, commanded)]
                if max(tracking) > TRACKING_ERROR_DEG:
                    raise RuntimeError(f'Return-home tracking error exceeded {TRACKING_ERROR_DEG} degrees: {tracking}')
                reply = live_command(servo_command(commanded))
                write_row(stream, dict(
                    kind='return_home', timestamp_utc=utc_now(), sample=return_sample,
                    home_target_deg=home, arm_output_deg=commanded,
                    remaining_error_deg=differences, arm_reply=reply,
                    cr5=compact_state(state), return_speed_deg_s=RETURN_HOME_SPEED_DEG_S,
                ))
                return_sample += 1
                time.sleep(PERIOD_S)
            report.update(
                automatic_return=True,
                return_home_target_deg=home,
                return_home_final=compact_state(state),
                return_home_max_error_deg=max(differences),
                return_home_samples=return_sample,
                stop_reason='returned_home',
            )
    except KeyboardInterrupt:
        report.update(passed=True, stop_reason='keyboard_interrupt')
    except BaseException as exc:
        report.update(passed=False, error=f'{type(exc).__name__}: {exc}', stop_reason='fault')
    finally:
        report['enable_requested'] = enable_requested
        # First cancel gripper and stop the arm. Never wait for a worker before Stop.
        if worker_stop is not None:
            worker_stop.set()
        if dashboard is not None and enable_requested:
            if not dashboard_io_healthy:
                try:
                    dashboard.close()
                except Exception as exc:
                    report['desynchronized_dashboard_close_error'] = repr(exc)
                dashboard = None
            for name, command in [('stop', 'Stop()'), ('disable', 'DisableRobot()')]:
                if dashboard is None:
                    try:
                        dashboard = open_dashboard(args.host)
                        report[f'finally_{name}_used_fresh_dashboard'] = True
                    except Exception as exc:
                        report[f'finally_{name}_connect_error'] = repr(exc)
                        report['passed'] = False
                        continue
                try:
                    report[f'finally_{name}_reply'] = dashboard.command(command)
                except Exception as exc:
                    report[f'finally_{name}_error'] = repr(exc)
                    report['passed'] = False
                    # Do not let a late Stop reply get mistaken for Disable.
                    try:
                        dashboard.close()
                    except Exception as close_exc:
                        report[f'finally_{name}_close_error'] = repr(close_exc)
                    dashboard = None
            if feedback is not None:
                try:
                    final = wait_enable(feedback, False)
                    require_disabled_preflight(final)
                    report.update(cr5_final=compact_state(final), cr5_final_disable_verified=True)
                except Exception as exc:
                    report.update(cr5_final_disable_verified=False, cr5_final_disable_error=repr(exc), passed=False)
        gripper_stop_unconfirmed = False
        if worker is not None:
            try:
                worker.join(timeout=7.0)
            except Exception as exc:
                report.update(gripper_worker_join_error=repr(exc), passed=False)
                gripper_stop_unconfirmed = True
            if worker.is_alive():
                report.update(gripper_worker_stop_timeout=True, passed=False)
                gripper_stop_unconfirmed = True
            elif not worker_recorded:
                report['gripper_action_count'] += 1
                report['last_gripper_action'] = worker_result
                if not worker_result.get('passed'):
                    report['passed'] = False
                    gripper_stop_unconfirmed = True
        elif report.get('last_gripper_action') and not report['last_gripper_action'].get('passed'):
            gripper_stop_unconfirmed = True
        if args.live_output and enable_requested and not args.arm_only and gripper_stop_unconfirmed:
            try:
                send_best_effort_stop(args.host)
                report['gripper_final_best_effort_stop_sent'] = True
            except Exception as exc:
                report.update(gripper_final_stop_error=repr(exc), passed=False)
        # Each resource is released even when another close operation fails.
        for name, resource, closer in [('dashboard', dashboard, 'close'), ('feedback', feedback, 'close'),
                                        ('uarm_port', port if opened else None, 'closePort')]:
            if resource is not None:
                try:
                    getattr(resource, closer)()
                except Exception as exc:
                    report[f'{name}_close_error'] = repr(exc)
                    report['passed'] = False
        # No CAN status request may race an active gripper action (status resets CAN).
        if args.live_output and enable_requested and not args.arm_only and not (worker is not None and worker.is_alive()):
            try:
                report['dm_final'] = read_dm_preflight(args.host, dm_config)
                report['dm_final_disabled_no_fault'] = True
            except Exception as exc:
                report.update(dm_final_error=repr(exc), dm_final_disabled_no_fault=False, passed=False)
        report.update(completed_at_utc=utc_now(), duration_actual_s=time.monotonic()-started)
    return 0 if report['passed'] else 1


def csv_ints(value):
    return [int(v.strip()) for v in value.split(',')]


def csv_floats(value):
    return [float(v.strip()) for v in value.split(',')]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',
                        help='robot controller address (required when starting the program)')
    parser.add_argument('--port', default='auto')
    parser.add_argument('--live-output', action='store_true')
    parser.add_argument('--arm-only', action='store_true',
                        help='skip DM gateway and all gripper commands; only selected CR5 joints can follow U-Arm')
    parser.add_argument('--enable-robot', action='store_true')
    parser.add_argument('--confirm', default='')
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--heartbeat-file', type=Path)
    parser.add_argument('--telemetry-log', type=Path, required=True)
    parser.add_argument('--joints', type=csv_ints, default=[1,2,3,4,5,6])
    parser.add_argument('--joint-signs', type=csv_ints)
    parser.add_argument('--joint-sources', type=csv_ints, default=[1,2,3,4,5,6])
    parser.add_argument('--joint-scales', type=csv_floats, default=[1.0]*6)
    parser.add_argument('--joint-offsets-deg', type=csv_floats, default=[0.0]*6)
    parser.add_argument('--max-speed-deg-s', type=float, default=DEFAULT_MAX_SPEED_DEG_S)
    parser.add_argument('--max-follow-lag-deg', type=float, default=20.0)
    args = parser.parse_args(argv)
    if not args.joints or len(set(args.joints)) != len(args.joints) or not set(args.joints) <= set(range(1,7)):
        parser.error('--joints must contain unique values in 1..6')
    if args.joint_signs is not None and (len(args.joint_signs) != 6 or any(v not in (-1,1) for v in args.joint_signs)):
        parser.error('--joint-signs requires six -1/+1 values')
    if sorted(args.joint_sources) != list(range(1,7)):
        parser.error('--joint-sources must be a permutation of 1..6')
    if len(args.joint_scales) != 6 or any(not math.isfinite(v) or not 0.1 <= v <= 2.0 for v in args.joint_scales):
        parser.error('--joint-scales requires six finite values within 0.1..2.0')
    if len(args.joint_offsets_deg) != 6 or any(not math.isfinite(v) or not -180.0 <= v <= 180.0 for v in args.joint_offsets_deg):
        parser.error('--joint-offsets-deg requires six finite values within -180..180')
    if not math.isfinite(args.max_speed_deg_s) or not MIN_MAX_SPEED_DEG_S <= args.max_speed_deg_s <= HARD_MAX_SPEED_DEG_S:
        parser.error('--max-speed-deg-s must be within 1..20')
    if not math.isfinite(args.max_follow_lag_deg) or not 0.0 <= args.max_follow_lag_deg <= 180.0:
        parser.error('--max-follow-lag-deg must be within 0..180; 0 disables backlog clipping')
    if args.live_output and not (args.enable_robot and args.confirm == LIVE_CONFIRM and args.stop_file and args.heartbeat_file):
        parser.error('Live output requires enable, continuous confirmation, stop file and GUI heartbeat')
    if not args.live_output and (args.enable_robot or args.confirm):
        parser.error('Enable and confirmation require --live-output')
    return args


def main(argv=None):
    args = parse_args(argv)
    if not args.host:
        raise SystemExit('--host is required; pass the controller address explicitly')
    config = load_config(ROOT/'config/uarm_dobot_teleop.json')
    args.joint_signs = args.joint_signs if args.joint_signs is not None else config['joint_signs']
    gripper = json.loads((ROOT/'config/uarm_gripper_calibration.json').read_text(encoding='utf-8'))
    dm = json.loads((ROOT/'config/dm4310_gripper_calibration.json').read_text(encoding='utf-8'))
    calibration = json.loads((ROOT/'config/uarm_cr5_full_calibration.json').read_text(encoding='utf-8'))
    operational_home = calibration['robot']['operational_home']
    if not operational_home.get('operator_designated_as_final_home'):
        raise RuntimeError('Operational home has not been confirmed by the operator')
    args.operational_home = validate_pose(operational_home['joint_deg'])
    if args.live_output:
        if not all(config.get(k) for k in ('power_validated','mapping_validated','full_teleop_unlocked')):
            raise RuntimeError('Existing power/mapping/preflight validation is required')
        if not args.arm_only and not all(gripper['endpoints'][k].get('confirmed') for k in ('maximum_open','fully_closed')):
            raise RuntimeError('Gripper endpoints must be confirmed')
    report = dict(timestamp_utc=utc_now(),
                  operation=('continuous U-Arm CR5 arm-only' if args.arm_only
                             else 'continuous U-Arm CR5 + guarded DM gripper'),
                  live_output=args.live_output, continuous=True, duration_requested_s=None,
                  arm_only=args.arm_only,
                  arm_max_relative_deg=None, arm_max_speed_deg_s=args.max_speed_deg_s, rate_hz=20,
                  arm_mapping_signs=args.joint_signs, joint_sources=args.joint_sources,
                  joint_scales=args.joint_scales, joint_offsets_deg=args.joint_offsets_deg,
                  selected_joints=args.joints,
                  max_follow_lag_deg=args.max_follow_lag_deg,
                  joint_limits_deg=GUARDED_LIMITS, tracking_error_limit_deg=TRACKING_ERROR_DEG,
                  collision_clearance_verified=False, telemetry_log=str(args.telemetry_log),
                  authorization_profile='explicit continuous confirmation; old 5deg unlock is not a range authorization',
                  gripper_output_enabled=not args.arm_only)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    report_path = ROOT/'logs'/f'uarm_cr5_continuous_{stamp}.json'
    with ProcessMutex():
        with args.telemetry_log.open('x', encoding='utf-8') as stream:
            write_row(stream, dict(kind='session', schema_version=1, **report))
            code = run_session(args, config, gripper, dm, report, stream)
            report['log_file'] = str(report_path)
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            cleanup = {k:v for k,v in report.items() if k in ('enable_requested', 'automatic_return') or k.startswith(('finally_', 'cr5_final', 'dm_final', 'gripper_final', 'gripper_worker', 'return_home'))}
            write_row(stream, dict(kind='terminal', timestamp_utc=utc_now(), continuous=True,
                live_output=args.live_output, passed=report['passed'], error=report.get('error'),
                stop_reason=report.get('stop_reason'), arm_sample_count=report['arm_sample_count'],
                gripper_action_count=report['gripper_action_count'], cleanup=cleanup, report_file=str(report_path)))
    print(json.dumps(dict(passed=report['passed'], error=report.get('error'),
                         stop_reason=report.get('stop_reason'), report_file=str(report_path)), ensure_ascii=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
