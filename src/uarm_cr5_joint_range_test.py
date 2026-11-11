"""Explicit, bounded ONE-joint 10/20 degree U-Arm -> CR5 range trial.

Default is read-only. Live output needs --live-output --enable-robot and the
exact confirmation token. No gripper request, mode switch, error clear, or
U-Arm register write is implemented. This is separate from the existing
five-degree six-joint controller; no existing unlock/configuration is changed.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
from feetech_port import resolve_uarm_port
from uarm_dobot_preview import read_positions, read_servo_health, shortest_tick_step
from uarm_dobot_teleop import FeedbackStream, load_config, require_uarm_health, uarm_health_warnings
from uarm_cr5_axis_follow_test import (
    Dashboard, compact_state, require_disabled_preflight, require_enabled_healthy,
    safe_return_to_baseline, validate_pose, wait_enable,
)
from uarm_cr5_full_teleop import telemetry_log

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config/uarm_dobot_teleop.json'
LIVE_CONFIRM = 'I_UNDERSTAND_SINGLE_JOINT_EXPANDED_RANGE'
PERIOD_S = 0.05
MAX_SPEED_DEG_S = 3.0
DEADBAND_TICKS = 2


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def six_finite(values):
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Expected six finite joint angles')
    return [float(v) for v in values]


def check_envelope(actual, baseline, selected_index, maximum_deg):
    actual = six_finite(actual)
    baseline = six_finite(baseline)
    validate_pose(actual)
    delta = [v-b for v,b in zip(actual,baseline)]
    for i, value in enumerate(delta):
        allowance = maximum_deg + .25 if i == selected_index else .3
        if abs(value) > allowance:
            raise RuntimeError(f'J{i+1} actual delta {value:.4f} exceeds {allowance:.2f} deg')
    return delta


class SingleJointMapping:
    """Pure 1:1 encoder delta mapping with only one movable target axis."""
    def __init__(self, baseline, selected_index, initial_tick, sign, maximum_deg):
        self.baseline = six_finite(baseline)
        if selected_index not in range(6) or sign not in (-1, 1) or maximum_deg not in (10, 20):
            raise ValueError('Invalid joint, sign or range')
        self.index, self.sign, self.maximum = selected_index, sign, maximum_deg
        self.previous_tick = self._tick(initial_tick)
        self.accumulated = 0
        self.last_output = self.baseline[:]
        for offset in (-maximum_deg, maximum_deg):
            endpoint = self.baseline[:]
            endpoint[selected_index] += offset
            validate_pose(endpoint)

    @staticmethod
    def _tick(value):
        if isinstance(value, bool) or int(value) != value or not 0 <= value < 4096:
            raise ValueError('Encoder tick must be an integer within 0..4095')
        return int(value)

    def update(self, tick):
        tick = self._tick(tick)
        self.accumulated += shortest_tick_step(self.previous_tick, tick)
        self.previous_tick = tick
        signed = self.sign*self.accumulated
        filtered = math.copysign(max(0, abs(signed)-DEADBAND_TICKS), signed)
        raw = filtered*360/4096
        offset = max(-self.maximum, min(self.maximum, raw))
        target = self.baseline[:]
        target[self.index] += offset
        output = self.baseline[:]
        previous = self.last_output[self.index]
        step = max(-MAX_SPEED_DEG_S*PERIOD_S,
                   min(MAX_SPEED_DEG_S*PERIOD_S, target[self.index]-previous))
        output[self.index] = previous+step
        self.last_output = output[:]
        return {'target_deg':target, 'output_deg':output, 'input_relative_deg':raw,
                'selected_offset_deg':offset}


def servo_command(q):
    q = six_finite(q)
    validate_pose(q)
    return 'ServoJ('+','.join(f'{v:.6f}' for v in q)+',t=0.050,aheadtime=50,gain=250)'


class GuardedFeedback:
    def __init__(self, inner, baseline, index, maximum):
        self.inner, self.baseline, self.index, self.maximum = inner, baseline, index, maximum
        self.last_controller_stamp = None
        self.last_advance = time.monotonic()

    def latest(self, maximum_age=.25):
        state = self.inner.latest(maximum_age=maximum_age)
        stamp = int(state['timestamp_ms'])
        now = time.monotonic()
        if self.last_controller_stamp is not None and stamp < self.last_controller_stamp:
            raise RuntimeError('Controller feedback timestamp moved backwards')
        if stamp != self.last_controller_stamp:
            self.last_controller_stamp, self.last_advance = stamp, now
        if now-self.last_advance > .25:
            raise RuntimeError('Controller feedback timestamp stopped advancing')
        check_envelope(state['q_actual_deg'], self.baseline, self.index, self.maximum)
        if int(state['error_status']):
            raise RuntimeError('CR5 ErrorStatus became nonzero')
        return state


def check_transport(transport):
    if len(transport) != 7 or any(int(x['comm_result']) != COMM_SUCCESS or int(x['packet_error']) for x in transport):
        raise RuntimeError('U-Arm communication error')


def run_session(args, config, report, stream):
    port = protocol = feedback = guarded = dashboard = None
    opened = enabled_requested = completed_normally = False
    baseline = None
    started = time.monotonic()
    try:
        port_name = resolve_uarm_port(args.port)
        port = PortHandler(port_name)
        protocol = sms_sts(port)
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f'Could not open {port_name} at 1 Mbps')
        feedback = FeedbackStream(args.host, timeout=3.0)
        initial = feedback.wait_first(3.0)
        require_disabled_preflight(initial)
        baseline = six_finite(initial['q_actual_deg'])
        validate_pose(baseline)
        health = read_servo_health(protocol)
        require_uarm_health(health, config)
        positions, transport = read_positions(protocol)
        check_transport(transport)
        mapping = SingleJointMapping(baseline, args.joint-1, positions[args.joint-1],
                                     config['joint_signs'][args.joint-1], args.max_relative_deg)
        guarded = GuardedFeedback(feedback, baseline, args.joint-1, args.max_relative_deg)
        report.update(uarm_port=port_name, uarm_baseline_ticks=positions,
                      cr5_initial=compact_state(initial), uarm_health=health,
                      uarm_health_warnings=uarm_health_warnings(health,config))
        if args.live_output:
            dashboard = Dashboard(args.host)
            report['robot_mode_reply'] = dashboard.command('RobotMode()')
            enabled_requested = True
            report['enable_reply'] = dashboard.command('EnableRobot()')
            enabled = wait_enable(guarded, True)
            require_enabled_healthy(enabled)
            report['cr5_enabled'] = compact_state(enabled)
        deadline = time.monotonic()+args.duration
        sample = 0
        while time.monotonic() < deadline:
            positions, transport = read_positions(protocol)
            check_transport(transport)
            result = mapping.update(positions[args.joint-1])
            state = guarded.latest()
            if args.live_output:
                require_enabled_healthy(state)
            else:
                require_disabled_preflight(state)
            relative = check_envelope(state['q_actual_deg'],baseline,args.joint-1,args.max_relative_deg)
            reply = dashboard.command(servo_command(result['output_deg'])) if args.live_output else None
            row = {'kind':'sample','timestamp_utc':utc_now(),'sample':sample,
                   'live_output':args.live_output,'selected_joint':args.joint,
                   'arm_max_relative_deg':args.max_relative_deg,'arm_max_speed_deg_s':MAX_SPEED_DEG_S,
                   'servo_ticks':positions,'arm_target_deg':result['target_deg'],
                   'arm_output_deg':result['output_deg'],'input_relative_deg':result['input_relative_deg'],
                   'arm_actual_relative_deg':relative,'arm_reply':reply,'cr5':compact_state(state),
                   'gripper_control_enabled':False,'gripper_desired':'unchanged','gripper_active_action':None}
            report['arm_samples'].append(row)
            if stream is not None:
                stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
                stream.flush()
            sample += 1
            # Enforce at least one period after each command/reply. A slow
            # transport must never cause catch-up bursts of 0.15-degree steps.
            time.sleep(PERIOD_S)
        completed_normally = True
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        # A fault/timeout/interrupt never starts an automatic return trajectory.
        if args.live_output and dashboard is not None:
            if completed_normally and enabled_requested and guarded is not None:
                try:
                    report['arm_baseline_return'] = safe_return_to_baseline(
                        dashboard,guarded,baseline,
                        timeout_s=args.max_relative_deg/MAX_SPEED_DEG_S+5.0)
                except BaseException as exc:
                    report['arm_baseline_return'] = {'passed':False,'error':f'{type(exc).__name__}: {exc}'}
            else:
                report['arm_baseline_return'] = {'passed':False,'skipped':'session not completed normally'}
            for command, key in [('Stop()','finally_stop'),('DisableRobot()','finally_disable')]:
                try:
                    report[key+'_reply'] = dashboard.command(command)
                except BaseException as exc:
                    report[key+'_error'] = f'{type(exc).__name__}: {exc}'
            if feedback is not None:
                try:
                    # Raw reader for shutdown: an envelope fault must not prevent disable verification.
                    state = wait_enable(feedback,False)
                    require_disabled_preflight(state)
                    report['cr5_final'] = compact_state(state)
                    report['cr5_final_disable_verified'] = True
                except BaseException as exc:
                    report['cr5_final_disable_verified'] = False
                    report['cr5_final_disable_error'] = f'{type(exc).__name__}: {exc}'
        if opened:
            try:
                health = read_servo_health(protocol)
                require_uarm_health(health, config)
                report['uarm_shutdown_health'] = health
            except BaseException as exc:
                report['uarm_shutdown_health_error'] = f'{type(exc).__name__}: {exc}'
        # Each resource close is attempted even if a previous close raises.
        for name, resource, method in [('dashboard',dashboard,'close'),('feedback',feedback,'close'),
                                       ('serial',port if opened else None,'closePort')]:
            if resource is not None:
                try:
                    getattr(resource,method)()
                except BaseException as exc:
                    report.setdefault('resource_close_errors',{})[name] = f'{type(exc).__name__}: {exc}'
        report['duration_actual_s'] = time.monotonic()-started
        report['completed_at_utc'] = utc_now()

    rows = report['arm_samples']
    offsets = [r['arm_actual_relative_deg'][args.joint-1] for r in rows]
    observed_positive = max([0.0]+offsets)
    observed_negative = min([0.0]+offsets)
    report['observed_selected_range_deg'] = [observed_negative, observed_positive]
    report['requested_range_observed_at_least_one_direction'] = bool(
        args.live_output and max(abs(observed_negative),observed_positive) >= args.max_relative_deg-.5)
    report['both_directions_observed_to_requested_range'] = bool(
        observed_positive >= args.max_relative_deg-.5 and observed_negative <= -args.max_relative_deg+.5)
    clean = completed_normally and not any(report.get(key) for key in (
        'error','finally_stop_error','finally_disable_error','cr5_final_disable_error',
        'uarm_shutdown_health_error','resource_close_errors'))
    if args.live_output:
        clean = clean and report.get('cr5_final_disable_verified',False) and report.get('arm_baseline_return',{}).get('passed',False)
    report['session_checks_passed'] = bool(clean)
    report['passed'] = bool(clean and rows and (
        not args.live_output or report['requested_range_observed_at_least_one_direction']))
    if clean and args.live_output and not report['requested_range_observed_at_least_one_direction']:
        report['acceptance_note'] = 'Session ended safely, but the requested range was not actually exercised.'
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='192.168.201.1')
    parser.add_argument('--port',default='auto')
    parser.add_argument('--joint',type=int,choices=range(1,7),required=True)
    parser.add_argument('--max-relative-deg',type=int,choices=(10,20),default=10)
    parser.add_argument('--duration',type=float,default=20)
    parser.add_argument('--live-output',action='store_true')
    parser.add_argument('--enable-robot',action='store_true')
    parser.add_argument('--confirm',default='')
    parser.add_argument('--telemetry-log',type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('duration must be within (0,30] seconds')
    if args.live_output and (not args.enable_robot or args.confirm != LIVE_CONFIRM):
        parser.error(f'live output requires --enable-robot --confirm {LIVE_CONFIRM}')
    if not args.live_output and (args.enable_robot or args.confirm):
        parser.error('enable/confirmation requires --live-output')
    config = load_config(CONFIG)
    signs = config.get('joint_signs',[])
    if len(signs) != 6 or any(v not in (-1,1) for v in signs):
        raise ValueError('Expected six validated mapping signs')
    if args.live_output and not (config.get('power_validated') and config.get('mapping_validated')):
        raise RuntimeError('Existing power and direction validation required; no lock is changed')
    report = {'timestamp_utc':utc_now(),'operation':'single-joint expanded range trial',
              'live_output':args.live_output,'selected_joint':args.joint,
              'duration_requested_s':args.duration,'rate_hz':1/PERIOD_S,
              'arm_max_relative_deg':args.max_relative_deg,'arm_max_speed_deg_s':MAX_SPEED_DEG_S,
              'arm_mapping_signs':signs,'gripper_control_enabled':False,
              'arm_samples':[],'gripper_actions':[],'passed':False,
              'scope':'Only selected joint follows; the other five hold baseline. No gripper commands.',
              'range_acceptance':'At least one direction must reach requested amplitude within 0.5 deg; both directions reported separately.'}
    with telemetry_log(args.telemetry_log,report) as stream:
        code = run_session(args,config,report,stream)
        path = ROOT/'logs'/f'uarm_cr5_joint_range_{datetime.now():%Y%m%d_%H%M%S_%f}.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        report['log_file'] = str(path)
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        summary = {key:value for key,value in report.items() if key not in ('arm_samples','uarm_health','uarm_shutdown_health')}
        summary['arm_sample_count'] = len(report['arm_samples'])
        print(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
