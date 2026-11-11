"""Offline safety/lifecycle coverage; network access is forbidden in mock runs."""
from contextlib import ExitStack
import io
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import uarm_cr5_continuous_teleop as c


class ContinuousTests(unittest.TestCase):
    @staticmethod
    def gateway_event(event, result, flags, position=0.0, target=0.0,
                      motor_error=0):
        body = (b'\xD5\x4D\x47\x50' + bytes((event, result, motor_error, flags))
                + struct.pack('<ff', position, target))
        checksum = 0
        for value in body:
            checksum ^= value
        return body + bytes((checksum,))

    def mapping(self, baseline=None, selected=None, signs=None):
        return c.ContinuousMapping(baseline or [0]*6, [1000]*7,
                                   signs or [1]*6, selected or [1,2,3,4,5,6], [2]*6)

    def test_no_five_degree_session_limit(self):
        m=self.mapping()
        row=m.update([1300]*7)
        self.assertGreater(row['target_deg'][0],25)
        for value in row['output_deg']:self.assertAlmostEqual(value,.15)

    def test_gripper_middle_half_holds_last_state(self):
        config = dict(endpoints={
            'midpoint_ticks': 2778.5,
            'maximum_open': {'ticks': 2591},
            'fully_closed': {'ticks': 2966},
        })
        hysteresis = c.gripper_trigger_hysteresis_ticks(config)
        self.assertAlmostEqual(hysteresis, 93.75)
        self.assertEqual(c.desired_state(2700, 2778.5, hysteresis, 'close'),
                         'close')
        self.assertEqual(c.desired_state(2680, 2778.5, hysteresis, 'close'),
                         'open')
        self.assertEqual(c.desired_state(2850, 2778.5, hysteresis, 'open'),
                         'open')
        self.assertEqual(c.desired_state(2880, 2778.5, hysteresis, 'open'),
                         'close')

    def test_hardware_margin_and_antiwindup(self):
        m=self.mapping([354,0,0,0,0,0])
        row=m.update([1500]+[1000]*6)
        self.assertEqual(row['target_deg'][0],355)
        self.assertEqual(row['limit_clipped_axes'],[1])
        row=m.update([1490]+[1000]*6)
        self.assertLess(row['target_deg'][0],355)
        self.assertGreater(row['target_deg'][0],354)

    def test_negative_limit_reverses_immediately(self):
        m=self.mapping([0,0,-154,0,0,0])
        row=m.update([1000,1000,500,1000,1000,1000,1000])
        self.assertEqual(row['target_deg'][2],-155)
        row=m.update([1000,1000,510,1000,1000,1000,1000])
        self.assertGreater(row['target_deg'][2],-155)

    def test_only_selected_axis_and_sign(self):
        m=self.mapping(selected=[4],signs=[1,1,1,-1,1,1])
        row=m.update([1200]*7)
        self.assertEqual(row['target_deg'][:3],[0]*3)
        self.assertLess(row['target_deg'][3],0)
        self.assertEqual(row['target_deg'][4:],[0]*2)

    def test_source_permutation_scale_and_operator_speed(self):
        mapping=c.ContinuousMapping(
            [0]*6,[1000]*7,[1]*6,[1,2,3,4,5,6],[0]*6,
            sources=[2,1,3,4,5,6],scales=[.5,2,1,1,1,1],max_speed_deg_s=12,
        )
        row=mapping.update([1100,1200,1000,1000,1000,1000,1000])
        self.assertAlmostEqual(row['target_deg'][0],100*360/4096)
        self.assertAlmostEqual(row['target_deg'][1],200*360/4096)
        for value in row['output_deg']:
            self.assertLessEqual(abs(value),.6000000001)

    def test_joint_offset_is_slewed_and_reaches_full_value(self):
        mapping=c.ContinuousMapping(
            [0]*6,[1000]*7,[1]*6,[2],[0]*6,
            max_speed_deg_s=12,offsets_deg=[0,180,0,0,0,0],
        )
        row=mapping.update([1000]*7)
        self.assertAlmostEqual(row['target_deg'][1],.6)
        self.assertAlmostEqual(row['output_deg'][1],.6)
        self.assertEqual(row['joint_offsets_deg'][1],180)
        self.assertAlmostEqual(row['joint_offset_progress_deg'][1],.6)
        for _ in range(299):
            row=mapping.update([1000]*7)
        self.assertAlmostEqual(row['target_deg'][1],180)
        self.assertAlmostEqual(row['output_deg'][1],180)
        self.assertAlmostEqual(row['joint_offset_progress_deg'][1],180)

    def test_follow_lag_drops_backlog_and_reverses_without_long_chase(self):
        mapping=c.ContinuousMapping(
            [0]*6,[1000]*7,[1]*6,[1],[0]*6,
            max_speed_deg_s=12,max_follow_lag_deg=20,
        )
        row=mapping.update([1500,1000,1000,1000,1000,1000,1000])
        self.assertEqual(row['target_deg'][0],20)
        self.assertEqual(row['follow_lag_clipped_axes'],[1])
        self.assertAlmostEqual(row['output_deg'][0],.6)
        # Returning the leader does not leave the discarded ~156-degree target
        # queued; the effective target immediately heads back toward zero.
        row=mapping.update([1000]*7)
        self.assertLess(row['target_deg'][0],20)
        self.assertLess(row['output_deg'][0],.6)

    def test_wrap_and_encoder_jump(self):
        m=c.ContinuousMapping([0]*6,[4094]*7,[1]*6,[1],[2]*6)
        self.assertAlmostEqual(m.update([3]*7)['target_deg'][0],3*360/4096)
        with self.assertRaises(RuntimeError):m.update([800]*7)

    def test_stationary_input_has_no_drift_and_slew_is_bounded(self):
        m=self.mapping();previous=[0]*6
        for ticks in ([1300]*7,)*200+([700]*7,):
            # Use a physically plausible intermediate encoder step on reversal.
            if ticks[0]==700:m.update([1000]*7);previous=m.output[:]
            row=m.update(ticks)
            for a,b in zip(previous,row['output_deg']):self.assertLessEqual(abs(a-b),.15000000001)
            previous=row['output_deg']
        self.assertEqual(m.accumulated,[-300]*6)

    def test_new_but_stale_heartbeat_is_rejected(self):
        path=MagicMock()
        path.stat.return_value=SimpleNamespace(st_mtime=100,st_mtime_ns=1000,st_size=5)
        with patch.object(c,'Path',return_value=path),patch.object(c.time,'time',return_value=100.2),patch.object(c.time,'monotonic',return_value=50):
            hb=c.Heartbeat('unused')
        path.stat.return_value=SimpleNamespace(st_mtime=101,st_mtime_ns=2000,st_size=5)
        with patch.object(c.time,'time',return_value=104),patch.object(c.time,'monotonic',return_value=54):
            with self.assertRaisesRegex(RuntimeError,'wall-clock'):hb.check()

    def test_frozen_controller_clock_and_backwards_clock(self):
        f=MagicMock();state=dict(timestamp_ms=10,q_actual_deg=[0]*6,error_status=0)
        f.latest.return_value=state
        with patch.object(c.time,'monotonic',return_value=1):
            g=c.FeedbackGuard(f);g.latest()
        with patch.object(c.time,'monotonic',return_value=1.3):
            with self.assertRaisesRegex(RuntimeError,'stopped advancing'):g.latest()
        f.latest.return_value=dict(state,timestamp_ms=9)
        with self.assertRaisesRegex(RuntimeError,'backwards'):g.latest()

    def test_cancel_during_identity_does_not_send_action(self):
        sock=MagicMock();stop=threading.Event();result={}
        def identity(_):
            stop.set();return dict(event=1,result=1,flags=16)
        with patch.object(c.socket,'create_connection',return_value=sock),patch.object(c,'receive_packet',return_value=b'ignored'),patch.object(c,'decode_gateway_event',side_effect=identity):
            c.cancellable_gripper_worker('unused','open',stop,result)
        self.assertTrue(result['cancelled_before_action'])
        sock.sendall.assert_called_once_with(c.PACKETS['identity'])
        sock.close.assert_called_once()

    def test_unsolicited_gateway_abort_is_reported_without_host_timeout(self):
        sock = MagicMock()
        sock.recv.side_effect = [
            self.gateway_event(0x01, 0x01, 0x10, position=3.7),
            self.gateway_event(0x10, 0x05, 0x02, position=-6.38, target=-6.38),
            self.gateway_event(0x12, 0x02, 0x01, position=-1.1, target=-0.98),
        ]
        result = {}
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker(
                'unused', 'open', threading.Event(), result
            )
        self.assertFalse(result['passed'])
        self.assertIn('Gateway aborted active gripper session', result['error'])
        self.assertNotIn('TimeoutError', result['error'])
        self.assertEqual(result['gateway_abort_event']['event'], c.EVENTS['stop'])
        sock.close.assert_called_once()

    def test_latest_gripper_target_supersedes_inflight_transition(self):
        sock = MagicMock()
        stop = threading.Event()
        targets = c.GripperTargetMailbox('open')
        result = {}
        packets = [
            self.gateway_event(0x01, 0x01, 0x10, position=3.8),
            self.gateway_event(0x10, 0x05, 0x02, position=-6.38, target=-6.38),
        ]
        calls = 0

        def recv(_size):
            nonlocal calls
            calls += 1
            if packets:
                return packets.pop(0)
            if calls == 3:
                targets.request('close')
                raise c.socket.timeout()
            if calls == 4:
                targets.request('open')
                raise c.socket.timeout()
            if calls == 5:
                return self.gateway_event(
                    0x10, 0x05, 0x02, position=-6.38, target=-6.38
                )
            if calls == 6:
                stop.set()
                raise c.socket.timeout()
            return self.gateway_event(
                0x12, 0x01, 0x01, position=-6.38, target=-6.38
            )

        sock.recv.side_effect = recv
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'open', stop, result, targets)
        sent = [call.args[0] for call in sock.sendall.call_args_list]
        self.assertIn(c.PACKETS['close'], sent)
        self.assertGreater(sent.index(c.PACKETS['open'], 2), sent.index(c.PACKETS['close']))
        self.assertTrue(result['passed'])
        self.assertEqual(result['current_action'], 'open')
        self.assertFalse(result['holding_active'])

    def test_contact_partial_release_reclamp_stays_one_enabled_session(self):
        sock, stop = MagicMock(), threading.Event()
        targets = c.GripperTargetMailbox('close')
        result = {}
        calls = 0

        def recv(_size):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.gateway_event(1, 1, 16, position=4.5)
            if calls == 2:
                return self.gateway_event(0x11, 4, 2, position=4.08, target=4.12)
            if calls == 3:
                targets.request('open')
                raise c.socket.timeout()
            if calls == 4:
                # Reverse during release, before an OPEN completion exists.
                targets.request('close')
                raise c.socket.timeout()
            if calls == 5:
                return self.gateway_event(0x11, 4, 2, position=4.07, target=4.11)
            if calls == 6:
                stop.set()
                raise c.socket.timeout()
            return self.gateway_event(0x12, 1, 1, position=4.07)

        sock.recv.side_effect = recv
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'close', stop, result, targets)
        actions = [call.args[0] for call in sock.sendall.call_args_list
                   if call.args[0] in (c.PACKETS['open'], c.PACKETS['close'], c.PACKETS['stop'])]
        self.assertEqual(actions, [c.PACKETS[x] for x in ('close', 'open', 'close', 'stop')])
        self.assertTrue(result['passed'])
        self.assertTrue(result['contact_detected'])
        self.assertTrue(result['stopped_by_operator'])
        self.assertTrue(stop.is_set())
        sock.close.assert_called_once()

    def test_startup_reversal_uses_capability_and_never_becomes_stop(self):
        for version in (4.4, 4.5):
            with self.subTest(gateway=version):
                sock, stop = MagicMock(), threading.Event()
                targets = c.GripperTargetMailbox('close')
                result = {}
                calls = 0

                def recv(_size):
                    nonlocal calls
                    calls += 1
                    if calls == 1:
                        return self.gateway_event(1, 1, 16, position=version)
                    if calls == 2:
                        targets.request('open')
                        raise c.socket.timeout()
                    if calls == 3:
                        sent = [call.args[0] for call in sock.sendall.call_args_list]
                        self.assertIn(c.PACKETS['hold_heartbeat'], sent)
                        self.assertNotIn(c.PACKETS['stop'], sent)
                        self.assertEqual(c.PACKETS['open'] in sent, version >= 4.5)
                        event = 0x10 if version >= 4.5 else 0x11
                        return self.gateway_event(event, 5, 2)
                    if calls == 4:
                        self.assertIn(c.PACKETS['open'], [x.args[0] for x in sock.sendall.call_args_list])
                        return self.gateway_event(0x10, 5, 2)
                    if calls == 5:
                        stop.set()
                        raise c.socket.timeout()
                    return self.gateway_event(0x12, 1, 1)

                sock.recv.side_effect = recv
                with patch.object(c.socket, 'create_connection', return_value=sock):
                    c.cancellable_gripper_worker('unused', 'close', stop, result, targets)
                self.assertTrue(result['passed'], result)
                self.assertGreater(result['heartbeat_count'], 0)
                self.assertIn('received_at_utc', result['events'][0])
                sock.close.assert_called_once()

    def test_can_recovery_diagnostic_during_release_is_not_an_action_or_stop(self):
        sock, stop = MagicMock(), threading.Event()
        targets = c.GripperTargetMailbox('close')
        result = {}
        calls = 0

        def recv(_size):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.gateway_event(1, 1, 16, position=4.6)
            if calls == 2:
                return self.gateway_event(0x11, 4, 2, position=4.08, target=4.12)
            if calls == 3:
                targets.request('open')
                raise c.socket.timeout()
            if calls == 4:
                # Diagnostic payload slots are elapsed milliseconds and retry
                # count, NOT a gripper pose or completed target.
                return self.gateway_event(0x20, 1, 0, position=130.0, target=2.0)
            if calls == 5:
                self.assertEqual(result['current_action'], 'close')
                self.assertTrue(result['transition_pending'])
                self.assertNotIn(c.PACKETS['stop'], [x.args[0] for x in sock.sendall.call_args_list])
                return self.gateway_event(0x10, 5, 2, position=-.09, target=-.098)
            if calls == 6:
                self.assertEqual(result['current_action'], 'open')
                self.assertFalse(result['transition_pending'])
                stop.set()
                raise c.socket.timeout()
            return self.gateway_event(0x12, 1, 1, position=-.09)

        sock.recv.side_effect = recv
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'close', stop, result, targets)
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['stopped_by_operator'])
        self.assertNotIn('fallback_stop_sent', result)
        actions = [x.args[0] for x in sock.sendall.call_args_list
                   if x.args[0] in (c.PACKETS['open'], c.PACKETS['close'], c.PACKETS['stop'])]
        self.assertEqual(actions, [c.PACKETS[x] for x in ('close', 'open', 'stop')])
        diagnostics = [event for event in result['events'] if event['event'] == 0x20]
        self.assertEqual(len(diagnostics), 1)
        diagnostic = diagnostics[0]
        self.assertEqual(diagnostic['diagnostic'], 'can_exchange')
        self.assertTrue(diagnostic['recovered'])
        self.assertEqual(diagnostic['attempt_count'], 2)
        self.assertEqual(diagnostic['elapsed_ms'], 130.0)
        self.assertNotIn('position_rad', diagnostic)
        self.assertNotIn('target_rad', diagnostic)
        self.assertIn('received_at_utc', diagnostic)
        sock.close.assert_called_once()

    def test_stop_during_reversal_send_cannot_be_cleared_or_overridden(self):
        sock, stop = MagicMock(), threading.Event()
        targets = c.GripperTargetMailbox('close')
        result = {}
        calls = 0

        def send(packet):
            if packet == c.PACKETS['open']:
                # Simulate STOP racing a blocked reversal send and a newer
                # target arriving afterward. STOP must remain terminal.
                stop.set()
                targets.request('close')

        def recv(_size):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.gateway_event(1, 1, 16, position=4.5)
            if calls == 2:
                return self.gateway_event(0x11, 4, 2)
            if calls == 3:
                targets.request('open')
                raise c.socket.timeout()
            if calls == 4:
                raise c.socket.timeout()
            return self.gateway_event(0x12, 1, 1)

        sock.sendall.side_effect = send
        sock.recv.side_effect = recv
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'close', stop, result, targets)
        actions = [call.args[0] for call in sock.sendall.call_args_list
                   if call.args[0] in (c.PACKETS['open'], c.PACKETS['close'], c.PACKETS['stop'])]
        self.assertEqual(actions, [c.PACKETS[x] for x in ('close', 'open', 'stop')])
        self.assertTrue(result['passed'], result)
        self.assertTrue(stop.is_set())
        self.assertEqual(sock.sendall.call_args_list[-1].args[0], c.PACKETS['stop'])

    def test_stop_wins_when_target_and_stop_are_pending_together(self):
        sock, stop = MagicMock(), threading.Event()
        targets = c.GripperTargetMailbox('close')
        result = {}
        calls = 0

        def recv(_size):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.gateway_event(1, 1, 16, position=4.5)
            if calls == 2:
                targets.request('open')
                stop.set()
                raise c.socket.timeout()
            return self.gateway_event(0x12, 1, 1)

        sock.recv.side_effect = recv
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'close', stop, result, targets)
        sent = [call.args[0] for call in sock.sendall.call_args_list]
        self.assertNotIn(c.PACKETS['open'], sent)
        self.assertEqual(sent[-1], c.PACKETS['stop'])
        self.assertTrue(result['passed'])

    def test_superseded_pending_target_does_not_replay_old_reverse(self):
        sock, stop = MagicMock(), threading.Event()
        targets = c.GripperTargetMailbox('close')
        result = {}
        calls = 0

        def recv(_size):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.gateway_event(1, 1, 16, position=4.5)
            if calls == 2:
                targets.request('open')
                targets.request('close')
                return self.gateway_event(0x11, 4, 2)
            if calls == 3:
                stop.set()
                raise c.socket.timeout()
            return self.gateway_event(0x12, 1, 1)

        sock.recv.side_effect = recv
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'close', stop, result, targets)
        self.assertNotIn(c.PACKETS['open'], [call.args[0] for call in sock.sendall.call_args_list])
        self.assertTrue(result['passed'])

    def test_heartbeat_covers_startup_and_hold_without_feedback_events(self):
        sock, stop = MagicMock(), threading.Event()
        result = {}
        now = [0.0]
        heartbeat_times = []
        calls = 0

        def send(packet):
            if packet == c.PACKETS['hold_heartbeat']:
                heartbeat_times.append(now[0])

        def recv(_size):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.gateway_event(1, 1, 16, position=4.5)
            now[0] += .05
            if calls == 15:
                return self.gateway_event(0x11, 4, 2, position=4.08, target=4.12)
            if calls == 40:
                stop.set()
            if calls == 41:
                return self.gateway_event(0x12, 1, 1)
            raise c.socket.timeout()

        sock.recv.side_effect = recv
        sock.sendall.side_effect = send
        with patch.object(c.socket, 'create_connection', return_value=sock), patch.object(c.time, 'monotonic', side_effect=lambda: now[0]):
            c.cancellable_gripper_worker('unused', 'close', stop, result)
        self.assertTrue(result['passed'], result)
        self.assertEqual(heartbeat_times[0], 0.0)
        self.assertGreater(len([t for t in heartbeat_times if t < .7]), 2)
        self.assertGreater(len([t for t in heartbeat_times if t > .7]), 3)
        self.assertLessEqual(result['max_heartbeat_gap_s'], .251)
        self.assertEqual(result['heartbeat_count'], len(heartbeat_times))

    def test_partial_action_send_failure_still_attempts_stop_and_closes(self):
        sock, stop = MagicMock(), threading.Event()
        result = {}
        sock.recv.return_value = self.gateway_event(1, 1, 16, position=4.5)

        def send(packet):
            if packet == c.PACKETS['open']:
                raise c.socket.timeout('partial delivery unknown')

        sock.sendall.side_effect = send
        with patch.object(c.socket, 'create_connection', return_value=sock):
            c.cancellable_gripper_worker('unused', 'open', stop, result)
        self.assertFalse(result['passed'])
        self.assertTrue(result['action_attempted'])
        self.assertTrue(result['fallback_stop_sent'])
        self.assertEqual(sock.sendall.call_args_list[-1].args[0], c.PACKETS['stop'])
        sock.close.assert_called_once()

    def test_live_requires_new_confirmation_and_gui_watchdog(self):
        with self.assertRaises(SystemExit):
            c.parse_args(['--telemetry-log','unused','--live-output','--enable-robot','--confirm',c.LIVE_CONFIRM])
        args=c.parse_args(['--telemetry-log','unused'])
        self.assertFalse(args.live_output)

    def run_mock(self, fault=False, close_fault=False, cancel_before_enable=False, cancel_after_servo=False,
                 initial_enabled=False, readonly=False, enable_timeout=False):
        state=dict(robot_mode=4,enable_status=0,running_status=0,error_status=0,q_actual_deg=[0]*6,timestamp_ms=1000)
        enabled=dict(state,robot_mode=5,enable_status=1)
        port=MagicMock();port.openPort.return_value=port.setBaudRate.return_value=True
        feedback=MagicMock();feedback.wait_first.return_value=state
        feedback.latest.side_effect=[state,enabled,enabled]
        dashboard=MagicMock();dashboard.command.return_value='0,{},OK();'
        cleanup_dashboard=MagicMock();cleanup_dashboard.command.return_value='0,{},OK();'
        if enable_timeout:
            dashboard.command.side_effect=lambda command: (
                (_ for _ in ()).throw(TimeoutError('late EnableRobot reply'))
                if command == 'EnableRobot()' else
                ('0,{4},RobotMode();' if command == 'RobotMode()' else '0,{},OK();')
            )
        if close_fault:dashboard.close.side_effect=RuntimeError('close failed')
        transport=[dict(comm_result=0,packet_error=0)]*7
        read=MagicMock(side_effect=[([1000]*7,transport),([1000]*7,transport),([1100]*7,transport),RuntimeError('read failed')])
        stop=MagicMock()
        # startup, after-DM, before-enable, after-enable pairing, first loop.
        stop.exists.side_effect=[False,False,cancel_before_enable,False,not fault]
        if initial_enabled:
            feedback.wait_first.return_value=enabled
            feedback.latest.side_effect=[enabled]*5
            stop.exists.side_effect=[False]*6+[True]
            modes=iter([5,4])
            dashboard.command.side_effect=lambda command: ('0,{'+str(next(modes))+'},RobotMode();') if command=='RobotMode()' else '0,{},OK();'
        if readonly:
            stop.exists.side_effect=[False,False,True]
        if cancel_after_servo:
            stop.exists.side_effect=None
            stop.exists.return_value=False
            read.side_effect=[([1000]*7,transport),([1000]*7,transport),([1100]*7,transport)]+[([1100]*6+[3000],transport)]*4
            feedback.latest.side_effect=[state]+[dict(enabled,timestamp_ms=1000+i) for i in range(10)]
            sends=[0]
            def command(value):
                if value.startswith('ServoJ('):
                    sends[0]+=1
                    if sends[0]==4:stop.exists.return_value=True
                return '0,{},OK();'
            dashboard.command.side_effect=command
        args=SimpleNamespace(host='unused',port='unused',live_output=not readonly,heartbeat_file=None,arm_only=False,
                             stop_file=stop,joint_signs=[1]*6,joints=[1,2,3,4,5,6],
                             joint_sources=[1,2,3,4,5,6],joint_scales=[1.0]*6,
                             max_speed_deg_s=12.0,max_follow_lag_deg=20.0)
        report={};stream=io.StringIO()
        with ExitStack() as stack:
            dashboard_factory=(MagicMock(side_effect=[dashboard,cleanup_dashboard])
                               if enable_timeout else MagicMock(return_value=dashboard))
            replacements={'resolve_uarm_port':lambda _: 'unused','PortHandler':lambda _:port,
                          'sms_sts':lambda _:object(),'FeedbackStream':lambda *a,**k:feedback,
                          'read_servo_health':lambda _:[],'require_uarm_health':lambda *a:None,
                          'uarm_health_warnings':lambda *a:[], 'read_positions':read,
                          'read_dm_preflight':lambda *a:dict(error_code=0),
                          'Dashboard':dashboard_factory,'wait_enabled_cancellable':lambda *a:enabled,
                          'wait_enable':lambda *a:state}
            for name,value in replacements.items():stack.enter_context(patch.object(c,name,value))
            network=stack.enter_context(patch.object(c.socket,'create_connection',side_effect=AssertionError('network forbidden')))
            stop_send=stack.enter_context(patch.object(c,'send_best_effort_stop'))
            worker=stack.enter_context(patch.object(c,'cancellable_gripper_worker'))
            c.run_session(args,{},dict(endpoints=dict(midpoint_ticks=2778.5)),{},report,stream)
            network.assert_not_called();worker.assert_not_called();stop_send.assert_not_called()
            if readonly:dashboard_factory.assert_not_called()
            elif enable_timeout: self.assertEqual(dashboard_factory.call_count,2)
            else:dashboard_factory.assert_called_once()
        if enable_timeout:
            report['_test_cleanup_commands']=[call.args[0] for call in cleanup_dashboard.command.call_args_list]
        return report,dashboard,feedback,port

    def test_normal_stop_never_returns_or_triggers_initial_gripper(self):
        report,dashboard,feedback,port=self.run_mock()
        self.assertTrue(report['passed'])
        self.assertTrue(report['cr5_final_disable_verified'])
        self.assertFalse(report['automatic_return'])
        self.assertEqual(report['uarm_baseline_ticks'],[1100]*7)
        self.assertEqual([call.args[0] for call in dashboard.command.call_args_list],
                         ['RobotMode()','EnableRobot()','Stop()','DisableRobot()'])
        feedback.close.assert_called_once();port.closePort.assert_called_once()

    def test_fault_stops_disables_and_close_failure_does_not_leak_others(self):
        report,dashboard,feedback,port=self.run_mock(fault=True,close_fault=True)
        self.assertFalse(report['passed'])
        self.assertTrue(report['enable_requested'])
        self.assertTrue(report['cr5_final_disable_verified'])
        self.assertEqual([call.args[0] for call in dashboard.command.call_args_list][-2:],['Stop()','DisableRobot()'])
        feedback.close.assert_called_once();port.closePort.assert_called_once()

    def test_cancel_after_dashboard_query_does_not_enable(self):
        report,dashboard,feedback,port=self.run_mock(cancel_before_enable=True)
        self.assertFalse(report['enable_requested'])
        self.assertEqual([call.args[0] for call in dashboard.command.call_args_list],['RobotMode()'])
        feedback.close.assert_called_once();port.closePort.assert_called_once()

    def test_enable_timeout_discards_stream_and_cleans_up_on_fresh_dashboard(self):
        report,dashboard,feedback,port=self.run_mock(enable_timeout=True)
        self.assertFalse(report['passed'])
        self.assertIn('late EnableRobot reply',report['error'])
        self.assertTrue(report['finally_stop_used_fresh_dashboard'])
        self.assertEqual(report['_test_cleanup_commands'],['Stop()','DisableRobot()'])
        self.assertEqual([call.args[0] for call in dashboard.command.call_args_list],
                         ['RobotMode()','EnableRobot()'])
        self.assertTrue(report['cr5_final_disable_verified'])

    def test_cancel_during_servoj_reply_prevents_new_gripper_action(self):
        report,dashboard,feedback,port=self.run_mock(cancel_after_servo=True)
        self.assertEqual(report['gripper_action_count'],0)
        self.assertEqual(report['arm_sample_count'],3)
        self.assertEqual(report['stop_reason'], 'operator_stop')
        self.assertTrue(report['cr5_final_disable_verified'])
        commands=[call.args[0] for call in dashboard.command.call_args_list]
        self.assertEqual(sum(x.startswith('ServoJ(') for x in commands),4)
        self.assertEqual(commands[-2:],['Stop()','DisableRobot()'])

    def test_already_enabled_live_normalizes_with_one_dashboard(self):
        report,dashboard,feedback,port=self.run_mock(initial_enabled=True)
        self.assertTrue(report['passed'])
        self.assertEqual(report['initial_normalization']['from_state']['robot_mode'],5)
        self.assertEqual(report['initial_normalization']['disabled_state']['robot_mode'],4)
        self.assertEqual([call.args[0] for call in dashboard.command.call_args_list],
            ['RobotMode()','DisableRobot()','RobotMode()','EnableRobot()','Stop()','DisableRobot()'])
        dashboard.close.assert_called_once()

    def test_already_enabled_readonly_never_opens_dashboard(self):
        report,dashboard,feedback,port=self.run_mock(initial_enabled=True,readonly=True)
        self.assertTrue(report['passed'])
        self.assertFalse(report['enable_requested'])
        self.assertNotIn('initial_normalization',report)
        dashboard.command.assert_not_called()
        feedback.close.assert_called_once();port.closePort.assert_called_once()

    def test_drag_moving_and_fault_states_cannot_be_adopted(self):
        state=dict(robot_mode=5,enable_status=1,running_status=0,error_status=0,q_actual_deg=[0]*6)
        for change in [dict(robot_mode=6),dict(robot_mode=7),dict(running_status=1),dict(error_status=1),dict(enable_status=0)]:
            with self.assertRaises(RuntimeError):c.require_idle_preflight(dict(state,**change))


if __name__=='__main__':unittest.main()
