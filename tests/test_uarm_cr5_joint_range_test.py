import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from contextlib import ExitStack

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from uarm_cr5_joint_range_test import SingleJointMapping, check_envelope, PERIOD_S, MAX_SPEED_DEG_S
import uarm_cr5_joint_range_test as program


class SingleJointRangeTests(unittest.TestCase):
    def test_only_selected_axis_moves_and_clamps(self):
        baseline = [12,-10,-30,20,40,-50]
        m = SingleJointMapping(baseline,2,1000,1,20)
        for _ in range(150):
            result = m.update(1500)
        self.assertEqual(result['target_deg'],[12,-10,-10,20,40,-50])
        self.assertEqual(result['output_deg'],result['target_deg'])

    def test_encoder_wrap_and_negative_direction(self):
        m = SingleJointMapping([0]*6,0,4094,1,10)
        self.assertAlmostEqual(m.update(3)['input_relative_deg'],3*360/4096)
        reverse = SingleJointMapping([0]*6,1,3,-1,10)
        self.assertAlmostEqual(reverse.update(4094)['input_relative_deg'],3*360/4096)

    def test_slew_and_reversal_are_limited(self):
        m = SingleJointMapping([0]*6,0,2000,1,20)
        previous = 0
        for tick in [2500]*100+[1500]*150:
            value = m.update(tick)['output_deg'][0]
            self.assertLessEqual(abs(value-previous),MAX_SPEED_DEG_S*PERIOD_S+1e-12)
            previous = value

    def test_two_tick_deadband(self):
        m = SingleJointMapping([0]*6,0,2000,1,10)
        for tick in [2001,2002,1999,1998]:
            self.assertEqual(m.update(tick)['target_deg'],[0]*6)

    def test_full_sweep_must_fit_guarded_limits(self):
        with self.assertRaises(RuntimeError):
            SingleJointMapping([0,0,150,0,0,0],2,1000,1,10)
        with self.assertRaises(ValueError):
            SingleJointMapping([math.nan]*6,0,1000,1,10)

    def test_actual_selected_and_held_envelopes(self):
        check_envelope([20.2,0,0,0,0,0],[0]*6,0,20)
        with self.assertRaises(RuntimeError):
            check_envelope([20.26,0,0,0,0,0],[0]*6,0,20)
        with self.assertRaises(RuntimeError):
            check_envelope([0,0.31,0,0,0,0],[0]*6,0,20)

    def run_mock_session(self, fail_read=False, fail_close=False):
        state = dict(robot_mode=4,enable_status=0,running_status=0,error_status=0,
                     q_actual_deg=[0.0]*6,timestamp_ms=1000)
        port = MagicMock()
        port.openPort.return_value = port.setBaudRate.return_value = True
        feedback = MagicMock()
        feedback.wait_first.return_value = state
        dashboard = MagicMock()
        dashboard.command.return_value = '0,{},Command();'
        if fail_close:
            dashboard.close.side_effect = RuntimeError('close failed')
        transport = [dict(comm_result=0,packet_error=0)]*7
        read = MagicMock(side_effect=[([1000]*7,transport),RuntimeError('runtime read failed')])
        args = SimpleNamespace(host='unused',port='unused',joint=1,max_relative_deg=20,
                               duration=1 if fail_read else 0,live_output=True)
        report = dict(arm_samples=[],gripper_actions=[],passed=False)
        with ExitStack() as stack:
            for name,value in [('resolve_uarm_port',lambda _: 'unused'),('PortHandler',lambda _:port),
                               ('sms_sts',lambda _:object()),('FeedbackStream',lambda *a,**k:feedback),
                               ('read_servo_health',lambda _:[]),('require_uarm_health',lambda *a:None),
                               ('uarm_health_warnings',lambda *a:[]),('Dashboard',lambda _:dashboard),
                               ('read_positions',read),
                               ('wait_enable',lambda _, enabled:dict(state,robot_mode=5 if enabled else 4,
                                                                   enable_status=int(enabled)))]:
                stack.enter_context(patch.object(program,name,value))
            return_mock = stack.enter_context(patch.object(program,'safe_return_to_baseline',return_value={'passed':True}))
            network = stack.enter_context(patch('socket.create_connection',side_effect=AssertionError('hardware forbidden')))
            code = program.run_session(args,{'joint_signs':[1]*6},report,None)
            network.assert_not_called()
        return report, dashboard, feedback, port, return_mock

    def test_fault_skips_return_but_stops_disables_and_closes_every_resource(self):
        report,dashboard,feedback,port,return_mock = self.run_mock_session(fail_read=True,fail_close=True)
        return_mock.assert_not_called()
        commands = [call.args[0] for call in dashboard.command.call_args_list]
        self.assertEqual(commands[-2:],['Stop()','DisableRobot()'])
        self.assertTrue(report['cr5_final_disable_verified'])
        self.assertFalse(report['passed'])
        feedback.close.assert_called_once()
        port.closePort.assert_called_once()

    def test_normal_return_has_time_for_twenty_degrees(self):
        report,dashboard,feedback,port,return_mock = self.run_mock_session()
        return_mock.assert_called_once()
        self.assertGreaterEqual(return_mock.call_args.kwargs['timeout_s'],20/3+5)
        self.assertTrue(report['cr5_final_disable_verified'])
        self.assertFalse(report['passed'])  # no sampled motion cannot pass range acceptance


if __name__ == '__main__':
    unittest.main()
