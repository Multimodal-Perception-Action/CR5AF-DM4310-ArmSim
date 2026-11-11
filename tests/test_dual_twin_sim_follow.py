"""Main-loop integration tests using only fake devices and an in-memory GUI.

No COM, TCP, controller, motor or desktop GUI is accessed. Real session logs are
not touched; each run writes to a TemporaryDirectory.
"""
from __future__ import annotations

from contextlib import ExitStack, redirect_stdout
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import uarm_cr5_dual_twin as twin


class Variable:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeModel:
    def __init__(self):
        limits = [[-359.8175, 359.8175]] * 6
        limits[2] = [-160.0271, 160.0271]
        self.jnt_range = np.radians(limits)
        self.geom_group = np.array([2])
        self.geom_rgba = np.zeros((1, 4))
        self.ngeom = 1

    def joint(self, name):
        return SimpleNamespace(id=int(name.removeprefix('a_joint')) - 1)


class FakeGui:
    def __init__(self, frames, actions, modes, close_error=False):
        self.frames = frames
        self.actions = actions
        self.modes = modes
        self.index = 0
        self.registration = SimpleNamespace(map_angles=lambda values: list(values))
        self.sim_mode = Variable('simulation')
        self.pause_text = Variable('暂停仿真')
        self.inspect_axis = Variable(4)
        self.inspection_text = Variable('')
        self.highlight_axis = Variable(True)
        self.sim_saved_text = Variable('')
        self.actual_joint_values = None
        self.virtual_joint_values = None
        self.rendered = []
        self.closed = False
        self.close_error = close_error

    def is_running(self):
        return self.index < len(self.frames)

    def consume_sim_actions(self):
        if self.index in self.modes:
            self.sim_mode.set(self.modes[self.index])
        return self.actions.get(self.index, [])

    def update(self, data, left_text, right_text):
        self.rendered.append({'joints':dict(data.joints), 'gripper':data.gripper,
                              'status':right_text})
        self.index += 1

    def set_joint_values(self, actual, virtual):
        self.actual_joint_values = list(actual)
        self.virtual_joint_values = list(virtual)

    def close(self):
        self.closed = True
        if self.close_error:
            raise RuntimeError('synthetic GUI close failure')


class DualTwinSimFollowIntegrationTests(unittest.TestCase):
    @staticmethod
    def pose(tick=1000, *, trigger=2966):
        return [tick] * 6 + [trigger]

    def run_session(self, frames, *, actions=None, modes=None, actual=None,
                    bad_read_index=None, close_error=False, sim_signs=None,
                    resume_records=None, resume_tail=''):
        actual = [0.0] * 6 if actual is None else list(actual)
        model = FakeModel()
        data = SimpleNamespace(joints={}, gripper=0.0)
        gui = FakeGui(frames, actions or {}, modes or {}, close_error=close_error)
        port = SimpleNamespace(ser=object(), openPort=Mock(return_value=True),
                               setBaudRate=Mock(return_value=True), closePort=Mock())
        feedback_row = {'q_actual_deg':actual[:], 'robot_mode':4,
                        'enable_status':0, 'error_status':0, 'timestamp_ms':123456}
        feedback = SimpleNamespace(wait_first=Mock(return_value=feedback_row),
                                   latest=Mock(return_value=feedback_row), close=Mock())
        dm = SimpleNamespace(start=Mock(), snapshot=Mock(return_value=(None,None)), close=Mock())
        sequence = [frames[0]] + frames
        read_index = -1

        def read_positions(_protocol):
            nonlocal read_index
            read_index += 1
            status = 1 if read_index == bad_read_index else 0
            return list(sequence[read_index]), [{'comm_result':0, 'packet_error':status} for _ in range(7)]

        def set_joint(_model, target, name, value):
            target.joints[name] = value

        def set_gripper(_model, target, _prefix, value):
            target.gripper = value

        error = None
        with tempfile.TemporaryDirectory(prefix='uarm_sim_integration_') as folder:
            # Isolate control files as well: a mistaken save destination must
            # fail the test without overwriting the user's real calibration.
            control_files = {}
            for constant in ('CALIBRATION','HEALTH_CONFIG'):
                before = getattr(twin,constant).read_bytes()
                temporary_control = Path(folder)/(constant.lower()+'.json')
                temporary_control.write_bytes(before)
                control_files[constant] = (temporary_control,before)
            simulation_config = Path(folder)/'simulation.json'
            simulation_config.write_text(json.dumps({
                'schema_version':1, 'purpose':'simulation_only',
                'joint_signs':[1,-1,1,1,1,1] if sim_signs is None else sim_signs,
            }),encoding='utf-8')
            arguments = ['twin','--gui','--duration','10','--rate','50']
            if resume_records is not None:
                resume_path = Path(folder)/'resume.jsonl'
                resume_path.write_text(''.join(json.dumps(row)+'\n' for row in resume_records)
                                       +resume_tail,encoding='utf-8')
                arguments.extend(['--resume-simulation',str(resume_path)])
            with ExitStack() as stack:
                replacements = {
                    'ROOT':Path(folder), 'build_scene':Mock(return_value=(model,data)),
                    'SIM_CONFIG':simulation_config,
                    'CALIBRATION':control_files['CALIBRATION'][0],
                    'HEALTH_CONFIG':control_files['HEALTH_CONFIG'][0],
                    'resolve_uarm_port':Mock(return_value='FAKE_COM_ONLY'),
                    'PortHandler':Mock(return_value=port), 'sms_sts':Mock(return_value=object()),
                    'read_servo_health':Mock(return_value=[]), 'require_uarm_health':Mock(),
                    'uarm_health_warnings':Mock(return_value=[]), 'read_positions':read_positions,
                    'FeedbackStream':Mock(return_value=feedback), 'DMStatusPoller':Mock(return_value=dm),
                    'TwinCanvas':Mock(return_value=gui), 'set_joint_rad':set_joint,
                    'set_tool_open_fraction':set_gripper,
                }
                for name, value in replacements.items():
                    stack.enter_context(patch.object(twin, name, value))
                stack.enter_context(patch.object(twin.mujoco,'mj_forward'))
                stack.enter_context(patch.object(twin.time,'sleep'))
                network = stack.enter_context(patch('socket.create_connection',
                    side_effect=AssertionError('Network access forbidden in offline test')))
                stack.enter_context(patch.object(sys,'argv',arguments))
                stack.enter_context(redirect_stdout(io.StringIO()))
                try:
                    twin.main()
                except Exception as exc:
                    error = exc
                network.assert_not_called()
            logs = list((Path(folder)/'logs').glob('*.jsonl'))
            rows = [json.loads(line) for line in logs[0].read_text(encoding='utf-8').splitlines()] if logs else []
            config_payload = json.loads(simulation_config.read_text(encoding='utf-8'))
            control_unchanged = {name:path.read_bytes()==before
                                 for name,(path,before) in control_files.items()}
        return SimpleNamespace(samples=[r for r in rows if r['kind']=='sample'], rows=rows,
                               gui=gui, port=port, feedback=feedback, dm=dm, error=error,
                               config_payload=config_payload, control_unchanged=control_unchanged)

    def assert_clean_closed(self, session):
        self.assertTrue(session.gui.closed)
        session.port.closePort.assert_called_once()
        session.feedback.close.assert_called_once()
        session.dm.close.assert_called_once()

    def test_startup_pair_and_input_follow_do_not_change_actual_feedback(self):
        session = self.run_session([self.pose(), self.pose(1128,trigger=2591), self.pose(1256)],
                                   actual=[10,20,0,0,0,0], modes={2:'actual'})
        self.assertIsNone(session.error)
        first, second, third = session.samples
        self.assertEqual(first['cr5_preview_joint_deg'], first['cr5_actual_joint_deg'])
        self.assertAlmostEqual(second['cr5_preview_joint_deg'][0],21.25)
        self.assertAlmostEqual(second['cr5_preview_joint_deg'][1],8.75)
        self.assertGreater(abs(second['simulation_delta_deg'][0]),5.0)
        self.assertEqual(second['simulation_gripper_open_fraction'],1.0)
        self.assertEqual(session.gui.rendered[1]['gripper'],1.0)
        self.assertEqual(second['cr5_render_source'],'simulation')
        self.assertEqual(third['cr5_render_source'],'actual_feedback')
        self.assertEqual(third['cr5_rendered_joint_deg'],[10,20,0,0,0,0])
        for row in session.samples:
            self.assertEqual(row['cr5_actual_joint_deg'],[10,20,0,0,0,0])
        self.assert_clean_closed(session)

    def test_candidate_pause_resume_and_actual_rebase(self):
        frames = [self.pose(1000+i*100,trigger=2591 if i==2 else 2966) for i in range(9)]
        session = self.run_session(frames,actions={1:['candidate'],3:['pause'],5:['pause'],7:['actual']})
        self.assertIsNone(session.error)
        samples = session.samples
        candidate = json.loads(twin.CALIBRATION.read_text(encoding='utf-8'))['robot']['safe_home_candidate']['joint_deg_canonical_equivalent']
        self.assertEqual(samples[1]['cr5_preview_joint_deg'],candidate)
        for index in (3,4,5):
            self.assertEqual(samples[index]['cr5_preview_joint_deg'],samples[2]['cr5_preview_joint_deg'])
        self.assertTrue(samples[3]['simulation_paused'])
        self.assertTrue(samples[4]['simulation_paused'])
        self.assertFalse(samples[5]['simulation_paused'])
        self.assertEqual(samples[3]['simulation_gripper_open_fraction'],1.0)
        self.assertEqual(samples[4]['simulation_gripper_open_fraction'],1.0)
        self.assertAlmostEqual(samples[6]['cr5_preview_joint_deg'][0]-samples[5]['cr5_preview_joint_deg'][0],100*360/4096)
        self.assertEqual(samples[7]['cr5_preview_joint_deg'],[0.0]*6)
        self.assertAlmostEqual(samples[8]['cr5_preview_joint_deg'][0],100*360/4096)
        reasons = [r['reference']['reason'] for r in session.rows if r['kind']=='simulation_reference']
        self.assertEqual(reasons,['startup_actual_mapped','candidate','resume_hold','actual'])
        self.assert_clean_closed(session)

    def test_model_limits_clip_simulated_joints_only(self):
        session = self.run_session([self.pose(),self.pose(1400)],actual=[355,0,155,0,0,0])
        self.assertIsNone(session.error)
        row = session.samples[-1]
        self.assertAlmostEqual(row['cr5_preview_joint_deg'][0],359.8175)
        self.assertAlmostEqual(row['cr5_preview_joint_deg'][2],160.0271)
        self.assertEqual(row['simulation_clipped_axes'],[1,3])
        self.assertEqual(row['cr5_actual_joint_deg'],[355,0,155,0,0,0])
        self.assert_clean_closed(session)

    def test_bad_input_stops_before_rendering_and_releases_resources(self):
        session = self.run_session([self.pose(),self.pose(1100)],bad_read_index=2)
        self.assertIsInstance(session.error,RuntimeError)
        self.assertIn('runtime read failed',str(session.error))
        self.assertEqual(len(session.samples),1)
        self.assert_clean_closed(session)

    def test_gui_cleanup_error_does_not_leak_device_resources(self):
        session = self.run_session([self.pose()],close_error=True)
        self.assertIsInstance(session.error,RuntimeError)
        self.assertIn('synthetic GUI close failure',str(session.error))
        self.assert_clean_closed(session)

    def test_virtual_direction_reversal_keeps_control_mapping_and_feedback_independent(self):
        frames = [self.pose(), self.pose(1128)]
        normal = self.run_session(frames,actual=[10,20,30,40,50,60])
        reversed_sim = self.run_session(frames,actual=[10,20,30,40,50,60],
                                        sim_signs=[-1,1,-1,1,1,1])
        self.assertIsNone(normal.error)
        self.assertIsNone(reversed_sim.error)
        before, after = normal.samples[-1], reversed_sim.samples[-1]
        self.assertEqual(after['simulation_delta_deg'],[-11.25,11.25,-11.25,11.25,11.25,11.25])
        self.assertEqual(after['cr5_preview_joint_deg'],[-1.25,31.25,18.75,51.25,61.25,71.25])
        self.assertEqual(after['cr5_actual_joint_deg'],before['cr5_actual_joint_deg'])
        self.assertEqual(after['mapped_relative_deg'],before['mapped_relative_deg'])
        self.assertEqual(after['uarm_display_angles_deg'],before['uarm_display_angles_deg'])
        self.assertEqual(after['simulation_reference']['signs'],[-1,1,-1,1,1,1])
        self.assertIn('虚拟 J4',reversed_sim.gui.inspection_text.get())
        self.assertIn('实机反馈映射 +40.0',reversed_sim.gui.inspection_text.get())
        self.assert_clean_closed(normal)
        self.assert_clean_closed(reversed_sim)

    def test_resume_preserves_last_virtual_pose_and_uses_new_signs_after_wrap(self):
        prior_target = [30,20,-10,40,50,60]
        records = [
            {'kind':'session'},
            {'kind':'sample', 'uarm_ticks':self.pose(4080),
             'cr5_preview_joint_deg':[5]*6},
            {'kind':'sample', 'uarm_ticks':self.pose(4090),
             'cr5_preview_joint_deg':prior_target},
        ]
        session = self.run_session([self.pose(4090),self.pose(10)],
            sim_signs=[-1,1,-1,1,1,1], resume_records=records,
            resume_tail='{"kind":"sample","uarm_ticks":')
        self.assertIsNone(session.error)
        first, second = session.samples
        self.assertEqual(first['cr5_preview_joint_deg'],prior_target)
        self.assertEqual(first['cr5_actual_joint_deg'],[0]*6)
        self.assertEqual(first['simulation_reference']['reason'],'resume_display')
        expected_delta = [sign*16*360/4096 for sign in [-1,1,-1,1,1,1]]
        self.assertEqual(second['simulation_delta_deg'],expected_delta)
        self.assertEqual(second['cr5_preview_joint_deg'],
                         [q+delta for q,delta in zip(prior_target,expected_delta)])
        self.assert_clean_closed(session)

    def test_gui_direction_toggle_preserves_pose_then_reverses_and_saves_only_simulation(self):
        frames = [self.pose(1000+index*128) for index in range(5)]
        session = self.run_session(frames,actual=[10,20,30,40,50,60],
                                  actions={2:[('sign',0,-1)],4:['save_signs']})
        self.assertIsNone(session.error)
        before, toggled, moved = session.samples[1:4]
        self.assertEqual(toggled['cr5_preview_joint_deg'],before['cr5_preview_joint_deg'])
        self.assertAlmostEqual(moved['cr5_preview_joint_deg'][0]-toggled['cr5_preview_joint_deg'][0],-11.25)
        self.assertEqual(toggled['simulation_reference']['reason'],'direction_J1')
        self.assertEqual(session.config_payload['purpose'],'simulation_only')
        self.assertEqual(session.config_payload['joint_signs'],[-1,-1,1,1,1,1])
        self.assertIn('仿真方向已保存',session.gui.sim_saved_text.get())
        self.assertEqual(session.control_unchanged,{'CALIBRATION':True,'HEALTH_CONFIG':True})
        for row in session.samples:
            self.assertEqual(row['cr5_actual_joint_deg'],[10,20,30,40,50,60])
        # Control preview sign remains positive for J1 despite the GUI-only flip.
        self.assertAlmostEqual(moved['mapped_relative_deg'][0]-toggled['mapped_relative_deg'][0],11.25)
        self.assert_clean_closed(session)


if __name__ == '__main__':
    unittest.main()
