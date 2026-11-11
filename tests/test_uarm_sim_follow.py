"""Offline tests of wraparound, rebasing, and simulation-only clipping."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from uarm_sim_follow import DEGREES_PER_TICK, SimFollowMapping


class SimFollowMappingTests(unittest.TestCase):
    def mapping(self, signs=None, limits=None):
        return SimFollowMapping(signs or [1] * 6, limits or [[-360, 360] for _ in range(6)])

    def test_requires_anchor(self):
        mapping = self.mapping()
        self.assertFalse(mapping.as_dict()['initialized'])
        self.assertEqual(mapping.as_dict()['version'], 0)
        with self.assertRaises(RuntimeError):
            mapping.update([0] * 6)

    def test_wrap_forward_and_backward(self):
        mapping = self.mapping()
        mapping.rebase([4095] * 6, [0] * 6, 'initial')
        self.assertEqual(mapping.update([0] * 6)['target_deg'], [DEGREES_PER_TICK] * 6)
        self.assertEqual(mapping.update([4095] * 6)['target_deg'], [0] * 6)
        self.assertEqual(mapping.update([4094] * 6)['target_deg'], [-DEGREES_PER_TICK] * 6)

    def test_signs_and_no_small_motion_clamp(self):
        mapping = self.mapping([1, -1, 1, -1, 1, -1])
        mapping.rebase([1000] * 6, [0] * 6, 'initial')
        result = mapping.update([2024] * 6)
        self.assertEqual(result['delta_deg'], [90, -90, 90, -90, 90, -90])
        self.assertEqual(result['target_deg'], result['unclamped_target_deg'])
        self.assertEqual(result['clipped_axes'], [])

    def test_model_limits_and_reversal(self):
        mapping = self.mapping([1, -1, 1, 1, 1, 1], [[-20, 30]] * 6)
        mapping.rebase([1000] * 6, [0] * 6, 'initial')
        result = mapping.update([1512, 1512, 1000, 1000, 1000, 1000])
        self.assertEqual(result['target_deg'], [30, -20, 0, 0, 0, 0])
        self.assertEqual(result['unclamped_target_deg'], [45, -45, 0, 0, 0, 0])
        self.assertEqual(result['clipped_axes'], [1, 2])
        result = mapping.update([1100, 1100, 1000, 1000, 1000, 1000])
        self.assertEqual(result['target_deg'][:2], [100 * DEGREES_PER_TICK, -100 * DEGREES_PER_TICK])

    def test_rebase_resume_does_not_jump(self):
        mapping = self.mapping()
        mapping.rebase([1000] * 6, [10, 20, 30, 40, 50, 60], 'initial')
        held = mapping.update([1100] * 6)['target_deg']
        mapping.rebase([1800] * 6, held, 'resume')
        self.assertEqual(mapping.update([1800] * 6)['target_deg'], held)
        self.assertEqual(mapping.as_dict()['version'], 2)
        self.assertEqual(mapping.as_dict()['reason'], 'resume')

    def test_accumulated_travel_can_exceed_half_turn(self):
        mapping = self.mapping(limits=[[-1000, 1000]] * 6)
        mapping.rebase([0] * 6, [0] * 6, 'initial')
        for tick in [1024, 2048, 3072, 0, 1024]:
            result = mapping.update([tick] * 6)
        self.assertEqual(result['target_deg'], [450] * 6)

    def test_invalid_rebase_keeps_previous_anchor(self):
        mapping = self.mapping()
        mapping.rebase([1000] * 6, [0] * 6, 'initial')
        before = mapping.as_dict()
        bad_anchors = [[361] * 6, [float('nan')] * 6, [0] * 5]
        for anchor in bad_anchors:
            with self.assertRaises(ValueError):
                mapping.rebase([2000] * 6, anchor, 'invalid')
            self.assertEqual(mapping.as_dict(), before)
        self.assertEqual(mapping.update([1000] * 6)['target_deg'], [0] * 6)

    def test_bad_ticks_do_not_advance_previous_sample(self):
        mapping = self.mapping()
        mapping.rebase([1000] * 6, [0] * 6, 'initial')
        for bad in [[0] * 5, [-1] * 6, [4096] * 6, [0.5] * 6,
                    [float('inf')] * 6, [True] * 6, ['1000'] * 6]:
            with self.assertRaises(ValueError):
                mapping.update(bad)
        self.assertEqual(mapping.update([1001] * 6)['target_deg'], [DEGREES_PER_TICK] * 6)

    def test_bad_constructor_inputs(self):
        for signs in [[0] * 6, [1] * 5, [float('nan')] * 6]:
            with self.assertRaises(ValueError):
                self.mapping(signs)
        for limits in [[[0, 0]] * 6, [[0, float('inf')]] * 6,
                       [[0, 1]] * 5, [['0', '1']] * 6]:
            with self.assertRaises(ValueError):
                self.mapping(limits=limits)

    def test_inputs_and_snapshots_cannot_mutate_mapping(self):
        signs = [1] * 6
        limits = [[-360, 360] for _ in range(6)]
        leader = [1000] * 6
        robot = [0] * 6
        mapping = SimFollowMapping(signs, limits)
        mapping.rebase(leader, robot, 'initial')
        signs[0] = -1; limits[0][1] = 0; leader[0] = 4000; robot[0] = 200
        snapshot = mapping.as_dict()
        snapshot['leader_anchor_ticks'][0] = 0
        snapshot['robot_anchor_deg'][0] = 300
        snapshot['limits_deg'][0][1] = 0
        self.assertEqual(mapping.update([1100] * 6)['target_deg'], [100 * DEGREES_PER_TICK] * 6)


if __name__ == '__main__':
    unittest.main()
