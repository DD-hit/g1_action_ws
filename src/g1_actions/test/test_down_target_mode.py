"""Target acquisition regressions; no hardware or motion commands."""
import copy
import time
import unittest
from types import SimpleNamespace
from g1_actions.actions.elevator.target_mode import down_target_options
from g1_actions.actions.elevator.task_node import PressTask


class DownTargetTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {'base_frame': 'torso_link', 'motion': {'max_target_age_s': .8}}
        self.up = dict(direction='up', point=[.4, -.2, .3], normal=[-1., 0., 0.],
                       stamp_sec=100., frame_id='torso_link', center_xy=[450, 180],
                       confidence=.9, quality={'plane_residual_m': .001},
                       camera_point=[.1, .2, .5], stable_frames=3)
        self.down = dict(self.up, direction='down', point=[.4, -.2, .21])

    def target(self, targets, direction='down', **changes):
        node = SimpleNamespace(cfg=self.cfg, direction=direction, locked_target=None,
            targets={'targets': targets}, target_arrival=time.monotonic(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100000000000)))
        node.__dict__.update(changes)
        return PressTask.target(node)

    def test_old_config_and_direct_mode_preserve_detected_down(self):
        self.assertEqual(self.target([self.up, self.down]), self.down)
        self.cfg['down_target'] = {'mode': 'direct'}
        self.assertEqual(self.target([self.up, self.down]), self.down)
        with self.assertRaises(RuntimeError): self.target([self.up])

    def test_from_up_selects_up_even_with_conflicting_direct_down(self):
        self.cfg['down_target'] = {'mode': 'from_up'}
        original = copy.deepcopy(self.up)
        result = self.target([self.up, self.down])
        self.assertAlmostEqual(result['point'][2], .24)
        self.assertEqual(result['point'][:2], self.up['point'][:2])
        self.assertEqual(result['direction'], 'down')
        for key in ('normal', 'stamp_sec', 'quality', 'center_xy', 'camera_point'):
            self.assertEqual(result[key], self.up[key])
        self.assertEqual(self.up, original)
        result['quality']['plane_residual_m'] = 99
        self.assertEqual(self.up, original)

    def test_custom_offset_and_up_task_unchanged(self):
        self.cfg['down_target'] = {'mode': 'from_up', 'z_offset_m': .08}
        self.assertAlmostEqual(self.target([self.up])['point'][2], .22)
        self.assertEqual(self.target([self.up], 'up'), self.up)

    def test_from_up_rejects_missing_ambiguous_stale_wrong_frame(self):
        self.cfg['down_target'] = {'mode': 'from_up'}
        cases = [[], [self.down], [self.up, copy.deepcopy(self.up)],
                 [dict(self.up, stamp_sec=99.)], [dict(self.up, stamp_sec=101.)],
                 [dict(self.up, frame_id='camera')],
                 [dict(self.up, point=[.4, 0, float('nan')])]]
        for targets in cases:
            with self.subTest(targets=targets), self.assertRaises(RuntimeError):
                self.target(targets)
        with self.assertRaises(RuntimeError):
            self.target([self.up], target_arrival=time.monotonic()-2)

    def test_locked_snapshot_is_not_offset_twice(self):
        self.cfg['down_target'] = {'mode': 'from_up'}
        locked = self.target([self.up])
        result = self.target([], locked_target=locked)
        self.assertEqual(result, locked)
        self.assertIsNot(result, locked)

    def test_invalid_mode_and_offsets_fail(self):
        for options in [{'mode': 'typo'}] + [
                {'mode': 'from_up', 'z_offset_m': x}
                for x in (0, -.06, float('nan'), float('inf'), None, True, 'bad')]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                down_target_options({'down_target': options})


if __name__ == '__main__':
    unittest.main()
