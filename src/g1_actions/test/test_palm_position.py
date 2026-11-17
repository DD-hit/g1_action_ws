"""Position-only regression cases independent of ROS and the MediaPipe model."""
import unittest
import numpy as np
from g1_actions.perception.palm_pose import estimate_palm_position, rays


def example():
    points = np.full((21, 3), .5)
    points[0, :2] = [.5, .65]
    for index, x in zip((5, 9, 13, 17), (.42, .47, .53, .58)):
        points[index, :2] = [x, .4]
    k = dict(width=320, height=240, fx=320., fy=320., cx=160., cy=120., d=[])
    return points, np.full((240, 320), .5), k


class PalmPositionTests(unittest.TestCase):
    def test_known_wrist_and_offset_no_fabricated_orientation(self):
        points, depth, k = example()
        result = estimate_palm_position(points, depth, k)
        self.assertTrue(result['valid'], result)
        wrist = rays([points[0, :2]*[320, 240]], k)[0]*.5
        np.testing.assert_allclose(result['position_m'], wrist+[0, -.06, 0], atol=1e-8)
        self.assertFalse(result['orientation_valid'])
        self.assertNotIn('rotation', result)
        self.assertNotIn('quaternion_xyzw', result)

    def test_direction_has_measured_out_of_image_component(self):
        points, depth, k = example()
        depth[88:105] = .46
        result = estimate_palm_position(points, depth, k)
        self.assertTrue(result['valid'], result)
        self.assertLess(result['direction'][2], -.3)
        self.assertAlmostEqual(np.linalg.norm(np.array(result['position_m'])-result['wrist_position_m']), .06)

    def test_missing_wrist_depth_never_falls_back_to_model(self):
        points, depth, k = example(); depth[148:165, 150:170] = 0
        result = estimate_palm_position(points, depth, k)
        self.assertFalse(result['valid']); self.assertNotIn('position_m', result)

    def test_background_at_bases_is_rejected(self):
        points, depth, k = example(); depth[88:105] = 1.
        self.assertFalse(estimate_palm_position(points, depth, k)['valid'])

    def test_one_occluded_base_is_allowed_but_two_are_not(self):
        points, depth, k = example()
        depth[90:103, 128:142] = 0
        result = estimate_palm_position(points, depth, k)
        self.assertTrue(result['valid'], result)
        self.assertEqual(result['accepted_finger_bases'], [9, 13, 17])
        depth[90:103, 144:157] = 0
        self.assertFalse(estimate_palm_position(points, depth, k)['valid'])

    def test_curled_or_offscreen_fingertips_do_not_change_position(self):
        points, depth, k = example();before = estimate_palm_position(points, depth, k)
        points[[4, 8, 12, 16, 20]] = [2., 2., 100.]
        after = estimate_palm_position(points, depth, k)
        self.assertTrue(after['valid']);np.testing.assert_allclose(after['position_m'], before['position_m'])

    def test_degenerate_direction_and_invalid_offset(self):
        points, depth, k = example(); points[[5, 9, 13, 17]] = points[0]
        self.assertFalse(estimate_palm_position(points, depth, k)['valid'])
        self.assertFalse(estimate_palm_position(points, depth, k, config={'wrist_offset': {'distance_m': -1}})['valid'])


if __name__ == '__main__':
    unittest.main()
