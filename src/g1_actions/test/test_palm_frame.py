"""Geometry and tracking regressions; no ROS, GPU, video, or robot required."""
import unittest

import numpy as np

from g1_actions.perception.palm_frame import estimate_model_frame, PalmFrameTracker, MCP


def sample(side='Right'):
    points = np.zeros((21, 3))
    points[1:5] = [[-.035, .025, 0], [-.05, .035, 0], [-.065, .045, 0], [-.08, .05, 0]]
    for start, x, y in [(5, -.03, .08), (9, -.01, .09), (13, .01, .085), (17, .03, .07)]:
        for offset in range(4):
            points[start+offset] = [x, y+.02*offset, 0]
    if side == 'Left':
        points[:, 0] *= -1
    return points


def hand(points=None, label='Right'):
    points = sample() if points is None else points
    image = points.copy() * 3
    image[:, :2] += .4
    return dict(landmarks_model_world=points.tolist(), landmarks_normalized=image.tolist(),
                handedness_raw=label, handedness_score=.95)


class PalmGeometryTests(unittest.TestCase):
    def test_both_hands_use_orthonormal_right_handed_frames(self):
        for side in ['Right', 'Left']:
            result = estimate_model_frame(sample(side), side, {})
            self.assertTrue(result['valid'])
            r = result['rotation']
            np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-10)
            self.assertAlmostEqual(np.linalg.det(r), 1.)
            np.testing.assert_allclose(np.cross(r[:, 1], r[:, 2]), r[:, 0], atol=1e-10)
            self.assertLess(r[2, 2], -.99)

    def test_mirroring_hand_does_not_reverse_outward_normal(self):
        right = estimate_model_frame(sample(), 'Right', {})
        left = estimate_model_frame(sample('Left'), 'Left', {})
        np.testing.assert_allclose(right['rotation'][:, 2], left['rotation'][:, 2], atol=1e-10)
        self.assertGreater(np.dot(right['rotation'][:, 0], right['thumb_direction']), 0)
        self.assertLess(np.dot(left['rotation'][:, 0], left['thumb_direction']), 0)

    def test_fingertip_bending_does_not_move_palm_origin_or_frame(self):
        points = sample()
        before = estimate_model_frame(points, 'Right', {})
        points[[4, 8, 12, 16, 20]] += [.03, -.08, .04]
        after = estimate_model_frame(points, 'Right', {})
        np.testing.assert_allclose(before['rotation'], after['rotation'])
        np.testing.assert_allclose(before['origin'], after['origin'])

    def test_rigid_transform_preserves_frame_geometry(self):
        points = sample()
        before = estimate_model_frame(points, 'Right', {})
        q, _ = np.linalg.qr(np.random.default_rng(42).normal(size=(3, 3)))
        q[:, -1] *= np.linalg.det(q)
        translation = np.array([.1, -.2, .4])
        after = estimate_model_frame(points@q.T+translation, 'Right', {})
        np.testing.assert_allclose(after['rotation'], q@before['rotation'], atol=1e-10)
        np.testing.assert_allclose(after['origin'], q@before['origin']+translation, atol=1e-10)

    def test_bad_geometry_and_unknown_side_are_rejected(self):
        self.assertFalse(estimate_model_frame(np.zeros((21, 3)), 'Right', {})['valid'])
        points = sample(); points[0, 0] = float('nan')
        self.assertFalse(estimate_model_frame(points, 'Right', {})['valid'])
        self.assertFalse(estimate_model_frame(sample(), 'Unknown', {})['valid'])


class PalmTrackingTests(unittest.TestCase):
    def confirmed(self):
        tracker = PalmFrameTracker({})
        for i in range(5):
            item = hand(); tracker.update([item], i*33)
        self.assertTrue(item['palm_frame']['valid'])
        return tracker, item['palm_frame']

    def test_single_raw_label_flip_does_not_flip_normal(self):
        tracker, before = self.confirmed()
        item = hand(label='Left'); tracker.update([item], 165)
        after = item['palm_frame']
        self.assertEqual(after['handedness_stable'], 'Right')
        self.assertTrue(after['raw_label_disagrees'])
        np.testing.assert_allclose(after['rotation'], before['rotation'], atol=1e-10)

    def test_actual_large_rotation_requires_consistent_reacquisition(self):
        tracker, before = self.confirmed()
        points = sample(); center = (points[0]+points[MCP].mean(axis=0))/2
        points = (points-center)@np.diag([1., -1., -1.])+center
        for index in range(3):
            item = hand(points); tracker.update([item], 165+index*33)
            self.assertEqual(item['palm_frame']['valid'], index==2)
        self.assertEqual(item['palm_frame']['status'], 'orientation_reacquired')
        self.assertLess(np.dot(np.array(before['rotation'])[:, 2], np.array(item['palm_frame']['rotation'])[:, 2]), -.99)

    def test_missing_hand_expires_identity_and_new_hand_must_confirm(self):
        tracker, before = self.confirmed()
        tracker.update([], 800)
        item = hand(sample('Left'), 'Left'); tracker.update([item], 833)
        self.assertNotEqual(item['palm_frame']['track_id'], before['track_id'])
        self.assertFalse(item['palm_frame']['valid'])
        for index in range(4):
            item = hand(sample('Left'), 'Left'); tracker.update([item], 866+index*33)
        self.assertTrue(item['palm_frame']['valid'])
        self.assertEqual(item['palm_frame']['handedness_stable'], 'Left')

    def test_smoothing_stays_on_rotation_group(self):
        tracker, _ = self.confirmed()
        points = sample(); a = .3
        q = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        center = (points[0]+points[MCP].mean(axis=0))/2
        item = hand((points-center)@q.T+center); tracker.update([item], 165)
        r = np.array(item['palm_frame']['rotation'])
        np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(r), 1.)
        self.assertLess(item['palm_frame']['raw_to_filtered_angle_deg'], 70)


if __name__ == '__main__':
    unittest.main()
