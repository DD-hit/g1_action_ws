"""RGB-D geometry and freshness checks with known metric ground truth."""
from types import SimpleNamespace
import unittest
import numpy as np

from g1_actions.perception.palm_pose import estimate_palm_pose, validate_pair, image_array, quaternion_xyzw


def example():
    points = np.zeros((21, 3)); points[:, :2] = [.5, .5]
    points[0, :2] = [.5, .6]
    for i, xy in zip([5, 9, 13, 17], [[.44, .4], [.48, .38], [.52, .38], [.56, .42]]): points[i, :2] = xy
    intrinsics = dict(width=320, height=240, fx=320., fy=320., cx=160., cy=120., d=[])
    depth = np.full((240, 320), .5)
    return points, depth, intrinsics


class PalmDepthTests(unittest.TestCase):
    def test_offscreen_fingertip_rejects_incomplete_hand(self):
        points, depth, k = example()
        points[12, 1] = 1.05
        result = estimate_palm_pose(points, depth, k, handedness='Right')
        self.assertFalse(result['valid'])
        self.assertIn('clipped', result['reason'])

    def test_front_plane_has_known_position_and_right_handed_axes(self):
        points, depth, k = example()
        result = estimate_palm_pose(points, depth, k, handedness='Right')
        self.assertTrue(result['valid'], result)
        center = (points[0, :2]+points[[5, 9, 13, 17], :2].mean(axis=0))/2
        expected = np.array([(center[0]*320-160)/320, (center[1]*240-120)/320, 1])*.5
        np.testing.assert_allclose(result['position_m'], expected, atol=1e-6)
        r = np.array(result['rotation']); np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-8)
        self.assertAlmostEqual(np.linalg.det(r), 1.)
        self.assertAlmostEqual(np.linalg.norm(result['quaternion_xyzw']), 1.)

    def test_tilted_plane_and_outliers(self):
        points, depth, k = example();yy, xx = np.mgrid[:240, :320]
        depth[:] = .5/(1-.25*(xx-160)/320)
        depth[::5, ::5] += .07
        result = estimate_palm_pose(points, depth, k, handedness='Right')
        self.assertTrue(result['valid'], result)
        normal = np.array([-.25, 0, 1.]);normal/=np.linalg.norm(normal)
        self.assertGreater(abs(np.array(result['rotation'])[:, 2]@normal), .999)

    def test_missing_center_depth_is_not_replaced_with_fake_distance(self):
        points, depth, k = example(); depth[105:135, 145:175] = 0
        result = estimate_palm_pose(points, depth, k, handedness='Right')
        self.assertFalse(result['valid']);self.assertNotIn('position_m', result)

    def test_depth_shape_range_and_clipped_palm(self):
        points, depth, k = example()
        self.assertFalse(estimate_palm_pose(points, depth[:20], k, handedness='Right')['valid'])
        self.assertFalse(estimate_palm_pose(points, depth*0, k, handedness='Right')['valid'])
        points[0, 0] = -1
        self.assertFalse(estimate_palm_pose(points, depth, k, handedness='Right')['valid'])

    def test_timestamp_and_coordinate_mismatch_rejected(self):
        a=dict(stamp_s=10.,frame_id='color_optical',width=320,height=240)
        self.assertEqual(validate_pair(a,a,a,10.1,{}),0.)
        for b,now in [(dict(a,stamp_s=9.),10.1),(dict(a,frame_id='depth_optical'),10.1),(a,11.)]:
            with self.assertRaises(ValueError):validate_pair(a,b,a,now,{})

    def test_padded_and_big_endian_depth_images(self):
        payload=np.array([[1000,2000,0],[3000,4000,0]],dtype='>u2').tobytes()
        msg=SimpleNamespace(encoding='16UC1',height=2,width=2,step=6,is_bigendian=True,data=payload)
        np.testing.assert_allclose(image_array(msg),[[1.,2.],[3.,4.]])
        msg.data=payload[:-1]
        with self.assertRaises(ValueError):image_array(msg)

    def test_quaternion_at_half_turn(self):
        q=quaternion_xyzw(np.diag([1.,-1.,-1.]))
        np.testing.assert_allclose(np.abs(q),[1.,0.,0.,0.],atol=1e-8)


if __name__=='__main__':unittest.main()
