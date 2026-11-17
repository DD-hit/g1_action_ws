from pathlib import Path
import unittest
import numpy as np
import yaml
from g1_actions.actions.elevator.config import for_direction
from scipy.spatial.transform import Rotation
from g1_actions.actions.elevator.geometry import resolve_conflicts, StableTargets, localize, press_poses, transform_point, arm_from_image
from g1_actions.actions.elevator.robot_model import Robot

ROOT=Path(__file__).resolve().parents[1]


def box(direction='up',score=.9,x=100):
    return {'direction':direction,'confidence':score,'xyxy':[x,100,x+30,130],
            'center_xy':[x+15,115]}


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.config=for_direction(yaml.safe_load((ROOT/'config/elevator.yaml').read_text()),'up')

    def test_cross_class_conflict_is_not_double_target(self):
        self.assertEqual(resolve_conflicts([box(),box('down',.85)]),[])
        result=resolve_conflicts([box(score=.99),box('down',.7)])
        self.assertEqual([x['direction'] for x in result],['up'])

    def test_arm_selection_uses_original_image_half_and_rejects_invalid_input(self):
        self.assertEqual(arm_from_image([160,240],640),'left')
        self.assertEqual(arm_from_image([465,197],640),'right')
        self.assertEqual(arm_from_image([465,197],1280),'left')
        for center,width in [(None,640),([465,197],None),([320,240],640),
                             ([float('nan'),240],640),([-1,240],640),([640,240],640)]:
            with self.subTest(center=center,width=width):
                with self.assertRaises(ValueError):arm_from_image(center,width)

    def test_tracker_resets_on_missing_ambiguous_or_jumping_targets(self):
        tracker=StableTargets(3,3)
        self.assertEqual(tracker.update([box()]),[])
        self.assertEqual(tracker.update([box(x=101)]),[])
        self.assertEqual(len(tracker.update([box(x=102)])),1)
        self.assertEqual(tracker.update([]),[])
        self.assertEqual(tracker.update([box()]),[])
        self.assertEqual(tracker.update([box(),box(x=200)]),[])
        self.assertEqual(tracker.update([box(x=300)]),[])

    def test_depth_plane_reconstruction_and_invalid_center_rejection(self):
        depth=np.full((480,640),.5,np.float32)
        point,normal,quality=localize(depth,[300,220,340,260],[320,240],
                                    (500,500,320,240),self.config['depth'])
        np.testing.assert_allclose(point,[0,0,.5],atol=1e-6)
        np.testing.assert_allclose(normal,[0,0,-1],atol=1e-6)
        self.assertLess(quality['plane_residual_m'],1e-6)
        depth[230:250,310:330]=0
        with self.assertRaisesRegex(ValueError,'face depth unavailable'):
            localize(depth,[300,220,340,260],[320,240],(500,500,320,240),self.config['depth'])

    def test_depth_plane_rejects_no_data(self):
        with self.assertRaises(ValueError):
            localize(np.zeros((480,640)),[300,220,340,260],[320,240],
                     (500,500,320,240),self.config['depth'])

    def test_raw_color_distortion_is_removed_before_backprojection(self):
        import cv2
        point=np.array([[.1,-.03,.5]],dtype=float)
        matrix=np.array([[500.,0.,320.],[0.,500.,240.],[0.,0.,1.]])
        coefficients=np.array([.2,.01,.001,.002,0.])
        pixel,_=cv2.projectPoints(point,np.zeros(3),np.zeros(3),matrix,coefficients)
        u,v=pixel.ravel()
        actual,_,_=localize(np.full((480,640),.5),[u-15,v-15,u+15,v+15],[u,v],
                            (500,500,320,240),self.config['depth'],('plumb_bob',coefficients))
        np.testing.assert_allclose(actual,point[0],atol=1e-7)

    def test_wrist_tcp_offset_and_press_direction(self):
        cfg=self.config['motion'];tip=[.17,-.03,.007];p=np.array([.35,.2,.15]);n=np.array([-1.,.3,.2])
        poses=press_poses(p,n,tip,cfg)
        for pose in poses.values():
            rx,ry,rz=pose['rpy']
            # Match the driver's explicit Eigen AngleAxis X * Y * Z convention.
            rotation=(Rotation.from_rotvec([rx,0,0]).as_matrix() @
                      Rotation.from_rotvec([0,ry,0]).as_matrix() @
                      Rotation.from_rotvec([0,0,rz]).as_matrix())
            actual=np.array(pose['xyz'])+rotation@tip
            np.testing.assert_allclose(rotation[:,0],-n/np.linalg.norm(n),atol=1e-10)
            np.testing.assert_allclose(actual,pose['tcp_xyz'],atol=1e-10)
        self.assertGreater(np.dot(np.array(poses['approach']['tcp_xyz'])-p,n),0)
        self.assertLess(np.dot(np.array(poses['press']['tcp_xyz'])-p,n),0)
        cfg['press_depth_m']=cfg['max_press_depth_m']+.001
        with self.assertRaises(ValueError):press_poses(p,n,tip,cfg)

    def test_palm_down_keeps_finger_normal_and_correct_tcp_for_both_arms(self):
        cfg=self.config['motion'];point=np.array([.35,.1,.15]);normal=np.array([-1.,.2,.15]);normal/=np.linalg.norm(normal)
        expected_down=np.array([0.,0.,-1.]);expected_down-=normal*(expected_down@normal);expected_down/=np.linalg.norm(expected_down)
        for arm in ('left','right'):
            tool=self.config['tools'][arm]
            palm=[0.,-1.,0.] if arm=='left' else [0.,1.,0.]
            tip=Robot(ROOT/self.config['model_file']).wrist_tip(tool)
            poses=press_poses(point,normal,tip,cfg,palm_normal=palm)
            for pose in poses.values():
                rotation=Rotation.from_euler('XYZ',pose['rpy']).as_matrix()
                np.testing.assert_allclose(rotation@palm,expected_down,atol=1e-10)
                np.testing.assert_allclose(rotation[:,0],-normal,atol=1e-10)
                np.testing.assert_allclose(np.array(pose['xyz'])+rotation@tip,pose['tcp_xyz'],atol=1e-10)
            self.assertAlmostEqual(np.linalg.norm(np.array(poses['press']['tcp_xyz'])-point),cfg['press_depth_m'])

    def test_workspace_is_checked(self):
        with self.assertRaisesRegex(ValueError,'workspace'):
            press_poses([1.,0,.1],[-1,0,0],[.17,0,0],self.config['motion'])

    def test_urdf_camera_and_middle_contact_chain(self):
        robot=Robot(ROOT/self.config['model_file'])
        camera=robot.camera(self.config)
        np.testing.assert_allclose(camera[:3,3],[.0576235,.01753,.42987],atol=1e-8)
        for arm,tool in self.config['tools'].items():
            expected=robot.relative(tool['wrist_link'],arm+'_middle_contact')[:3,3]
            np.testing.assert_allclose(robot.wrist_tip(tool),expected,atol=1e-7)
        with self.assertRaisesRegex(ValueError,'Missing measured joint'):
            robot.relative('torso_link','left_wrist_yaw_link')


if __name__=='__main__':unittest.main()
