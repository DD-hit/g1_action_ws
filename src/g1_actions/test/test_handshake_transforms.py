"""Reference-frame and tool-center regression cases for nominal previews."""
import copy
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
import yaml
from g1_actions.actions.handshake.transforms import (
    rigid_transform, fixed_urdf_transform, camera_transform,
    wrist_to_palm_center, precontact_preview)

PACKAGE = Path(__file__).resolve().parents[1]


class HandshakeTransformTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((PACKAGE/'config/handshake.yaml').read_text())

    def test_full_state_sdk_mapping_matches_model_and_existing_driver(self):
        from g1_actions.actions.handshake.read_state import SDK_JOINT_NAMES
        import xml.etree.ElementTree as ET
        root = ET.parse(PACKAGE/'robot/g1_29dof_mode_15.urdf').getroot()
        self.assertEqual(set(SDK_JOINT_NAMES), {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'})
        robot = yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())['pipelines'][0]['robot']
        names = robot['left_arm_joint_names']+robot['right_arm_joint_names']+robot['waist_joint_names']
        self.assertEqual([SDK_JOINT_NAMES[index] for index in robot['sdk_joint_indices']], names)

    def test_tool_center_includes_urdf_mount_translation(self):
        for arm, sign in [('left', 1.), ('right', -1.)]:
            hand = np.array(self.config['tools'][arm]['hand_center_xyz_m'])
            np.testing.assert_allclose(wrist_to_palm_center(self.config, PACKAGE, arm), hand+[.0415, sign*.003, 0.])

    def test_known_rotated_camera_and_wrist_center_round_trip(self):
        self.config['execution']['precontact_distance_m'] = .10
        self.config['camera_extrinsics']['measured_torso_to_optical'] = dict(
            xyz_m=[.1, .2, .3], quaternion_xyzw=[0., 0., math.sin(math.pi/4), math.cos(math.pi/4)])
        for arm in ('left', 'right'):
            result = precontact_preview(self.config, PACKAGE, [.2, 0., .5], arm,
                                        source_frame='camera_color_optical_frame')
            np.testing.assert_allclose(result['human_palm_xyz_m'], [.1, .4, .8], atol=1e-12)
            np.testing.assert_allclose(result['precontact_palm_xyz_m'], [0., .4, .8], atol=1e-12)
            r = rigid_transform([0, 0, 0], result['wrist_quaternion_xyzw'])[:3, :3]
            recovered = np.array(result['wrist_xyz_m'])+r@result['wrist_to_palm_center_xyz_m']
            np.testing.assert_allclose(recovered, result['precontact_palm_xyz_m'], atol=1e-12)
            self.assertFalse(result['executable']);self.assertFalse(result['geometry_verified'])

    def test_zero_offset_targets_human_palm_without_dropping_tool_offset(self):
        self.config['execution']['precontact_distance_m'] = 0.
        result = precontact_preview(self.config, PACKAGE, [0., 0., .5], 'right', source_frame='camera_color_optical_frame')
        np.testing.assert_allclose(result['precontact_palm_xyz_m'], result['human_palm_xyz_m'])
        r = rigid_transform([0, 0, 0], result['wrist_quaternion_xyzw'])[:3, :3]
        np.testing.assert_allclose(np.array(result['wrist_xyz_m'])+r@result['wrist_to_palm_center_xyz_m'], result['human_palm_xyz_m'])
        for invalid in (-.01, .31, float('nan')):
            self.config['execution']['precontact_distance_m'] = invalid
            with self.assertRaises(ValueError):precontact_preview(self.config, PACKAGE, [0,0,.5], 'right', source_frame='camera_color_optical_frame')

    def test_factory_translation_is_not_lost(self):
        original = camera_transform(self.config, PACKAGE)
        changed = copy.deepcopy(self.config)
        changed['camera_extrinsics']['camera_link_to_optical']['xyz_m'] = [0, 0, 0]
        no_offset = camera_transform(changed, PACKAGE)
        self.assertGreater(np.linalg.norm(original[:3, 3]-no_offset[:3, 3]), .014)
        np.testing.assert_allclose(original[:3, :3], no_offset[:3, :3])

    def test_fixed_chain_uses_order_and_rejects_movable_joint(self):
        xml = '''<robot name="test"><link name="a"/><link name="b"/><link name="c"/>
        <joint name="ab" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>
        <joint name="bc" type="fixed"><parent link="b"/><child link="c"/><origin xyz="1 0 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'test.urdf';path.write_text(xml)
            np.testing.assert_allclose(fixed_urdf_transform(path, 'a', 'c')[:3, 3], [1, 1, 0], atol=1e-12)
            path.write_text(xml.replace('name="bc" type="fixed"','name="bc" type="revolute"'))
            with self.assertRaises(ValueError):fixed_urdf_transform(path, 'a', 'c')

    def test_unverified_parts_each_block_verified_request(self):
        for section in ('camera', 'orientation', 'tool'):
            cfg = copy.deepcopy(self.config)
            cfg['camera_extrinsics']['verified'] = section != 'camera'
            cfg['execution']['fixed_orientation']['verified'] = section != 'orientation'
            cfg['tools']['right']['verified'] = section != 'tool'
            with self.assertRaises(ValueError):precontact_preview(cfg, PACKAGE, [0, 0, .5], 'right',
                source_frame='camera_color_optical_frame', require_verified=True)

    def test_invalid_frames_points_and_transform_are_rejected(self):
        for point, frame in [([0, 0, .5], 'camera_depth_optical_frame'),
                             ([0, 0, float('nan')], 'camera_color_optical_frame'),
                             ([0, 0, -.5], 'camera_color_optical_frame')]:
            with self.assertRaises(ValueError):precontact_preview(self.config, PACKAGE, point, 'right', source_frame=frame)
        with self.assertRaises(ValueError):rigid_transform([0, 0, 0], [0, 0, 0, 0])


if __name__ == '__main__':unittest.main()
