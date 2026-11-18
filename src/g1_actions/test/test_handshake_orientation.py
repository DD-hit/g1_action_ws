"""Check physical directions and source frame semantics, not just Euler values."""
import copy
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from g1_actions.actions.handshake.geometry import fixed_orientation

PACKAGE = Path(__file__).resolve().parents[1]


class HandshakeOrientationTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((PACKAGE/'config/handshake.yaml').read_text())

    def test_fingers_up_20_thumb_up_and_palms_mirrored(self):
        root = ET.parse(PACKAGE/'robot/g1_29dof_mode_15.urdf').getroot()
        results = []
        for arm in ('left', 'right'):
            joint = root.find(f"joint[@name='{arm}_hand_palm_joint']")
            self.assertEqual(joint.find('parent').get('link'), f'{arm}_wrist_yaw_link')
            self.assertEqual(joint.find('child').get('link'), f'{arm}_rubber_hand')
            self.assertEqual(joint.get('type'), 'fixed')
            np.testing.assert_allclose([float(x) for x in joint.find('origin').get('rpy').split()], 0.)
            result = fixed_orientation(self.config, arm);results.append(result)
            r = np.array(result['rotation'])
            np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1.)
            np.testing.assert_allclose(result['finger_direction'], [math.cos(math.radians(20.)), 0., math.sin(math.radians(20.))], atol=1e-12)
            self.assertGreater(result['thumb_side_direction'][2], 0.)
            self.assertAlmostEqual(result['palm_normal'][2], 0.)
        np.testing.assert_allclose(results[0]['palm_normal'], -np.array(results[1]['palm_normal']))
        np.testing.assert_allclose(results[0]['quaternion_xyzw'], results[1]['quaternion_xyzw'])

    def test_unverified_configuration_cannot_be_requested_as_verified(self):
        with self.assertRaises(ValueError):fixed_orientation(self.config, 'right', require_verified=True)

    def test_rejects_wrong_frame_quaternion_and_nonfinite_angle(self):
        for key, value in [('frame_id','camera_color_optical_frame'),
                           ('right_quaternion_xyzw',[0., 0., 0., 1.]),
                           ('finger_downward_deg',float('nan')),
                           ('finger_downward_deg',-90.), ('finger_downward_deg',90.)]:
            config = copy.deepcopy(self.config);config['execution']['fixed_orientation'][key] = value
            with self.assertRaises(ValueError):fixed_orientation(config, 'right')


if __name__ == '__main__':unittest.main()
