"""Same-handed selection and fallback checks; no SDK or motion."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import numpy as np
import yaml
from g1_actions.actions.handshake.selection import select_arm, fixed_fallback
from g1_actions.actions.handshake.transforms import rigid_transform

PACKAGE=Path(__file__).resolve().parents[1]


class HandshakeSelectionTests(unittest.TestCase):
    def test_auto_uses_anatomical_label_not_image_position(self):
        self.assertEqual(select_arm('auto','Left'),'left')
        self.assertEqual(select_arm('auto','Right'),'right')
        self.assertEqual(select_arm('left','Right'),'left')
        self.assertEqual(select_arm('right','Left'),'right')
        for unknown in ('',None,'unknown'):
            with self.assertRaises(ValueError):select_arm('auto',unknown)

    def test_fixed_fallback_is_same_arm_and_keeps_tool_center_offset(self):
        config=yaml.safe_load((PACKAGE/'config/handshake.yaml').read_text())
        for arm in ('left','right'):
            result=fixed_fallback(config,PACKAGE,arm,{'arm':arm,'human_palm_xyz_m':[.4,0,0]})
            center=config['execution']['fixed_fallback'][arm]['palm_xyz_m']
            rotation=rigid_transform([0,0,0],result['wrist_quaternion_xyzw'])[:3,:3]
            from g1_actions.actions.handshake.transforms import wrist_to_palm_center
            np.testing.assert_allclose(np.array(result['wrist_xyz_m'])+rotation@wrist_to_palm_center(config,PACKAGE,arm),center)
            self.assertEqual(result['arm'],arm)
            self.assertFalse(result['executable'])


if __name__=='__main__':unittest.main()
