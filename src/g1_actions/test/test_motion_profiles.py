import copy
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import yaml
from builtin_interfaces.msg import Time
from g1_actions.actions.elevator.config import motion_scales, validate_motion_profiles
from g1_actions.actions.elevator.task_node import PressTask

ROOT=Path(__file__).resolve().parents[1]

class MotionProfileTests(unittest.TestCase):
    def setUp(self):
        self.motion=yaml.safe_load((ROOT/'config/elevator.yaml').read_text())['motion']

    def test_production_profiles_and_recovery_independence(self):
        validate_motion_profiles(self.motion)
        for stage in ('WITHDRAW','RECOVERY_WITHDRAW'):
            self.assertEqual(motion_scales(self.motion,stage),(.35,.30))
        self.assertEqual(motion_scales(self.motion,'APPROACH'),(.45,.35))
        self.assertEqual(motion_scales(self.motion,'PRESS'),(.60,.45))
        for stage in ('RETURN','RECOVERY_RETURN'):
            self.assertEqual(motion_scales(self.motion,stage),(.15,.12))
        for stage in ('HOVER_APPROACH','HOVER_RETURN','HEIGHT_MOVE','HEIGHT_RETURN'):
            self.assertEqual(motion_scales(self.motion,stage),(.25,.15))
        self.assertEqual(self.motion['contact_timeout_s'],1.)
        self.assertEqual(self.motion['dwell_s'],1.)

    def test_legacy_and_partial_override_do_not_mutate_configuration(self):
        m={'speed':.12,'acceleration':.13}
        self.assertEqual(motion_scales(m,'PRESS'),(.12,.13))
        self.assertEqual(motion_scales(m,'HOVER_RETURN'),(.20,.15))
        m['stage_profiles']={'PRESS':{'speed':.3}}
        old=copy.deepcopy(m)
        self.assertEqual(motion_scales(m,'PRESS'),(.3,.13))
        self.assertEqual(m,old)

    def test_invalid_configuration_rejected_including_inactive_stage(self):
        for value in (0,-.1,1.1,float('nan'),float('inf'),True,'bad'):
            m=copy.deepcopy(self.motion)
            m['stage_profiles']['RECOVERY_RETURN']['acceleration']=value
            with self.subTest(value=value),self.assertRaises((ValueError,TypeError)):
                validate_motion_profiles(m)
        for profiles in (None,[],{'PREES':{}},{'PRESS':{'speeed':.2}}):
            m=copy.deepcopy(self.motion);m['stage_profiles']=profiles
            with self.assertRaises(ValueError):validate_motion_profiles(m)

    def test_actual_cartesian_command_uses_stage_scales_and_logs_them(self):
        for stage,linear in (('PRESS',True),('WITHDRAW',True),
                             ('RECOVERY_WITHDRAW',True),('APPROACH',False),
                             ('HOVER_RETURN',False)):
            messages=[];events=[]
            publisher=SimpleNamespace(publish=messages.append)
            stamp=Time(sec=1,nanosec=2)
            node=SimpleNamespace(cfg={'motion':self.motion,'base_frame':'torso_link'},
                arm='right',health=lambda **kw:None,
                status=SimpleNamespace(current_state=0),status_sequence=0,
                create_publisher=lambda *a:publisher,destroy_publisher=lambda p:None,
                wait=lambda *a:None,
                get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:stamp)),
                event=lambda *a,**kw:events.append((a,kw)),motion_feedback={})
            with patch('g1_actions.actions.elevator.task_node.rclpy.ok',return_value=False):
                with self.assertRaises(RuntimeError):
                    PressTask.move(node,{'xyz':[.3,0.,.2],'rpy':[0.,0.,0.]},linear,stage)
            self.assertEqual(len(messages),1)
            speed,acceleration=motion_scales(self.motion,stage)
            self.assertAlmostEqual(messages[0].speed,speed,places=6)
            self.assertAlmostEqual(messages[0].a,acceleration,places=6)
            self.assertAlmostEqual(events[0][1]['speed_scale'],speed,places=6)

if __name__=='__main__':unittest.main()
