"""Withdrawal verification and cached-retrace recovery; no hardware commands."""
import copy
import time
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import numpy as np
from g1_actions.actions.elevator.task_node import PressTask


class RetainedReleaseTests(unittest.TestCase):
    def test_configured_20mm_releases_and_19_9mm_blocks(self):
        task=self.fixture();task.cfg['motion']['withdraw_release_clearance_m']=.020
        task.hover_guard.current_clearance=.020
        PressTask.release_if_safely_withdrawn(task)
        task.trigger.assert_called_once_with('/dual_arm/end')
        task=self.fixture();task.cfg['motion']['withdraw_release_clearance_m']=.020
        task.hover_guard.current_clearance=.0199
        with self.assertRaisesRegex(RuntimeError,'below 20 mm'):PressTask.release_if_safely_withdrawn(task)
        task.trigger.assert_not_called()
    def fixture(self):
        task=SimpleNamespace(status_sequence=0,status_arrival=time.monotonic(),waist_arrival=time.monotonic(),
            waist=SimpleNamespace(sdk_connected=True,has_upper_body_authority=True),
            status=SimpleNamespace(current_state=0,current_joint=SimpleNamespace(left_velocities=[0.]*7,right_velocities=[0.]*7)),
            cfg={'motion':{'max_joint_speed':.08}},hover_guard=SimpleNamespace(current_clearance=.04),
            health=lambda **kwargs:None,contact_pending=True,acquired=True,event=Mock(),trigger=Mock())
        def wait(predicate,*args):task.status_sequence+=1;assert predicate()
        task.wait=wait
        task.verify_withdrawal=lambda:PressTask.verify_withdrawal(task)
        return task

    def test_fresh_stationary_clear_hand_releases_without_moving(self):
        task=self.fixture();PressTask.release_if_safely_withdrawn(task)
        task.trigger.assert_called_once_with('/dual_arm/end')
        self.assertFalse(task.acquired);self.assertFalse(task.contact_pending)

    def test_unclear_moving_fast_or_disconnected_never_releases(self):
        for change in ('clearance','moving','velocity','sdk','stale'):
            task=self.fixture()
            if change=='clearance':task.hover_guard.current_clearance=.0399
            if change=='moving':task.status.current_state=1
            if change=='velocity':task.status.current_joint.right_velocities[0]=.09
            if change=='sdk':task.waist.sdk_connected=False
            if change=='stale':task.wait=Mock(side_effect=RuntimeError('stale feedback'))
            with self.assertRaises(RuntimeError):PressTask.release_if_safely_withdrawn(task)
            task.trigger.assert_not_called();self.assertTrue(task.acquired)

    def test_external_release_is_recognized_without_another_request(self):
        task=self.fixture();task.waist.has_upper_body_authority=False
        PressTask.release_if_safely_withdrawn(task)
        task.trigger.assert_not_called();self.assertFalse(task.acquired)

class RecoveryTests(unittest.TestCase):
    def fixture(self):
        task=SimpleNamespace(contact_pending=True,status=SimpleNamespace(current_state=0),
            cfg={'motion':{'approach_m':.06}},
            locked_target={'normal':[-1.,0.,0.]},
            hover_guard=SimpleNamespace(current_clearance=-.002,begin_withdrawal=lambda:None),
            retreat_pose={'xyz':[.238,-.2,.1],'rpy':[0.,0.,0.]},
            initial_pose={'xyz':[.01,-.2,-.1],'rpy':[.5,1.3,-.3]},
            wait=lambda *a:None,health=lambda **k:None,
            current_pose=lambda:{'xyz':[.3,-.2,.1],'rpy':[.01,-.02,.03]})
        calls=[]
        task.configure_panel=lambda target,phase:calls.append(phase)
        task.move=lambda pose,linear,stage:calls.append((stage,linear,copy.deepcopy(pose)))
        def verified():task.contact_pending=False;calls.append('verified')
        task.verify_withdrawal=verified
        def recorded_withdrawal(stage):
            task.configure_panel(task.locked_target,'withdraw')
            calls.append(stage)
            task.verify_withdrawal()
        task.withdraw_contact=Mock(side_effect=recorded_withdrawal)
        task.finish_withdrawal=lambda **kwargs:PressTask.finish_withdrawal(task,**kwargs)
        task.event=lambda stage,**kwargs:calls.append(stage)
        return task,calls
    def test_recovery_uses_recorded_retrace_before_return(self):
        task,calls=self.fixture();PressTask.recover_contact(task)
        task.withdraw_contact.assert_called_once_with('RECOVERY_WITHDRAW')
        self.assertEqual(calls[:4],['withdraw','RECOVERY_WITHDRAW','verified','return'])
        self.assertEqual(calls[4],('RECOVERY_RETURN',False,task.initial_pose))
    def test_retreat_failure_blocks_return_and_keeps_contact_pending(self):
        task,calls=self.fixture()
        task.withdraw_contact=Mock(side_effect=RuntimeError('No accepted retrace cache'))
        with self.assertRaisesRegex(RuntimeError,'No accepted retrace cache'):PressTask.recover_contact(task)
        self.assertTrue(task.contact_pending);self.assertNotIn('return',calls)
    def test_clearance_below_40mm_cannot_authorize_return(self):
        task=SimpleNamespace(cfg={'motion':{}},status=SimpleNamespace(current_state=0),health=lambda **k:None,hover_guard=SimpleNamespace(current_clearance=.039),contact_pending=True,event=Mock())
        with self.assertRaisesRegex(RuntimeError,'below 40 mm'):PressTask.verify_withdrawal(task)
        self.assertTrue(task.contact_pending)
    def test_verified_clearance_authorizes_return(self):
        task=SimpleNamespace(cfg={'motion':{}},status=SimpleNamespace(current_state=0),health=lambda **k:None,hover_guard=SimpleNamespace(current_clearance=.05),contact_pending=True,event=Mock())
        PressTask.verify_withdrawal(task);self.assertFalse(task.contact_pending)

    def test_recovery_releases_from_verified_retreat_without_return(self):
        task,calls=self.fixture();task.cfg['motion']['return_to_start_after_withdraw']=False
        PressTask.recover_contact(task)
        self.assertEqual(calls[0],'withdraw')
        self.assertEqual(calls[1],'RECOVERY_WITHDRAW')
        self.assertEqual(calls[2:],['verified','READY_TO_RELEASE'])
        self.assertFalse(task.contact_pending)

    def test_unverified_retreat_cannot_skip_return_for_release(self):
        task,calls=self.fixture();task.cfg['motion']['return_to_start_after_withdraw']=False
        with self.assertRaisesRegex(RuntimeError,'withdrawal is verified'):
            PressTask.finish_withdrawal(task)
        self.assertNotIn('READY_TO_RELEASE',calls)

    def test_moving_arm_cannot_authorize_release(self):
        task=SimpleNamespace(status=SimpleNamespace(current_state=1),health=lambda **k:None,hover_guard=SimpleNamespace(current_clearance=.06),contact_pending=True,event=Mock())
        with self.assertRaisesRegex(RuntimeError,'not stationary'):PressTask.verify_withdrawal(task)
        self.assertTrue(task.contact_pending)
