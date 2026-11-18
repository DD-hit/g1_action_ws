"""Freshness retries and one shared contact budget; no motor interfaces."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from g1_actions.actions.elevator.task_node import PressTask

class TargetPreparationTests(unittest.TestCase):
    def fixture(self):
        now=[1.];latest=[{'stamp_sec':.6,'center_xy':[400.,200.]}]
        node=SimpleNamespace(cfg={'motion':{'max_target_age_s':.8}},
            requested_arm='auto',arm='auto',targets={'camera_calibration_id':'id','image_width':640},
            event=Mock(),lock_snapshot=Mock(),current_pose=lambda:{'xyz':[0.,0.,0.],'rpy':[0.,0.,0.]})
        node.target=lambda:copy.deepcopy(latest[0])
        node.poses=Mock(return_value={'approach':{},'press':{}})
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(now[0]*1e9)))
        return node,now,latest
    def test_target_expiring_during_preparation_is_replaced_before_lock(self):
        node,now,latest=self.fixture();calls=[0]
        def poses(target):
            calls[0]+=1
            if calls[0]==1:now[0]+=.5
            return {'approach':{},'press':{}}
        node.poses=poses
        def spin(*a,**k):now[0]+=.01;latest[0]['stamp_sec']=now[0]
        with patch('g1_actions.actions.elevator.task_node.camera_signature',return_value='id'),patch('g1_actions.actions.elevator.task_node.time.monotonic',side_effect=lambda:now[0]),patch.object(node,'spin_once',side_effect=spin,create=True):
            target,poses,selection=PressTask.prepare_target(node,2.,True)
        self.assertEqual(calls[0],2);self.assertEqual(node.arm,'right')
        self.assertGreater(target['stamp_sec'],.6)
        node.lock_snapshot.assert_called_once_with(target)
    def test_missing_fresh_target_times_out_without_locking(self):
        node,now,latest=self.fixture();node.target=Mock(side_effect=RuntimeError('No fresh vision result'))
        def spin(*a,**k):now[0]+=.2
        with patch('g1_actions.actions.elevator.task_node.time.monotonic',side_effect=lambda:now[0]),patch.object(node,'spin_once',side_effect=spin,create=True):
            with self.assertRaisesRegex(RuntimeError,'Timed out waiting for a fresh target snapshot'):
                PressTask.prepare_target(node,.4,True)
        node.lock_snapshot.assert_not_called()
    def test_calibration_mismatch_still_aborts_without_locking(self):
        node,now,latest=self.fixture()
        with patch('g1_actions.actions.elevator.task_node.camera_signature',return_value='different'),patch('g1_actions.actions.elevator.task_node.time.monotonic',side_effect=lambda:now[0]):
            with self.assertRaisesRegex(RuntimeError,'Camera configuration changed'):
                PressTask.prepare_target(node,2.,True)
        node.lock_snapshot.assert_not_called()

class ContactBudgetTests(unittest.TestCase):
    def test_waiting_and_dwell_share_the_same_one_second_budget(self):
        node=SimpleNamespace(cfg={'motion':{'contact_timeout_s':1.}},press_contact_started_at=10.)
        for now,expected in ((10.,1.),(10.7,.3),(11.,0.),(12.,0.)):
            with self.subTest(now=now),patch('g1_actions.actions.elevator.task_node.time.monotonic',return_value=now):
                self.assertAlmostEqual(PressTask.contact_remaining(node),expected)
    def test_budget_does_not_start_before_contact_region_is_entered(self):
        node=SimpleNamespace(cfg={'motion':{'contact_timeout_s':1.}},press_contact_started_at=None)
        with patch('g1_actions.actions.elevator.task_node.time.monotonic',return_value=100.):
            self.assertEqual(PressTask.contact_remaining(node),1.)
    def test_invalid_budget_is_rejected(self):
        for value in (0.,-1.,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaises(RuntimeError):
                PressTask.contact_remaining(SimpleNamespace(cfg={'motion':{'contact_timeout_s':value}}))
class ContactLimitTransitionTests(unittest.TestCase):
    def test_one_second_limit_cancels_forward_motion_and_returns_for_withdrawal(self):
        from builtin_interfaces.msg import Time
        from brsd_msgs.msg import DualArmStatus
        now=[0.];stamp=Time(sec=1,nanosec=2)
        node=SimpleNamespace(cfg={'base_frame':'torso_link','tools':{'right':{}},
            'motion':{'speed':.35,'acceleration':.3,'hover_m':.012,'contact_timeout_s':1.,'command_timeout_s':5.}},
            arm='right',status_sequence=0,press_contact_started_at=None,motion_feedback={},
            locked_target={'point':[0.,0.,0.],'normal':[1.,0.,0.]},event=Mock(),health=Mock(),
            robot=SimpleNamespace(wrist_tip=lambda tool:[0.,0.,0.]),
            current_pose=lambda:{'xyz':[0.,0.,0.],'rpy':[0.,0.,0.]},
            close_to=Mock(return_value=False),destroy_publisher=Mock(),count_publishers=lambda topic:1)
        node.status=SimpleNamespace(current_state=DualArmStatus.IDLE)
        def publish(msg):
            node.status.current_state=DualArmStatus.MOVING
            node.motion_feedback[('MoveL',2,1,2)]={'phase':'accepted'}
        node.create_publisher=lambda *a:SimpleNamespace(publish=publish,get_subscription_count=lambda:1)
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:stamp))
        node.wait=lambda predicate,*a: self.assertTrue(predicate())
        node.contact_remaining=lambda:PressTask.contact_remaining(node)
        def cancel():node.status.current_state=DualArmStatus.IDLE
        node.cancel_motion=Mock(side_effect=cancel)
        def spin(*a,**k):now[0]+=.25;node.status_sequence+=1
        with patch('g1_actions.actions.elevator.task_node.time.monotonic',side_effect=lambda:now[0]),patch.object(node,'spin_once',side_effect=spin,create=True),patch('g1_actions.actions.elevator.task_node.rclpy.ok',return_value=True):
            reached=PressTask.move(node,{'xyz':[.1,0.,0.],'rpy':[0.,0.,0.]},True,'PRESS')
        self.assertFalse(reached);node.cancel_motion.assert_called_once()
        self.assertEqual(node.press_completion['termination'],'contact_time_limit')
        self.assertFalse(node.press_completion['endpoint_reached'])
        self.assertAlmostEqual(now[0]-node.press_contact_started_at,1.)

    def test_driver_checks_precede_target_selection(self):
        order=[]
        class StopBeforeMotion(Exception):pass
        node=SimpleNamespace(cfg={'motion':{'pause_inference_on_lock':False}},
            check_execution_ready=lambda:order.append('driver_checks'))
        def prepare(*a):order.append('target_selection');raise StopBeforeMotion()
        node.prepare_target=prepare
        with self.assertRaises(StopBeforeMotion):PressTask.run(node,True,2.,False)
        self.assertEqual(order,['driver_checks','target_selection'])

class HoverReturnTests(unittest.TestCase):
    def test_noncontact_trial_returns_to_its_captured_initial_pose(self):
        initial={'xyz':[.01,-.2,-.11],'rpy':[0.,1.3,0.]}
        approach={'xyz':[.2,-.1,.1],'rpy':[0.,0.,0.]}
        node=SimpleNamespace(cfg={'camera':{'verified':False},'tools':{'right':{'verified':False}},
            'motion':{'pause_inference_on_lock':False,
            'approach_m':.06,'command_timeout_s':5.}},arm='right',initial_pose=initial,
            check_execution_ready=Mock(),prepare_target=Mock(return_value=({},
                {'approach':approach},{})),event=Mock(),health=Mock(),
            configure_panel=Mock(),move=Mock(),spin_once=Mock(),
            waist=SimpleNamespace(has_upper_body_authority=True))
        clock=iter([0.,1.,2.,3.])
        with patch('g1_actions.actions.elevator.task_node.motion_scales',return_value=(.25,.15)),patch('g1_actions.actions.elevator.hover_guard.HoverGuard',return_value=SimpleNamespace(minimum_seen=.06)),patch('g1_actions.actions.elevator.task_node.time.monotonic',side_effect=lambda:next(clock)):
            PressTask.run(node,execute=True,wait_s=2.,hover_only=True)
        self.assertEqual(node.move.call_count,2)
        self.assertEqual(node.move.call_args_list[0].args,(approach,False,'HOVER_APPROACH'))
        self.assertEqual(node.move.call_args_list[1].args,(initial,False,'HOVER_RETURN'))

if __name__=='__main__':unittest.main()
