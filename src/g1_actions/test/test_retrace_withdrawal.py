"""Retraction uses the cached joint path; never submits a second MoveL."""
import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from g1_actions.actions.elevator.task_node import PressTask
from brsd_msgs.msg import DualArmStatus

class RetraceWithdrawalTests(unittest.TestCase):
    def fixture(self):
        task=SimpleNamespace(arm='right',locked_target={'normal':[-1.,0.,0.]},
            status_sequence=0,cfg={'motion':{'command_timeout_s':2.}},
            hover_guard=SimpleNamespace(begin_withdrawal=Mock()),health=Mock(),
            configure_panel=Mock(),event=Mock(),verify_withdrawal=Mock(),move=Mock())
        task.status=SimpleNamespace(current_state=DualArmStatus.IDLE,
            current_joint=SimpleNamespace(right_positions=[0.]*7))
        task.trigger=Mock(return_value=SimpleNamespace(message=json.dumps({'arm_mask':2,'target_positions':[0.]*17})))
        return task
    def test_withdrawal_calls_retrace_and_confirms_fresh_measured_endpoint(self):
        task=self.fixture();now=[0.]
        def spin(*args,**kwargs):now[0]+=.15;task.status_sequence+=1
        with patch('g1_actions.actions.elevator.task_node.time.monotonic',side_effect=lambda:now[0]),patch('g1_actions.actions.elevator.task_node.rclpy.spin_once',side_effect=spin):
            PressTask.withdraw_contact(task)
        task.trigger.assert_called_once_with('/dual_arm/retrace_right')
        task.move.assert_not_called();task.verify_withdrawal.assert_called_once()
        task.configure_panel.assert_called_once_with(task.locked_target,'withdraw')
    def test_missing_cache_never_falls_back_to_new_cartesian_plan(self):
        task=self.fixture();task.trigger.side_effect=RuntimeError('no matching accepted linear trajectory')
        with self.assertRaisesRegex(RuntimeError,'no matching'):
            PressTask.withdraw_contact(task)
        task.move.assert_not_called();task.verify_withdrawal.assert_not_called()
    def test_wrong_arm_acknowledgement_is_rejected(self):
        task=self.fixture();task.trigger.return_value.message=json.dumps({'arm_mask':1,'target_positions':[0.]*17})
        with self.assertRaisesRegex(RuntimeError,'Invalid recorded-retrace'):
            PressTask.withdraw_contact(task)
        task.verify_withdrawal.assert_not_called()
if __name__=='__main__':unittest.main()
