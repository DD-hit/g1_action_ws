"""Slow driver attestation must not reuse pre-scan task feedback."""
from types import SimpleNamespace
from unittest import TestCase,main
from unittest.mock import patch
from g1_actions.actions.elevator import task_node as module


class FeedbackBarrierTests(TestCase):
    def test_both_streams_must_advance_after_slow_verification(self):
        clock=[10.]
        status=SimpleNamespace(current_state=module.DualArmStatus.IDLE,
            current_joint=SimpleNamespace(left_velocities=[0.]*7,right_velocities=[0.]*7))
        task=SimpleNamespace(cfg={'motion':{'max_joint_speed':.1}},status=status,
            waist=SimpleNamespace(),status_arrival=10.,waist_arrival=10.)
        checked=[]
        def verify(_):clock[0]=11.
        def wait(predicate,*_):
            self.assertFalse(predicate())
            clock[0]=11.01;task.status_arrival=clock[0]
            self.assertFalse(predicate())  # One new arm frame cannot admit stale waist data.
            clock[0]=11.02;task.waist_arrival=clock[0]
            self.assertTrue(predicate())
        task.wait=wait;task.health=lambda:checked.append(True)
        with patch.object(module,'verify_driver_runtime',side_effect=verify),patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            module.PressTask.check_execution_ready(task)
        self.assertEqual(checked,[True])

    def test_missing_waist_still_times_out_before_health_or_motion(self):
        task=SimpleNamespace(cfg={'motion':{}},status=object(),waist=None,
            status_arrival=10.,waist_arrival=0.)
        def wait(predicate,*_):
            self.assertFalse(predicate())
            raise RuntimeError('fresh feedback timed out')
        task.wait=wait;task.health=lambda:self.fail('Must not pass readiness without both streams')
        with patch.object(module,'verify_driver_runtime'),patch.object(module.time,'monotonic',return_value=10.):
            with self.assertRaisesRegex(RuntimeError,'fresh feedback timed out'):
                module.PressTask.check_execution_ready(task)


if __name__=='__main__':main()
