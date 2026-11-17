"""Start/stop state, cancellation and RPC tests in an isolated ROS domain."""
import os
os.environ.update(ROS_DOMAIN_ID='199',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
import threading
import time
from types import SimpleNamespace
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from std_msgs.msg import String
from g1_actions.actions.elevator.service_node import TaskService,TaskCancelled
from g1_actions.actions.elevator.task_node import PressTask


class FakeTask:
    instances=[]
    def __init__(self,cfg,direction,arm,stop_event):
        self.stop_event=stop_event;self.executing=False;self.acquired=False
        self.status=object();self.waist=SimpleNamespace(has_upper_body_authority=False)
        self.cancelled=False;self.released=False;self.destroyed=False
        self.event=lambda *a,**k:None
        self.instances.append(self)
    def wait(self,predicate,*args):assert predicate()
    def run(self,**kwargs):
        self.executing=True;self.acquired=True
        self.stop_event.wait(3)
        if self.stop_event.is_set():raise TaskCancelled('stopped')
    def cancel_motion(self):self.cancelled=True
    def trigger(self,name):self.released=name=='/dual_arm/end'
    def destroy_node(self):self.destroyed=True


class TaskServiceTests(unittest.TestCase):
    def test_exit_rechecks_safe_release_without_repeated_cancellation(self):
        from unittest.mock import Mock
        task=SimpleNamespace(release_if_safely_withdrawn=Mock(side_effect=RuntimeError('clearance below 40 mm')),destroy_node=Mock())
        self.node.retained_task=task;self.node.last_result={'authority_retained':True}
        original=self.node.stop;self.node.stop=Mock(wraps=original)
        with self.assertRaisesRegex(RuntimeError,'clearance below 40'):self.node.close()
        self.assertIs(self.node.retained_task,task);task.destroy_node.assert_not_called()
        task.release_if_safely_withdrawn.side_effect=None
        self.node.close()
        self.node.stop.assert_called_once()
        task.destroy_node.assert_called_once()
        self.assertTrue(self.node.last_result['authority_released'])
        self.assertFalse(self.node.last_result['authority_retained'])
        self.node.close();self.node.stop.assert_called_once()

    def test_exit_retries_automatically_without_another_ctrl_c(self):
        import ast
        from pathlib import Path
        from unittest.mock import Mock
        tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/serve.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        cleanup=next(n for n in main.body if isinstance(n,ast.Try) and n.finalbody)
        retry=next(n for n in cleanup.finalbody if isinstance(n,ast.While))
        assignments=[n for n in cleanup.finalbody if isinstance(n,ast.Assign)]
        clock=[0.];calls=[]
        def close():
            calls.append('close')
            if clock[0]<.2:raise RuntimeError('waiting for fresh safe state')
        def spin(**kwargs):clock[0]+=.1
        env={'node':SimpleNamespace(close=close),'executor':SimpleNamespace(spin_once=spin),
             'stopping':[True],'time':SimpleNamespace(monotonic=lambda:clock[0]),'rclpy':SimpleNamespace(ok=lambda:True)}
        exec(compile(ast.Module(body=assignments+[retry],type_ignores=[]),'shutdown-retry-test','exec'),env)
        self.assertEqual(calls,['close','close'])

    def setUp(self):
        rclpy.init();FakeTask.instances=[]
        self.node=TaskService({},task_factory=FakeTask)
        self.node.waist=SimpleNamespace(has_upper_body_authority=False)
    def tearDown(self):
        self.node.close();self.node.destroy_node();rclpy.shutdown()
    def start(self):return self.node.start(Trigger.Request(),Trigger.Response())
    def stop(self):return self.node.stop(Trigger.Request(),Trigger.Response())
    def core_ready(self):
        self.node.status=object()
        self.node.status_arrival=self.node.waist_arrival=time.monotonic()
        self.node.count_publishers=lambda topic:1
        self.node.resume_client=SimpleNamespace(service_is_ready=lambda:True)
        self.node.freeze_client=SimpleNamespace(service_is_ready=lambda:True)
    def make_ready(self):
        self.core_ready()
        now=time.monotonic();wall=self.node.get_clock().now().nanoseconds*1e-9
        for i in range(15):
            ago=(14-i)*.15
            self.node.vision_gate.observe({'sequence':i+1,'stamp_sec':wall-ago-.1},
                                          now-ago,wall-ago)
        self.node.ready=True
    def finished(self):
        until=time.monotonic()+3
        while self.node.worker is not None and time.monotonic()<until:time.sleep(.01)
        self.assertIsNone(self.node.worker)
    def test_services_without_fresh_rgbd_are_not_ready(self):
        self.core_ready()
        self.assertFalse(self.node.dependencies_ready())
        self.assertIn('视觉未就绪',self.node.dependency_error)
        self.make_ready()
        self.assertTrue(self.node.dependencies_ready())

    def test_cached_ready_does_not_admit_a_task_after_camera_error(self):
        self.make_ready()
        self.node.on_vision(String(data='{"reason":"camera frames stopped"}'))
        self.assertTrue(self.node.ready)  # Timer has not run yet.
        self.assertFalse(self.start().success)
        self.assertEqual(FakeTask.instances,[])
        self.assertIn('camera frames stopped',self.node.dependency_error)

    def test_camera_restart_blocks_start_until_requalified(self):
        self.make_ready();self.node.perception_recovering=True
        self.assertFalse(self.start().success)
        self.assertEqual(FakeTask.instances,[])
        self.assertIn('自动重启',self.node.dependency_error)
        self.node.perception_recovering=False
        self.node.vision_gate.invalidate('相机已重启')
        self.assertFalse(self.node.refresh_readiness())
        from unittest.mock import patch
        now=time.monotonic()+3
        wall=self.node.get_clock().now().nanoseconds*1e-9+3
        clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(wall*1e9)))
        with patch('g1_actions.actions.elevator.service_node.time.monotonic',return_value=now),patch.object(self.node,'get_clock',return_value=clock):
            self.make_ready();self.assertTrue(self.node.refresh_readiness())

    def test_cached_ready_does_not_admit_a_task_after_feedback_expires(self):
        self.make_ready()
        self.node.status_arrival=time.monotonic()-1.
        self.assertFalse(self.start().success)
        self.assertEqual(FakeTask.instances,[])

    def test_camera_loss_aborts_an_unlocked_job(self):
        self.make_ready()
        self.node.worker=object()
        self.node.task=SimpleNamespace(locked_target=None)
        try:
            self.node.on_vision(String(data='{"reason":"camera frames stopped"}'))
            self.node.monitor()
            self.assertTrue(self.node.stop_event.is_set())
        finally:self.node.worker=None;self.node.task=None

    def test_locked_snapshot_keeps_existing_pause_behavior(self):
        self.core_ready()
        self.node.task=SimpleNamespace(locked_target={'stamp_sec':1.})
        try:self.assertTrue(self.node.dependencies_ready())
        finally:self.node.task=None

    def test_malformed_vision_clears_readiness(self):
        self.make_ready()
        self.node.on_vision(String(data='[]'))
        self.assertFalse(self.start().success)

    def test_waiting_job_does_not_hide_camera_loss(self):
        self.node.status=object()
        self.node.status_arrival=self.node.waist_arrival=time.monotonic()
        self.node.count_publishers=lambda topic:1
        self.node.resume_client=SimpleNamespace(service_is_ready=lambda:True)
        self.node.freeze_client=SimpleNamespace(service_is_ready=lambda:True)
        self.node.worker=object()
        self.node.task=SimpleNamespace(locked_target=None)
        try:self.assertFalse(self.node.dependencies_ready())
        finally:self.node.worker=None;self.node.task=None

    def test_not_ready_does_not_start(self):
        self.assertFalse(self.start().success);self.assertEqual(FakeTask.instances,[])

    def test_target_rejection_reports_direction_specific_depth_reason(self):
        task=SimpleNamespace(locked_target=None,cfg={'motion':{'max_target_age_s':.8}},
            targets={'targets':[], 'target_status':{
                'down':{'ready':False,'reason':'button depth inconsistent with panel'}}},
            target_arrival=time.monotonic(),direction='down',
            get_clock=self.node.get_clock)
        with self.assertRaisesRegex(RuntimeError,'下行目标未就绪：button depth inconsistent with panel'):
            PressTask.target(task)
    def test_duplicate_start_rejected_and_stop_releases(self):
        self.make_ready()
        self.assertTrue(self.start().success)
        until=time.monotonic()+1
        while not FakeTask.instances and time.monotonic()<until:time.sleep(.01)
        self.assertFalse(self.start().success)
        self.assertTrue(self.stop().success);self.finished()
        task=FakeTask.instances[0]
        self.assertTrue(task.cancelled and task.released and task.destroyed)
        self.assertEqual(self.node.last_result['result'],'stopped')
        self.assertTrue(self.stop().success)
    def test_sync_rpc_remains_pending_and_stop_is_responsive(self):
        from rclpy.executors import SingleThreadedExecutor
        import json
        self.make_ready()
        self.node.refresh_readiness=lambda:True
        executor=SingleThreadedExecutor();executor.add_node(self.node)
        client=Node('sync_contract_test');executor.add_node(client)
        start=client.create_client(Trigger,'/elevator/start')
        stop=client.create_client(Trigger,'/elevator/stop')
        try:
            self.assertTrue(start.wait_for_service(timeout_sec=2))
            pending=start.call_async(Trigger.Request())
            until=time.monotonic()+2
            while not FakeTask.instances and time.monotonic()<until:executor.spin_once(timeout_sec=.02)
            self.assertTrue(FakeTask.instances)
            self.assertFalse(pending.done())
            duplicate=start.call_async(Trigger.Request())
            executor.spin_until_future_complete(duplicate,timeout_sec=2)
            self.assertTrue(duplicate.done());self.assertFalse(duplicate.result().success)
            stopping=stop.call_async(Trigger.Request())
            executor.spin_until_future_complete(stopping,timeout_sec=2)
            self.assertTrue(stopping.done());self.assertTrue(stopping.result().success)
            executor.spin_until_future_complete(pending,timeout_sec=3)
            self.assertTrue(pending.done());self.assertFalse(pending.result().success)
            result=json.loads(pending.result().message)
            self.assertEqual(result['result'],'stopped')
            self.assertTrue(result['authority_released'])
        finally:
            executor.remove_node(client);client.destroy_node()
            executor.remove_node(self.node);executor.shutdown()

    def test_sync_success_is_returned_only_after_release(self):
        from rclpy.executors import SingleThreadedExecutor
        import json
        released=threading.Event()
        class SuccessTask(FakeTask):
            def run(self,**kwargs):self.executing=True;self.acquired=True
            def trigger(self,name):
                time.sleep(.05)
                released.set()
        self.make_ready();self.node.refresh_readiness=lambda:True
        self.node.task_factory=SuccessTask
        executor=SingleThreadedExecutor();executor.add_node(self.node)
        client=Node('sync_success_test');executor.add_node(client)
        rpc=client.create_client(Trigger,'/elevator/start')
        try:
            self.assertTrue(rpc.wait_for_service(timeout_sec=2))
            pending=rpc.call_async(Trigger.Request())
            executor.spin_until_future_complete(pending,timeout_sec=3)
            self.assertTrue(pending.done());self.assertTrue(pending.result().success)
            self.assertTrue(released.is_set())
            self.assertTrue(json.loads(pending.result().message)['authority_released'])
        finally:
            executor.remove_node(client);client.destroy_node()
            executor.remove_node(self.node);executor.shutdown()

    def test_contact_failure_cancels_and_releases_without_recovery(self):
        calls=[]
        class ContactTask(FakeTask):
            def run(self,**kwargs):
                self.executing=self.acquired=self.contact_pending=True
                raise RuntimeError('Contact travel timed out')
            def cancel_motion(self):calls.append('cancel')
            def recover_contact(self):
                calls.append('withdraw_verified');self.contact_pending=False
            def trigger(self,name):calls.append(name)
        self.node.task_factory=ContactTask
        self.node.run_task('down','right',threading.Event())
        self.assertEqual(calls,['cancel','/dual_arm/end'])
        self.assertTrue(self.node.last_result['authority_released'])

    def test_rejected_lin_without_cached_path_releases_authority(self):
        class RejectedTask(FakeTask):
            def run(self,**kwargs):
                self.executing=True;self.acquired=True;self.contact_pending=True
                raise RuntimeError('Driver reports error: Pilz LIN failed: PLANNING_FAILED (code -1)')
            def recover_contact(self):
                raise RuntimeError('/dual_arm/retrace_right: no matching accepted linear trajectory to retrace')
        self.node.task_factory=RejectedTask
        self.node.record=lambda *a,**k:None
        self.node.run_task('up','right',threading.Event())
        task=FakeTask.instances[-1]
        self.assertTrue(task.cancelled)
        self.assertTrue(task.released)
        self.assertTrue(task.destroyed)
        self.assertEqual(self.node.last_result['result'],'aborted')
        self.assertTrue(self.node.last_result['authority_released'])
        self.assertFalse(self.node.last_result['authority_retained'])

    def test_release_failure_reports_retained_authority(self):
        calls=[]
        class ContactTask(FakeTask):
            def run(self,**kwargs):
                self.executing=self.acquired=self.contact_pending=True
                raise RuntimeError('Contact travel timed out')
            def cancel_motion(self):calls.append('cancel')
            def recover_contact(self):raise RuntimeError('Driver safety status forbids motion')
            def trigger(self,name):
                calls.append(name)
                raise RuntimeError('release RPC failed')
        self.node.task_factory=ContactTask
        self.node.run_task('down','right',threading.Event())
        self.assertIn('/dual_arm/end',calls)
        self.assertTrue(self.node.last_result['authority_retained'])
        self.assertEqual(self.node.last_result['result'],'cleanup_failed')
        # Test does not own real authority; clear the marker before tearDown.
        self.node.last_result=None

    def test_retained_authority_blocks_ready_and_start(self):
        self.make_ready()
        self.node.last_result={'authority_retained':True}
        self.node.waist.has_upper_body_authority=True
        self.assertFalse(self.node.refresh_readiness())
        self.assertEqual(self.node.stage,'RECOVERY_REQUIRED')
        self.assertFalse(self.stop().success)
        self.assertFalse(self.start().success)
        self.assertFalse(FakeTask.instances)
        # Resolved only by fresh feedback confirming release, not by a stale
        # false ownership sample from before the failed task.
        self.node.waist.has_upper_body_authority=False
        self.node.waist_arrival=time.monotonic()-1
        self.assertFalse(self.node.refresh_readiness())
        self.node.waist_arrival=time.monotonic()
        self.assertTrue(self.node.refresh_readiness())
        self.assertTrue(self.node.last_result['recovery_resolved'])

    def test_invalid_direction_rejected(self):
        self.make_ready()
        self.node.set_parameters([Parameter('direction',value='invalid')])
        self.assertFalse(self.start().success)
        self.assertEqual(FakeTask.instances,[])
    def test_external_authority_not_taken_over(self):
        self.make_ready();self.node.waist.has_upper_body_authority=True
        self.assertFalse(self.start().success)
    def test_worker_start_failure_returns_to_idle(self):
        def fail(*args):raise ValueError('factory failed')
        self.node.task_factory=fail;self.make_ready()
        self.assertTrue(self.start().success);self.finished()
        self.assertEqual(self.node.last_result['result'],'aborted')
    def test_ros_rpc_stop_remains_responsive_during_job(self):
        self.node.dependencies_ready=lambda:True;self.node.ready=True
        executor=SingleThreadedExecutor();executor.add_node(self.node)
        thread=threading.Thread(target=executor.spin);thread.start()
        client=Node('task_service_test_client')
        try:
            def call(name):
                service=client.create_client(Trigger,name)
                self.assertTrue(service.wait_for_service(timeout_sec=3))
                future=service.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(client,future,timeout_sec=3)
                self.assertTrue(future.done());answer=future.result()
                client.destroy_client(service);return answer
            starting=client.create_client(Trigger,'/elevator/start')
            self.assertTrue(starting.wait_for_service(timeout_sec=3))
            pending=starting.call_async(Trigger.Request())
            until=time.monotonic()+1
            while not FakeTask.instances and time.monotonic()<until:
                rclpy.spin_once(client,timeout_sec=.02)
            self.assertTrue(FakeTask.instances);self.assertFalse(pending.done())
            started=time.monotonic()
            self.assertTrue(call('/elevator/stop').success)
            self.assertLess(time.monotonic()-started,1.)
            rclpy.spin_until_future_complete(client,pending,timeout_sec=3)
            self.assertTrue(pending.done());self.assertFalse(pending.result().success)
            client.destroy_client(starting)
            self.finished()
        finally:
            client.destroy_node();executor.shutdown(timeout_sec=3);thread.join(3)


if __name__=='__main__':unittest.main(verbosity=2)
