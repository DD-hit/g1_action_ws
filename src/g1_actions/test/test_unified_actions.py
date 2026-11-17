"""Real ROS transports + existing elevator lifecycle; synthetic task, no SDK.

Runs only in localhost ROS domain 207. Never starts a camera or hardware driver.
"""
import os
os.environ.update(ROS_DOMAIN_ID='207', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')

import json
from pathlib import Path
import signal
import subprocess
import threading
import time
from types import SimpleNamespace
import unittest
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from g1_action_interfaces.action import Execute
from g1_action_interfaces.msg import ExecutionGoal, ExecutionResult as Result
from g1_action_interfaces.srv import GetExecution, ListActions, StopActive
from g1_actions.actions.elevator.service_node import TaskService, TaskCancelled
from g1_actions.server import ActionLibraryServer
from g1_actions.protocol import validate, InvalidRequest, map_result


class SyntheticTask:
    instances = []

    def __init__(self, cfg, direction, arm, stop_event):
        self.mode = cfg.get('test_mode', 'normal')
        self.direction = direction
        self.arm = 'right' if arm == 'auto' else arm
        self.stop_event = stop_event
        self.executing = False
        self.acquired = False
        self.destroyed = False
        self.status = SimpleNamespace(current_joint=SimpleNamespace(left_positions=[0.]*7, right_positions=[0.]*7))
        self.waist = SimpleNamespace(has_upper_body_authority=False)
        self.locked_target = {'point': [.4, -.1, .1]}
        self.event = lambda *args, **kwargs: None
        self.press_completion = {'termination': 'contact_time_limit', 'button_activation_verified': False}
        self.instances.append(self)

    def current_pose(self):
        return {'xyz': [.2, -.1, .1], 'rpy': [0., 0., 0.]}

    def wait(self, predicate, *args):
        assert predicate()

    def run(self, **kwargs):
        self.acquired = self.executing = True
        self.event('ACQUIRE_AUTHORITY')
        self.event('APPROACH')
        if self.mode == 'collision':
            raise RuntimeError('轨迹规划/接收失败：PILZ_PATH_COLLISION: Pilz LIN failed')
        self.event('PRESS')
        delay = 5. if self.mode in ('hold', 'slow_cleanup') else .3
        if self.stop_event.wait(delay):
            raise TaskCancelled(getattr(self.stop_event, 'reason', '用户请求停止'))
        self.event('WITHDRAW')
        self.executing = False
        self.event('READY_TO_RELEASE')

    def cancel_motion(self):
        self.executing = False

    def trigger(self, name):
        if self.mode == 'release_failed':
            raise RuntimeError('synthetic release failure')
        if self.mode == 'slow_cleanup':
            time.sleep(.5)

    def release_if_safely_withdrawn(self):
        self.acquired = False

    def destroy_node(self):
        self.destroyed = True


class SyntheticBackend(TaskService):
    def __init__(self):
        self.test_ready = True
        super().__init__({}, task_factory=SyntheticTask)
        self.waist = SimpleNamespace(has_upper_body_authority=False)

    def dependencies_ready(self):
        self.dependency_error = '' if self.test_ready else '模拟 RGB-D 尚未稳定'
        return self.test_ready


class UnifiedActionsTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        SyntheticTask.instances = []
        self.backend = SyntheticBackend()
        self.server = ActionLibraryServer(self.backend)
        self.client_node = Node('unified_acceptance_client')
        self.client = ActionClient(self.client_node, Execute, '/g1_actions/execute')
        self.executor = SingleThreadedExecutor()
        for node in [self.backend, self.server, self.client_node]:
            self.executor.add_node(node)
        self.assertTrue(self.client.wait_for_server(timeout_sec=3))

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail('Timed out waiting for synthetic test condition')

    def tearDown(self):
        self.server.begin_shutdown()
        self.wait(lambda: self.backend.worker is None)
        for _ in range(10):
            self.executor.spin_once(timeout_sec=.01)
        self.backend.close()
        self.client.destroy()
        self.server.destroy_node()
        self.backend.destroy_node()
        self.client_node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def goal(self, action='elevator', direction='up', **kwargs):
        request = ExecutionGoal(request_id=str(uuid.uuid4()), action_name=action,
                                arm=2, parameters_json=json.dumps({'direction': direction} if action == 'elevator' else {}))
        for key, value in kwargs.items():
            setattr(request, key, value)
        return request

    def send(self, request):
        future = self.client.send_goal_async(Execute.Goal(request=request))
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle

    def result(self, handle):
        future = handle.get_result_async()
        self.wait(future.done)
        return future.result()

    def invoke(self, cls, endpoint, request):
        client = self.client_node.create_client(cls, endpoint)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            future = client.call_async(request)
            self.wait(future.done)
            return future.result()
        finally:
            self.client_node.destroy_client(client)

    def test_up_and_down_complete_after_authority_release(self):
        for direction in ['up', 'down']:
            answer = self.result(self.send(self.goal(direction=direction))).result.result
            self.assertTrue(answer.success)
            self.assertEqual(answer.authority_state, Result.AUTHORITY_RELEASED)
            self.assertTrue(answer.motion_completed and answer.cleanup_completed)
            self.assertEqual(json.loads(answer.details_json)['direction'], direction)
        self.assertEqual([t.direction for t in SyntheticTask.instances], ['up', 'down'])

    def test_busy_and_duplicate_requests_do_not_create_a_second_task(self):
        self.backend.cfg['test_mode'] = 'hold'
        request = self.goal()
        handle = self.send(request)
        self.wait(lambda: bool(SyntheticTask.instances))
        self.assertEqual(self.result(self.send(self.goal())).result.result.code, Result.BUSY)
        self.assertEqual(self.result(self.send(request)).result.result.code, Result.DUPLICATE_REQUEST)
        future = handle.cancel_goal_async()
        self.wait(future.done)
        result = self.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.result.code, Result.CANCELLED)
        self.assertEqual(len(SyntheticTask.instances), 1)

    def test_explicit_left_arm_is_preserved(self):
        answer = self.result(self.send(self.goal(arm=1))).result.result
        self.assertTrue(answer.success)
        self.assertEqual(answer.selected_arm, 1)
        self.assertEqual(SyntheticTask.instances[-1].arm, 'left')

    def test_completed_request_can_be_queried_but_not_reexecuted(self):
        request = self.goal()
        original = self.result(self.send(request)).result.result
        self.assertEqual(self.result(self.send(request)).result.result.code, Result.DUPLICATE_REQUEST)
        found = self.invoke(GetExecution, '/g1_actions/get_execution', GetExecution.Request(request_id=request.request_id))
        self.assertTrue(found.found and found.terminal)
        self.assertEqual(found.result, original)
        self.assertEqual(len(SyntheticTask.instances), 1)

    def test_invalid_unimplemented_unknown_and_not_ready_do_not_move(self):
        cases = [(self.goal(parameters_json='{"direction":"down","speed":2}'), Result.INVALID_ARGUMENT),
                 (self.goal(action='handshake'), Result.NOT_IMPLEMENTED),
                 (self.goal(action='dance'), Result.UNKNOWN_ACTION),
                 (self.goal(api_version=2), Result.UNSUPPORTED_VERSION)]
        for request, code in cases:
            self.assertEqual(self.result(self.send(request)).result.result.code, code)
        self.backend.test_ready = False
        self.assertEqual(self.result(self.send(self.goal())).result.result.code, Result.NOT_READY)
        self.assertFalse(SyntheticTask.instances)

    def test_planning_collision_returns_failure_after_release(self):
        self.backend.cfg['test_mode'] = 'collision'
        answer = self.result(self.send(self.goal())).result.result
        self.assertFalse(answer.success)
        self.assertEqual(answer.code, Result.COLLISION)
        self.assertEqual(answer.authority_state, Result.AUTHORITY_RELEASED)

    def test_stop_service_waits_for_final_result(self):
        self.backend.cfg['test_mode'] = 'hold'
        request = self.goal()
        handle = self.send(request)
        self.wait(lambda: self.backend.task is not None)
        stop = self.invoke(StopActive, '/g1_actions/stop_active', StopActive.Request(request_id=request.request_id))
        self.assertTrue(stop.accepted)
        answer = self.result(handle)
        self.assertEqual(answer.result.result.code, Result.CANCELLED)
        self.assertEqual(answer.result.result.authority_state, Result.AUTHORITY_RELEASED)
        self.assertEqual(answer.status, GoalStatus.STATUS_CANCELED)

    def test_work_deadline_stops_and_releases(self):
        self.backend.cfg['test_mode'] = 'hold'
        answer = self.result(self.send(self.goal(timeout_s=1.))).result.result
        self.assertEqual(answer.code, Result.DEADLINE_EXCEEDED)
        self.assertEqual(answer.authority_state, Result.AUTHORITY_RELEASED)

    def test_release_failure_latches_fault_and_blocks_new_goals(self):
        self.backend.cfg['test_mode'] = 'release_failed'
        answer = self.result(self.send(self.goal())).result.result
        self.assertEqual(answer.code, Result.CLEANUP_FAILED)
        self.assertEqual(answer.authority_state, Result.AUTHORITY_RETAINED)
        self.assertEqual(self.result(self.send(self.goal())).result.result.code, Result.NOT_READY)
        self.assertEqual(len(SyntheticTask.instances), 1)

    def test_cleanup_watchdog_does_not_unlock_running_backend(self):
        self.server.cleanup_timeout_s = .1
        self.backend.cfg['test_mode'] = 'slow_cleanup'
        handle = self.send(self.goal())
        self.wait(lambda: self.backend.task is not None)
        cancelled = handle.cancel_goal_async()
        self.wait(cancelled.done)
        answer = self.result(handle).result.result
        self.assertEqual(answer.code, Result.CLEANUP_FAILED)
        self.assertFalse(answer.cleanup_completed)
        self.assertEqual(self.result(self.send(self.goal())).result.result.code, Result.NOT_READY)

    def test_legacy_start_and_stop_share_the_same_scheduler(self):
        self.backend.cfg['test_mode'] = 'hold'
        legacy = self.client_node.create_client(Trigger, '/elevator/start')
        self.assertTrue(legacy.wait_for_service(timeout_sec=3))
        future = legacy.call_async(Trigger.Request())
        self.wait(lambda: self.server.active is not None)
        self.assertEqual(self.result(self.send(self.goal())).result.result.code, Result.BUSY)
        self.invoke(Trigger, '/elevator/stop', Trigger.Request())
        self.wait(future.done)
        raw = json.loads(future.result().message)
        self.assertFalse(future.result().success)
        self.assertTrue(raw['authority_released'])
        self.assertIn(raw['request_id'], self.server.executions)
        self.client_node.destroy_client(legacy)

    def test_list_reports_elevator_ready_and_handshake_unimplemented(self):
        answer = self.invoke(ListActions, '/g1_actions/list_actions', ListActions.Request())
        self.assertTrue(answer.success)
        info = {x.name: x for x in answer.actions}
        self.assertTrue(info['elevator'].implemented and info['elevator'].ready)
        self.assertFalse(info['handshake'].implemented)

    def test_legacy_waiter_is_released_when_cleanup_watchdog_expires(self):
        self.server.cleanup_timeout_s = .1
        self.backend.cfg['test_mode'] = 'slow_cleanup'
        legacy = self.client_node.create_client(Trigger, '/elevator/start')
        self.assertTrue(legacy.wait_for_service(timeout_sec=3))
        future = legacy.call_async(Trigger.Request())
        self.wait(lambda: self.backend.task is not None)
        self.invoke(Trigger, '/elevator/stop', Trigger.Request())
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.assertEqual(json.loads(future.result().message)['result'], 'cleanup_failed')
        self.wait(lambda: self.backend.worker is None)
        self.assertTrue(self.server.fault)
        self.client_node.destroy_client(legacy)

    def test_cli_ctrl_c_requests_cancel_and_waits_for_release(self):
        root = Path(__file__).resolve().parents[3]
        self.backend.cfg['test_mode'] = 'hold'
        env = dict(os.environ, ELE_ROS_DOMAIN_ID='207')
        process = subprocess.Popen(['bash', str(root / 'scripts/g1.sh'), 'run', 'elevator', '--json'],
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.wait(lambda: self.backend.task is not None, timeout=15)
            process.send_signal(signal.SIGINT)
            self.wait(lambda: process.poll() is not None, timeout=15)
            stdout, stderr = process.communicate()
            self.assertEqual(process.returncode, 1, stderr)
            answer = json.loads(stdout)
            self.assertEqual(answer['code'], Result.CANCELLED)
            self.assertEqual(answer['authority_state'], Result.AUTHORITY_RELEASED)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)

    def test_cli_json_is_one_final_object(self):
        root = Path(__file__).resolve().parents[3]
        env = dict(os.environ, ELE_ROS_DOMAIN_ID='207')
        process = subprocess.Popen(['bash', str(root / 'scripts/g1.sh'), 'run', 'elevator', '--direction', 'down', '--json'],
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.wait(lambda: process.poll() is not None, timeout=15)
        stdout, stderr = process.communicate()
        self.assertEqual(process.returncode, 0, stderr)
        result = json.loads(stdout)
        self.assertTrue(result['success'])
        self.assertEqual(result['authority_state'], Result.AUTHORITY_RELEASED)
        self.assertEqual(len(stdout.strip().splitlines()), 1)


class ProtocolTests(unittest.TestCase):
    def test_missing_ik_candidates_is_not_a_proven_path_collision(self):
        request = ExecutionGoal(request_id=str(uuid.uuid4()), action_name='elevator')
        raw = dict(result='aborted', arm='right', authority_acquired=True, authority_released=True,
                   reason='ELEVATOR_CONTACT_UNREACHABLE: no transit posture with a valid complete press: no collision-free deterministic IK candidate')
        answer = map_result(request, raw, 1.)
        self.assertEqual(answer.code, Result.PLANNING_FAILED)
        self.assertIn('固定候选', answer.message)

    def test_json_rejects_duplicates_nonfinite_wrong_types_and_unknown_fields(self):
        for params in ['[]', '{"direction":"up","direction":"down"}', '{"direction":NaN}',
                       '{"direction":"up","arm":"right"}', '{"direction":true}', '"up"']:
            request = ExecutionGoal(request_id=str(uuid.uuid4()), action_name='elevator', parameters_json=params)
            with self.assertRaises(InvalidRequest):
                validate(request)

    def test_work_and_cleanup_flags_cannot_falsely_report_success(self):
        request = ExecutionGoal(request_id=str(uuid.uuid4()), action_name='elevator')
        raw = dict(result='completed', arm='right', authority_acquired=True, authority_released=True)
        self.assertTrue(map_result(request, raw, 1.).success)
        for modified in [dict(raw, authority_released=False), dict(raw, cancel_error='unconfirmed'),
                         dict(raw, authority_retained=True), dict(raw, authority_acquired=False)]:
            self.assertFalse(map_result(request, modified, 1.).success)


if __name__ == '__main__':
    unittest.main()
