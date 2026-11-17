"""Unified Action -> actual elevator -> C++ planner/executor -> fake SDK.

Opt-in only, no camera/video or hardware SDK connection. Domain 208 localhost.
"""
import os
import json
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid


@unittest.skipUnless(os.environ.get('ELE_TEST_DRIVER_INTEGRATION') == '1',
                     'Set ELE_TEST_DRIVER_INTEGRATION=1 to use the isolated C++ fake driver')
class UnifiedElevatorDriverFakeTests(unittest.TestCase):
    def test_up_and_down_through_unified_api(self):
        os.environ.update(ROS_DOMAIN_ID='208', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        import yaml
        import copy
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from brsd_msgs.msg import DualArmMoveJ, DualArmStatus
        from g1_action_interfaces.action import Execute
        from g1_action_interfaces.msg import ExecutionGoal, ExecutionResult
        from g1_actions.actions.elevator.config import load, camera_signature
        from g1_actions.actions.elevator.service_node import TaskService, ManagedPressTask
        from g1_actions.server import ActionLibraryServer
        from g1_actions.driver_runtime import verify_driver_runtime

        root = Path(__file__).resolve().parents[1]
        case_name = os.environ.get('G1_REPLAY_CASE', 'baseline')
        case = json.loads((root / 'test/fixtures/ik_replays.json').read_text())[case_name]
        cfg = load(root / 'config/elevator.yaml')
        cfg['motion'].update(require_business_driver=False, pause_inference_on_lock=False)
        driver_cfg = yaml.safe_load((root / 'config/robot.yaml').read_text())
        pipe = driver_cfg['pipelines'][0]
        pipe['fake_mode'] = True
        pipe['fake_has_upper_body_authority'] = True
        pipe['unitree']['connect_on_init'] = False
        rclpy.init()
        # C++ fake mode exposes fixed permission/weight telemetry. For this
        # fixture only, supply ownership state after successful REAL service
        # replies from its start/end endpoints. Production telemetry is intact.
        owner = [False]
        def ownership(message):
            value = copy.deepcopy(message)
            value.has_upper_body_authority = owner[0]
            return value
        class FakeOwnershipTask(ManagedPressTask):
            def on_waist(self, message):
                super().on_waist(ownership(message))
            def trigger(self, name):
                answer = super().trigger(name)
                if name in ('/dual_arm/start', '/dual_arm/end'):
                    owner[0] = name == '/dual_arm/start'
                return answer
        class FakeOwnershipBackend(TaskService):
            def on_waist(self, message):
                super().on_waist(ownership(message))
        backend = FakeOwnershipBackend(cfg, task_factory=FakeOwnershipTask)
        # Synthetic image stream: production motion/task code is used, while
        # the readiness of missing physical RGB-D services is supplied here.
        def ready():
            now = time.monotonic()
            ok = backend.status is not None and backend.waist is not None and now-backend.status_arrival < .6 and now-backend.waist_arrival < .6
            backend.dependency_error = '' if ok else '等待隔离的 fake SDK'
            return ok
        backend.dependencies_ready = ready
        server = ActionLibraryServer(backend)
        client_node = Node('unified_fake_driver_fixture')
        executor = SingleThreadedExecutor()
        for node in [backend, server, client_node]:
            executor.add_node(node)
        targets = client_node.create_publisher(String, '/elevator/targets', 1)
        sequence = [0]
        def publish():
            sequence[0] += 1
            stamp = client_node.get_clock().now().nanoseconds*1e-9
            targets.publish(String(data=json.dumps(dict(sequence=sequence[0], stamp_sec=stamp,
                camera_verified=bool(cfg['camera']['verified']), camera_calibration_id=camera_signature(cfg),
                image_width=640, targets=[dict(direction='up', point=case['point'],
                normal=case['normal'], center_xy=[430.,173.],
                stamp_sec=stamp, frame_id='torso_link')]))))
        timer = client_node.create_timer(.1, publish)
        action = ActionClient(client_node, Execute, '/g1_actions/execute')
        def wait(predicate, timeout=15):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
                if predicate():
                    return
            self.fail('Timed out in isolated unified fake-driver test')
        def trigger(endpoint):
            client = client_node.create_client(Trigger, endpoint)
            try:
                self.assertTrue(client.wait_for_service(timeout_sec=5))
                future = client.call_async(Trigger.Request())
                wait(future.done)
                self.assertTrue(future.result().success, future.result().message)
                owner[0] = endpoint == '/dual_arm/start'
            finally:
                client_node.destroy_client(client)

        label = case_name + ('_repeat' if os.environ.get('G1_REPLAY_REPEAT') == '1' else '')
        output = root.parents[1] / 'data/validation/unified_driver_fake' / label
        output.mkdir(parents=True, exist_ok=True)
        results = []
        driver = None
        try:
            with tempfile.TemporaryDirectory(prefix='g1_unified_fake_') as directory, (output/'driver.log').open('w') as log:
                config = Path(directory)/'driver.yaml'
                config.write_text(yaml.safe_dump(driver_cfg))
                env = dict(os.environ, CONFIG_FILE=str(config), BRSD_UPBODY_MODE='fake',
                           UNITREE_SDK2_ROOT='/opt/unitree_sdk2', UNITREE_DDS_ROOT='/opt/unitree_sdk2')
                driver = subprocess.Popen(['bash','/workspace/src/brsd_manipulation/src/brsd_upbody_control/script/run_driver.sh','unitree_g1','fake'],
                    env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                wait(lambda: backend.status is not None and backend.waist is not None, 45)
                verify_driver_runtime(dict(require_business_driver=True, driver_config=str(config),
                    driver_install_prefix='/workspace/install/brsd_upbody_control'))
                trigger('/dual_arm/start')
                wait(lambda:backend.waist.has_upper_body_authority)
                pub = client_node.create_publisher(DualArmMoveJ, '/dual_arm/move_j', 1)
                wait(lambda:pub.get_subscription_count()==1)
                setup = DualArmMoveJ(arm_mask=3, speed=.5, a=.5)
                setup.joint.left_positions=[.2844,.1302,.0099,.9844,.0613,.0845,-.0472]
                setup.joint.right_positions=case['initial_right']
                pub.publish(setup)
                wait(lambda:backend.status.current_state==DualArmStatus.IDLE and
                    max(abs(a-b) for a,b in zip(backend.status.current_joint.right_positions,setup.joint.right_positions))<.01)
                trigger('/dual_arm/end')
                wait(lambda:not backend.waist.has_upper_body_authority)
                self.assertTrue(action.wait_for_server(timeout_sec=5))
                for direction in case['directions']:
                    future = action.send_goal_async(Execute.Goal(request=ExecutionGoal(request_id=str(uuid.uuid4()),
                        action_name='elevator', arm=2, parameters_json=json.dumps({'direction':direction}))))
                    wait(future.done)
                    handle = future.result()
                    self.assertTrue(handle.accepted)
                    result = handle.get_result_async()
                    wait(result.done, 75)
                    value = result.result().result.result
                    from rosidl_runtime_py.convert import message_to_ordereddict
                    results.append(message_to_ordereddict(value))
                    self.assertTrue(value.success, value.message+' '+value.details_json)
                    self.assertEqual(value.authority_state, ExecutionResult.AUTHORITY_RELEASED)
                    self.assertTrue(value.motion_completed and value.cleanup_completed)
                text=(output/'driver.log').read_text()
                self.assertIn('[RecordedRetraceAccepted]',text)
                self.assertIn('[ElevatorContactLookahead]',text)
                chosen = re.findall(r'\[ElevatorJointGoal\].*positions=\[([^]]+)\].*seed_phase=(\w+)', text)
                self.assertEqual(len(chosen), len(case['directions']))
                self.assertTrue(all(phase == case['expected_seed_phase'] for _, phase in chosen), chosen)
        finally:
            server.begin_shutdown()
            try:
                wait(lambda:backend.worker is None, 15)
                backend.close()
            finally:
                if driver is not None and driver.poll() is None:
                    os.killpg(driver.pid,signal.SIGINT)
                    try:driver.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
                (output/'results.json').write_text(json.dumps(dict(hardware_commands_sent=False, ros_domain=208,
                    case=case_name, expected_seed_phase=case['expected_seed_phase'],
                    driver_prefix='/workspace/install/brsd_upbody_control', results=results),ensure_ascii=False,indent=2))
                action.destroy()
                server.destroy_node()
                backend.destroy_node()
                client_node.destroy_node()
                executor.shutdown()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
