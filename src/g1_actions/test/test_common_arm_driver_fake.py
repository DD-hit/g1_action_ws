"""Common planning/cache -> actual C++ executor, fake SDK only, domain 219."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


@unittest.skipUnless(os.environ.get('G1_COMMON_ARM_DRIVER_TEST') == '1',
                     'Opt in to isolated C++ fake SDK test')
class CommonArmDriverFakeTests(unittest.TestCase):
    def test_preplan_execute_and_cache_guards(self):
        os.environ.update(ROS_DOMAIN_ID='219', ROS_LOCALHOST_ONLY='1',
                          RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        import numpy as np
        import yaml
        import rclpy
        from brsd_msgs.msg import DualArmMoveJ, DualArmStatus
        from g1_actions.actions.handshake.approach_once import Approach
        package=Path(__file__).resolve().parents[1]
        profile=yaml.safe_load((package/'config/robot.yaml').read_text())
        pipe=profile['pipelines'][0]
        pipe.update(fake_mode=True,fake_has_upper_body_authority=True)
        pipe['unitree']['connect_on_init']=False
        driver=None
        with tempfile.TemporaryDirectory(prefix='g1_common_fake_') as folder:
            folder=Path(folder);config=folder/'robot.yaml'
            config.write_text(yaml.safe_dump(profile))
            rclpy.init()
            node=Approach(yaml.safe_load((package/'config/handshake.yaml').read_text()),package,'right',folder)
            def wait(predicate,timeout=30):
                until=time.monotonic()+timeout
                while time.monotonic()<until:
                    rclpy.spin_once(node,timeout_sec=.02)
                    if predicate():return
                self.fail('Timeout waiting for isolated fake driver')
            def ack(msg,phase):
                key=node.feedback_key(msg)
                wait(lambda:node.feedback.get(key,{}).get('phase')==phase,15)
                return node.feedback[key]
            def clear_error():
                node.cleaning_up=True
                try:node.cancel()
                finally:node.cleaning_up=False
                wait(lambda:node.status is not None and not node.status.left_has_error and not node.status.right_has_error)
            try:
                with (folder/'driver.log').open('w') as log:
                    driver=subprocess.Popen(['bash','/workspace/src/brsd_manipulation/src/brsd_upbody_control/script/run_driver.sh','unitree_g1','fake'],
                        env=dict(os.environ,CONFIG_FILE=str(config),UNITREE_SDK2_ROOT='/opt/unitree_sdk2',UNITREE_DDS_ROOT='/opt/unitree_sdk2'),
                        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    wait(lambda:node.status is not None and node.waist is not None,45)
                    pub=node.create_publisher(DualArmMoveJ,'/dual_arm/move_j',1)
                    wait(lambda:pub.get_subscription_count()==1)
                    setup=DualArmMoveJ(arm_mask=3,speed=.5,a=.5)
                    setup.joint.left_positions=[.2844,.1302,.0099,.9844,.0613,.0845,-.0472]
                    setup.joint.right_positions=[.2904,-.1219,.0129,.9785,-.0545,-.0249,-.0351]
                    pub.publish(setup)
                    wait(lambda:node.status.current_state==DualArmStatus.IDLE and
                         len(node.status.current_joint.right_positions)==7 and
                         max(abs(a-b) for a,b in zip(node.status.current_joint.right_positions,setup.joint.right_positions))<.01)
                    # A direct non-panel request must not silently replan.
                    bad=node.motion_request(node.pose());node.publisher.publish(bad)
                    self.assertIn('PREPARED_PLAN_INVALID:',ack(bad,'rejected')['reason'])
                    clear_error()
                    for arm in ('right','left'):
                        node.arm=arm
                        initial=node.pose()
                        target=list(initial);target[0]+=.02
                        before=np.array(list(node.status.current_joint.left_positions)+list(node.status.current_joint.right_positions))
                        node.owned=False
                        node.plan_motion(target)
                        expected=yaml.safe_load((package/'config/elevator.yaml').read_text())['motion']['stage_profiles']['APPROACH']
                        self.assertAlmostEqual(node.prepared_request.speed,expected['speed'],places=6)
                        self.assertAlmostEqual(node.prepared_request.a,expected['acceleration'],places=6)
                        key=node.feedback_key(node.prepared_request)
                        self.assertEqual(node.feedback[key]['phase'],'planned')
                        np.testing.assert_allclose(list(node.status.current_joint.left_positions)+list(node.status.current_joint.right_positions),before,atol=.001)
                        node.owned=True
                        node.move(target,'APPROACH')
                        self.assertTrue(node.close_to(target))
                        node.move(initial,'RETURN')
                        self.assertTrue(node.close_to(initial))
                    # Same stamp with a changed target consumes/rejects the cache.
                    node.plan_motion(node.pose())
                    changed=copy.deepcopy(node.prepared_request);changed.plan_only=False
                    changed.pose.left_x+=.01
                    node.publisher.publish(changed)
                    self.assertIn('request differs',ack(changed,'rejected')['reason'])
                    clear_error()
                    node.plan_motion(node.pose())
                    cancelled=copy.deepcopy(node.prepared_request);cancelled.plan_only=False
                    clear_error()  # Explicit cancellation revokes the prepared plan.
                    node.feedback.pop(node.feedback_key(cancelled),None)
                    node.publisher.publish(cancelled)
                    self.assertIn('PREPARED_PLAN_INVALID:',ack(cancelled,'rejected')['reason'])
                    text=(folder/'driver.log').read_text()
                    self.assertIn('[ArmTransitSelected]',text)
                    self.assertIn('[MotionPreparedExecute]',text)
                    self.assertIn('arm_velocity_limit=1.5 arm_acceleration_limit=2.5',text)
                    self.assertIn('exact_cached_trajectory=true replanning=false',text)
                    print('Fake SDK: both arms cached approach/return passed; request mutation and cancellation rejected; no hardware commands.',flush=True)
            finally:
                if driver is not None and driver.poll() is None:
                    os.killpg(driver.pid,signal.SIGINT)
                    try:driver.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
                output=package.parents[1]/'data/validation/common_arm_planning'
                output.mkdir(parents=True,exist_ok=True)
                if (folder/'driver.log').exists():(output/'driver.log').write_bytes((folder/'driver.log').read_bytes())
                node.journal.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
