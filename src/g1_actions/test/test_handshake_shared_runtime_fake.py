"""Persistent provider -> real C++ driver; isolated fake SDK, never hardware."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get('G1_SHARED_HANDSHAKE_TEST')=='1','Opt in to isolated fake SDK integration')
class SharedRuntimeTests(unittest.TestCase):
    def test_warm_execution_and_cancel_without_vision_after_lock(self):
        os.environ.update(ROS_DOMAIN_ID='220',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        import numpy as np
        import yaml
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from brsd_msgs.msg import DualArmMoveJ, DualArmStatus
        from g1_actions.actions.handshake import provider as module
        from g1_actions.actions.handshake.transforms import camera_transform
        from g1_actions.server import Completion
        package=Path(__file__).resolve().parents[1]
        cfg=yaml.safe_load((package/'config/handshake.yaml').read_text())
        profile=yaml.safe_load((package/'config/robot.yaml').read_text())
        pipe=profile['pipelines'][0];pipe.update(fake_mode=True,fake_has_upper_body_authority=True)
        pipe['unitree']['connect_on_init']=False
        owner=[False];vision_enabled=[True];cancel_hold=[False];events=[]
        class Provider(module.HandshakeProvider):
            def on_waist(self,msg):
                msg=copy.deepcopy(msg);msg.has_upper_body_authority=owner[0];msg.authority_weight=1. if owner[0] else 0.
                super().on_waist(msg)
        class Approach(module.Approach):
            def trigger(self,name):
                result=super().trigger(name)
                if name in ('/dual_arm/start','/dual_arm/end'):owner[0]=name=='/dual_arm/start'
                return result
        rclpy.init();driver=None
        with tempfile.TemporaryDirectory(prefix='g1_shared_fake_') as tmp:
            folder=Path(tmp);config=folder/'robot.yaml';config.write_text(yaml.safe_dump(profile))
            provider=Provider(package.parents[1],folder/'trials');fixture=Node('shared_runtime_fixture')
            executor=SingleThreadedExecutor();executor.add_node(provider);executor.add_node(fixture)
            pub=fixture.create_publisher(DualArmMoveJ,'/dual_arm/move_j',1)
            vision=fixture.create_publisher(String,'/handshake/perception_status',1)
            optical=(np.linalg.inv(camera_transform(cfg,package))@np.array([.35,-.12,.16,1.]))[:3].tolist()
            def publish():
                stamp=fixture.get_clock().now().nanoseconds*1e-9
                payload=dict(stamp_s=stamp,frame_id=cfg['frames']['camera_optical'],palms=[dict(
                    handedness='Right',geometry_valid=True,stable=True,pose=dict(position_m=optical))]) if vision_enabled[0] else dict(valid=False,reason='camera disconnected')
                vision.publish(String(data=json.dumps(payload)))
            fixture.create_timer(.1,publish)
            def event(value):
                events.append(value)
                if value['stage']=='TARGET_LOCKED':vision_enabled[0]=False
                if value['stage']=='HOLD' and cancel_hold[0]:provider.stop_event.set()
            provider.action_event_callback=event
            def wait(predicate,timeout=40):
                until=time.monotonic()+timeout
                while time.monotonic()<until:
                    executor.spin_once(timeout_sec=.01)
                    if predicate():return
                self.fail('Timeout in shared runtime fixture; events='+str(events[-5:]))
            try:
                with (folder/'driver.log').open('w') as log:
                    driver=subprocess.Popen(['bash','/workspace/src/brsd_manipulation/src/brsd_upbody_control/script/run_driver.sh','unitree_g1','fake'],
                        env=dict(os.environ,CONFIG_FILE=str(config),UNITREE_SDK2_ROOT='/opt/unitree_sdk2',UNITREE_DDS_ROOT='/opt/unitree_sdk2'),
                        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    wait(lambda:provider.status is not None and provider.waist is not None and pub.get_subscription_count()==1)
                    setup=DualArmMoveJ(arm_mask=3,speed=.5,a=.5)
                    setup.joint.left_positions=[.2844,.1302,.0099,.9844,.0613,.0845,-.0472]
                    setup.joint.right_positions=[.2904,-.1219,.0129,.9785,-.0545,-.0249,-.0351]
                    pub.publish(setup)
                    wait(lambda:provider.status.current_state==DualArmStatus.IDLE and
                         max(abs(a-b) for a,b in zip(provider.status.current_joint.right_positions,setup.joint.right_positions))<.01)
                    for cancel in (False,True):
                        vision_enabled[0]=True;cancel_hold[0]=cancel;events.clear();received=[]
                        wait(lambda:provider.refresh_readiness() and provider.vision and provider.vision.get('palms'))
                        # Only bypass production process-path attestation for this temporary
                        # fake config. Actual driver parameter verification remains active.
                        with patch.object(module,'Approach',Approach),patch('g1_actions.driver_runtime.verify_driver_runtime'):
                            started=time.monotonic()
                            accepted,reason=provider.start('auto',dict(hold_s=.25,subject_handedness='any'),Completion(received.append),request_id='shared-fake')
                            self.assertTrue(accepted,reason);wait(lambda:bool(received),50)
                        result=received[0]
                        self.assertEqual(result['result'],'stopped' if cancel else 'completed',result)
                        self.assertTrue(result['authority_released'],result);self.assertFalse(owner[0])
                        self.assertEqual(result['arm'],'right')
                        approach=next(item for item in events if item['stage']=='APPROACH')
                        print('Warm fake request -> APPROACH: %.3f s; total %.3f s; cancelled=%s; camera disabled after lock' %
                              (approach['task_elapsed_s'],time.monotonic()-started,cancel),flush=True)
                    self.assertIn('[FullBodyPreflight]',(folder/'driver.log').read_text())
            finally:
                if driver is not None and driver.poll() is None:
                    os.killpg(driver.pid,signal.SIGINT)
                    try:driver.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
                output=Path('/tmp/g1-shared-fake-driver.log')
                if (folder/'driver.log').exists():output.write_bytes((folder/'driver.log').read_bytes())
                executor.shutdown();provider.destroy_node();fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
