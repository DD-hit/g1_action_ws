"""Isolated ROS integration tests. No Unitree SDK or hardware publishers."""
import os
os.environ['ROS_DOMAIN_ID']='188'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import copy
import json
from pathlib import Path
import subprocess
import signal
import tempfile
import threading
import time
import unittest
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from brsd_msgs.msg import DualArmStatus, WaistStatus, DualArmMovePJ, DualArmMoveL, DualArmMotionSequence
from g1_actions.actions.elevator.config import camera_signature
from g1_actions.actions.elevator.task_node import driver_config_match

ROOT=Path(__file__).resolve().parents[1]


class MockRobot(Node):
    def __init__(self):
        super().__init__('elevator_mock_robot')
        self.authority=False
        self.moves=[];self.cancels=0;self.starts=0;self.ends=0;self.events=[]
        self.arm_masks=[];self.target_center=[160.,240.];self.shift_image_after_move=False
        self.target_point=[.35,.20,.15]
        self.waist_change_on_authority=0.
        self.waist_feedback_override=None
        self.vision_paused=False;self.freeze_calls=0;self.resume_calls=0
        self.honor_freeze=True;self.change_target_on_move=False
        self.fail_move=0
        self.block_move=0
        self.blocked=False
        self.fail=False
        self.target_enabled=True
        self.camera_id=''
        self.complete_at=0
        self.pose=[.20,.20,.10,0.,0.,0.]
        self.goal=self.pose[:]
        self.stat=self.create_publisher(DualArmStatus,'/dual_arm/status',10)
        self.waist=self.create_publisher(WaistStatus,'/dual_arm/waist_status',10)
        self.targets=self.create_publisher(String,'/elevator/targets',1)
        self.create_subscription(DualArmMovePJ,'/dual_arm/move_pj',self.move,10)
        self.create_subscription(DualArmMoveL,'/dual_arm/move_l',self.move,10)
        self.create_subscription(DualArmMotionSequence,'/dual_arm/motion_sequence',self.cancel,10)
        self.create_subscription(String,'/elevator/task_status',lambda m:self.events.append(json.loads(m.data)),10)
        self.create_service(Trigger,'/dual_arm/start',self.start)
        self.create_service(Trigger,'/dual_arm/end',self.end)
        self.create_service(Trigger,'/elevator/vision/freeze',self.freeze)
        self.create_service(Trigger,'/elevator/vision/resume',self.resume)
        self.create_timer(.04,self.tick)

    def start(self,request,response):
        self.starts+=1;self.authority=True
        response.success=True;return response

    def freeze(self,request,response):
        self.freeze_calls+=1;self.vision_paused=self.honor_freeze
        response.success=True;return response

    def resume(self,request,response):
        self.resume_calls+=1;self.vision_paused=False
        response.success=True;return response

    def end(self,request,response):
        self.ends+=1;self.authority=False
        response.success=True;return response

    def move(self,message):
        arm='left' if message.arm_mask==1 else 'right'
        self.goal=[getattr(message.pose,arm+'_'+k) for k in ['x','y','z','rx','ry','rz']]
        self.moves.append(self.goal)
        self.arm_masks.append(message.arm_mask)
        if self.shift_image_after_move:self.target_center=[160.,240.]
        if self.change_target_on_move:self.target_point=[.8,-.4,.5]
        self.blocked=self.block_move==len(self.moves)
        self.complete_at=time.monotonic()+.25
        if self.fail_move and len(self.moves)==self.fail_move:self.fail=True

    def cancel(self,message):
        if message.cancel:
            self.cancels+=1;self.complete_at=0

    def tick(self):
        moving=time.monotonic()<self.complete_at
        if not moving and not self.blocked:self.pose=self.goal[:]
        stamp=self.get_clock().now().to_msg()
        s=DualArmStatus();s.header.stamp=stamp
        s.current_state=1 if moving else 0
        s.left_enabled=s.right_enabled=self.authority
        s.left_has_error=self.fail
        s.left_error_message='injected planning failure' if self.fail else ''
        for arm in ('left','right'):
            for k,v in zip(['x','y','z','rx','ry','rz'],self.pose):setattr(s.current_pose,arm+'_'+k,float(v))
        s.current_joint.left_positions=[0.]*7;s.current_joint.right_positions=[0.]*7
        s.current_joint.left_velocities=[0.]*7;s.current_joint.right_velocities=[0.]*7
        self.stat.publish(s)
        w=WaistStatus();w.header.stamp=stamp
        w.sdk_connected=True;w.has_upper_body_authority=self.authority
        w.enabled=self.authority;w.current_joint.positions=[0.]*3
        if self.authority:w.current_joint.positions[2]=self.waist_change_on_authority
        if self.waist_feedback_override is not None:w.current_joint.positions=self.waist_feedback_override
        self.waist.publish(w)
        if self.target_enabled and not self.vision_paused:
            target={'direction':'up','point':self.target_point,'normal':[-1.,0.,0.],
                    'center_xy':self.target_center,
                    'stamp_sec':self.get_clock().now().nanoseconds*1e-9,'frame_id':'torso_link'}
            self.targets.publish(String(data=json.dumps({'targets':[target],'camera_verified':True,
                                                          'image_width':640,'image_height':480,
                                                          'camera_calibration_id':self.camera_id})))


class MockIntegration(unittest.TestCase):
    def test_driver_identity_ignores_navigation_components(self):
        expected='/workspace/src/g1_action_ws/src/g1_actions/config/elevator.yaml'
        self.assertIsNone(driver_config_match(['/workspace/mfla_meta/install/mfla_quadruped/bin/app_run_component','--component_config=/nav.yaml'],expected))
        binary='/workspace/install/brsd_upbody_control/lib/brsd_upbody_control/app_run_component'
        self.assertTrue(driver_config_match([binary,'--component_config='+expected],expected))
        self.assertFalse(driver_config_match([binary,'--component_config=/other_driver.yaml'],expected))

    def setUp(self):
        rclpy.init()
        self.mock=MockRobot()
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.mock)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True)
        self.thread.start()
        self.directory=tempfile.TemporaryDirectory(prefix='ele_mock_')

    def tearDown(self):
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.mock.destroy_node();rclpy.shutdown()
        self.directory.cleanup()

    def command(self,execute=False,verified=False,wait=4,interrupt=False,old_calibration=False,hover_only=False):
        config=yaml.safe_load((ROOT/'config/elevator.yaml').read_text())
        config['camera']['verified']=verified
        for tool in config['tools'].values():tool['verified']=verified
        config['motion']['require_business_driver']=False
        config['motion']['dwell_s']=.05
        config['motion']['contact_timeout_s']=.6
        self.mock.camera_id='previous_config' if old_calibration else camera_signature(config)
        path=Path(self.directory.name)/'test.yaml';path.write_text(yaml.safe_dump(config))
        args=['ros2','run','ele_elevator','press','--config',str(path),'--direction','up','--wait',str(wait)]
        if execute:args.append('--execute')
        if hover_only:args.append('--hover-only')
        if not interrupt:
            return subprocess.run(args,capture_output=True,text=True,timeout=35)
        process=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
        until=time.monotonic()+8
        while time.monotonic()<until and not self.mock.moves:
            time.sleep(.01)
        if not self.mock.moves:
            process.kill();out,err=process.communicate()
            raise AssertionError(out+err)
        os.killpg(process.pid,signal.SIGINT)
        out,err=process.communicate(timeout=15)
        return subprocess.CompletedProcess(args,process.returncode,out,err)

    def test_preview_never_gets_authority_or_moves(self):
        result=self.command()
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.mock.moves,[])
        self.assertEqual(self.mock.starts,0)

    def test_uncalibrated_execution_is_blocked_before_authority(self):
        result=self.command(execute=True)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.mock.starts,0)
        self.assertEqual(self.mock.moves,[])

    def test_full_cycle_and_release(self):
        result=self.command(execute=True,verified=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(len(self.mock.moves),6)
        self.assertEqual(self.mock.starts,1)
        self.assertEqual(self.mock.ends,1)
        self.assertIn('CONTACT_SEQUENCE_COMPLETED',result.stdout)
        self.assertIn('"press_verified": false',result.stdout)
        self.assertEqual(self.mock.arm_masks,[1]*6)
        self.assertEqual(self.mock.freeze_calls,1)
        self.assertIn('TARGET_LOCKED',result.stdout)

    def test_auto_right_selection_is_locked_even_if_image_target_crosses_midline(self):
        self.mock.target_center=[465.,197.]
        self.mock.shift_image_after_move=True
        result=self.command(execute=True,verified=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.mock.arm_masks,[2]*6)
        self.assertIn('"arm": "right"',result.stdout)
        self.assertEqual(self.mock.ends,1)

    def test_auto_midline_does_not_acquire_control(self):
        self.mock.target_center=[320.,240.]
        result=self.command(execute=True,verified=True)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.mock.starts,0)
        self.assertEqual(self.mock.moves,[])

    def test_nominal_hover_only_approaches_and_returns_without_calibration_or_contact(self):
        self.mock.target_point=[.55,.20,.15]
        self.mock.target_center=[465.,197.]
        result=self.command(execute=True,hover_only=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.mock.arm_masks,[2,2])
        self.assertIn('HOVER_COMPLETED',result.stdout)
        self.assertNotIn('"stage": "PRESS"',result.stdout)
        self.assertEqual(self.mock.ends,1)
        self.assertTrue(self.mock.vision_paused)
        self.assertEqual(self.mock.freeze_calls,1)

    def test_snapshot_ignores_changed_vision_after_motion_starts(self):
        self.mock.honor_freeze=False
        self.mock.change_target_on_move=True
        result=self.command(execute=True,verified=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(len(self.mock.moves),6)
        self.assertTrue(all(goal[0]<.5 for goal in self.mock.moves))

    def test_next_task_resumes_vision_and_acquires_another_snapshot(self):
        first=self.command()
        self.assertEqual(first.returncode,0,first.stdout+first.stderr)
        self.mock.vision_paused=True
        second=self.command()
        self.assertEqual(second.returncode,0,second.stdout+second.stderr)
        self.assertEqual(self.mock.resume_calls,2)

    def test_nominal_hover_guard_blocks_close_hand_before_authority(self):
        self.mock.target_point=[.35,.20,.15]
        result=self.command(execute=True,hover_only=True)
        self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.mock.starts,0)
        self.assertEqual(self.mock.moves,[])

    def test_hover_allows_small_waist_change_during_authority_handoff(self):
        self.mock.target_point=[.55,.20,.15]
        self.mock.waist_change_on_authority=.025
        result=self.command(execute=True,hover_only=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('HOVER_COMPLETED',result.stdout)
        self.assertEqual(len(self.mock.moves),2)

    def test_hover_does_not_abort_when_waist_change_exceeds_old_limit(self):
        self.mock.target_point=[.55,.20,.15]
        self.mock.waist_change_on_authority=.08
        result=self.command(execute=True,hover_only=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.mock.ends,1)
        self.assertIn('HOVER_COMPLETED',result.stdout)

    def test_contact_execution_does_not_abort_due_to_waist_change(self):
        self.mock.waist_change_on_authority=.04
        result=self.command(execute=True,verified=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('CONTACT_SEQUENCE_COMPLETED',result.stdout)
        self.assertEqual(self.mock.ends,1)

    def test_invalid_waist_feedback_still_blocks_authority(self):
        self.mock.waist_feedback_override=[float('nan'),0.,0.]
        result=self.command(execute=True,verified=True)
        self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('Invalid measured waist joints',result.stdout+result.stderr)
        self.assertEqual(self.mock.starts,0)
        self.assertEqual(self.mock.moves,[])

    def test_driver_error_cancels_and_releases_without_pressing(self):
        self.mock.fail_move=2
        result=self.command(execute=True,verified=True)
        self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(len(self.mock.moves),2)
        self.assertGreaterEqual(self.mock.cancels,1)
        self.assertEqual(self.mock.ends,1)

    def test_missing_targets_never_move(self):
        self.mock.target_enabled=False
        result=self.command(execute=True,verified=True,wait=.5)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.mock.moves,[])

    def test_interrupt_sends_cancel_before_ros_shutdown(self):
        result=self.command(execute=True,verified=True,interrupt=True)
        self.assertNotEqual(result.returncode,0)
        self.assertGreaterEqual(self.mock.cancels,1,result.stdout+result.stderr)
        self.assertEqual(self.mock.ends,1,result.stdout+result.stderr)

    def test_camera_recalibration_requires_vision_restart(self):
        result=self.command(execute=True,verified=True,old_calibration=True)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.mock.starts,0)
        self.assertEqual(self.mock.moves,[])

    def test_blocked_contact_is_time_bounded(self):
        self.mock.block_move=3
        start=time.monotonic()
        result=self.command(execute=True,verified=True)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(len(self.mock.moves),3)
        self.assertGreaterEqual(self.mock.cancels,1)
        self.assertEqual(self.mock.ends,1)
        self.assertLess(time.monotonic()-start,8)


if __name__=='__main__':unittest.main(verbosity=2)
