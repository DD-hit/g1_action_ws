"""Actual C++ planner/executor, isolated fake SDK, current unverified parameter fixture.

Never starts real mode; production calibration files are not modified.
"""
import os
os.environ.update(ROS_DOMAIN_ID=os.environ.get('ELE_TEST_DOMAIN','186'),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import copy
import json
import signal
import subprocess
import tempfile
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from brsd_msgs.msg import DualArmMoveJ,WaistMoveJ
from g1_actions.actions.elevator.config import load,camera_signature,for_direction
from g1_actions.actions.elevator.task_node import PressTask

ROOT=Path(__file__).resolve().parents[1]
cfg=for_direction(load(ROOT/'config/elevator.yaml'),'up')
cfg['motion'].update(require_business_driver=False,pause_inference_on_lock=False)
if os.environ.get('ELE_TEST_PRODUCTION_DEPTH')=='1':cfg['motion']['press_depth_m']=.013
driver_cfg=yaml.safe_load((ROOT/'config/robot.yaml').read_text())
driver_cfg['pipelines'][0]['fake_mode']=True
driver_cfg['pipelines'][0]['fake_has_upper_body_authority']=True
import fcntl
_test_lock=open("/tmp/ele_press_safety_fake_"+os.environ["ROS_DOMAIN_ID"]+".lock","a")
fcntl.flock(_test_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
selected_arm=os.environ.get('ELE_TEST_ARM','right')
production_depth=os.environ.get('ELE_TEST_PRODUCTION_DEPTH')=='1'
escape_offset=float(os.environ.get('ELE_TEST_ESCAPE_OFFSET_M','0'))
waist_regression=os.environ.get('ELE_TEST_WAIST_REGRESSION')=='1'
reject_press=os.environ.get('ELE_TEST_REJECT_PRESS')=='1'
inject_contact_timeout=os.environ.get('ELE_TEST_INJECT_CONTACT_TIMEOUT')=='1'
if inject_contact_timeout and not escape_offset:cfg['motion']['contact_timeout_s']=.001
assert selected_arm in ('left','right')
suffix=('_left' if selected_arm=='left' else '_right')+('_recovery' if inject_contact_timeout else '')+('_extended' if production_depth else '')+('_rejected' if reject_press else '')+('_waist' if waist_regression else '')+('_offset' if escape_offset else '')
report={'fixture_press_depth_m':cfg['motion']['press_depth_m'],'production_depth_m':.013,'production_speed':cfg['motion']['speed'],'production_acceleration':cfg['motion']['acceleration'],
    'contact_timeout_s':cfg['motion']['contact_timeout_s'],'selected_arm':selected_arm,'ros_domain':int(os.environ['ROS_DOMAIN_ID']),'hardware_commands_sent':False,'uses_current_unverified_parameters':True,'events':[]}
with tempfile.TemporaryDirectory(prefix='two_stage_fake_') as temp:
    config=Path(temp)/'driver.yaml';config.write_text(yaml.safe_dump(driver_cfg))
    env=os.environ.copy();env.update(CONFIG_FILE=str(config),UNITREE_SDK2_ROOT='/opt/unitree_sdk2',UNITREE_DDS_ROOT='/opt/unitree_sdk2')
    (ROOT/'data'/'validation').mkdir(parents=True,exist_ok=True)
    log_path=ROOT/('data/validation/two_stage_driver_fake'+suffix+'.log')
    rclpy.init();node=PressTask(cfg,'up',selected_arm)
    pub=node.create_publisher(String,'/elevator/targets',1)
    target_point=[.3903156344458282,.0587344167978942 if selected_arm=='left' else -.0587344167978942,.16598064724022144]
    target_normal=[-.9952993497549503,.05713786111690998 if selected_arm=='left' else -.05713786111690998,-.07819507148380801]
    def target():
        stamp=node.get_clock().now().nanoseconds*1e-9
        pub.publish(String(data=json.dumps({'camera_verified':bool(cfg['camera']['verified']),'camera_calibration_id':camera_signature(cfg),
            'image_width':640,'targets':[{'direction':'up','point':target_point,'normal':target_normal,
            'center_xy':[160.,240.],'stamp_sec':stamp,'frame_id':'torso_link'}]})))
    node.create_timer(.1,target)
    if escape_offset:
        original_move=node.move
        def measured_offset_failure(pose,linear,stage):
            original_move(pose,linear,stage)
            if stage=='PRESS':
                import numpy as np
                normal=np.asarray(node.locked_target['normal']);tangent=np.cross(normal,[0.,0.,1.]);tangent/=np.linalg.norm(tangent)
                node.locked_target['point']=(np.asarray(node.locked_target['point'])+tangent*escape_offset).tolist()
                node.hover_guard.centre+=tangent*escape_offset
                raise RuntimeError('Contact travel timed out; injected measured lateral offset')
        node.move=measured_offset_failure
    if reject_press:
        original_poses=node.poses
        def bad_endpoint(target):
            poses=original_poses(target)
            # Fake SDK only: request a point beyond the 15 mm wall and prove
            # rejection reaches the matching task without a 120-second wait.
            normal=__import__('numpy').asarray(target['normal'],float)
            poses['press']['xyz']=(__import__('numpy').asarray(poses['press']['xyz'])-.025*normal).tolist()
            return poses
        node.poses=bad_endpoint
    rejection_started=[None]
    original=node.event
    def event(stage,**extra):
        if stage=='PRESS':rejection_started[0]=time.monotonic()
        report['events'].append({'stage':stage,'actual_pose':node.current_pose() if node.status else None,
            'actual_left_joints':list(node.status.current_joint.left_positions) if node.status else None,**extra});original(stage,**extra)
    node.event=event
    driver=None
    try:
        with log_path.open('w') as log:
            driver=subprocess.Popen(['bash','/workspace/src/brsd_manipulation/src/brsd_upbody_control/script/run_driver.sh','unitree_g1','fake'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            node.wait(lambda:node.status is not None and node.waist is not None,45,'fake driver')
            # Fake driver defaults to zero joints (hands forward). Initialize the
            # synthetic fixture to the recorded neutral posture, SDK stays fake.
            node.trigger('/dual_arm/start');node.acquired=True
            setup=node.create_publisher(DualArmMoveJ,'/dual_arm/move_j',1)
            node.wait(lambda:setup.get_subscription_count()==1,5,'fake setup subscriber')
            msg=DualArmMoveJ();msg.arm_mask=3;msg.speed=.5;msg.a=.5
            msg.joint.left_positions=[.2844,.1302,.0099,.9844,.0613,.0845,-.0472]
            msg.joint.right_positions=[.2904,-.1219,.0129,.9785,-.0545,-.0249,-.0351]
            setup.publish(msg)
            node.wait(lambda:node.status.current_state==0 and max(abs(a-b) for a,b in zip(node.status.current_joint.left_positions,msg.joint.left_positions))<.01,15,'fake neutral posture')
            node.destroy_publisher(setup)
            if waist_regression:
                waist_pub=node.create_publisher(WaistMoveJ,'/dual_arm/waist_move_j',1)
                node.wait(lambda:waist_pub.get_subscription_count()==1,5,'fake waist setup subscriber')
                waist_msg=WaistMoveJ();waist_msg.header.frame_id=cfg['base_frame'];waist_msg.header.stamp=node.get_clock().now().to_msg()
                waist_msg.waist_positions=[-.03,-.01,-.02];waist_msg.duration=0.;waist_msg.speed=.2
                waist_pub.publish(waist_msg)
                node.wait(lambda:node.status.current_state==0 and max(abs(a-b) for a,b in zip(node.waist.current_joint.positions,waist_msg.waist_positions))<.0001,10,'fake nonzero waist pose')
                node.destroy_publisher(waist_pub)
                report['nonzero_waist_positions']=list(node.waist.current_joint.positions)
            report['initial_left_joints']=list(node.status.current_joint.left_positions)
            try:
                node.run(execute=True,wait_s=10,hover_only=False)
                assert not inject_contact_timeout, 'Injected contact timeout did not occur'
            except RuntimeError as error:
                if not inject_contact_timeout or 'Contact travel timed out' not in str(error):raise
                assert node.contact_pending, 'Contact must remain pending after timeout'
                node.cancel_motion()
                node.recover_contact()
                assert not node.contact_pending, 'Recovery failed to verify withdrawal'
                stages=[x['stage'] for x in report['events']]
                assert stages.index('RECOVERY_WITHDRAW')<stages.index('WITHDRAW_VERIFIED')<stages.index('RECOVERY_RETURN')
                report['cancel_recovery_order_verified']=True
            stages=[x['stage'] for x in report['events']]
            motions=[x for x in stages if x in ('APPROACH','PRESS','WITHDRAW','RETURN','HOVER','RETREAT')]
            assert motions==(['APPROACH','PRESS'] if inject_contact_timeout else ['APPROACH','PRESS','WITHDRAW','RETURN']),motions
            if not inject_contact_timeout:assert stages.index('APPROACH')<stages.index('PRESS')<stages.index('WITHDRAW')
            text=log_path.read_text()
            used_ompl='source=ompl_rrtconnect_fallback' in text
            assert '[ElevatorTwoStagePreflight]' not in text,'No repeated lookahead planning'
            assert 'penetration=0.015' in text,'Depth bound must remain active'
            assert text.count('[ElevatorPreflight] passed')>=4,'Every submitted motion must be prechecked'
            selected_plans=[line for line in text.splitlines() if '[MotionPlanned]' in line and ('command=MovePJ' in line or 'command=MoveL' in line)]
            expected_mask='arm=LEFT(1)' if selected_arm=='left' else 'arm=RIGHT(2)'
            assert len(selected_plans)==4 and all(expected_mask in line for line in selected_plans),selected_plans
            report['selected_arm_preserved_in_all_commands']=True
            report['ompl_transit_planner_used']=used_ompl
            report['all_four_motion_trajectories_prechecked']=True
            report['forward_motion_count']=2
            report['result']='passed'
    except Exception as exc:
        report.update(result='failed',reason=str(exc))
        if node.executing:node.cancel_motion()
        stages=[x['stage'] for x in report['events']]
        if reject_press and '轨迹规划/接收失败' in str(exc) and rejection_started[0] is not None:
            elapsed=time.monotonic()-rejection_started[0]
            assert elapsed<8.,elapsed
            assert not node.contact_pending,'Rejected press must not be mistaken for executed contact'
            report.update(result='passed', rejection_reported_within_s=elapsed,
                          no_press_accepted=not any(e['stage']=='MOTION_ACCEPTED' and e.get('motion_stage')=='PRESS' for e in report['events']))
            assert report['no_press_accepted']
        else:raise
    finally:
        if node.acquired:
            try:node.trigger('/dual_arm/end');report['authority_released']=True
            except Exception as exc:report['release_error']=str(exc)
        if driver is not None and driver.poll() is None:
            os.killpg(driver.pid,signal.SIGINT)
            try:driver.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
        node.destroy_node();rclpy.shutdown()
        (ROOT/('data/validation/two_stage_driver_fake'+suffix+'.json')).write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
