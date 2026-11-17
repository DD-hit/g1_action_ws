"""Actual C++ planner/executor, isolated fake SDK, current unverified parameter fixture.

Never starts real mode; production calibration files are not modified.
"""
import os
os.environ.update(ROS_DOMAIN_ID='186',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
from pathlib import Path
import copy
import math
import re
import json
import signal
import subprocess
import tempfile
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from brsd_msgs.msg import DualArmMoveJ,DualArmStatus
from g1_actions.actions.elevator.config import load,camera_signature
from g1_actions.actions.elevator.target_mode import source_direction
from g1_actions.actions.elevator.task_node import PressTask

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=Path(os.environ.get('ELE_TEST_OUTPUT',ROOT/'data'/'validation'));OUTPUT.mkdir(parents=True,exist_ok=True)
cfg=load(ROOT/'config/elevator.yaml')
cfg['motion'].update(require_business_driver=False,pause_inference_on_lock=False)
# Isolated fake SDK only: exercise full completion without the production 1 s cutoff.
if os.environ.get('ELE_TEST_CONTACT_TIMEOUT'):
    cfg['motion']['contact_timeout_s']=float(os.environ['ELE_TEST_CONTACT_TIMEOUT'])
if os.environ.get('ELE_TEST_SPEED'):
    for profile in cfg['motion'].get('stage_profiles',{}).values():
        profile.update(speed=float(os.environ['ELE_TEST_SPEED']),acceleration=float(os.environ['ELE_TEST_SPEED']))
if os.environ.get('ELE_TEST_PRESS_SPEED'):
    cfg['motion']['stage_profiles']['PRESS'].update(speed=float(os.environ['ELE_TEST_PRESS_SPEED']),acceleration=float(os.environ['ELE_TEST_PRESS_SPEED']))
driver_cfg=yaml.safe_load((ROOT/'config/elevator.yaml').read_text())
driver_cfg['pipelines'][0]['fake_mode']=True
if os.environ.get('ELE_TEST_TRANSIT_PLANNER'):
    driver_cfg['pipelines'][0]['moveit']['elevator_transit_planner']=os.environ['ELE_TEST_TRANSIT_PLANNER']
if os.environ.get('ELE_TEST_APS_EARLY_EXIT'):
    driver_cfg['pipelines'][0]['moveit']['elevator_aps_early_exit_enabled']=os.environ['ELE_TEST_APS_EARLY_EXIT']=='1'
if os.environ.get('ELE_TEST_BASELINE_EXECUTION')=='1':
    driver_cfg['pipelines'][0]['moveit'].update(elevator_execution_velocity_scale=.90,elevator_execution_acceleration_scale=.80)
    cfg['motion']['stage_profiles']['APPROACH'].update(speed=.35,acceleration=.30)

driver_cfg['pipelines'][0]['fake_has_upper_body_authority']=True
import fcntl
_test_lock=open("/tmp/ele_two_stage_fake.lock","a")
fcntl.flock(_test_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
cancelled_press=os.environ.get('ELE_TEST_CANCEL_PRESS')=='1'
selected_arm=os.environ.get('ELE_TEST_ARM','left')
selected_direction=os.environ.get('ELE_TEST_DIRECTION','up')
assert selected_arm in ('left','right')
suffix=('' if selected_arm=='left' else '_right')+('_down' if selected_direction=='down' else '')+('_cancelled' if cancelled_press else '')
report={'test_stage_speed_override':os.environ.get('ELE_TEST_SPEED'),'production_speed':cfg['motion']['speed'],'production_acceleration':cfg['motion']['acceleration'],
    'contact_timeout_s':cfg['motion']['contact_timeout_s'],'selected_arm':selected_arm,'ros_domain':186,'hardware_commands_sent':False,'uses_current_unverified_parameters':True,'events':[]}
with tempfile.TemporaryDirectory(prefix='two_stage_fake_') as temp:
    config=Path(temp)/'driver.yaml';config.write_text(yaml.safe_dump(driver_cfg))
    env=os.environ.copy();env.update(CONFIG_FILE=str(config),UNITREE_SDK2_ROOT='/opt/unitree_sdk2',UNITREE_DDS_ROOT='/opt/unitree_sdk2')
    log_path=OUTPUT/('retrace_driver_fake'+suffix+'.log')
    rclpy.init();node=PressTask(cfg,selected_direction,selected_arm)
    pub=node.create_publisher(String,'/elevator/targets',1)
    target_point=[.35,.20 if selected_arm=='left' else -.20,.15]
    target_normal=[-1.,0.,0.]
    for key,destination in (("ELE_TEST_TARGET_POINT",target_point),("ELE_TEST_TARGET_NORMAL",target_normal)):
        if os.environ.get(key):
            value=json.loads(os.environ[key])
            assert len(value)==3 and all(math.isfinite(float(x)) for x in value)
            destination[:]=[float(x) for x in value]

    def target():
        stamp=node.get_clock().now().nanoseconds*1e-9
        pub.publish(String(data=json.dumps({'camera_verified':bool(cfg['camera']['verified']),'camera_calibration_id':camera_signature(cfg),
            'image_width':640,'targets':[{'direction':source_direction(cfg,selected_direction),'point':target_point,'normal':target_normal,
            'center_xy':[160.,240.],'stamp_sec':stamp,'frame_id':'torso_link'}]})))
    node.create_timer(.1,target)
    approach_samples=[];recording=[False]
    def capture(message):
        if recording[0]:approach_samples.append(list(getattr(message.current_joint,selected_arm+'_positions')))
    node.create_subscription(DualArmStatus,'/dual_arm/status',capture,10)
    original=node.event
    def event(stage,**extra):
        if stage=='APPROACH':recording[0]=True
        if stage=='MOTION_REACHED' and extra.get('motion_stage')=='APPROACH':recording[0]=False
        report['events'].append({'stage':stage,'monotonic_s':time.monotonic(),'actual_pose':node.current_pose() if node.status else None,
            'actual_left_joints':list(node.status.current_joint.left_positions) if node.status else None,
            'actual_right_joints':list(node.status.current_joint.right_positions) if node.status else None,**extra});original(stage,**extra)
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
            if os.environ.get('ELE_TEST_INITIAL_ARM_JOINTS'):
                initial=json.loads(os.environ['ELE_TEST_INITIAL_ARM_JOINTS'])
                assert len(initial)==7 and all(math.isfinite(x) for x in initial)
                setattr(msg.joint,selected_arm+'_positions',initial)
            setup.publish(msg)
            node.wait(lambda:node.status.current_state==0 and max(abs(a-b) for a,b in zip(node.status.current_joint.left_positions,msg.joint.left_positions))<.01,15,'fake neutral posture')
            node.destroy_publisher(setup)
            report['initial_left_joints']=list(node.status.current_joint.left_positions)
            if cancelled_press:
                class InjectedPressCancel(Exception):pass
                cutoff=[None];old_event=node.event;old_health=node.health
                def cancellable_event(stage,**extra):
                    old_event(stage,**extra)
                    if stage=='MOTION_ACCEPTED' and extra.get('motion_stage')=='PRESS':cutoff[0]=time.monotonic()+float(os.environ.get('ELE_TEST_CANCEL_DELAY','.4'))
                def cancellable_health(moving=False):
                    old_health(moving)
                    if cutoff[0] is not None and time.monotonic()>=cutoff[0]:raise InjectedPressCancel()
                node.event=cancellable_event;node.health=cancellable_health
                try:node.run(execute=True,wait_s=10,hover_only=False)
                except InjectedPressCancel:
                    node.health=old_health
                    node.cancel_motion();node.recover_contact()
                    report['partial_press_cancel_recovered']=True
                else:raise AssertionError('Press cancellation was not exercised')
            else:
                node.run(execute=True,wait_s=10,hover_only=False)
            stages=[x['stage'] for x in report['events']]
            motions=[x for x in stages if x in ('APPROACH','PRESS','WITHDRAW','RECOVERY_WITHDRAW','RETURN','HOVER','RETREAT')]
            withdraw_stage='RECOVERY_WITHDRAW' if cancelled_press else 'WITHDRAW'
            assert motions==['APPROACH','PRESS',withdraw_stage],motions
            assert stages.index('APPROACH')<stages.index('PRESS')<stages.index(withdraw_stage)
            text=log_path.read_text()
            used_ompl=any(source in text for source in ('source=ompl_rrtconnect_fallback','source=ompl_rrtstar_fallback','source=ompl_anytime_path_shortening'))
            assert text.count('[ElevatorPreflight] passed')>=2,'Forward motions must be prechecked'
            selected_plans=[line for line in text.splitlines() if '[MotionPlanned]' in line and ('command=MovePJ' in line or 'command=MoveL' in line)]
            expected_mask='arm=LEFT(1)' if selected_arm=='left' else 'arm=RIGHT(2)'
            assert len(selected_plans)==2 and all(expected_mask in line for line in selected_plans),selected_plans
            report['selected_arm_preserved_in_all_commands']=True
            report['ompl_transit_planner_used']=used_ompl
            report['transit_planner']='PilzPTP' if 'planner=PilzPTP fixed_joint_goal=true' in text else 'AnytimePathShortening' if 'planner=AnytimePathShortening fixed_joint_goal=true' in text else 'legacy'
            assert '[RecordedRetraceValidated]' in text and '[RecordedRetraceAccepted]' in text
            report['withdrawal_reuses_recorded_trajectory_without_replanning']=True
            report['forward_trajectories_prechecked']=True
            count=text.count('[PilzPlan] stage=request algorithm=LIN')
            retrace_at=text.index('[RecordedRetrace')
            assert count>=1 and text[:retrace_at].count('[PilzPlan] stage=request algorithm=LIN')==count,'Withdrawal must not replan LIN'
            if len(approach_samples)>=2:
                distance=lambda a,b:math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
                length=sum(distance(a,b) for a,b in zip(approach_samples,approach_samples[1:]))
                direct=distance(approach_samples[0],approach_samples[-1])
                report['observed_approach_path']={'length_rad':length,'direct_rad':direct,'ratio':length/direct if direct>1e-8 else 0.,'end_joints':approach_samples[-1]}
            report['direction']=selected_direction
            if os.environ.get('ELE_TEST_EXPECT_PTP')=='1':assert report['transit_planner']=='PilzPTP',report['transit_planner']
            report['planning_searches']=[{'elapsed_ms':int(elapsed),'budget_ms':float(budget)} for elapsed,budget in re.findall(r'\[OmplSearchTiming\] planner=AnytimePathShorteningkConfigDefault elapsed_ms=(\d+) budget_ms=([0-9.]+)',text)]
            report['aps_early_exit']=[{'enabled':enabled in ('true','1'),'lower_bound':float(lower),'threshold':float(threshold)} for enabled,lower,threshold in re.findall(r'\[APSEarlyExit\] enabled=(true|false|0|1) lower_bound=([0-9.eE+-]+) threshold=([0-9.eE+-]+)',text)]
            report['forward_motion_count']=2
            report['result']='passed'
    except Exception as exc:
        report.update(result='failed',reason=str(exc))
        if node.executing:node.cancel_motion()
        raise
    finally:
        if node.acquired:
            try:node.trigger('/dual_arm/end');report['authority_released']=True
            except Exception as exc:report['release_error']=str(exc)
        if driver is not None and driver.poll() is None:
            os.killpg(driver.pid,signal.SIGINT)
            try:driver.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait()
        node.destroy_node();rclpy.shutdown()
        (OUTPUT/('retrace_driver_fake'+suffix+'.json')).write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
