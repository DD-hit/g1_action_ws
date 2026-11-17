"""One target diagnostic: fresh palm -> configured target -> return -> release.

Zero target offset can cause contact; this is not a non-contact test.

The unified scheduler owns production execution. Default is preview only.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from brsd_msgs.msg import DualArmStatus, WaistStatus, DualArmMovePJ, DualArmMotionSequence
from .transforms import precontact_preview, point3


def euler_xyz_matrix(rpy):
    x,y,z=map(float,rpy);cx,sx,cy,sy,cz,sz=math.cos(x),math.sin(x),math.cos(y),math.sin(y),math.cos(z),math.sin(z)
    return np.array([[cy*cz,-cy*sz,sy],[cx*sz+sx*sy*cz,cx*cz-sx*sy*sz,-sx*cy],
                     [sx*sz-cx*sy*cz,sx*cz+cx*sy*sz,cx*cy]])


class ObservationUnavailable(RuntimeError):
    """A fresh, stable observation is temporarily unavailable."""


class TargetRejected(RuntimeError):
    """Only a completed geometric/planning rejection permits a fixed fallback."""


class Approach(Node):
    def __init__(self, config, package, arm, output):
        super().__init__('g1_handshake_precontact_once')
        self.cfg,self.package,self.arm,self.output=config,package,arm,output
        self.request_id=None
        self.cancel_requested=False;self.cleaning_up=False;self.subject_handedness="any"
        self.selected_handedness=None;self.used_fallback=False
        self.status=self.waist=self.vision=None
        self.arm_at=self.waist_at=self.vision_at=0.
        self.feedback={};self.locked=None;self.locked_stamp=-1.;self.owned=False;self.motion_sent=False;self.command_dispatched=False
        self.create_subscription(DualArmStatus,'/dual_arm/status',self.on_arm,1)
        self.create_subscription(WaistStatus,'/dual_arm/waist_status',self.on_waist,1)
        self.vision_subscription=self.create_subscription(String,'/handshake/perception_status',self.on_vision,1)
        self.create_subscription(String,'/elevator/motion_feedback',self.on_feedback,10)
        self.publisher=self.create_publisher(DualArmMovePJ,'/dual_arm/move_pj',1)
        self.cancel_pub=self.create_publisher(DualArmMotionSequence,'/dual_arm/motion_sequence',1)
        self.journal=(output/'events.jsonl').open('w',buffering=1)

    def on_arm(self,msg):self.status,self.arm_at=msg,time.monotonic()
    def on_waist(self,msg):self.waist,self.waist_at=msg,time.monotonic()
    def on_vision(self,msg):
        if self.locked is not None:return
        try:self.vision=json.loads(msg.data);self.vision_at=time.monotonic()
        except (ValueError,TypeError):self.vision=None
    def on_feedback(self,msg):
        try:
            value=json.loads(msg.data)
            key=(value['command'],value['arm_mask'],value['request_sec'],value['request_nanosec'])
            self.feedback[key]=value
        except (KeyError,ValueError,TypeError):pass
    def event(self,stage,**values):
        record=dict(stage=stage,**values)
        if self.request_id:record["request_id"]=self.request_id
        print(json.dumps(record,ensure_ascii=False),flush=True)
        self.journal.write(json.dumps(record,ensure_ascii=False)+'\n')
    def check_cancel(self):
        if self.cancel_requested and not self.cleaning_up:raise RuntimeError('用户请求停止')
    def wait(self,predicate,seconds,label,monitor=False):
        until=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<until:
            self.check_cancel()
            rclpy.spin_once(self,timeout_sec=.03)
            if monitor:
                self.health()
            if predicate():return
        raise RuntimeError('Timeout: '+label)
    def target(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.vision is None:raise ObservationUnavailable('未收到手部识别状态 /handshake/perception_status，请启动 g1.sh serve')
        if time.monotonic()-self.vision_at>.6:raise ObservationUnavailable('手部识别状态已过期，请检查相机和识别进程')
        if self.vision.get('valid') is False:raise ObservationUnavailable(self.vision.get('reason','Vision is unavailable'))
        if self.vision.get('frame_id')!=self.cfg['frames']['camera_optical']:raise RuntimeError('Unexpected camera frame')
        if not 0<=now-self.vision['stamp_s']<=.6:raise ObservationUnavailable('Hand image is stale')
        candidates=[p for p in self.vision.get('palms',[]) if p.get('geometry_valid') and p.get('stable')]
        if self.subject_handedness!='any':
            candidates=[p for p in candidates if str(p.get('handedness','')).lower()==self.subject_handedness]
        if len(candidates)>1:raise ObservationUnavailable('检测到多个稳定目标，请只展示一只手')
        if not candidates:
            palms=self.vision.get('palms',[])
            if not palms:raise ObservationUnavailable('相机和识别已运行，但画面中未检测到手')
            reasons=[p.get('pose',{}).get('reason','等待位置稳定') for p in palms if not p.get('geometry_valid')]
            raise ObservationUnavailable('手部目标尚未就绪：'+('; '.join(reasons) if reasons else '等待连续稳定的位置'))
        label=str(candidates[0].get('handedness','')).lower()
        if self.arm=='auto' and label not in ('left','right'):
            raise ObservationUnavailable('等待稳定的人手左右识别，不能按画面左右猜测')
        self.selected_handedness=label
        return point3(candidates[0]['pose']['position_m'])
    def health(self,owned=False):
        self.check_cancel()
        now=time.monotonic()
        if self.status is None or self.waist is None:raise RuntimeError('未收到双臂或腰部驱动反馈，请启动 g1.sh serve')
        if now-self.arm_at>.6 or now-self.waist_at>.6:raise RuntimeError('Driver feedback is stale')
        if not self.waist.sdk_connected:raise RuntimeError('SDK disconnected')
        if self.status.emergency_stop_active or self.status.joint_limits_exceeded or self.status.collision_detected:raise RuntimeError('Driver safety check failed')
        if self.status.left_has_error or self.status.right_has_error:raise RuntimeError(self.status.left_error_message+'; '+self.status.right_error_message)
        if owned and not self.waist.has_upper_body_authority:raise RuntimeError('Control authority lost')
        if self.count_publishers('/dual_arm/status')!=1 or self.count_publishers('/dual_arm/waist_status')!=1:raise RuntimeError('Exactly one driver is required')
    def readiness_problem(self):
        problems=[]
        try:self.health()
        except (ValueError,RuntimeError,KeyError) as error:problems.append('驱动：'+str(error))
        try:self.target()
        except (ValueError,RuntimeError,KeyError) as error:problems.append('视觉：'+str(error))
        return '；'.join(problems)
    def wait_for_target(self,seconds=20.):
        reason=['尚未收到就绪状态']
        previous=[None]
        def ready():
            problem=self.readiness_problem()
            if not problem:return True
            reason[0]=problem
            if problem!=previous[0]:
                self.event('WAITING_TARGET',reason=problem)
                previous[0]=problem
            return False
        try:self.wait(ready,seconds,'initial target')
        except RuntimeError as error:
            raise RuntimeError('首次目标未就绪：'+reason[0]) from error
    def lock_target(self):
        if self.locked is not None:raise RuntimeError('Target is already locked')
        snapshot=self.target().copy()
        from .selection import select_arm
        self.arm=select_arm(self.arm,self.selected_handedness)
        self.event("ARM_SELECTED",arm=self.arm,human_handedness=self.selected_handedness,policy="same_handedness")
        self.locked_stamp=float(self.vision['stamp_s'])
        snapshot.setflags(write=False)
        self.locked=snapshot
        self.destroy_subscription(self.vision_subscription)
        self.vision_subscription=None
        self.vision=None
        self.event('TARGET_LOCKED',camera_position_m=snapshot.tolist(),source_stamp_s=self.locked_stamp,
                   vision_required_after_lock=False)
        return snapshot
    def hold(self,seconds):
        if isinstance(seconds,bool) or not math.isfinite(seconds) or not 0<=seconds<=10:
            raise ValueError('hold_s must be between 0 and 10 seconds')
        self.phase='hold'
        self.event('HOLD',duration_s=float(seconds))
        until=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<until:
            rclpy.spin_once(self,timeout_sec=.03)
            self.health(owned=True)
        self.event('HOLD_COMPLETED')
    def trigger(self,name):
        client=self.create_client(Trigger,name)
        try:
            self.wait(client.service_is_ready,5,name)
            future=client.call_async(Trigger.Request());self.wait(future.done,8,name)
            result=future.result()
            if result is None or not result.success:raise RuntimeError(name+': '+(result.message if result else 'no response'))
        finally:self.destroy_client(client)
    def pose(self):
        value=self.status.current_pose
        return [float(getattr(value,self.arm+'_'+axis)) for axis in ('x','y','z','rx','ry','rz')]
    def close_to(self,target):
        current=self.pose()
        angle=math.acos(float(np.clip((np.trace(euler_xyz_matrix(current[3:]).T@euler_xyz_matrix(target[3:]))-1)/2,-1,1)))
        return np.linalg.norm(np.array(current[:3])-target[:3])<.015 and angle<.10
    def move(self,target,stage):
        self.phase=stage.lower()
        self.health(owned=True)
        if self.status.current_state!=DualArmStatus.IDLE:raise RuntimeError('Driver is not idle')
        if self.publisher.get_subscription_count()!=1:raise RuntimeError('Exactly one motion subscriber is required')
        msg=DualArmMovePJ();msg.header.frame_id='torso_link';msg.header.stamp=self.get_clock().now().to_msg()
        msg.arm_mask=1 if self.arm=='left' else 2;msg.body_avoidance=msg.PILZ
        msg.speed=.15;msg.a=.10
        for axis,value in zip(('x','y','z','rx','ry','rz'),target):setattr(msg.pose,self.arm+'_'+axis,float(value))
        key=('MovePJ',msg.arm_mask,msg.header.stamp.sec,msg.header.stamp.nanosec)
        self.event(stage,target_wrist=target,speed=.15,acceleration=.10)
        self.motion_sent=True;self.command_dispatched=True;self.publisher.publish(msg);started=time.monotonic();accepted=False;moving=False
        while rclpy.ok() and time.monotonic()-started<60:
            rclpy.spin_once(self,timeout_sec=.03);self.health(owned=True)
            feedback=self.feedback.get(key,{})
            if feedback.get('phase')=='rejected':raise RuntimeError(feedback.get('reason','Motion rejected'))
            accepted |= feedback.get('phase')=='accepted'
            moving |= self.status.current_state==DualArmStatus.MOVING
            if not accepted and time.monotonic()-started>8:raise RuntimeError('Motion planning acknowledgement timed out')
            if accepted and self.status.current_state==DualArmStatus.IDLE and self.close_to(target) and (moving or time.monotonic()-started>.5):
                self.motion_sent=False;self.event(stage+'_REACHED');return
        raise RuntimeError('Motion completion timed out')
    def cancel(self):
        self.wait(lambda:self.cancel_pub.get_subscription_count()==1,2,'cancel subscriber')
        msg=DualArmMotionSequence();msg.arm_mask=3;msg.cancel=True;msg.enable=False;self.cancel_pub.publish(msg)
        sent=time.monotonic()
        self.wait(lambda:time.monotonic()-sent>.25 and self.status is not None and self.status.current_state!=DualArmStatus.MOVING,3,'motion cancellation')
    def run_process(self,command,logname,monitor=True,timeout=30):
        self.phase=logname.removesuffix('.log')
        started=time.monotonic()
        self.event('PREFLIGHT_STAGE',operation=self.phase,status='started')
        with (self.output/logname).open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                                     env=dict(os.environ,ROS_DOMAIN_ID='211',ROS_LOCALHOST_ONLY='1'))
            try:
                self.wait(lambda:process.poll() is not None,timeout,logname,monitor=monitor)
                if process.returncode:raise RuntimeError(logname+': '+(self.output/logname).read_text()[-1500:])
                self.event('PREFLIGHT_STAGE',operation=self.phase,status='completed',elapsed_s=time.monotonic()-started)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    def verify_driver(self,allow_scene=False):
        from g1_actions.driver_runtime import verify_driver_runtime
        from rcl_interfaces.srv import GetParameters
        workspace=self.package.parents[3]
        verify_driver_runtime(dict(require_business_driver=True,
            driver_config=str(self.package/'config/robot.yaml'),
            driver_install_prefix=str(workspace/'install/brsd_upbody_control')))
        peers=self.get_publishers_info_by_topic('/dual_arm/status')
        if len(peers)!=1:raise RuntimeError('Exactly one driver is required')
        peer=peers[0];remote=peer.node_namespace.rstrip('/')+'/'+peer.node_name
        client=self.create_client(GetParameters,remote+'/get_parameters')
        try:
            self.wait(client.service_is_ready,5,'driver parameter query')
            request=GetParameters.Request();request.names=['elevator_panel']
            future=client.call_async(request);self.wait(future.done,5,'driver panel status')
            answer=future.result()
            if answer is None or len(answer.values)!=1 or (answer.values[0].double_array_value and not allow_scene):
                raise RuntimeError('Scoped elevator panel scene was not cleared before handshake')
        finally:self.destroy_client(client)

    def clear_action_scene(self):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        peers=self.get_publishers_info_by_topic('/dual_arm/status')
        if len(peers)!=1:raise RuntimeError('Exactly one driver is required')
        peer=peers[0];remote=peer.node_namespace.rstrip('/')+'/'+peer.node_name
        client=self.create_client(SetParametersAtomically,remote+'/set_parameters_atomically')
        try:
            self.wait(client.service_is_ready,5,'clear previous action scene')
            request=SetParametersAtomically.Request()
            request.parameters=[Parameter('elevator_panel',Parameter.Type.DOUBLE_ARRAY,[]).to_parameter_msg()]
            future=client.call_async(request);self.wait(future.done,5,'clear previous action scene')
            answer=future.result()
            if answer is None or not answer.result.successful:
                raise RuntimeError('Could not clear previous action scene')
        finally:self.destroy_client(client)

    def prepare_target(self):
        target=precontact_preview(self.cfg,self.package,self.lock_target(),self.arm,
                                  source_frame=self.cfg['frames']['camera_optical'])
        self.event('PREVIEW',**target)
        return target

    def checked_target(self,target):
        try:self.preflight(target)
        except TargetRejected as error:
            from .selection import fixed_fallback
            target=fixed_fallback(self.cfg,self.package,self.arm,target)
            self.event('FIXED_FALLBACK',arm=self.arm,reason=str(error),target_wrist=target['wrist_xyz_m'],
                       provisional=True)
            self.preflight(target)
            self.used_fallback=True
        return target

    def preflight(self,target):
        workspace=self.package.parents[3];binary=workspace/'build/handshake_preflight/handshake_preflight'
        reader=workspace/'build/handshake_preflight/handshake_full_state_reader'
        if not binary.is_file() or not reader.is_file():raise RuntimeError('Build handshake preflight helpers first')
        self.run_process([str(reader),'eth0',str(self.output/'motors.yaml')],'read_state.log')
        from .read_state import SDK_JOINT_NAMES
        state=yaml.safe_load((self.output/'motors.yaml').read_text());state['joint_positions']=dict(zip(SDK_JOINT_NAMES,state.pop('motor_q')))
        if len(state['joint_positions'])!=29:raise RuntimeError('Incomplete real joint snapshot')
        (self.output/'state.yaml').write_text(yaml.safe_dump(state))
        target.update(stamp_s=self.locked_stamp)
        (self.output/'target.jsonl').write_text(json.dumps(target)+'\n')
        self.run_process([str(binary),str(self.package/'robot/g1_29dof_mode_15.urdf'),str(self.package/'robot/g1_29dof_mode_15.srdf'),
                         str(workspace/'src/brsd_manipulation/src/brsd_upbody_control/robot/moveit/unitree_g1/joint_limits.yaml'),
                         str(self.output/'target.jsonl'),str(self.output/'preflight.yaml'),'1',str(self.output/'state.yaml')], 'preflight.log')
        report=yaml.safe_load((self.output/'preflight.yaml').read_text())
        if report['accepted_targets']!=1:raise TargetRejected('Current target rejected: '+str(report['results'][0]['reason']))
        for side in ('left','right'):
            names=[name for name in SDK_JOINT_NAMES if name.startswith(side) and ('shoulder' in name or 'elbow' in name or 'wrist' in name)]
            current=np.array(getattr(self.status.current_joint,side+'_positions'))
            if len(current)!=7 or max(abs(current-np.array([state['joint_positions'][n] for n in names])))>.035:raise RuntimeError('Robot moved during preflight')
        measured_waist=np.array([state['joint_positions']['waist_'+name+'_joint'] for name in ('yaw','roll','pitch')])
        if len(self.waist.current_joint.positions)!=3 or max(abs(np.array(self.waist.current_joint.positions)-measured_waist))>.02:raise RuntimeError('Waist moved during preflight')
        self.event('PREFLIGHT_PASSED',source='fresh_real_29_joint_state',planner='PilzPTP')


def main():
    package=Path(__file__).resolve().parents[3]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm',choices=('auto','left','right'),default='auto')
    parser.add_argument('--request-id')
    parser.add_argument('--hold-s',type=float)
    parser.add_argument('--subject-handedness',choices=('any','left','right'),default='any')
    parser.add_argument('--managed',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.execute and not args.managed:
        # All public execution routes must acquire the common scheduler.
        argv=['bash',str(package.parents[1]/'scripts/g1.sh'),'run','handshake','--arm',args.arm,
              '--subject-handedness',args.subject_handedness]
        if args.hold_s is not None:argv.extend(['--hold-s',str(args.hold_s)])
        print('执行已转交统一调度；诊断 --output 不用于生产任务，日志见 data/runtime。',flush=True)
        os.execvp('bash',argv)
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((package/'config/handshake.yaml').read_text())
    hold_s=config['execution']['hold_s'] if args.hold_s is None else args.hold_s
    if isinstance(hold_s,bool) or not isinstance(hold_s,(int,float)) or not math.isfinite(hold_s) or not 0<=hold_s<=10:
        parser.error('hold_s must be between 0 and 10 seconds')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Approach(config,package,args.arm,args.output)
    node.request_id=args.request_id
    node.subject_handedness=args.subject_handedness
    def interrupted(*_):node.cancel_requested=True
    previous_int=signal.signal(signal.SIGINT,interrupted)
    previous_term=signal.signal(signal.SIGTERM,interrupted)
    node.phase='waiting_target'
    result=dict(success=False,authority_released=False,hardware_motion_commands_sent=False)
    try:
        node.wait_for_target()
        target=node.prepare_target()
        if args.execute:
            if node.waist.has_upper_body_authority:raise RuntimeError('Another controller already owns the arms')
            velocities=list(node.status.current_joint.left_velocities)+list(node.status.current_joint.right_velocities)
            if len(velocities)!=14 or not np.isfinite(velocities).all() or max(map(abs,velocities))>.08:raise RuntimeError('Arms must be stationary')
            node.verify_driver(allow_scene=True)
            node.clear_action_scene()
            node.verify_driver()
            initial=node.pose()
            if not np.isfinite(initial).all():raise RuntimeError('Invalid initial wrist pose')
            target=node.checked_target(target)
            node.health()
            node.phase='acquire_authority'
            node.event('ACQUIRE_AUTHORITY',arm=node.arm)
            node.owned=True;node.trigger('/dual_arm/start')
            node.wait(lambda:node.waist.has_upper_body_authority and node.waist.authority_weight>=.98 and node.status.current_state==DualArmStatus.IDLE,5,'full authority')
            node.move(target['wrist_xyz_m']+[0.,math.radians(config['execution']['fixed_orientation']['finger_downward_deg']),0.],'APPROACH')
            node.hold(hold_s)
            node.move(initial,'RETURN')
        result.update(success=True,hardware_motion_commands_sent=args.execute)
    except (Exception,KeyboardInterrupt) as error:
        result['reason']=str(error) or 'Interrupted'
        result['failure_stage']=node.phase
        result['target_rejected']=isinstance(error,TargetRejected)
        node.cleaning_up=True
        if node.motion_sent:
            try:node.cancel()
            except Exception as cancel_error:result['cancel_error']=str(cancel_error)
    finally:
        node.cleaning_up=True
        if node.owned:
            try:
                node.event('RELEASE_AUTHORITY',arm=node.arm)
                node.trigger('/dual_arm/end')
                node.wait(lambda:node.waist is not None and time.monotonic()-node.waist_at<.6 and not node.waist.has_upper_body_authority and node.waist.authority_weight<.02,5,'authority released feedback')
                result['authority_released']=True
            except Exception as release_error:result['release_error']=str(release_error)
        result['hardware_motion_commands_sent']=bool(node.command_dispatched)
        result['authority_requested']=bool(node.owned)
        result.update(request_id=args.request_id,arm=node.arm,used_fixed_fallback=node.used_fallback,
                      human_handedness=node.selected_handedness,cancelled=node.cancel_requested)
        if result.get('release_error') or result.get('cancel_error'):result['success']=False
        (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True);node.journal.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        signal.signal(signal.SIGINT,previous_int);signal.signal(signal.SIGTERM,previous_term)
    return 0 if result['success'] and not result.get('release_error') else 1


if __name__=='__main__':raise SystemExit(main())
