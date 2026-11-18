"""One-shot task controller. Default invocation only previews geometric targets."""
import argparse
import copy
import json
from pathlib import Path
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from brsd_msgs.msg import DualArmStatus, WaistStatus
from scipy.spatial.transform import Rotation
from .config import load, resource, camera_signature, for_direction, withdrawal_clearance, motion_scales, validate_motion_profiles
from .target_mode import down_target_options, source_direction, acquired_target
from .robot_model import Robot
from .geometry import press_poses, arm_from_image


from g1_actions.driver_runtime import driver_config_match, verify_driver_runtime


class PressTask(Node):
    def __init__(self, config, direction, arm):
        config = for_direction(config, direction)
        down_target_options(config)
        validate_motion_profiles(config['motion'])
        super().__init__('ele_elevator_press_task')
        self.cfg, self.direction, self.arm = config, direction, arm
        self.requested_arm = arm
        self.robot = Robot(resource(config, config['model_file']))
        self.targets = self.status = self.waist = None
        self.target_arrival = self.status_arrival = self.waist_arrival = 0.
        self.status_sequence = 0
        self.motion_feedback = {}
        self.acquired = False
        self.executing = False
        self.locked_target = None
        self.contact_pending = False
        self.press_contact_started_at = None
        self.initial_pose = None
        self.retreat_pose = None
        self.events = self.create_publisher(String, '/elevator/task_status', 10)
        self.create_subscription(String, '/elevator/targets', self.on_targets, 1)
        self.create_subscription(String, '/elevator/motion_feedback', self.on_motion_feedback, 10)
        self.create_subscription(DualArmStatus, '/dual_arm/status', self.on_status, 10)
        self.create_subscription(WaistStatus, '/dual_arm/waist_status', self.on_waist, 10)

    def on_targets(self, msg):
        if self.locked_target is not None:
            return
        try:
            self.targets = json.loads(msg.data)
            self.target_arrival = time.monotonic()
        except (ValueError, TypeError):
            self.targets = None

    def on_motion_feedback(self, msg):
        try:
            data=json.loads(msg.data)
            if data['phase'] not in ('planning','accepted','rejected'):
                return
            key=(data['command'],int(data['arm_mask']),int(data['request_sec']),int(data['request_nanosec']))
            self.motion_feedback[key]=data
        except (ValueError,TypeError,KeyError):
            return

    def on_status(self, msg):
        self.status, self.status_arrival = msg, time.monotonic()
        self.status_sequence += 1

    def on_waist(self, msg):
        self.waist, self.waist_arrival = msg, time.monotonic()

    def event(self, stage, **extra):
        from .log_view import log_line
        record = {'stage': stage, 'direction': self.direction, 'arm': self.arm,
                  'stamp_sec': self.get_clock().now().nanoseconds*1e-9, **extra}
        self.events.publish(String(data=json.dumps(record)))
        if getattr(self,'quiet_console',False):return
        if stage == 'PREVIEW':
            direction = {'up':'上行', 'down':'下行'}.get(self.direction, self.direction)
            arm = {'left':'左手', 'right':'右手'}.get(self.arm, self.arm)
            offset = self.cfg['motion'].get('target_z_offset_m', 0.0)*1000
            from .panel_motion import lateral_offset
            lateral = lateral_offset(self.cfg['motion'], self.arm)*1000
            print(log_line('目标',f'{direction}｜{arm}｜按压深度 {self.cfg["motion"]["press_depth_m"]*1000:g} mm｜Y 补偿 {lateral:+.0f} mm｜Z 补偿 {offset:+.0f} mm'),flush=True)
        elif stage == 'ABORTED':
            pass  # Failure reason is printed once in the final result.
        elif stage == 'WAITING_TARGET':
            print(log_line('目标','等待：'+str(extra.get('reason','正在获取稳定 RGB-D 目标'))),flush=True)
        else:
            labels = {'ACQUIRE_AUTHORITY':'1/5 接管控制权',
                      'HOVER_APPROACH':'靠近按钮', 'HOVER_HOLD':'悬停保持 3 秒',
                      'HOVER_RETURN':'返回起始位置', 'HOVER_COMPLETED':'悬停完成',
                      'APPROACH':'2/5 规划并运动到预备点', 'HOVER':'到达预按压位置',
                      'PRESS':'3/5 直线按压', 'WITHDRAW':'4/5 沿原轨迹撤回', 'RETURN':'返回起始位置',
                      'RECOVERY_WITHDRAW':'异常恢复：沿面板法向直线撤回',
                      'WITHDRAW_VERIFIED':'已确认退出接触',
                      'READY_TO_RELEASE':'已撤回并停止，准备归还控制权',
                      'RECOVERY_RETURN':'异常恢复：返回起始位置',
                      'MOTION_ACCEPTED':'轨迹已下发，开始运动',
                      'CONTACT_SEQUENCE_COMPLETED':'按压动作完成'}
            quiet={'MOTION_ACCEPTED','CONTACT_SEQUENCE_COMPLETED','READY_TO_RELEASE','WITHDRAW_VERIFIED'}
            if stage in labels and stage not in quiet:
                print(log_line('阶段',labels[stage]),flush=True)

    def spin_once(self, timeout_sec=.04):
        # Keep this worker registered across callbacks. rclpy.spin_once(node)
        # adds/removes it on every call, which can starve later subscriptions
        # when the arm-status topic is continuously ready at 600 Hz.
        executor = getattr(self, '_task_executor', None)
        if executor is None:
            executor = self._task_executor = SingleThreadedExecutor(context=self.context)
        if self.executor is not executor:
            executor.add_node(self)
        executor.spin_once(timeout_sec=timeout_sec)

    def destroy_node(self):
        executor = getattr(self, '_task_executor', None)
        if executor is not None:
            executor.remove_node(self)
            executor.shutdown()
            self._task_executor = None
        return super().destroy_node()

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_once( timeout_sec=.04)
            if predicate():
                return
        raise RuntimeError('Timed out: '+label)

    def target(self):
        if self.locked_target is not None:
            return copy.deepcopy(self.locked_target)
        limit = self.cfg['motion']['max_target_age_s']
        if self.targets is None or time.monotonic()-self.target_arrival > limit:
            raise RuntimeError('No fresh vision result')
        now = self.get_clock().now().nanoseconds*1e-9
        source = source_direction(self.cfg, self.direction)
        candidates = [x for x in self.targets.get('targets', []) if x['direction']==source]
        if len(candidates) != 1:
            if self.targets.get('reason'):
                raise RuntimeError('视觉未就绪：'+str(self.targets['reason']))
            detail = self.targets.get('target_status', {}).get(source, {}).get('reason')
            if detail:
                direction = {'up': '上行', 'down': '下行'}[source]
                raise RuntimeError(direction+'目标未就绪：'+detail)
            raise RuntimeError('未获得该方向唯一、稳定且深度有效的按钮目标')
        target = candidates[0]
        if not 0 <= now-target['stamp_sec'] <= limit:
            raise RuntimeError('Target image timestamp is stale or in the future')
        if target['frame_id'] != self.cfg['base_frame']:
            raise RuntimeError('Unexpected target coordinate frame')
        return acquired_target(self.cfg, self.direction, target)

    def lock_snapshot(self, target):
        self.locked_target = copy.deepcopy(target)
        if self.cfg['motion'].get('pause_inference_on_lock',False):
            self.trigger('/elevator/vision/freeze')
        self.event('TARGET_LOCKED',target=self.locked_target,
                   inference_paused=bool(self.cfg['motion'].get('pause_inference_on_lock',False)),
                   note='Fixed target for this task; subsequent vision loss or changes do not update it.')

    def poses(self, target):
        from .panel_motion import compensated_point
        point = compensated_point(target, self.cfg['motion'], self.arm)
        return press_poses(point, target['normal'],
            self.robot.wrist_tip(self.cfg['tools'][self.arm]), self.cfg['motion'],
            palm_normal=self.cfg['tools'][self.arm].get('palm_normal_xyz'))

    def health(self, moving=False):
        now = time.monotonic()
        if self.status is None or self.waist is None or now-self.status_arrival > .6 or now-self.waist_arrival > .6:
            raise RuntimeError('Driver feedback is missing or stale')
        s, w = self.status, self.waist
        if not w.sdk_connected:
            raise RuntimeError('Unitree SDK feedback disconnected')
        if s.emergency_stop_active or s.joint_limits_exceeded or s.collision_detected:
            raise RuntimeError('Driver safety status forbids motion')
        if s.left_has_error or s.right_has_error or s.current_state in (3,6,11):
            raise RuntimeError(f'Driver reports error: {s.left_error_message}; {s.right_error_message}')
        if moving and not w.has_upper_body_authority:
            raise RuntimeError('Upper-body authority was lost')
        if len(w.current_joint.positions) != 3 or not np.all(np.isfinite(w.current_joint.positions)):
            raise RuntimeError('Invalid measured waist joints')
        if self.count_publishers('/dual_arm/status') != 1 or self.count_publishers('/dual_arm/waist_status') != 1:
            raise RuntimeError('Exactly one driver instance is required')
        if hasattr(self, 'hover_guard'):
            self.hover_guard()

    def current_pose(self):
        p = self.status.current_pose
        return {'xyz': [getattr(p,self.arm+'_'+axis) for axis in ('x','y','z')],
                'rpy': [getattr(p,self.arm+'_'+axis) for axis in ('rx','ry','rz')]}

    def close_to(self, target, contact=False, ready_for_contact=False):
        actual = self.current_pose()
        distance = np.linalg.norm(np.asarray(actual['xyz'])-target['xyz'])
        error = Rotation.from_euler('XYZ',actual['rpy']).inv()*Rotation.from_euler('XYZ',target['rpy'])
        tolerance = self.cfg['motion']['position_tolerance_m']
        if ready_for_contact or contact:
            tip = self.robot.wrist_tip(self.cfg['tools'][self.arm])
            actual_tip = np.asarray(actual['xyz'])+Rotation.from_euler('XYZ',actual['rpy']).apply(tip)
            wanted_tip = np.asarray(target['xyz'])+Rotation.from_euler('XYZ',target['rpy']).apply(tip)
            if contact:
                normal=np.asarray(self.locked_target['normal'],float)
                normal/=np.linalg.norm(normal)
                delta=actual_tip-wanted_tip
                normal_error=float(delta@normal)
                tangent_error=np.linalg.norm(delta-normal*normal_error)
                contact_tolerance=float(self.cfg['motion'].get('contact_position_tolerance_m',.005))
                if not 0 < contact_tolerance <= .005:
                    raise RuntimeError('Contact position tolerance must be within (0, 5 mm]')
                return abs(normal_error)<contact_tolerance and tangent_error<float(self.cfg['motion'].get('button_aperture_half_width_m',.020)) and error.magnitude()<.08
            return distance<tolerance and np.linalg.norm(actual_tip-wanted_tip)<.015 and error.magnitude()<.06
        return distance < tolerance and error.magnitude() < self.cfg['motion']['orientation_tolerance_rad']

    def trigger(self, name):
        client = self.create_client(Trigger,name)
        try:
            self.wait(client.service_is_ready,5,name+' service')
            future = client.call_async(Trigger.Request())
            self.wait(future.done,8,name+' response')
            answer = future.result()
            if not answer.success:
                raise RuntimeError(name+': '+answer.message)
            return answer
        finally:
            self.destroy_client(client)

    def configure_panel(self, target, phase):
        """Send one atomic scene snapshot to the same node publishing status."""
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        from .panel_motion import panel_snapshot
        values = panel_snapshot(self.cfg, self.arm, target, phase)
        peers = self.get_publishers_info_by_topic('/dual_arm/status')
        if len(peers) != 1:
            raise RuntimeError('Exactly one driver is required for panel scene setup')
        peer = peers[0]
        remote = peer.node_namespace.rstrip('/')+'/'+peer.node_name
        client = self.create_client(SetParametersAtomically, remote+'/set_parameters_atomically')
        try:
            self.wait(client.service_is_ready, 5, 'driver panel parameter service')
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter('elevator_panel', value=values).to_parameter_msg()]
            request.parameters.append(Parameter('elevator_withdraw_clearance_m',value=withdrawal_clearance(self.cfg)).to_parameter_msg())
            future = client.call_async(request)
            self.wait(future.done, 5, 'driver panel snapshot acknowledgement')
            answer = future.result()
            if answer is None or not answer.result.successful:
                raise RuntimeError('Driver rejected panel scene: '+str(answer.result.reason if answer else 'no response'))
        finally:
            self.destroy_client(client)
        if phase=='withdraw':
            self.hover_guard.begin_withdrawal()
        else:
            self.hover_guard.phase = 'contact' if phase=='contact' else 'transit'
        self.event('PANEL_SCENE_SET', phase=phase)

    def move(self, target, linear, stage):
        from brsd_msgs.msg import DualArmMovePJ, DualArmMoveL
        self.health(moving=True)
        if self.status.current_state == DualArmStatus.MOVING:
            raise RuntimeError('Another motion is already active')
        message_type = DualArmMoveL if linear else DualArmMovePJ
        topic = '/dual_arm/move_l' if linear else '/dual_arm/move_pj'
        pub = self.create_publisher(message_type,topic,1)
        try:
            self.wait(lambda: pub.get_subscription_count()==1 and self.count_publishers('/elevator/motion_feedback')==1,5,'one motion subscriber and correlated feedback publisher')
            msg = message_type()
            msg.header.frame_id = self.cfg['base_frame']
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.arm_mask = 1 if self.arm=='left' else 2
            for name,value in zip(('x','y','z','rx','ry','rz'),target['xyz']+target['rpy']):
                setattr(msg.pose,self.arm+'_'+name,float(value))
            msg.speed, msg.a = motion_scales(self.cfg['motion'], stage)
            # Never rely on plan_only: the installed G1 handler does not implement it.
            seq = self.status_sequence
            self.event(stage,wrist_target=target,speed_scale=msg.speed,acceleration_scale=msg.a)
            key=('MoveL' if linear else 'MovePJ',int(msg.arm_mask),msg.header.stamp.sec,msg.header.stamp.nanosec)
            self.motion_feedback.pop(key,None)
            pub.publish(msg)
            accepted=False
            observed_motion = False
            motion_started = None
            command_sent = time.monotonic()
            planning_deadline=command_sent+float(self.cfg['motion'].get('planning_timeout_s',8.))
            deadline = command_sent+self.cfg['motion']['command_timeout_s']
            while time.monotonic()<deadline and rclpy.ok():
                self.spin_once(timeout_sec=.04)
                feedback=self.motion_feedback.get(key,{})
                if feedback.get('phase')=='rejected':
                    # A correlated rejection proves PRESS was never submitted.
                    # We are still at the verified preparation point.
                    if stage=='PRESS' and not accepted and self.hover_guard.current_clearance>=.04:
                        self.contact_pending=False
                    raise RuntimeError('轨迹规划/接收失败：'+feedback.get('reason','未知原因'))
                if feedback.get('phase')=='accepted' and not accepted:
                    accepted=True
                    self.event('MOTION_ACCEPTED',motion_stage=stage)
                if not accepted and time.monotonic()>=planning_deadline:
                    raise RuntimeError('轨迹规划确认超时：'+stage)
                self.health(moving=True)
                if not accepted or self.status_sequence <= seq:
                    continue
                observed_motion |= self.status.current_state==DualArmStatus.MOVING
                if observed_motion and motion_started is None:
                    motion_started = time.monotonic()
                if stage=='PRESS':
                    actual=self.current_pose()
                    tip=np.asarray(actual['xyz'])+Rotation.from_euler('XYZ',actual['rpy']).apply(
                        self.robot.wrist_tip(self.cfg['tools'][self.arm]))
                    normal=np.asarray(self.locked_target['normal'],float)
                    normal/=np.linalg.norm(normal)
                    gap=float((tip-np.asarray(self.locked_target['point']))@normal)
                    if gap<=self.cfg['motion']['hover_m'] and self.press_contact_started_at is None:
                        self.press_contact_started_at=time.monotonic()
                        self.event('CONTACT_STARTED',tip_panel_gap_m=gap,
                                   contact_limit_s=self.cfg['motion']['contact_timeout_s'])
                    if self.press_contact_started_at is not None and self.contact_remaining()<=0.:
                        self.press_completion={'endpoint_reached':False,'termination':'contact_time_limit',
                                               'measured_tip_gap_m':gap,'button_activation_verified':False}
                        self.event('CONTACT_TIME_LIMIT',press_completion=self.press_completion)
                        self.cancel_motion()
                        self.wait(lambda:self.status is not None and self.status.current_state==DualArmStatus.IDLE,
                                  3.,'contact time limit cancel acknowledgement')
                        return False
                if self.status.current_state==DualArmStatus.IDLE and self.close_to(target,contact=stage=='PRESS',ready_for_contact=stage=='APPROACH'):
                    # Feedback must be newer than this publication; a no-op is valid.
                    if observed_motion or time.monotonic()-command_sent>.5:
                        if stage=='PRESS':
                            actual=self.current_pose()
                            tip=self.robot.wrist_tip(self.cfg['tools'][self.arm])
                            actual_tip=np.asarray(actual['xyz'])+Rotation.from_euler('XYZ',actual['rpy']).apply(tip)
                            wanted_tip=np.asarray(target['xyz'])+Rotation.from_euler('XYZ',target['rpy']).apply(tip)
                            normal=np.asarray(self.locked_target['normal'],float);normal/=np.linalg.norm(normal)
                            self.press_completion={
                                'endpoint_reached':True,
                                'normal_error_m':float((actual_tip-wanted_tip)@normal),
                                'measured_tip_gap_m':float((actual_tip-np.asarray(self.locked_target['point']))@normal),
                                'normal_tolerance_m':float(self.cfg['motion'].get('contact_position_tolerance_m',.005)),
                                'button_activation_verified':False}
                        self.event('MOTION_REACHED',motion_stage=stage,wrist_target=target,
                                   **({'press_completion':self.press_completion} if stage=='PRESS' else {}))
                        return True
            self.event('MOTION_TIMEOUT',motion_stage=stage,wrist_target=target)
            raise RuntimeError('Motion did not reach the measured endpoint: '+stage)
        finally:
            self.destroy_publisher(pub)

    def move_joints(self, target, stage):
        """Return to a measured joint pose without re-solving a singular wrist IK."""
        from brsd_msgs.msg import DualArmMoveJ
        self.health(moving=True)
        if self.status.current_state != DualArmStatus.IDLE:
            raise RuntimeError('Another motion is already active')
        for arm in ('left', 'right'):
            if len(target[arm]) != 7 or not np.all(np.isfinite(target[arm])):
                raise RuntimeError('Invalid measured return joint pose')
        pub = self.create_publisher(DualArmMoveJ, '/dual_arm/move_j', 1)
        try:
            self.wait(lambda: pub.get_subscription_count()==1, 5, 'one joint motion subscriber')
            msg = DualArmMoveJ()
            msg.header.frame_id = self.cfg['base_frame']
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.arm_mask = 1 if self.arm=='left' else 2
            msg.joint.left_positions = list(target['left'])
            msg.joint.right_positions = list(target['right'])
            msg.speed, msg.a = motion_scales(self.cfg['motion'], stage)
            self.event(stage, joint_target=target,speed_scale=msg.speed,acceleration_scale=msg.a)
            seq = self.status_sequence
            pub.publish(msg)
            observed = False
            sent = time.monotonic()
            while time.monotonic()-sent < self.cfg['motion']['command_timeout_s']:
                self.spin_once( timeout_sec=.04)
                self.health(moving=True)
                if self.status_sequence <= seq:
                    continue
                observed |= self.status.current_state==DualArmStatus.MOVING
                current = getattr(self.status.current_joint, self.arm+'_positions')
                error = max(abs(a-b) for a,b in zip(current, target[self.arm]))
                if (self.status.current_state==DualArmStatus.IDLE and error < .035 and
                    (observed or time.monotonic()-sent>.5)):
                    self.event('MOTION_REACHED', motion_stage=stage, max_joint_error_rad=float(error))
                    return
            raise RuntimeError('Joint return did not reach measured endpoint')
        finally:
            self.destroy_publisher(pub)

    def cancel_motion(self):
        from brsd_msgs.msg import DualArmMotionSequence
        pub = self.create_publisher(DualArmMotionSequence,'/dual_arm/motion_sequence',1)
        try:
            self.wait(lambda: pub.get_subscription_count()==1,2,'cancel subscriber')
            msg = DualArmMotionSequence()
            msg.arm_mask, msg.cancel, msg.enable = 3, True, False
            pub.publish(msg)
            for _ in range(5):
                self.spin_once(timeout_sec=.05)
        finally:
            self.destroy_publisher(pub)

    def verify_withdrawal(self):
        self.health(moving=True)
        if self.status.current_state != DualArmStatus.IDLE:
            raise RuntimeError('Withdrawal not verified: arm is not stationary')
        clearance = self.hover_guard.current_clearance
        required=withdrawal_clearance(self.cfg)
        if clearance < required:
            raise RuntimeError(f'Withdrawal not verified: measured model hand clearance below {required*1000:g} mm')
        self.contact_pending = False
        self.event('WITHDRAW_VERIFIED', hand_clearance_m=float(clearance))

    def release_if_safely_withdrawn(self):
        """Refresh feedback and release from a verified stationary clear pose.

        No trajectory, recovery motion or return is sent by this method.
        """
        sequence=self.status_sequence
        self.wait(lambda:self.status is not None and self.waist is not None
                  and self.status_sequence>sequence
                  and time.monotonic()-self.status_arrival<.6
                  and time.monotonic()-self.waist_arrival<.6,
                  2.,'fresh feedback before retained-authority release')
        if not self.waist.sdk_connected:
            raise RuntimeError('Unitree SDK feedback disconnected')
        if not self.waist.has_upper_body_authority:
            self.acquired=False
            return
        self.verify_withdrawal()
        for arm in ('left','right'):
            velocities=getattr(self.status.current_joint,arm+'_velocities')
            if len(velocities)!=7 or not np.all(np.isfinite(velocities)) or max(abs(x) for x in velocities)>self.cfg['motion']['max_joint_speed']:
                raise RuntimeError('Arms are not stationary enough for authority release')
        print('已确认手部间隙和停稳状态，自动归还控制权',flush=True)
        self.trigger('/dual_arm/end')
        self.acquired=False

    def withdraw_contact(self, stage='WITHDRAW'):
        # Drop only the inward pressing corridor. Driver feedback, whole-hand
        # penetration, joint/scene collisions and link clearances remain.
        self.hover_guard.begin_withdrawal()
        self.health(moving=True)
        self.configure_panel(self.locked_target,'withdraw')
        # The driver reverses the accepted linear joint path, including only
        # its executed prefix after cancellation. No new Cartesian IK/plan.
        seq=self.status_sequence
        self.event(stage,trajectory_retrace=True)
        answer=self.trigger('/dual_arm/retrace_'+self.arm)
        details=json.loads(answer.message)
        mask=1 if self.arm=='left' else 2
        if details.get('arm_mask')!=mask or len(details.get('target_positions',[]))!=17:
            raise RuntimeError('Invalid recorded-retrace acknowledgement')
        offset=0 if self.arm=='left' else 7
        target=details['target_positions'][offset:offset+7]
        if not np.all(np.isfinite(target)):
            raise RuntimeError('Non-finite retrace endpoint')
        self.event('MOTION_ACCEPTED',motion_stage=stage,trajectory_retrace=True)
        started=time.monotonic()
        while time.monotonic()-started<self.cfg['motion']['command_timeout_s']:
            self.spin_once(timeout_sec=.04)
            self.health(moving=True)
            current=getattr(self.status.current_joint,self.arm+'_positions')
            if (self.status_sequence>seq and time.monotonic()-started>.25 and
                    self.status.current_state==DualArmStatus.IDLE and len(current)==7 and
                    max(abs(a-b) for a,b in zip(current,target))<=.035):
                self.event('MOTION_REACHED',motion_stage=stage,trajectory_retrace=True)
                break
        else:
            raise RuntimeError('Recorded reverse trajectory did not reach its measured endpoint')
        self.verify_withdrawal()

    def recover_contact(self):
        if not self.contact_pending:
            return
        self.wait(lambda:self.status is not None and self.status.current_state==DualArmStatus.IDLE,
                  3.,'cancelled motion idle acknowledgement')
        self.withdraw_contact('RECOVERY_WITHDRAW')
        self.finish_withdrawal(recovery=True)

    def finish_withdrawal(self, recovery=False):
        if self.contact_pending:
            raise RuntimeError('Cannot finish before measured withdrawal is verified')
        if self.cfg['motion'].get('return_to_start_after_withdraw',True):
            self.configure_panel(self.locked_target,'return')
            self.move(self.initial_pose,False,'RECOVERY_RETURN' if recovery else 'RETURN')
        else:
            self.event('READY_TO_RELEASE',business_return_skipped=True)

    def check_execution_ready(self):
        verify_driver_runtime(self.cfg['motion'])
        # Process attestation can outlast the freshness window while this
        # worker is not spinning. Require both subscriptions to advance after
        # it completes; cached non-None messages are not a readiness barrier.
        checked_at = time.monotonic()
        self.wait(lambda: self.status is not None and self.waist is not None
                  and self.status_arrival >= checked_at and self.waist_arrival >= checked_at
                  and time.monotonic()-self.status_arrival <= .6
                  and time.monotonic()-self.waist_arrival <= .6,
                  5,'fresh arm and waist feedback after driver verification')
        self.health()
        if self.status.current_state == DualArmStatus.MOVING:
            raise RuntimeError('Robot is already moving')
        velocities = self.status.current_joint.left_velocities+self.status.current_joint.right_velocities
        if velocities and max(abs(x) for x in velocities)>self.cfg['motion']['max_joint_speed']:
            raise RuntimeError('Arms must be stationary before starting')

    def prepare_target(self, wait_s, execute):
        """Select and lock a fresh target after slow startup checks finish."""
        deadline=time.monotonic()+wait_s
        reason='等待新鲜稳定按钮目标'
        last_report=0.
        while time.monotonic()<deadline:
            try:
                target=self.target()
            except RuntimeError as error:
                reason=str(error)
            else:
                if execute and self.targets.get('camera_calibration_id') != camera_signature(self.cfg):
                    raise RuntimeError('Camera configuration changed; restart vision before physical execution')
                selection={'mode':self.requested_arm}
                if self.requested_arm=='auto':
                    width=self.targets.get('image_width')
                    self.arm=arm_from_image(target.get('center_xy'),width)
                    selection.update(center_xy=target['center_xy'],image_width=width,
                                     image_midline_x=width/2,locked_for_task=True)
                poses=self.poses(target)
                if not execute:
                    return target,poses,selection
                initial=self.current_pose()
                if not np.all(np.isfinite(initial['xyz']+initial['rpy'])):
                    raise RuntimeError('Invalid measured wrist pose')
                age=self.get_clock().now().nanoseconds*1e-9-target['stamp_sec']
                if 0 <= age <= self.cfg['motion']['max_target_age_s']:
                    self.initial_pose=copy.deepcopy(initial)
                    self.lock_snapshot(target)
                    return target,poses,selection
                reason='准备期间目标已过期，等待新帧后重新锁定'
            now=time.monotonic()
            if now-last_report>=5. or last_report==0.:
                self.event('WAITING_TARGET',reason=reason,remaining_s=max(0.,deadline-now))
                last_report=now
            self.spin_once(timeout_sec=.04)
        raise RuntimeError('Timed out waiting for a fresh target snapshot: '+reason)

    def contact_remaining(self):
        limit=float(self.cfg['motion']['contact_timeout_s'])
        if not np.isfinite(limit) or limit<=0:
            raise RuntimeError('Contact time limit must be positive and finite')
        started=getattr(self,'press_contact_started_at',None)
        return limit if started is None else max(0.,limit-(time.monotonic()-started))

    def run(self, execute=False, wait_s=15, hover_only=False):
        if hover_only and not execute:
            raise RuntimeError('Hover-only is physical motion and requires --execute')
        self.press_contact_started_at = None
        self.locked_target = None
        self.targets = None
        if self.cfg['motion'].get('pause_inference_on_lock',False):
            # TensorRT loading can outlast the normal 5-second RPC discovery.
            ready = self.create_client(Trigger, '/elevator/vision/resume')
            try:
                self.wait(ready.service_is_ready, wait_s, 'vision initialization')
            finally:
                self.destroy_client(ready)
            self.trigger('/elevator/vision/resume')
        # Discovery, process scanning and robot checks must not consume the
        # lifetime of a target already selected from a camera frame.
        if execute:self.check_execution_ready()
        target,poses,selection=self.prepare_target(wait_s,execute)
        self.retreat_pose=copy.deepcopy(poses['approach'])
        self.event('PREVIEW',target=target,poses=poses,arm_selection=selection,
                   calibration_verified=bool(self.cfg['camera']['verified'] and self.cfg['tools'][self.arm]['verified']),
                   note='Geometric target; motion has not started.')
        if not execute:return
        if hover_only:
            from .hover_guard import HoverGuard
            if self.cfg['motion']['approach_m'] < .06:
                raise RuntimeError('Nominal-model hover requires at least 60 mm clearance')
            speed, acceleration = motion_scales(self.cfg['motion'], 'HOVER_APPROACH')
            self.cfg['motion']['command_timeout_s'] = max(self.cfg['motion']['command_timeout_s'],90.)
            self.hover_guard = HoverGuard(self, target)
            self.health()
            self.event('NOMINAL_MODEL_HOVER',clearance_m=self.cfg['motion']['approach_m'],
                       speed_scale=speed,acceleration_scale=acceleration,
                       note='Non-contact model validation; calibration remains unverified.')
        if not hover_only:
            from .hover_guard import HoverGuard
            self.hover_guard = HoverGuard(self, target)
            self.health()
        self.configure_panel(target, 'transit')
        if not self.waist.has_upper_body_authority:
            self.event('ACQUIRE_AUTHORITY')
            # Own cleanup as soon as the request is sent, including interruption
            # while the service is ramping up authority.
            self.acquired = True
            self.trigger('/dual_arm/start')
            self.wait(lambda: self.waist.has_upper_body_authority,5,'authority feedback')
        self.executing = True
        self.health(moving=True)
        if hover_only:
            self.move(poses['approach'],False,'HOVER_APPROACH')
            self.event('HOVER_HOLD',nominal_tip_clearance_m=self.cfg['motion']['approach_m'],
                       wrist_target=poses['approach'])
            until = time.monotonic()+3.
            while time.monotonic()<until:
                self.spin_once(timeout_sec=.04)
                self.health(moving=True)
            self.configure_panel(target, 'return')
            self.move(self.initial_pose,False,'HOVER_RETURN')
            self.event('HOVER_COMPLETED',contact_attempted=False,calibration_verified=False,
                       nominal_min_hand_clearance_m=self.hover_guard.minimum_seen)
            return
        # Exactly two forward motions: safe free-space approach, then normal LIN.
        self.move(poses['approach'],False,'APPROACH')
        self.configure_panel(target, 'contact')
        # From here through measured withdrawal, release could let the factory
        # controller swing the hand home across the button panel.
        self.contact_pending = True
        press_reached=self.move(poses['press'],True,'PRESS')
        remaining=min(float(self.cfg['motion']['dwell_s']),self.contact_remaining()) if press_reached is not False else 0.
        self.event('DWELL',remaining_contact_s=remaining)
        until = time.monotonic()+remaining
        while time.monotonic()<until:
            self.spin_once(timeout_sec=.04)
            self.health(moving=True)
        self.withdraw_contact('WITHDRAW')
        self.finish_withdrawal()
        self.event('CONTACT_SEQUENCE_COMPLETED', press_verified=False,
                   note='Motion completed. Button activation still requires lamp/contact feedback or operator confirmation.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--direction',choices=['up','down'],required=True)
    parser.add_argument('--arm',choices=['auto','left','right'],default='auto',
                        help='Auto: source image left half uses left arm; right half uses right arm; locked per task')
    parser.add_argument('--execute',action='store_true',help='Explicitly acquire control and move the selected arm')
    parser.add_argument('--hover-only',action='store_true',help='Non-contact nominal-model validation: 60 mm approach, hold, return; never press')
    parser.add_argument('--wait',type=float,default=15)
    parser.add_argument('--config')
    args, ros_args = parser.parse_known_args()
    # Keep DDS alive during Ctrl+C/SIGTERM cleanup so cancellation and release
    # actually reach the driver before shutting down the ROS context.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    def interrupted(signum, frame):
        if not stopping[0]:
            stopping[0] = True
            raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node, error = None, None
    try:
        node = PressTask(load(args.config),args.direction,args.arm)
        node.run(args.execute,args.wait,args.hover_only)
    except (Exception,KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
        if node:
            node.event('ABORTED',reason=error)
            if node.executing:
                try:
                    node.cancel_motion()
                    node.recover_contact()
                except Exception as stop_error:
                    node.get_logger().error('Contact cleanup failed: '+str(stop_error))
                    try:node.cancel_motion()
                    except Exception as cancel_error:node.get_logger().error(str(cancel_error))
    finally:
        if node:
            if node.acquired and node.contact_pending:
                node.get_logger().error('撤回未确认：保留控制权保持当前位置，禁止自动归位；需要人工处理')
                error = error or 'Contact recovery failed; authority retained'
            if node.acquired and not node.contact_pending:
                try:
                    print('归还控制权',flush=True)
                    node.trigger('/dual_arm/end')
                except Exception as release_error:
                    node.get_logger().error('Authority release failed: '+str(release_error))
                    error = error or str(release_error)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if error:
        if node is None:
            print(error, flush=True)
        raise SystemExit(1)
