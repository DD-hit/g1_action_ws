"""Handshake worker shares the persistent server's ROS connections.

Perception stays in the project venv/GPU process. Motion code only requires the
same ROS/NumPy environment as the elevator worker; no per-task subprocess.
"""
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace

import yaml
from brsd_msgs.msg import DualArmStatus, WaistStatus, DualArmMovePJ, DualArmMotionSequence
from rclpy.node import Node
from std_msgs.msg import String
from .approach_once import Approach, execute_once


class HandshakeProvider(Node):
    def __init__(self, project, output):
        super().__init__('g1_handshake_provider')
        self.project,self.output=Path(project),Path(output)
        self.worker=self.task=None
        self.stop_event=threading.Event()
        self.condition=threading.Condition(threading.RLock())
        self.status=self.waist=self.vision=None
        self.status_at=self.waist_at=self.vision_at=0.
        self.feedback={}
        self.ready=False;self.dependency_error='等待驱动和手部识别'
        self.action_event_callback=None;self.closed=False;self.cleanup_uncertain=False
        self.create_subscription(DualArmStatus,'/dual_arm/status',self.on_status,1)
        self.create_subscription(WaistStatus,'/dual_arm/waist_status',self.on_waist,1)
        self.create_subscription(String,'/handshake/perception_status',self.on_vision,1)
        self.create_subscription(String,'/elevator/motion_feedback',self.on_feedback,10)
        self.motion_publisher=self.create_publisher(DualArmMovePJ,'/dual_arm/move_pj',1)
        self.cancel_publisher=self.create_publisher(DualArmMotionSequence,'/dual_arm/motion_sequence',1)

    def on_status(self,message):
        with self.condition:
            self.status,self.status_at=message,time.monotonic();self.condition.notify_all()

    def on_waist(self,message):
        with self.condition:
            self.waist,self.waist_at=message,time.monotonic();self.condition.notify_all()

    def on_vision(self,message):
        try:value=json.loads(message.data)
        except ValueError:value=None
        with self.condition:
            self.vision,self.vision_at=value,time.monotonic();self.condition.notify_all()

    def on_feedback(self,message):
        try:
            value=json.loads(message.data)
            key=(value['command'],value['arm_mask'],value['request_sec'],value['request_nanosec'])
        except (ValueError,TypeError,KeyError):return
        with self.condition:
            self.feedback[key]=value
            while len(self.feedback)>128:self.feedback.pop(next(iter(self.feedback)))
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return dict(status=self.status,status_at=self.status_at,waist=self.waist,waist_at=self.waist_at,
                        vision=self.vision,vision_at=self.vision_at,feedback=dict(self.feedback))

    def wait_update(self,timeout):
        with self.condition:self.condition.wait(timeout=timeout)

    def refresh_readiness(self):
        now=time.monotonic()
        if self.closed:reason='服务正在退出'
        elif self.status is None or self.waist is None or now-self.status_at>.6 or now-self.waist_at>.6:
            reason='未收到新鲜驱动反馈'
        elif not self.waist.sdk_connected:reason='机器人 SDK 未连接'
        elif self.status.emergency_stop_active or self.status.joint_limits_exceeded or self.status.collision_detected:
            reason='驱动安全检查未通过'
        elif self.status.left_has_error or self.status.right_has_error:reason='驱动处于错误状态'
        elif self.vision is None or now-self.vision_at>1. or 'stamp_s' not in self.vision:
            reason='等待 RGB-D 手部识别进程'
        else:reason=''
        self.dependency_error=reason;self.ready=not reason
        return self.ready

    def start(self,arm,params,completion,request_id=None):
        if self.worker is not None:return False,'握手动作正在执行'
        if not self.refresh_readiness():return False,self.dependency_error
        self.stop_event=threading.Event()
        self.task=SimpleNamespace(arm=arm,acquired=False)
        self.worker=threading.Thread(target=self.run,args=(arm,params,completion,request_id),daemon=True)
        self.worker.start()
        return True,'握手任务已开始'

    def event(self,value):
        if value.get('arm') in ('left','right'):self.task.arm=value['arm']
        if value.get('stage')=='ACQUIRE_AUTHORITY':self.task.acquired=True
        if self.action_event_callback is not None:
            try:self.action_event_callback(value)
            except Exception as error:self.get_logger().error('动作日志回调失败：'+str(error))

    def run(self,arm,params,completion,request_id=None):
        started_at=time.monotonic()
        node=None;raw=None
        try:
            self.output.mkdir(parents=True,exist_ok=True)
            folder=Path(tempfile.mkdtemp(prefix='handshake_'+((request_id+'_') if request_id else ''),dir=self.output))
            trial=folder/'trial';trial.mkdir()
            package=self.project/'src/g1_actions'
            config=yaml.safe_load((package/'config/handshake.yaml').read_text())
            node=Approach(config,package,arm,trial,feed=self)
            node.started_at=started_at
            node.acquired=False;self.task=node
            node.request_id=request_id;node.subject_handedness=params['subject_handedness']
            result=execute_once(node,execute=True,hold_s=params['hold_s'])
            acquired=bool(result.get('authority_requested'))
            released=bool(result.get('authority_released')) or not acquired
            raw=dict(result,result='completed' if result.get('success') else
                     'stopped' if result.get('cancelled') else 'aborted',
                     authority_acquired=acquired,authority_released=released,
                     authority_retained=acquired and not released,
                     arm=result.get('arm',self.task.arm),failed_stage=result.get('failure_stage',''))
            try:(trial/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            except OSError as error:raw['diagnostic_write_error']=str(error)
        except BaseException as error:
            # Unexpected worker failure must not claim that authority was released.
            acquired=bool(node is not None and node.owned)
            raw=dict(result='cleanup_failed' if acquired else 'aborted',reason=str(error),
                     arm=self.task.arm,authority_acquired=acquired,
                     authority_released=not acquired,authority_retained=acquired)
        finally:
            if node is not None:
                try:node.journal.close();node.destroy_node()
                except Exception as error:
                    raw.update(result='cleanup_failed',node_cleanup_error=str(error))
            self.task.acquired=bool(raw.get('authority_retained'))
            self.cleanup_uncertain=not bool(raw.get('authority_released'))
            self.worker=None
            completion.set_result(raw)

    def _stop_impl(self,request,response):
        self.stop_event.set()
        with self.condition:self.condition.notify_all()
        response.success=True
        return response
