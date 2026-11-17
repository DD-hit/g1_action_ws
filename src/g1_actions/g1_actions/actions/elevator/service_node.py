"""Persistent start/stop ROS 2 services with one cancellable task worker."""
import copy
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger
from brsd_msgs.msg import DualArmStatus, WaistStatus, DualArmMotionSequence
from .task_node import PressTask
from .log_view import format_result, log_line
from .camera_health import FrameReadiness,CameraReceipts
from .config import withdrawal_clearance


class TaskCancelled(Exception):
    pass


class ManagedPressTask(PressTask):
    def __init__(self, cfg, direction, arm, stop_event):
        super().__init__(cfg,direction,arm)
        self.stop_event=stop_event

    def check_stop(self):
        if self.stop_event.is_set():raise TaskCancelled(getattr(self.stop_event,'reason','用户请求停止'))

    def health(self,moving=False):
        self.check_stop()
        return super().health(moving)

    def target(self):
        self.check_stop()
        return super().target()

    def wait(self,predicate,timeout,label):
        until=time.monotonic()+timeout
        while time.monotonic()<until and rclpy.ok():
            self.check_stop()
            rclpy.spin_once(self,timeout_sec=.04)
            if predicate():return
        raise RuntimeError('Timed out: '+label)

    def trigger(self,name):
        # Cancellation must not cancel the cleanup service request itself.
        if name=='/dual_arm/end':
            saved=self.stop_event
            self.stop_event=threading.Event()
            try:return super().trigger(name)
            finally:self.stop_event=saved
        return super().trigger(name)

    def resume_idle(self):
        saved=self.stop_event
        self.stop_event=threading.Event()
        try:return super().trigger('/elevator/vision/resume')
        finally:self.stop_event=saved

    def recover_contact(self):
        # A user stop cancels forward motion, but must not interrupt guarded
        # withdrawal and leave the hand at the panel. Hardware faults still stop it.
        saved=self.stop_event
        self.stop_event=threading.Event()
        try:return super().recover_contact()
        finally:self.stop_event=saved

    def cancel_motion(self):
        saved=self.stop_event
        self.stop_event=threading.Event()
        try:return super().cancel_motion()
        finally:self.stop_event=saved

    def release_if_safely_withdrawn(self):
        saved=self.stop_event
        self.stop_event=threading.Event()
        try:return super().release_if_safely_withdrawn()
        finally:self.stop_event=saved


class TaskService(Node):
    def __init__(self,cfg,direction='up',arm='auto',task_factory=ManagedPressTask):
        super().__init__('ele_elevator_service')
        self.cfg=cfg;self.task_factory=task_factory
        self.declare_parameter('direction',direction)
        self.declare_parameter('arm',arm)
        self.ready=False;self.dependency_error='等待相机、视觉和驱动就绪'
        self.lock=threading.Lock();self.worker=None;self.task=None
        self.retained_task=None
        self.stop_event=threading.Event();self.closed=False
        self.perception_recovering=False
        self.camera_recovery_status={}
        self.status=None;self.waist=None
        self.status_arrival=self.waist_arrival=0.
        camera=cfg.get('camera',{})
        self.vision_gate=FrameReadiness(
            max_frame_age=camera.get('max_frame_age',.6),
            stable_seconds=camera.get('ready_stable_s',2.),
            min_frames=camera.get('ready_min_frames',6))
        self.vision_health={}
        self.camera_receipts=CameraReceipts()
        self.stage='INITIALIZING';self.last_result=None
        self.record_callback=None
        self.events=self.create_publisher(String,'/elevator/service_status',10)
        self.cancel_pub=self.create_publisher(DualArmMotionSequence,'/dual_arm/motion_sequence',1)
        self.create_subscription(DualArmStatus,'/dual_arm/status',self.on_status,10)
        self.create_subscription(WaistStatus,'/dual_arm/waist_status',self.on_waist,10)
        self.create_subscription(String,'/elevator/targets',self.on_vision,1)
        self.create_subscription(String,'/elevator/camera_health',self.on_camera_health,1)
        self.resume_client=self.create_client(Trigger,'/elevator/vision/resume')
        self.freeze_client=self.create_client(Trigger,'/elevator/vision/freeze')
        self.start_group=ReentrantCallbackGroup()
        self.create_service(Trigger,'/elevator/start',self.execute_start,callback_group=self.start_group)
        self.create_service(Trigger,'/elevator/stop',self.stop)
        self.create_timer(.25,self.monitor)

    def on_status(self,msg):self.status=msg;self.status_arrival=time.monotonic()
    def on_waist(self,msg):self.waist=msg;self.waist_arrival=time.monotonic()

    def on_vision(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('Expected object')
            self.vision_health=data.get('camera_health',{})
            self.camera_receipts.observe(self.vision_health,time.monotonic())
            self.vision_gate.observe(data,time.monotonic(),self.get_clock().now().nanoseconds*1e-9)
        except (ValueError,TypeError):
            self.vision_gate.invalidate('视觉消息格式无效')

    def on_camera_health(self,msg):
        try:
            health=json.loads(msg.data)
            if not isinstance(health,dict):return
            self.camera_receipts.observe(health,time.monotonic())
            self.vision_health=health
        except (ValueError,TypeError):return

    def dependencies_ready(self):
        now=time.monotonic()
        if self.last_result and self.last_result.get('authority_retained'):
            released=(self.waist is not None and now-self.waist_arrival<.6
                      and not self.waist.has_upper_body_authority)
            if not released:
                self.dependency_error='撤回状态未确认，驱动仍保持控制权；退出时自动复核，禁止再次 start'
                self.stage='RECOVERY_REQUIRED'
                return False
            self.last_result.update(authority_retained=False,recovery_resolved=True)
        if self.perception_recovering:
            self.dependency_error='相机断流，正在自动重启相机和视觉；等待重新出图'
            return False
        core_ready=(self.status is not None and self.waist is not None and
            now-self.status_arrival<.6 and now-self.waist_arrival<.6 and
            self.count_publishers('/dual_arm/status')==1 and
            self.count_publishers('/elevator/targets')==1 and
            self.count_publishers('/elevator/motion_feedback')==1 and
            self.resume_client.service_is_ready() and self.freeze_client.service_is_ready())
        target_locked=self.task is not None and getattr(self.task,'locked_target',None) is not None
        frames_ready,vision_reason=(True,'') if target_locked else self.vision_gate.check(
            now,self.get_clock().now().nanoseconds*1e-9)
        if core_ready and not frames_ready:
            self.dependency_error='视觉未就绪：'+vision_reason
        elif not core_ready and not self.dependency_error.startswith('组件退出'):
            self.dependency_error='等待视觉服务和驱动反馈就绪'
        elif not self.dependency_error.startswith('组件退出'):
            self.dependency_error=''
        return core_ready and frames_ready

    def refresh_readiness(self):
        ready=self.dependencies_ready() and not self.dependency_error.startswith('组件退出')
        if ready!=self.ready:
            self.ready=ready
            if ready:
                if self.worker is None:
                    self.stage='READY'
                    print('已就绪：RGB-D 已连续稳定；调用 /elevator/start 开始，/elevator/stop 停止。',flush=True)
            else:
                print('未就绪：'+self.dependency_error,flush=True)
                if self.worker is None:
                    self.stage='RECOVERY_REQUIRED' if self.last_result and self.last_result.get('authority_retained') else 'NOT_READY'
                else:
                    self.stop_event.reason=self.dependency_error
                    self.stop_event.set()
        return ready

    def monitor(self):
        self.refresh_readiness()
        self.events.publish(String(data=json.dumps({'ready':self.ready,'stage':self.stage,
            'busy':self.worker is not None,'last_result':self.last_result,
            'reason':self.dependency_error,
            'camera_recovery':self.camera_recovery_status,
            'camera_readiness':self.vision_gate.diagnostics(
                time.monotonic(),self.get_clock().now().nanoseconds*1e-9),
            'camera_health':self.vision_health},ensure_ascii=False)))

    async def execute_start(self,request,response):
        completion=Future(executor=self.executor)
        accepted=self.start(request,response,completion)
        if not accepted.success:return accepted
        result=await completion
        response.success=result.get('result')=='completed' and result.get('authority_released') is True
        response.message=json.dumps(result,ensure_ascii=False)
        return response

    def start(self,request,response,completion=None):
        guard=getattr(self,'start_guard',None)
        if guard is not None:return guard(request,response,completion)
        return self._start_impl(request,response,completion)

    def _start_impl(self,request,response,completion=None,direction=None,arm=None):
        with self.lock:
            if self.closed:response.message='服务正在退出';return response
            if self.worker is not None:response.message='已有任务执行中，请先 stop';return response
            if not self.refresh_readiness():response.message=self.dependency_error;return response
            if self.waist is not None and self.waist.has_upper_body_authority:
                response.message='上半身控制权已被占用，拒绝接管其他任务';return response
            if self.retained_task is not None:
                self.retained_task.destroy_node();self.retained_task=None
            direction=self.get_parameter('direction').value if direction is None else direction
            arm=self.get_parameter('arm').value if arm is None else arm
            if direction not in ('up','down') or arm not in ('auto','left','right'):
                response.message='direction 必须为 up/down，arm 必须为 auto/left/right';return response
            self.stop_event=threading.Event();self.stage='STARTING'
            self.worker=threading.Thread(target=self.run_task,args=(direction,arm,self.stop_event,completion),daemon=False)
            self.worker.start()
            response.success=True
            arm_label={'auto':'自动选择','left':'左手','right':'右手'}[arm]
            response.message=(f'已接受按压任务：目标={"上行" if direction=="up" else "下行"}（{direction}），机械臂={arm_label}。'
                              '正在获取新鲜、稳定的按钮目标，尚未确认执行成功。'
                              '后续流程：接管控制权 → 到预备点 → 直线按压 → 撤回 → 归还控制权。'
                              '执行进度、最终结果及失败原因见 /elevator/service_status；'
                              '查询：bash scripts/ele.sh status；停止：bash scripts/ele.sh stop。')
        return response

    def stop(self,request,response):
        guard=getattr(self,'stop_guard',None)
        if guard is not None:return guard(request,response)
        return self._stop_impl(request,response)

    def _stop_impl(self,request,response):
        with self.lock:
            active=self.worker is not None
            self.stop_event.reason='用户请求停止'
            self.stop_event.set()
            if active:
                self.stage='STOPPING'
                if self.task is not None and self.task.executing:
                    msg=DualArmMotionSequence();msg.arm_mask=3;msg.cancel=True;msg.enable=False
                    self.cancel_pub.publish(msg)
        retained=self.last_result is not None and self.last_result.get('authority_retained')
        response.success=not (retained and not active)
        response.message=('撤回状态未确认，控制权仍保持；退出时会自动复核安全释放条件' if retained and not active
                          else '已请求停止，正在取消运动并归还控制权；最终结果见 start 响应或 service_status' if active
                          else '当前没有执行中的任务')
        return response

    def record(self,task,stage,**extra):
        record={'time':time.time(),'stage':stage,**extra}
        if task is not None and task.status is not None and task.arm in ('left','right'):
            record['actual_pose']=task.current_pose()
            record['actual_joints']=list(getattr(task.status.current_joint,task.arm+'_positions'))
            record['locked_target']=task.locked_target
        if self.record_callback is not None:self.record_callback(record)
        observer=getattr(self,'action_event_callback',None)
        if observer is not None:
            try:observer(record)
            except Exception as error:self.get_logger().error('统一动作反馈转发失败：'+str(error))

    def run_task(self,direction,arm,stop_event,completion=None):
        started=time.monotonic();failed_stage='STARTING';owned=False
        task=None;result={'direction':direction,'arm':arm}
        try:
            task=self.task_factory(copy.deepcopy(self.cfg),direction,arm,stop_event)
            with self.lock:self.task=task
            original=task.event
            def event(stage,**extra):
                nonlocal failed_stage
                if stage not in ('MOTION_ACCEPTED','MOTION_REACHED','CONTACT_STARTED','CONTACT_TIME_LIMIT','WITHDRAW_VERIFIED'):
                    failed_stage=stage
                self.stage=stage
                self.record(task,stage,**extra)
                original(stage,**extra)
            task.event=event
            task.wait(lambda:task.status is not None and task.waist is not None,8,'driver feedback')
            if task.waist.has_upper_body_authority:raise RuntimeError('控制权已被其他任务占用')
            task.run(execute=True,wait_s=45,hover_only=False)
            result['result']='completed'
            if getattr(task,'press_completion',None) is not None:
                result['press_completion']=task.press_completion
        except (Exception,KeyboardInterrupt) as exc:
            result.update(result='stopped' if stop_event.is_set() and getattr(stop_event,'reason','用户请求停止')=='用户请求停止' else 'aborted',reason=str(exc))
            result['failed_stage']=failed_stage
            if self.record_callback is not None:self.record(task,'ABORTED',reason=str(exc))
            if task is not None and task.executing:
                try:
                    task.cancel_motion()
                except Exception as cleanup:
                    result['cancel_error']=str(cleanup)
        finally:
            if task is not None:
                result['arm']=getattr(task,'arm',arm)
                owned=bool(task.acquired)
                if task.acquired:
                    try:
                        print(log_line('5/5','归还控制权'),flush=True)
                        task.trigger('/dual_arm/end')
                        task.acquired=False
                        result.update(authority_released=True,authority_retained=False)
                    except Exception as cleanup:
                        result.setdefault('failed_stage','RELEASE_AUTHORITY')
                        result.update(result='cleanup_failed',release_error=str(cleanup),authority_released=False,
                                      authority_retained=True,recovery_required='控制权释放接口失败，请检查驱动并手动归还控制权')
                else:
                    result.setdefault('authority_released',True)
                    result.setdefault('authority_retained',False)
                if self.cfg.get('motion',{}).get('pause_inference_on_lock') and hasattr(task,'resume_idle'):
                    self.vision_gate.invalidate('目标检测已恢复，等待新鲜稳定 RGB-D（相机持续运行）')
                    try:task.resume_idle()
                    except Exception as cleanup:
                        result['vision_resume_error']=str(cleanup)
                        if result.get('result')=='completed':
                            result['result']='cleanup_failed';result['failed_stage']='RESUME_VISION'
                if result.get('authority_retained'):
                    self.retained_task=task
                else:
                    try:task.destroy_node()
                    except Exception as cleanup:
                        result['node_cleanup_error']=str(cleanup)
                        if result.get('result')=='completed':
                            result['result']='cleanup_failed';result['failed_stage']='NODE_CLEANUP'
            if task is None:
                result.update(authority_released=True,authority_retained=False)
            result['authority_acquired']=owned
            result['elapsed_s']=round(time.monotonic()-started,3)
            self.record(None,'TASK_FINISHED',result=result)
            print(log_line('结果',format_result(result)),flush=True)
            with self.lock:
                self.last_result=result;self.task=None;self.worker=None
                if result.get('authority_retained'):
                    self.ready=False;self.stage='RECOVERY_REQUIRED'
                    self.dependency_error=result['recovery_required']
                else:
                    self.stage='READY' if self.ready else 'NOT_READY'
            if completion is not None:completion.set_result(copy.deepcopy(result))

    def close(self):
        with self.lock:
            first=not self.closed
            self.closed=True
        if first:self.stop(None,Trigger.Response())
        with self.lock:worker=self.worker
        if worker is not None:worker.join(timeout=.1)
        if worker is not None and worker.is_alive():raise RuntimeError('任务清理尚未完成，保留 ROS 上下文')
        if self.last_result and self.last_result.get('authority_retained'):
            task=self.retained_task
            if task is None:
                raise RuntimeError('缺少原任务安全检查上下文，保留驱动')
            try:task.release_if_safely_withdrawn()
            except Exception as error:
                raise RuntimeError('尚未满足自动释放条件：'+str(error)) from error
            self.last_result.update(authority_released=True,authority_retained=False,recovery_resolved=True)
            print(json.dumps(self.last_result,ensure_ascii=False),flush=True)
        if self.retained_task is not None:
            self.retained_task.destroy_node();self.retained_task=None
