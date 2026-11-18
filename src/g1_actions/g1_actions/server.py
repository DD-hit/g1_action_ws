"""One scheduler for unified Action calls and the elevator compatibility service."""
import copy
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal

from g1_action_interfaces.action import Execute
from g1_action_interfaces.msg import ExecutionGoal, ExecutionResult as Result, ExecutionFeedback, ActionInfo, ServerStatus
from g1_action_interfaces.srv import ListActions, GetExecution, StopActive
from .console import STAGE_LABELS, ARMS as ARM_LABELS, line as console_line, reason_text
from .protocol import ARMS, ARM_IDS, STAGES, InvalidRequest, validate, make_result, map_result


@dataclass
class Execution:
    request: object
    created: float
    done: object
    handle: object = None
    feedback: object = None
    result: object = None
    cancel_requested: bool = False
    deadline_exceeded: bool = False
    cancel_started: float = 0.
    started: bool = False
    legacy_completion: object = None
    raw_result: object = None
    transport_cancel_sent: bool = False
    transport_cancel_future: object = None
    ever_acquired: bool = False
    last_stage_at: float = 0.
    last_logged_stage: str = ''
    target_locked: bool = False
    notice_at: float = 0.
    notice_reason: str = ''


class Completion:
    def __init__(self, callback):
        self.callback = callback

    def set_result(self, value):
        self.callback(value)


class ActionLibraryServer(Node):
    def __init__(self, backend, cleanup_timeout_s=10., log_path=None, providers=None):
        super().__init__('g1_action_server')
        self.backend = backend
        backend.console_managed = True
        self.component_notices = {}
        self.providers = dict(providers or {})
        self.providers["elevator"] = backend
        self.cleanup_timeout_s = cleanup_timeout_s
        self.mutex = threading.RLock()
        self.executions = {}
        self.active = None
        self.fault = ''
        self.closed = False
        self.last_result = None
        self.log_path = Path(log_path) if log_path else None
        self.group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ServerStatus, '/g1_actions/status', qos)
        self.action_server = ActionServer(
            self, Execute, '/g1_actions/execute', execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=self.cancel, callback_group=self.group, result_timeout=1800.)
        self.cancel_client = self.create_client(CancelGoal, '/g1_actions/execute/_action/cancel_goal')
        self.create_service(ListActions, '/g1_actions/list_actions', self.list_actions)
        self.create_service(GetExecution, '/g1_actions/get_execution', self.get_execution)
        self.create_service(StopActive, '/g1_actions/stop_active', self.stop_active)
        self.timer = self.create_timer(.2, self.tick)
        backend.start_guard = self.legacy_start
        backend.stop_guard = self.legacy_stop
        for name, provider in self.providers.items():
            provider.action_event_callback = lambda event, name=name: self.observe(event, name)

    def observe(self, event, action_name="elevator"):
        with self.mutex:
            record = self.active
            if record is None or record.result is not None or record.request.action_name != action_name:
                return
            raw_stage = event['stage']
            if raw_stage == 'TARGET_LOCKED':
                record.target_locked = True
                self.log(record,raw_stage,'后续运动不再依赖视觉；继续监测驱动及控制权',detail=True)
            elif raw_stage == 'PREVIEW' and action_name == 'handshake':
                point=event.get('precontact_palm_xyz_m')
                if point:self.log(record,raw_stage,'掌心目标（torso_link）：['+', '.join(f'{x:.3f}' for x in point)+'] m',detail=True)
            elif raw_stage == 'FIXED_FALLBACK':
                self.log(record,raw_stage,'识别目标规划失败，尝试同侧固定握手位姿；'+reason_text(event.get('reason','')),detail=True)
            elif raw_stage == 'DRIVER_PLAN_CACHED':
                source=event.get('source','')
                planner='固定关节目标 PTP' if source=='pilz_PTP' else '有限绕行' if 'ompl' in source.lower() else source
                direction='返回' if event.get('motion_stage')=='RETURN' else '接近'
                self.log(record,raw_stage,direction+'｜'+planner+'｜将执行已检查的缓存轨迹',detail=True)
            elif raw_stage == 'WAITING_TARGET' and event.get('reason'):
                now=time.monotonic();reason=reason_text(event['reason'])
                if reason!=record.notice_reason and now-record.notice_at>=2.:
                    self.log(record,'WAITING_TARGET',reason,detail=True)
                    record.notice_at,record.notice_reason=now,reason
            stage = raw_stage
            if stage in ('MOTION_ACCEPTED', 'MOTION_REACHED'):
                stage = event.get('motion_stage', '')
            stage = {'CONTACT_STARTED': 'PRESS', 'CONTACT_TIME_LIMIT': 'PRESS',
                     'WITHDRAW_VERIFIED': 'WITHDRAW', 'TARGET_LOCKED': 'PLANNING',
                     'PREVIEW': 'PLANNING', 'TASK_FINISHED': 'FINALIZING',
                     'ARM_SELECTED': 'PLANNING', 'FIXED_FALLBACK': 'PLANNING',
                     'DRIVER_PREFLIGHT': 'PLANNING', 'DRIVER_PLAN_CACHED': 'PLANNING',
                     'PREFLIGHT_STAGE': 'PLANNING', 'PREFLIGHT_PASSED': 'PLANNING',
                     'HOLD': 'DWELL', 'RETURN': 'RETURN'}.get(stage, stage)
            if stage in STAGES or stage in ('PLANNING', 'FINALIZING'):
                feedback = self.feedback(record, STAGES.get(stage, stage))
                if record.feedback is None or record.feedback.stage != feedback.stage:
                    self.log(record, feedback.stage, feedback.message)
                record.feedback = feedback
                if record.handle is not None and record.handle.is_active:
                    record.handle.publish_feedback(Execute.Feedback(feedback=feedback))

    def feedback(self, record, stage):
        task = self.provider(record).task
        arm = ARM_IDS.get(getattr(task, 'arm', ''), record.request.arm)
        acquired = getattr(task, 'acquired', False)
        record.ever_acquired |= acquired
        authority = Result.AUTHORITY_ACQUIRED if acquired else Result.AUTHORITY_RELEASED if record.ever_acquired else Result.AUTHORITY_NOT_ACQUIRED
        if record.cancel_requested:
            stage = 'RECOVERING' if stage == 'RECOVERING' else 'CANCELLING'
        return ExecutionFeedback(request_id=record.request.request_id,
            action_name=record.request.action_name, selected_arm=arm,
            stage=stage, progress=-1., elapsed_s=time.monotonic() - record.created,
            authority_state=authority, message='执行超时，正在收尾' if record.deadline_exceeded else STAGE_LABELS.get(stage,stage))

    def log(self, record, stage, message, result=None, detail=False):
        now=time.monotonic()
        elapsed=now-record.created
        stage_elapsed=now-(record.last_stage_at or record.created)
        arm=record.feedback.selected_arm if record.feedback is not None else record.request.arm
        if result is not None:arm=result.selected_arm
        event=dict(timestamp=self.get_clock().now().nanoseconds*1e-9,
                   request_id=record.request.request_id,action_name=record.request.action_name,
                   stage=stage,level='INFO' if result is None or result.success else 'ERROR',
                   message=message,elapsed_s=elapsed,stage_elapsed_s=stage_elapsed,selected_arm=arm,
                   detail=detail)
        if result is not None:event['result']=message_to_ordereddict(result)
        if self.log_path:
            try:
                with self.log_path.open('a') as stream:stream.write(json.dumps(event,ensure_ascii=False)+'\n')
            except OSError as error:self.get_logger().error('写入动作日志失败：'+str(error))
        text=reason_text(message)
        if result is not None:
            states={Result.AUTHORITY_NOT_ACQUIRED:'本次未接管',Result.AUTHORITY_RELEASED:'已归还',
                    Result.AUTHORITY_RETAINED:'仍持有，需人工处理',Result.AUTHORITY_UNKNOWN:'未确认'}
            text=f"{ARM_LABELS.get(arm,'未选定')}｜总耗时 {result.elapsed_s:.2f} s｜控制权：{states.get(result.authority_state,'未归还')}"
            if not result.success:text+=f"｜错误码 {result.code}｜{reason_text(result.message)}"
        elif not detail:
            text=f"{ARM_LABELS.get(arm,'未选定')}｜总计 {elapsed:.2f} s"
            if record.last_logged_stage:text+=f"｜上一阶段 {stage_elapsed:.2f} s"
            if stage=='VALIDATING':
                try:direction=json.loads(record.request.parameters_json).get('direction')
                except (ValueError,AttributeError):direction=None
                if direction in ('up','down'):text+='｜'+{'up':'上行','down':'下行'}[direction]
                text+='｜请求 ID：'+record.request.request_id
        print(console_line(record.request.action_name,record.request.request_id,stage,text),flush=True)
        if not detail:record.last_stage_at=now;record.last_logged_stage=stage

    def component_notice(self,name,provider,active):
        reason=provider.dependency_error or ''
        if '等待连续稳定' in reason:reason='等待连续稳定 RGB-D'
        key=(bool(provider.ready),reason)
        state=self.component_notices.setdefault(name,dict(candidate=key,since=time.monotonic(),reported=None))
        now=time.monotonic()
        if key!=state['candidate']:state.update(candidate=key,since=now)
        if active is not None and active.target_locked:return
        if key!=state['reported'] and now-state['since']>=1.:
            state['reported']=key
            print(console_line(name,'','READY' if key[0] else 'NOT_READY',
                               '可接收任务' if key[0] else reason_text(reason)),flush=True)

    def provider(self, record):
        return self.providers.get(record.request.action_name, self.backend)

    def claim(self, request, handle=None):
        params = validate(request)
        now = time.monotonic()
        with self.mutex:
            self.prune(now)
            if request.request_id in self.executions:
                raise InvalidRequest(Result.DUPLICATE_REQUEST, '请求 ID 已存在，请用 get_execution 查询；不会重复运动')
            if self.closed or self.fault:
                raise InvalidRequest(Result.NOT_READY, self.fault or '服务正在退出')
            if self.active is not None or any(p.worker is not None for p in self.providers.values()):
                raise InvalidRequest(Result.BUSY, '已有动作执行中；请等待结束或 stop')
            record = Execution(copy.deepcopy(request), now, Future(executor=self.executor), handle=handle)
            record.feedback = self.feedback(record, 'VALIDATING')
            self.executions[request.request_id] = record
            self.active = record
            self.log(record, 'VALIDATING', '开始校验任务')
            return record, params

    def prune(self, now):
        terminal = [key for key, value in self.executions.items() if value.result is not None]
        for key in terminal:
            if now - self.executions[key].created > 1800:
                del self.executions[key]
        terminal = [key for key, value in self.executions.items() if value.result is not None]
        for key in terminal[:-100]:
            del self.executions[key]

    def finish(self, record, result, fault=False):
        with self.mutex:
            if record.result is not None:
                return
            record.result = result
            self.last_result = result
            if fault or result.code == Result.CLEANUP_FAILED:
                self.fault = result.message
            if self.active is record:
                self.active = None
            self.log(record, 'COMPLETED' if result.success else 'CANCELLED' if result.code == Result.CANCELLED else 'FAILED', result.message, result)
            record.done.set_result(result)
            if record.legacy_completion is not None:
                raw = record.raw_result or dict(result='cleanup_failed', reason=result.message,
                    arm=ARMS.get(result.selected_arm, 'auto'),
                    authority_released=False, authority_retained=result.authority_state == Result.AUTHORITY_RETAINED,
                    authority_acquired=False, elapsed_s=result.elapsed_s,
                    release_error=result.message)
                record.legacy_completion.set_result(dict(raw, request_id=record.request.request_id, action_name='elevator'))
            self.prune(time.monotonic())

    def raw_complete(self, record, raw, legacy_completion=None):
        try:
            result = map_result(record.request, raw, time.monotonic() - record.created,
                                record.cancel_requested, record.deadline_exceeded)
        except Exception as error:
            result = make_result(record.request, Result.INTERNAL_ERROR, '结果转换失败：' + str(error), time.monotonic() - record.created)
            result.cleanup_completed = False
            result.authority_state = Result.AUTHORITY_UNKNOWN
            result.code = Result.CLEANUP_FAILED
        record.raw_result = raw
        self.finish(record, result)

    def begin(self, record, params, legacy_completion=None):
        record.legacy_completion = legacy_completion
        if record.request.action_name not in self.providers:
            self.finish(record, make_result(record.request, Result.NOT_IMPLEMENTED, '动作提供者未加载'))
            return False, '动作提供者未加载'
        if record.cancel_requested or (record.handle is not None and record.handle.is_cancel_requested):
            self.finish(record, make_result(record.request, Result.CANCELLED, '任务开始前已取消'))
            return False, '任务开始前已取消'
        with self.mutex:
            completion = Completion(lambda raw: self.raw_complete(record, raw, legacy_completion))
            if record.request.action_name == 'elevator':
                answer = self.backend._start_impl(Trigger.Request(), Trigger.Response(), completion,
                    direction=params['direction'], arm=ARMS[record.request.arm])
            else:
                success, message = self.provider(record).start(ARMS[record.request.arm], params, completion, request_id=record.request.request_id)
                answer = Trigger.Response(success=success, message=message)
            if answer.success:
                record.started = True
            else:
                self.finish(record, make_result(record.request, Result.NOT_READY, answer.message))
        if record.cancel_requested and record.result is None:
            self.request_stop(record)
        return answer.success, answer.message

    async def execute(self, handle):
        request = handle.request.request
        try:
            record, params = self.claim(request, handle)
        except InvalidRequest as error:
            result = make_result(request, error.code, str(error))
            rejected = Execution(request, time.monotonic(), None)
            self.log(rejected, 'FAILED', result.message, result)
            handle.abort()
            return Execute.Result(result=result)
        try:
            self.begin(record, params)
        except Exception as error:
            # A start exception must not unlock a still-running backend.
            if self.provider(record).worker is not None:
                self.request_stop(record)
                result = make_result(request, Result.CLEANUP_FAILED, '启动异常，需确认后台收尾：' + str(error))
                result.cleanup_completed = False
                result.authority_state = Result.AUTHORITY_UNKNOWN
                self.finish(record, result, fault=True)
            else:
                self.finish(record, make_result(request, Result.INTERNAL_ERROR, str(error)))
        result = await record.done
        if result.code == Result.CANCELLED and record.transport_cancel_future is not None:
            await record.transport_cancel_future
        if result.success:
            handle.succeed()
        elif result.code == Result.CANCELLED and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return Execute.Result(result=result)

    def legacy_start(self, request, response, completion):
        goal = ExecutionGoal(request_id=str(uuid.uuid4()), action_name='elevator',
                             arm=ARM_IDS.get(self.backend.get_parameter('arm').value, 255),
                             parameters_json=json.dumps({'direction': self.backend.get_parameter('direction').value}))
        try:
            record, params = self.claim(goal)
        except InvalidRequest as error:
            response.message = str(error)
            return response
        try:
            accepted, message = self.begin(record, params, completion)
        except Exception as error:
            result = make_result(goal, Result.CLEANUP_FAILED if self.provider(record).worker is not None else Result.INTERNAL_ERROR, str(error))
            if self.provider(record).worker is not None:
                self.request_stop(record)
                result.cleanup_completed = False
                result.authority_state = Result.AUTHORITY_UNKNOWN
            self.finish(record, result)
            accepted, message = False, str(error)
        response.success, response.message = accepted, message
        return response

    def request_stop(self, record, deadline=False):
        with self.mutex:
            if record.result is not None:
                return False
            first = not record.cancel_requested
            record.deadline_exceeded |= deadline
            record.cancel_requested = True
            if not record.cancel_started:
                record.cancel_started = time.monotonic()
            if first:
                self.provider(record)._stop_impl(Trigger.Request(), Trigger.Response())
                record.feedback=self.feedback(record,'CANCELLING')
                self.log(record,'CANCELLING','超过执行时限，等待停止和控制权归还' if deadline else '已请求停止，等待收尾和控制权归还')
            if record.deadline_exceeded:
                self.provider(record).stop_event.reason = '动作执行超时'
            if record.handle is not None and not record.transport_cancel_sent:
                record.transport_cancel_sent = True
                request = CancelGoal.Request()
                request.goal_info.goal_id = record.handle.goal_id
                record.transport_cancel_future = self.cancel_client.call_async(request)
            return True

    def cancel(self, handle):
        with self.mutex:
            record = self.active
            if record is not None and record.handle is handle:
                record.transport_cancel_sent = True
                self.request_stop(record)
        return CancelResponse.ACCEPT

    def stop_active(self, request, response):
        with self.mutex:
            record = self.active
            if record is None or (request.request_id and request.request_id != record.request.request_id):
                response.message = '没有匹配的执行中任务；请查询最终结果'
                return response
            response.request_id = record.request.request_id
            response.accepted = self.request_stop(record)
            response.message = '已请求停止；须查询该请求的最终结果确认收尾和控制权'
            return response

    def legacy_stop(self, request, response):
        with self.mutex:
            if self.active is not None:
                identifier = self.active.request.request_id
                self.request_stop(self.active)
                response.success = True
                response.message = '已请求停止，等待收尾和控制权归还；request_id=' + identifier
                return response
        return self.backend._stop_impl(request, response)

    def get_execution(self, request, response):
        with self.mutex:
            self.prune(time.monotonic())
            record = self.executions.get(request.request_id)
            if record is None:
                response.message = '未找到请求，或已超出当前进程的结果缓存范围'
                return response
            response.found = True
            response.terminal = record.result is not None
            if record.feedback is not None:
                response.feedback = record.feedback
            if response.terminal:
                response.result = record.result
            return response

    def list_actions(self, request, response):
        if request.api_version != 1:
            response.message = '只支持 API v1'
            return response
        idle = not self.closed and not self.fault and self.active is None
        share = Path(get_package_share_directory('g1_actions'))
        for name, description in [('elevator', '上下行电梯按钮按压'), ('handshake', '握手')]:
            provider = self.providers.get(name)
            ready = bool(provider and provider.refresh_readiness() and idle)
            info = ActionInfo(name=name, api_version=1, description=description,
                              implemented=provider is not None, ready=ready,
                              supported_arms=[0, 1, 2] if provider else [],
                              parameter_schema_json=(share / 'schemas' / (name + '.schema.json')).read_text())
            info.unavailable_reason = ('' if info.ready else self.fault or ('已有任务执行中' if self.active else provider.dependency_error)) if info.implemented else '尚未实现'
            response.actions.append(info)
        response.success = True
        return response

    def tick(self):
        now = time.monotonic()
        with self.mutex:
            record = self.active
            if record is not None:
                if not record.cancel_requested and now - record.created >= record.request.timeout_s:
                    self.request_stop(record, deadline=True)
                if record.cancel_started and now - record.cancel_started >= self.cleanup_timeout_s:
                    result = make_result(record.request, Result.CLEANUP_FAILED, '停止收尾超时，禁止新动作；需检查驱动', now - record.created)
                    result.cleanup_completed = False
                    result.failed_stage = 'RECOVERING'
                    result.authority_state = Result.AUTHORITY_UNKNOWN
                    self.finish(record, result, fault=True)
                    record = None
            if record is not None:
                stage = record.feedback.stage if record.feedback is not None else 'VALIDATING'
                feedback = self.feedback(record, stage)
                if record.feedback is None or record.feedback.stage != stage:
                    self.log(record, stage, feedback.message)
                record.feedback = feedback
                if record.handle is not None and record.handle.is_active:
                    record.handle.publish_feedback(Execute.Feedback(feedback=feedback))
            status = ServerStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.state = ServerStatus.FAULT if self.fault else ServerStatus.CANCELLING if record and record.cancel_requested else ServerStatus.RUNNING if record else ServerStatus.IDLE
            available = [p.refresh_readiness() for p in self.providers.values()]
            ready = any(available)
            for name,provider in self.providers.items():self.component_notice(name,provider,record)
            status.accepting_goals = bool(not self.closed and not self.fault and record is None and all(p.worker is None for p in self.providers.values()) and ready)
            status.message = self.fault or (record.feedback.message if record else "；".join(name+"："+p.dependency_error for name,p in self.providers.items() if p.dependency_error))
            if record:
                status.active_request_id = record.request.request_id
                status.active_action_name = record.request.action_name
                status.stage = record.feedback.stage
                status.authority_state = record.feedback.authority_state
            else:
                status.stage = 'FAULT' if self.fault else 'READY' if ready else 'NOT_READY'
                status.authority_state = self.last_result.authority_state if self.last_result else Result.AUTHORITY_NOT_ACQUIRED
            if self.last_result is not None:
                status.last_result_valid = True
                status.last_result = self.last_result
            self.status_pub.publish(status)

    def begin_shutdown(self):
        with self.mutex:
            self.closed = True
            if self.active is not None:
                self.request_stop(self.active)

    def destroy_node(self):
        self.backend.start_guard = None
        self.backend.stop_guard = None
        for provider in self.providers.values():
            provider.action_event_callback = None
        self.action_server.destroy()
        return super().destroy_node()
