"""Validation and lossless mapping at the unified action boundary."""
import json
import math
import uuid

from g1_action_interfaces.msg import ExecutionResult as Result

ARMS = {0: 'auto', 1: 'left', 2: 'right'}
ARM_IDS = {value: key for key, value in ARMS.items()}
STAGES = {
    'STARTING': 'VALIDATING', 'WAITING_TARGET': 'WAITING_TARGET',
    'ACQUIRE_AUTHORITY': 'ACQUIRING_AUTHORITY', 'APPROACH': 'APPROACHING',
    'PRESS': 'PRESSING', 'DWELL': 'CONTACT_HOLD', 'WITHDRAW': 'WITHDRAWING',
    'RECOVERY_WITHDRAW': 'RECOVERING', 'RECOVERY_RETURN': 'RECOVERING',
    'STOPPING': 'CANCELLING', 'READY_TO_RELEASE': 'RELEASING_AUTHORITY',
    'CONTACT_SEQUENCE_COMPLETED': 'RELEASING_AUTHORITY',
    'RELEASE_AUTHORITY': 'RELEASING_AUTHORITY', 'RESUME_VISION': 'FINALIZING',
    'NODE_CLEANUP': 'FINALIZING',
}


class InvalidRequest(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def strict_json(text, maximum=4096):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('重复 JSON 字段：' + key)
            result[key] = value
        return result

    def constant(value):
        raise ValueError('不允许非有限 JSON 数值：' + value)

    if len(text.encode('utf-8')) > maximum:
        raise ValueError('参数超过 4096 字节')
    result = json.loads(text, object_pairs_hook=pairs, parse_constant=constant)
    if not isinstance(result, dict):
        raise ValueError('动作参数必须是 JSON 对象')
    return result


def validate(request):
    if request.api_version != 1:
        raise InvalidRequest(Result.UNSUPPORTED_VERSION, '只支持 API v1')
    try:
        identifier = uuid.UUID(request.request_id)
        if identifier.version != 4 or str(identifier) != request.request_id:
            raise ValueError()
    except (ValueError, AttributeError):
        raise InvalidRequest(Result.INVALID_ARGUMENT, 'request_id 必须是小写 UUID v4')
    if request.action_name not in ('elevator', 'handshake'):
        raise InvalidRequest(Result.UNKNOWN_ACTION, '未知动作：' + request.action_name)
    if request.arm not in ARMS:
        raise InvalidRequest(Result.INVALID_ARGUMENT, 'arm 必须为 0/1/2')
    if not math.isfinite(request.timeout_s) or not 1 <= request.timeout_s <= 120:
        raise InvalidRequest(Result.INVALID_ARGUMENT, 'timeout_s 必须为 1～120 s')
    try:
        params = strict_json(request.parameters_json)
        if request.action_name == 'elevator':
            if set(params) != {'direction'} or params['direction'] not in ('up', 'down'):
                raise ValueError('电梯参数只能为 {"direction":"up"} 或 {"direction":"down"}')
        else:
            if set(params) - {'subject_handedness', 'hold_s'}:
                raise ValueError('握手参数包含未知字段')
            params.setdefault('subject_handedness', 'any')
            params.setdefault('hold_s', 1.)
            hold = params['hold_s']
            if params['subject_handedness'] not in ('any', 'left', 'right'):
                raise ValueError('subject_handedness 必须为 any/left/right')
            if isinstance(hold, bool) or not isinstance(hold, (int, float)) or not math.isfinite(hold) or not 0 <= hold <= 3:
                raise ValueError('hold_s 必须为 0～3 s')
    except (ValueError, TypeError, KeyError) as error:
        raise InvalidRequest(Result.INVALID_ARGUMENT, str(error))
    return params


def make_result(request, code, message, elapsed=0.):
    return Result(request_id=request.request_id, action_name=request.action_name,
                  code=code, success=code == Result.SUCCEEDED,
                  selected_arm=request.arm if request.arm in ARMS else 0,
                  failed_stage='' if code == Result.SUCCEEDED else 'VALIDATING',
                  message=message, elapsed_s=float(elapsed),
                  authority_state=Result.AUTHORITY_NOT_ACQUIRED,
                  cleanup_completed=True)


def failure_code(raw):
    reason = str(raw.get('reason', ''))
    lower = reason.lower()
    if raw.get('result') == 'stopped':
        return Result.CANCELLED
    if 'ELEVATOR_CONTACT_UNREACHABLE:' in reason and 'PILZ_PATH_COLLISION:' not in reason:
        return Result.PLANNING_FAILED
    if 'PILZ_PATH_COLLISION' in reason or 'collision' in lower:
        return Result.COLLISION
    if any(word in lower for word in ('joint limit', 'joint bounds', 'acceleration limit', 'velocity limit')):
        return Result.JOINT_LIMIT
    if 'tracking error' in lower:
        return Result.TRACKING_ERROR
    if 'workspace' in lower or 'target expired' in lower or 'target exceeds' in lower or 'invalid target' in lower:
        return Result.TARGET_INVALID
    if 'target' in lower and ('timed out' in lower or 'timeout' in lower):
        return Result.TARGET_TIMEOUT
    if '规划' in reason or 'planning' in lower or 'motion_plan' in lower:
        return Result.PLANNING_FAILED
    if '控制权' in reason or 'authority' in lower:
        return Result.AUTHORITY_ERROR
    if 'Driver reports error' in reason or 'sdk' in lower or 'driver' in lower:
        return Result.DRIVER_ERROR
    return Result.INTERNAL_ERROR


def map_result(request, raw, elapsed, cancel=False, deadline=False):
    cleanup_errors = any(raw.get(key) for key in (
        'cancel_error', 'recovery_error', 'release_error',
        'vision_resume_error', 'node_cleanup_error'))
    acquired = raw.get('authority_acquired', False)
    released = raw.get('authority_released') is True
    retained = raw.get('authority_retained') is True
    completed = raw.get('result') == 'completed'
    cleanup = released and not retained and not cleanup_errors and raw.get('result') != 'cleanup_failed'
    if not cleanup:
        code = Result.CLEANUP_FAILED
    elif deadline:
        code = Result.DEADLINE_EXCEEDED
    elif cancel or raw.get('result') == 'stopped':
        code = Result.CANCELLED
    elif completed and acquired:
        code = Result.SUCCEEDED
    elif completed:
        code = Result.INTERNAL_ERROR
    else:
        code = failure_code(raw)
    message = raw.get('reason') or ('运动流程完成' if completed else '任务失败')
    if 'no collision-free deterministic IK candidate' in str(message):
        message = '预备点姿态求解失败：固定候选未找到满足关节和碰撞约束的 IK 解'
    if deadline and cleanup:
        message = '执行超时，已停止并完成收尾'
    elif cancel and cleanup:
        message = '任务已停止并完成收尾'
    if code == Result.CLEANUP_FAILED:
        message = '任务收尾或控制权归还未确认：' + str(raw.get('release_error') or raw.get('cancel_error') or raw.get('reason') or raw.get('result'))
    result = make_result(request, code, str(message), elapsed)
    result.selected_arm = ARM_IDS.get(raw.get('arm'), request.arm)
    result.motion_completed = completed
    result.cleanup_completed = cleanup
    result.authority_state = (Result.AUTHORITY_RETAINED if retained else
                              Result.AUTHORITY_RELEASED if acquired and released else
                              Result.AUTHORITY_NOT_ACQUIRED if not acquired and released else
                              Result.AUTHORITY_UNKNOWN)
    result.failed_stage = STAGES.get(raw.get('failed_stage'), raw.get('failed_stage', ''))
    if not result.success and not result.failed_stage:
        result.failed_stage = 'CANCELLING' if cancel else 'FINALIZING'
    if result.success:
        result.failed_stage = ''
    press = raw.get('press_completion', {})
    result.effect_verification = Result.EFFECT_VERIFIED if press.get('button_activation_verified') else Result.EFFECT_UNVERIFIED
    result.details_json = json.dumps(raw, ensure_ascii=False, allow_nan=False)
    if len(result.details_json.encode()) > 16384:
        result.details_json = json.dumps({'diagnostic_truncated': True, 'reason': str(raw.get('reason', ''))[:2000]}, ensure_ascii=False)
    return result
