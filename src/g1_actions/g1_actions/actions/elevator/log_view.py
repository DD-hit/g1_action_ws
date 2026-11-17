"""Human-readable task logs; structured ROS results remain unchanged."""
from datetime import datetime
from zoneinfo import ZoneInfo

STAGES = {
    'STARTING':'准备任务', 'WAITING_TARGET':'获取稳定目标', 'PREVIEW':'确认目标',
    'ACQUIRE_AUTHORITY':'接管控制权', 'APPROACH':'到预备点', 'PRESS':'直线按压',
    'DWELL':'接触保持', 'WITHDRAW':'沿原轨迹撤回', 'RETURN':'返回起始位置',
    'RECOVERY_WITHDRAW':'异常撤回', 'RECOVERY_RETURN':'异常返回',
    'RESUME_VISION':'恢复视觉', 'NODE_CLEANUP':'清理任务', 'RELEASE_AUTHORITY':'归还控制权', 'CONTACT_SEQUENCE_COMPLETED':'运动流程结束',
    'READY_TO_RELEASE':'准备归还控制权', 'READY':'就绪', 'NOT_READY':'未就绪',
    'INITIALIZING':'初始化', 'STOPPING':'停止中', 'RECOVERY_REQUIRED':'需要处理控制权',
}

def log_line(level, text):
    stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S')
    return f'[{stamp}] [{level}] {text}'

def result_lines(result):
    state=result.get('result','unknown')
    title={'completed':'按压流程完成','aborted':'按压失败','stopped':'任务已停止',
           'cleanup_failed':'任务收尾异常'}.get(state,'任务状态未知')
    direction={'up':'上行','down':'下行'}.get(result.get('direction'),'未知方向')
    arm={'left':'左手','right':'右手','auto':'自动选择（未确认）'}.get(result.get('arm'),'未确认')
    lines=[f'{title}｜{direction}｜{arm}']
    stage=result.get('failed_stage')
    if stage:lines.append('失败阶段：'+STAGES.get(stage,stage))
    if result.get('reason'):
        reason=str(result['reason'])
        if reason.startswith('Driver reports error: '):
            parts=[part.strip() for part in reason[len('Driver reports error: '):].split(';') if part.strip()]
            reason='驱动报错：'+'；'.join(dict.fromkeys(parts))
        lines.append('原因：'+reason)
    for key,label in [('cancel_error','取消运动失败'),('recovery_error','恢复失败'),
                      ('release_error','归还控制权失败'),('vision_resume_error','恢复视觉失败'),
                      ('node_cleanup_error','清理任务失败')]:
        if result.get(key):lines.append(label+'：'+str(result[key]))
    if result.get('authority_retained'):
        authority='归还失败，仍保留控制权'
    elif result.get('authority_released') is True:
        authority='已归还' if result.get('authority_acquired') is not False else '本次未接管'
    else:authority='未确认，请查询 status'
    lines.append('控制权：'+authority)
    if 'elapsed_s' in result:lines.append(f'总耗时：{result["elapsed_s"]:.1f} s')
    press=result.get('press_completion',{})
    if press.get('termination')=='contact_time_limit':
        lines.append('接触：达到时间上限，已转入撤回')
    return lines

def format_result(result):
    return '\n'.join(result_lines(result))
