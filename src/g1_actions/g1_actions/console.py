"""Shared Chinese stage names and compact console lines, no ROS dependencies."""
from datetime import datetime
from zoneinfo import ZoneInfo

STAGE_LABELS = {
    'VALIDATING':'准备任务','WAITING_TARGET':'获取稳定目标','PLANNING':'规划轨迹',
    'ACQUIRING_AUTHORITY':'接管控制权','APPROACHING':'到达目标',
    'RETURNING':'返回初始位姿','PRESSING':'直线按压','CONTACT_HOLD':'接触保持',
    'WITHDRAWING':'沿原轨迹撤回','RELEASING_AUTHORITY':'归还控制权','FINALIZING':'任务收尾',
    'CANCELLING':'停止中','RECOVERING':'异常收尾','READY':'就绪','NOT_READY':'未就绪',
    'FAULT':'需要人工检查','COMPLETED':'完成','FAILED':'失败','CANCELLED':'已停止',
    'TARGET_LOCKED':'目标锁定','PREVIEW':'目标','FIXED_FALLBACK':'备用位姿',
    'DRIVER_PLAN_CACHED':'规划通过',
}
ACTION_LABELS={'elevator':'电梯按压','handshake':'握手'}
ARMS={0:'自动选臂',1:'左手',2:'右手'}


def line(action,identifier,stage,message):
    stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S')
    prefix=f'[{stamp}][{ACTION_LABELS.get(action,action)}]'
    if identifier:prefix+=f'[{identifier[:8]}]'
    return prefix+f'[{STAGE_LABELS.get(stage,stage)}] '+message


def reason_text(reason):
    reason=str(reason)
    replacements={
        'No fresh vision result':'等待新鲜视觉结果',
        'Hand image is stale':'图像已过期，等待新鲜 RGB-D',
        'RGB-D is stale or clock is inconsistent':'RGB-D 未满足新鲜度要求，等待新帧',
        'Landmark 0 depth patch is clipped':'手腕深度区域超出画面',
        'Insufficient reliable finger-base depths for 3D direction':'四指根部有效深度不足',
    }
    for source,target in replacements.items():reason=reason.replace(source,target)
    if reason.startswith('Driver reports error: '):reason='驱动报错：'+reason[len('Driver reports error: '):]
    return '；'.join(dict.fromkeys(part.strip() for part in reason.replace('；',';').split(';') if part.strip()))
