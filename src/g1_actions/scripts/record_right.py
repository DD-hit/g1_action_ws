#!/usr/bin/env python3
"""Record only: subscribes to feedback/events and never publishes or calls services."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import time
from g1_actions.actions.elevator.trajectory_recording import ApproachCapture,save_json,select_record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'data/right_arm_recordings')
    action=parser.add_mutually_exclusive_group()
    action.add_argument('--list',action='store_true')
    action.add_argument('--select',type=int,metavar='NUMBER')
    parser.add_argument('--end-s',type=float,help='Select prefix ending at this elapsed second')
    args=parser.parse_args()
    files=sorted(args.output.glob('right_*.json'))
    if args.end_s is not None and args.select is None:parser.error('--end-s requires --select')
    if args.list:
        for number,path in enumerate(files,1):
            record=json.loads(path.read_text())
            print(f"{number}: {path.name} {record['direction']} {record['duration_s']:.2f}s {len(record['samples'])}帧 "+('记录完整' if record['capture_valid'] else '不可选：'+','.join(record['errors'])+' '+record['end_reason']))
        if not files:print('尚无记录；先运行 bash scripts/ele.sh record-right')
        return
    if args.select is not None:
        if not 1<=args.select<=len(files):parser.error('编号不存在，请先 --list')
        source=files[args.select-1]
        try:record=select_record(json.loads(source.read_text()),args.end_s)
        except ValueError as error:parser.error(str(error))
        record['source_file']=str(source)
        destination=args.output/'selected_right.json';save_json(destination,record)
        print(f'已选取 {source.name} 的前 {record["duration_s"]:.2f} 秒：{destination}；仅保存供检查，未启用回放。')
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from brsd_msgs.msg import DualArmStatus,WaistStatus
    from std_msgs.msg import String
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('elevator_right_approach_recorder')
    capture=ApproachCapture();state={'status':None,'waist':None,'status_time':None,'waist_time':None,'saved':0,'target':None,'busy_seen':False}
    stopping=[False]
    def stop(*args):stopping[0]=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def finish(reason,complete=False):
        record=capture.finish(reason,complete)
        if record is None:return
        record['ros_domain_id']=__import__('os').environ.get('ROS_DOMAIN_ID','0')
        record['joint_order']=['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']
        record['waist_joint_order']=['waist_yaw','waist_roll','waist_pitch']
        name='right_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json'
        save_json(args.output/name,record);state['saved']+=1
        print(f'已保存 {name}：{record["duration_s"]:.2f} 秒，{len(record["samples"])} 帧，'+('记录完整' if record['capture_valid'] else '不可选：'+','.join(record['errors'])+' '+reason),flush=True)
    def sample(now):
        if capture.active is None:return
        s,w=state['status'],state['waist']
        if s is None or w is None:
            capture.error('开始记录时缺少双臂或腰部反馈');return
        if now-state['status_time']>.6:capture.error('双臂反馈超过 0.6 秒未更新')
        age=now-state['waist_time']
        fault=any((s.emergency_stop_active,s.joint_limits_exceeded,s.collision_detected,s.left_has_error,s.right_has_error,w.emergency_stop_active,w.joint_limits_exceeded,not w.sdk_connected))
        capture.sample(now,s.current_joint.right_positions,s.current_joint.left_positions,w.current_joint.positions,age,s.current_state,fault)
    def status(message):
        now=time.monotonic();state.update(status=message,status_time=now)
        if capture.active is not None:
            samples=capture.active['samples']
            if not samples or now-capture.active['start_monotonic']-samples[-1]['t_s']>=.02:sample(now)
    def waist(message):state.update(waist=message,waist_time=time.monotonic())
    def event(message):
        try:record=json.loads(message.data)
        except (ValueError,TypeError):return
        if record.get('arm')!='right':return
        stage=record.get('stage');now=time.monotonic()
        if stage=='TARGET_LOCKED':state['target']=record.get('target')
        elif stage=='APPROACH':
            finish('新任务到来，上一段未正常结束')
            capture.begin(record,now)
            state['busy_seen']=False
            capture.active['locked_target']=state['target']
            sample(now)
            print('开始记录右臂第一阶段：'+str(record.get('direction')),flush=True)
        elif capture.active is not None:
            if stage=='MOTION_REACHED' and record.get('motion_stage')=='APPROACH':
                sample(now);finish('预备点已到达',True)
            elif stage in ('ABORTED','PRESS','RECOVERY_WITHDRAW','READY_TO_RELEASE'):
                sample(now);finish('缺少第一阶段到位确认：'+str(stage))
    def service(message):
        try:record=json.loads(message.data)
        except (ValueError,TypeError):return
        if capture.active is not None and record.get('busy') is True:state['busy_seen']=True
        if capture.active is not None and state['busy_seen'] and record.get('busy') is False and record.get('last_result'):
            finish('任务已结束但缺少第一阶段到位确认')
    subscriptions=[node.create_subscription(DualArmStatus,'/dual_arm/status',status,10),
                   node.create_subscription(WaistStatus,'/dual_arm/waist_status',waist,10),
                   node.create_subscription(String,'/elevator/task_status',event,100),
                   node.create_subscription(String,'/elevator/service_status',service,10)]
    print('只读记录已启动：自动分段保存右臂 APPROACH；左臂不录制。Ctrl+C 停止记录，不会停止业务任务。',flush=True)
    try:
        while not stopping[0] and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            if capture.active is not None:
                now=time.monotonic()
                if state['status_time'] is None or now-state['status_time']>.6:
                    capture.error('双臂反馈中断')
                if now-capture.active['start_monotonic']>120:finish('第一阶段记录超过 120 秒')
    finally:
        finish('记录器停止，第一阶段未完整记录')
        node.destroy_node();rclpy.shutdown()
        print(f'记录器已退出，本次保存 {state["saved"]} 段；未发送运动或停止指令。',flush=True)


if __name__=='__main__':main()
