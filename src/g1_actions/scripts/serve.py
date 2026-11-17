#!/usr/bin/env python3
"""One terminal owns the persistent service and only the components it starts."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from brsd_msgs.msg import DualArmStatus
from g1_actions.actions.elevator.config import load
from g1_actions.actions.elevator.service_node import TaskService
from g1_actions.server import ActionLibraryServer
from g1_actions.driver_runtime import verify_driver_runtime
from managed_service import stop_owned_perception
from camera_recovery import CameraRecovery,startup_components


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--direction',choices=('up','down'),default='up')
    parser.add_argument('--arm',choices=('auto','left','right'),default='auto')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    runtime=Path('/run/ele_elevator');runtime.mkdir(exist_ok=True)
    lock=(runtime/('server-'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock')).open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('常驻服务已启动，无需重复运行')
    output=root.parents[1]/'data'/'elevator'/time.strftime('server_%Y%m%d_%H%M%S');output.mkdir(parents=True)
    children=[];logs=[];log_files={};stopping=[False]
    def interrupted(signum,frame):stopping[0]=True
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=TaskService(load(),args.direction,args.arm)
    event_log=(output/'task_events.jsonl').open('a',buffering=1)
    node.record_callback=lambda record:event_log.write(json.dumps(record,ensure_ascii=False)+'\n')
    executor=SingleThreadedExecutor();executor.add_node(node)
    unified=ActionLibraryServer(node,log_path=output/'action_events.jsonl')
    executor.add_node(unified)
    recovery=CameraRecovery(stop_owned_perception)
    node.perception_recovering=False
    def start(command):
        if command not in log_files:
            log_files[command]=(output/(command+'.log')).open('a');logs.append(log_files[command])
        log=log_files[command]
        argv=['bash',str(root/'scripts/ele.sh'),command]
        if command=='driver':argv.append('real')
        child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((command,child))
    try:
        until=time.monotonic()+2
        while time.monotonic()<until and not stopping[0]:executor.spin_once(timeout_sec=.05)
        if stopping[0]:return
        if node.count_publishers('/dual_arm/status')>1:raise RuntimeError('存在多个机器人驱动')
        if not node.count_publishers('/dual_arm/status'):start('driver')
        else:verify_driver_runtime(node.cfg['motion'])
        camera=node.cfg['camera']
        camera_present=any(node.count_publishers(camera[key]) for key in
                           ('color_topic','depth_topic','info_topic'))
        vision_present=bool(node.count_publishers('/elevator/camera_health') or
                            node.count_publishers('/elevator/targets'))
        initial=startup_components(camera_present,vision_present)
        for component in initial:start(component)
        print('启动时仅补齐缺失组件：'+(', '.join(initial) if initial else '复用已有相机和视觉')+'；持续断流时才恢复重启。',flush=True)
        print('正在初始化相机、TensorRT 视觉和驱动；就绪前 start 会返回未就绪。',flush=True)
        print('日志：'+str(output),flush=True)
        print('统一动作库：bash scripts/g1.sh run elevator --direction up；stop 停止；status 查询。',flush=True)
        while not stopping[0] and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
            for name,child in children:
                if name=='driver' and child.poll() is not None:
                    node.dependency_error='组件退出：'+name+'，请查看 '+str(output/(name+'.log'))
                    node.stop_event.set()
                    raise RuntimeError(node.dependency_error)
            now=time.monotonic()
            safe=(node.worker is None and not node.closed and node.waist is not None
                  and now-node.waist_arrival<.6 and not node.waist.has_upper_body_authority
                  and node.status is not None and now-node.status_arrival<.6
                  and node.status.current_state in (DualArmStatus.IDLE,DualArmStatus.DISABLED)
                  and not (node.last_result and node.last_result.get('authority_retained')))
            action=recovery.tick(node.camera_receipts.last_frame(),node.ready,safe)
            if action=='stopping':
                node.perception_recovering=True
                node.vision_gate.invalidate('相机正在自动恢复，等待连续稳定 RGB-D')
                print(f'相机自动恢复：第 {recovery.attempts} 次，停止所属相机/视觉进程。',flush=True)
            elif action=='start':
                children[:]=[(name,child) for name,child in children if name=='driver']
                node.vision_health={}
                node.camera_receipts.reset()
                node.vision_gate.invalidate('相机已重启，等待连续稳定 RGB-D')
                start('perception')
                node.perception_recovering=False
                print('相机自动恢复：已重启相机和视觉，等待连续稳定出图。',flush=True)
            elif action=='failed':
                node.perception_recovering=False
                print('相机自动恢复失败：'+str(recovery.error)+'；稍后重试。',flush=True)
            node.camera_recovery_status={'attempts':recovery.attempts,'restarting':recovery.active,
                                         'last_error':recovery.error}
    finally:
        unified.begin_shutdown()
        recovery.join()
        last_cleanup_message=None
        last_cleanup_report=0.
        while True:
            try:
                node.close()
                break
            except RuntimeError as error:
                # Failed cleanup is not permission to abandon the live driver.
                # Keep status/feedback available; starts remain disabled by close().
                now=time.monotonic()
                if str(error)!=last_cleanup_message or now-last_cleanup_report>=5.:
                    print('尚未安全退出：'+str(error),flush=True)
                    print('服务和驱动继续保留；满足安全间隙和停稳条件后自动释放并退出。',flush=True)
                    last_cleanup_message=str(error);last_cleanup_report=now
                stopping[0]=False
                retry_at=time.monotonic()+.25
                while not stopping[0] and rclpy.ok() and time.monotonic()<retry_at:
                    executor.spin_once(timeout_sec=.1)
                if not rclpy.ok():
                    raise RuntimeError('ROS 上下文已失效，无法确认安全退出；保留驱动') from error
        for _ in range(10):executor.spin_once(timeout_sec=.05)
        executor.remove_node(unified);unified.destroy_node()
        executor.remove_node(node);executor.shutdown();node.destroy_node()
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        event_log.close();rclpy.shutdown();lock.close()


if __name__=='__main__':main()
