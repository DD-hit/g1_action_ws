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
from std_srvs.srv import Trigger
from g1_actions.actions.handshake.provider import HandshakeProvider
from g1_actions.actions.elevator.config import load
from g1_actions.actions.elevator.service_node import TaskService
from g1_actions.server import ActionLibraryServer
from g1_actions.console import line as console_line
from g1_actions.driver_runtime import verify_driver_runtime, existing_driver_processes
from managed_service import stop_owned_perception
from camera_recovery import CameraRecovery,startup_components


def server_log(stage,message):
    print(console_line('服务','',stage,message),flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--direction',choices=('up','down'),default='up')
    parser.add_argument('--arm',choices=('auto','left','right'),default='auto')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    runtime=Path('/run/g1_actions');runtime.mkdir(exist_ok=True)
    lock=(runtime/('server-'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock')).open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('常驻服务已启动，无需重复运行')
    hardware_lock=(runtime/'hardware.lock').open('a')
    try:fcntl.flock(hardware_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('已有统一后台占用机器人资源，请先关闭旧后台')
    output=root.parents[1]/'data'/'runtime'/time.strftime('server_%Y%m%d_%H%M%S');output.mkdir(parents=True)
    children=[];logs=[];log_files={};stopping=[False]
    def interrupted(signum,frame):stopping[0]=True
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=TaskService(load(),args.direction,args.arm)
    event_log=(output/'task_events.jsonl').open('a',buffering=1)
    node.record_callback=lambda record:event_log.write(json.dumps(record,ensure_ascii=False)+'\n')
    executor=SingleThreadedExecutor();executor.add_node(node)
    handshake=HandshakeProvider(root.parents[1], output/'executions')
    executor.add_node(handshake)
    unified=ActionLibraryServer(node,log_path=output/'action_events.jsonl',providers={'handshake':handshake})
    def shutdown(request,response):
        stopping[0]=True
        unified.begin_shutdown()
        response.success=True
        response.message='已请求关闭；当前动作收尾并归还控制权后停止本服务所属组件'
        return response
    unified.create_service(Trigger,'/g1_actions/shutdown',shutdown)
    executor.add_node(unified)
    recovery=CameraRecovery(stop_owned_perception)
    node.perception_recovering=False
    def start(command):
        if command not in log_files:
            log_files[command]=(output/(command+'.log')).open('a');logs.append(log_files[command])
        log=log_files[command]
        argv=(['bash',str(root.parents[1]/'scripts/handshake_live.sh'),'--duration','0','--production',
               '--output',str(output/'handshake_rgbd')] if command=='handshake_vision' else
              ['bash',str(root/'scripts/ele.sh'),command])
        if command=='driver':argv.append('real')
        child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((command,child))
    try:
        until=time.monotonic()+2
        while time.monotonic()<until and not stopping[0]:executor.spin_once(timeout_sec=.05)
        if stopping[0]:return
        if node.count_publishers('/dual_arm/status')>1:raise RuntimeError('存在多个机器人驱动')
        existing=existing_driver_processes()
        if any(domain!=os.environ.get('ROS_DOMAIN_ID','0') for _,domain in existing):
            raise RuntimeError('已有其他 ROS 域的双臂驱动，禁止重复启动同一机器人')
        if not node.count_publishers('/dual_arm/status'):
            if existing:raise RuntimeError('检测到已有双臂驱动，但未发现其反馈；请检查 ROS 域和驱动状态')
            start('driver')
        else:verify_driver_runtime(node.cfg['motion'])
        camera=node.cfg['camera']
        camera_present=any(node.count_publishers(camera[key]) for key in
                           ('color_topic','depth_topic','info_topic'))
        vision_present=bool(node.count_publishers('/elevator/camera_health') or
                            node.count_publishers('/elevator/targets'))
        initial=startup_components(camera_present,vision_present)
        for component in initial:start(component)
        if not node.count_publishers('/handshake/perception_status'):start('handshake_vision')
        server_log('启动','缺失组件已启动：'+(', '.join(initial) if initial else '无，复用已有相机和视觉'))
        server_log('初始化','等待驱动、相机及识别就绪；查询：bash scripts/g1.sh list')
        server_log('日志',str(output))
        server_log('接口','run elevator --direction up/down | run handshake --arm auto；stop 停止动作；shutdown 关闭后台')
        while not stopping[0] and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
            for name,child in children:
                if name in ('driver','handshake_vision') and child.poll() is not None:
                    node.dependency_error='组件退出：'+name+'，请查看 '+str(output/(name+'.log'))
                    node.stop_event.set()
                    raise RuntimeError(node.dependency_error)
            now=time.monotonic()
            safe=(unified.active is None and handshake.worker is None and node.worker is None and not node.closed and node.waist is not None
                  and now-node.waist_arrival<.6 and not node.waist.has_upper_body_authority
                  and node.status is not None and now-node.status_arrival<.6
                  and node.status.current_state in (DualArmStatus.IDLE,DualArmStatus.DISABLED)
                  and not (node.last_result and node.last_result.get('authority_retained')))
            action=recovery.tick(node.camera_receipts.last_frame(),node.ready,safe)
            if action=='stopping':
                node.perception_recovering=True
                node.vision_gate.invalidate('相机正在自动恢复，等待连续稳定 RGB-D')
                server_log('相机恢复',f'第 {recovery.attempts} 次：停止所属相机和视觉进程')
            elif action=='start':
                children[:]=[(name,child) for name,child in children if name in ('driver','handshake_vision')]
                node.vision_health={}
                node.camera_receipts.reset()
                node.vision_gate.invalidate('相机已重启，等待连续稳定 RGB-D')
                start('perception')
                node.perception_recovering=False
                server_log('相机恢复','已重启，等待连续稳定 RGB-D')
            elif action=='failed':
                node.perception_recovering=False
                server_log('相机恢复失败',str(recovery.error)+'；稍后重试')
            node.camera_recovery_status={'attempts':recovery.attempts,'restarting':recovery.active,
                                         'last_error':recovery.error}
    finally:
        unified.begin_shutdown()
        recovery.join()
        while handshake.worker is not None:
            executor.spin_once(timeout_sec=.1)
        if handshake.task is not None and (handshake.task.acquired or handshake.cleanup_uncertain):
            print('握手控制权未确认归还，保留驱动；请检查并手动归还控制权。',flush=True)
            while (handshake.waist is None or time.monotonic()-handshake.waist_at>.6 or
                   handshake.waist.has_upper_body_authority or handshake.waist.authority_weight>.02):
                executor.spin_once(timeout_sec=.1)
        handshake.closed=True
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
        executor.remove_node(handshake);handshake.destroy_node()
        executor.remove_node(node);executor.shutdown();node.destroy_node()
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        event_log.close();rclpy.shutdown();hardware_lock.close();lock.close()


if __name__=='__main__':main()
