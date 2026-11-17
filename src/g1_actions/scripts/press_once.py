#!/usr/bin/env python3
"""One-shot real press using the current configured transforms."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from g1_actions.actions.elevator.config import load
from g1_actions.actions.elevator.task_node import PressTask


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--direction',choices=('up','down'),default='up')
    parser.add_argument('--arm',choices=('auto','left','right'),default='auto')
    args=parser.parse_args()
    cfg=load()
    root=Path(__file__).resolve().parents[1]
    # Prevent concurrent launchers even before ROS discovery settles.
    import fcntl
    lock=(root/'data').resolve();lock.mkdir(exist_ok=True)
    lockfile=(lock/'press_once.lock').open('a')
    try:fcntl.flock(lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('已有一键按压任务正在运行')
    output=lock/time.strftime('press_%Y%m%d_%H%M%S');output.mkdir()
    children=[];logs=[];error=None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=PressTask(cfg,args.direction,args.arm)
    report={'events':[],'hardware_motion_command_requested':False,'contact_requested':False}
    original=node.event
    def event(stage,**extra):
        report['events'].append({'stage':stage,'time':time.time(),**extra})
        if stage in ('APPROACH','PRESS','WITHDRAW','RETURN'):
            report['hardware_motion_command_requested']=True
        if stage=='PRESS':report['contact_requested']=True
        original(stage,**extra)
    node.event=event
    def interrupted(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    def start(command):
        log=(output/(command+'.log')).open('w');logs.append(log)
        argv=['bash',str(root/'scripts/ele.sh'),command]
        if command=='driver':argv.append('real')
        children.append(subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    try:
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        if node.count_publishers('/dual_arm/status')>1:raise RuntimeError('存在多个手臂驱动')
        if node.count_publishers('/dual_arm/status')==0:start('driver')
        node.wait(lambda:node.status is not None and node.waist is not None,35,'driver feedback')
        if node.waist.has_upper_body_authority:raise RuntimeError('控制权已被占用；拒绝接管其他任务')
        if node.count_publishers('/elevator/targets')==0:
            start('vision' if node.count_publishers(cfg['camera']['color_topic']) else 'perception')
        node.run(execute=True,wait_s=45,hover_only=False)
        report['result']='motion_completed_button_activation_unverified'
    except (Exception,KeyboardInterrupt) as exc:
        error=str(exc) or 'Interrupted'
        report.update(result='aborted',reason=error)
        node.event('ABORTED',reason=error)
        if node.executing:
            try:node.cancel_motion()
            except Exception as exc:report['cancel_error']=str(exc)
    finally:
        if node.acquired:
            try:node.trigger('/dual_arm/end');report['authority_released']=True
            except Exception as exc:
                error=error or str(exc);report['release_error']=str(exc)
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        (output/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
        print('日志：'+str(output),flush=True)
        lockfile.close()
    if error:raise SystemExit(1)


if __name__=='__main__':main()
