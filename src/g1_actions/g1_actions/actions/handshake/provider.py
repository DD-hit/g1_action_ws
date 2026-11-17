"""Handshake provider owned by the common scheduler.

A child process isolates MediaPipe's Python environment from the ROS server.
Only the scheduler starts/stops it; stdout events feed the common Action API.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from types import SimpleNamespace

from brsd_msgs.msg import DualArmStatus, WaistStatus
from rclpy.node import Node
from std_msgs.msg import String


class HandshakeProvider(Node):
    def __init__(self, project, output):
        super().__init__('g1_handshake_provider')
        self.project, self.output = Path(project), Path(output)
        self.worker = self.task = self.process = None
        self.stop_event = threading.Event()
        self.status = self.waist = self.vision = None
        self.status_at = self.waist_at = self.vision_at = 0.
        self.ready = False
        self.dependency_error = '等待驱动和手部识别'
        self.action_event_callback = None
        self.closed = False
        self.cleanup_uncertain = False
        self.create_subscription(DualArmStatus, '/dual_arm/status', self.on_status, 1)
        self.create_subscription(WaistStatus, '/dual_arm/waist_status', self.on_waist, 1)
        self.create_subscription(String, '/handshake/perception_status', self.on_vision, 1)

    def on_status(self, message):
        self.status, self.status_at = message, time.monotonic()

    def on_waist(self, message):
        self.waist, self.waist_at = message, time.monotonic()

    def on_vision(self, message):
        try:
            self.vision = json.loads(message.data)
            self.vision_at = time.monotonic()
        except ValueError:
            self.vision = None

    def refresh_readiness(self):
        now = time.monotonic()
        if self.closed:
            reason = '服务正在退出'
        elif self.status is None or self.waist is None or now-self.status_at > .6 or now-self.waist_at > .6:
            reason = '未收到新鲜驱动反馈'
        elif not self.waist.sdk_connected:
            reason = '机器人 SDK 未连接'
        elif self.status.emergency_stop_active or self.status.joint_limits_exceeded or self.status.collision_detected:
            reason = '驱动安全检查未通过'
        elif self.status.left_has_error or self.status.right_has_error:
            reason = '驱动处于错误状态'
        elif self.vision is None or now-self.vision_at > 1. or 'stamp_s' not in self.vision:
            reason = '等待 RGB-D 手部识别进程'
        elif not (self.project/'.venv/bin/python').is_file():
            reason = '项目虚拟环境缺失'
        elif not all((self.project.parents[1]/'build/handshake_preflight'/name).is_file()
                     for name in ('handshake_preflight', 'handshake_full_state_reader')):
            reason = '握手预检工具尚未构建'
        else:
            reason = ''
        self.dependency_error = reason
        self.ready = not reason
        return self.ready

    def start(self, arm, params, completion, request_id=None):
        if self.worker is not None:
            return False, '握手动作正在执行'
        if not self.refresh_readiness():
            return False, self.dependency_error
        self.stop_event = threading.Event()
        self.task = SimpleNamespace(arm=arm, acquired=False)
        self.worker = threading.Thread(target=self.run, args=(arm, params, completion, request_id), daemon=True)
        self.worker.start()
        return True, '握手任务已开始'

    def event(self, value):
        if value.get('arm') in ('left', 'right'):
            self.task.arm = value['arm']
        if value.get('stage') == 'ACQUIRE_AUTHORITY':
            # Mark acquisition as attempted until the final release is confirmed.
            self.task.acquired = True
        if self.action_event_callback is not None:
            self.action_event_callback(value)

    def run(self, arm, params, completion, request_id=None):
        import tempfile
        self.output.mkdir(parents=True, exist_ok=True)
        folder = Path(tempfile.mkdtemp(prefix='handshake_'+((request_id+'_') if request_id else ''), dir=self.output))
        trial = folder/'trial'
        raw = None
        try:
            command = ['bash', str(self.project/'scripts/handshake_approach.sh'), '--managed',
                       '--arm', arm, '--execute', '--hold-s', str(params['hold_s']),
                       '--subject-handedness', params['subject_handedness'], '--output', str(trial)]
            if request_id:command[2:2]=['--request-id',request_id]
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, start_new_session=True)
            if self.stop_event.is_set():
                self.process.send_signal(signal.SIGINT)
            with (folder/'worker.log').open('w') as log:
                for line in self.process.stdout:
                    log.write(line);log.flush()
                    try:
                        event = json.loads(line)
                        if isinstance(event, dict) and 'stage' in event:
                            self.event(event)
                    except (ValueError, TypeError):
                        pass
            returncode = self.process.wait()
            if not (trial/'result.json').is_file():
                raise RuntimeError(f'握手执行进程异常退出（{returncode}）；日志：{folder}/worker.log')
            result = json.loads((trial/'result.json').read_text())
            acquired = bool(result.get('authority_requested'))
            released = bool(result.get('authority_released')) or not acquired
            raw = dict(result, result='completed' if result.get('success') and returncode == 0 else
                       'stopped' if result.get('cancelled') else 'aborted',
                       authority_acquired=acquired, authority_released=released,
                       authority_retained=acquired and not released,
                       arm=result.get('arm', self.task.arm), failed_stage=result.get('failure_stage',''))
        except Exception as error:
            if self.process is not None and self.process.poll() is None:
                # Keep the scheduler occupied until the child has finished its own cleanup.
                try:self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:pass
                self.process.communicate()
            # Missing final report cannot establish that an abruptly exited child released control.
            raw = dict(result='cleanup_failed', reason=str(error), arm=self.task.arm,
                       authority_acquired=self.task.acquired, authority_released=False,
                       authority_retained=self.task.acquired)
        finally:
            self.task.acquired = bool(raw.get('authority_retained'))
            self.cleanup_uncertain = not bool(raw.get('authority_released'))
            if self.process is not None and self.process.stdout is not None:
                self.process.stdout.close()
            self.process = None
            self.worker = None
            completion.set_result(raw)

    def _stop_impl(self, request, response):
        self.stop_event.set()
        child = self.process
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        response.success = True
        return response
