"""Synchronous CLI over the unified ROS 2 Action API; no direct arm commands."""
import argparse
import json
import math
import signal
import sys
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from g1_action_interfaces.action import Execute
from g1_action_interfaces.msg import ExecutionGoal, ExecutionResult as Result, ServerStatus
from g1_action_interfaces.srv import ListActions, GetExecution, StopActive

STAGE_LABELS = {
    'VALIDATING': '准备任务', 'WAITING_TARGET': '获取稳定目标', 'PLANNING': '规划轨迹',
    'ACQUIRING_AUTHORITY': '接管控制权', 'APPROACHING': '到预备点',
    'PRESSING': '直线按压', 'CONTACT_HOLD': '接触保持', 'WITHDRAWING': '沿原轨迹撤回',
    'RELEASING_AUTHORITY': '归还控制权', 'FINALIZING': '任务收尾',
    'CANCELLING': '停止中', 'RECOVERING': '异常收尾', 'READY': '就绪',
    'NOT_READY': '未就绪', 'FAULT': '需要人工检查',
}


def format_result(result):
    details = json.loads(result.details_json)
    action = {'elevator': '电梯按压', 'handshake': '握手'}.get(result.action_name, result.action_name)
    direction = {'up': '上行', 'down': '下行'}.get(details.get('direction'), '')
    arm = {0: '未选定机械臂', 1: '左手', 2: '右手'}.get(result.selected_arm, '未知机械臂')
    title = '动作完成' if result.success else '任务已停止' if result.code == Result.CANCELLED else '动作失败'
    lines = ['｜'.join(x for x in [title, action, direction, arm] if x), '请求 ID：' + result.request_id]
    if not result.success:
        if result.failed_stage:
            lines.append('失败阶段：' + STAGE_LABELS.get(result.failed_stage, result.failed_stage))
        lines.extend(['错误码：' + str(result.code), '原因：' + result.message])
    labels = {Result.AUTHORITY_UNKNOWN: '未确认，请检查驱动', Result.AUTHORITY_NOT_ACQUIRED: '本次未接管',
              Result.AUTHORITY_ACQUIRED: '仍持有', Result.AUTHORITY_RELEASING: '正在归还',
              Result.AUTHORITY_RELEASED: '已归还', Result.AUTHORITY_RETAINED: '仍持有，禁止新动作'}
    lines.extend(['控制权：' + labels.get(result.authority_state, '未知'), f'总耗时：{result.elapsed_s:.1f} s'])
    if details.get('press_completion', {}).get('termination') == 'contact_time_limit':
        lines.append('接触：达到时间上限，已转入撤回')
    return '\n'.join(lines)


class Client:
    def __init__(self):
        self.interrupted = False
        self.node = Node('g1_action_command_' + uuid.uuid4().hex[:10])
        self.action = ActionClient(self.node, Execute, '/g1_actions/execute')

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.05)
            if self.interrupted:
                self.interrupted = False
                raise KeyboardInterrupt()
            if predicate():
                return
        raise TimeoutError('等待超时；任务可能仍在执行，请使用原请求 ID 查询或 stop，不要重复发送')

    def invoke(self, srv_type, endpoint, request, timeout=5.):
        client = self.node.create_client(srv_type, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('统一动作服务未运行；请先在另一终端执行 bash scripts/g1.sh serve')
            future = client.call_async(request)
            self.wait(future.done, timeout)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def query(self, identifier):
        return self.invoke(GetExecution, '/g1_actions/get_execution', GetExecution.Request(request_id=identifier))

    def print_result(self, result, as_json):
        print(json.dumps(message_to_ordereddict(result), ensure_ascii=False) if as_json else format_result(result), flush=True)
        return 0 if result.success else 1

    def run(self, args):
        if not math.isfinite(args.timeout_s) or not 1 <= args.timeout_s <= 120:
            raise ValueError('timeout-s 必须为 1～120 s')
        if args.parameters is not None and args.direction is not None:
            raise ValueError('--parameters 与 --direction 不能同时使用')
        if args.direction is not None and args.action_name != 'elevator':
            raise ValueError('--direction 仅适用于 elevator 动作')
        params = args.parameters if args.parameters is not None else json.dumps({'direction': args.direction or 'up'} if args.action_name == 'elevator' else {})
        identifier = args.request_id or str(uuid.uuid4())
        goal = Execute.Goal(request=ExecutionGoal(request_id=identifier,
            action_name=args.action_name, arm={'auto': 0, 'left': 1, 'right': 2}[args.arm],
            timeout_s=args.timeout_s, parameters_json=params))
        print('请求 ID：' + identifier, file=sys.stderr, flush=True)
        if not self.action.wait_for_server(timeout_sec=3):
            raise RuntimeError('统一动作服务未运行；请先执行 bash scripts/g1.sh serve')
        stage = [None]
        def feedback(message):
            data = message.feedback.feedback
            if args.feedback and data.stage != stage[0]:
                print('阶段：' + STAGE_LABELS.get(data.stage, data.stage), file=sys.stderr, flush=True)
                stage[0] = data.stage
        future = self.action.send_goal_async(goal, feedback_callback=feedback)
        cancel_after_acceptance = False
        try:
            self.wait(future.done, 5)
        except KeyboardInterrupt:
            # Finish retrieving the handle before cancelling. Raising inside
            # a Humble pybind wait can corrupt the returned Python value.
            self.wait(future.done, 5)
            cancel_after_acceptance = True
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('ROS Action goal 被拒绝，尚无最终执行结果；请按请求 ID 查询')
        result = handle.get_result_async()
        if cancel_after_acceptance:
            handle.cancel_goal_async()
            print('已请求停止，等待收尾和控制权归还。', file=sys.stderr, flush=True)
        try:
            self.wait(result.done, args.timeout_s + 15.)
        except KeyboardInterrupt:
            # Request cancellation and wait for its final result instead of
            # interpreting an accepted cancellation as completed cleanup.
            handle.cancel_goal_async()
            print('已请求停止，等待收尾和控制权归还。', file=sys.stderr, flush=True)
            self.wait(result.done, 15.)
        return self.print_result(result.result().result.result, args.json)

    def command(self, args):
        if args.command == 'run':
            return self.run(args)
        if args.command == 'list':
            response = self.invoke(ListActions, '/g1_actions/list_actions', ListActions.Request())
            if args.json:
                print(json.dumps(message_to_ordereddict(response), ensure_ascii=False))
            elif not response.success:
                print('查询失败：' + response.message)
            else:
                for action in response.actions:
                    state = '就绪' if action.ready else '未就绪' if action.implemented else '未实现'
                    print(f'{action.name}｜{action.description}｜{state}' + ('｜' + action.unavailable_reason if action.unavailable_reason else ''))
            return 0 if response.success else 1
        if args.command == 'status':
            latest = []
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            subscription = self.node.create_subscription(ServerStatus, '/g1_actions/status', lambda value: latest.append(value), qos)
            self.wait(lambda: bool(latest), 4)
            status = latest[-1]
            if args.json:
                print(json.dumps(message_to_ordereddict(status), ensure_ascii=False))
            else:
                print('服务状态：' + STAGE_LABELS.get(status.stage, status.stage))
                print('任务：' + (status.active_request_id or '空闲'))
                if status.message:
                    print('说明：' + status.message)
                if status.last_result_valid:
                    print('最近结果：\n' + format_result(status.last_result))
            return 0
        if args.command == 'stop':
            response = self.invoke(StopActive, '/g1_actions/stop_active', StopActive.Request(request_id=args.request_id or ''))
            if not response.accepted:
                if args.request_id:
                    found = self.query(args.request_id)
                    if found.found and found.terminal:
                        return self.print_result(found.result, args.json)
                print(json.dumps({'accepted': False, 'message': response.message}, ensure_ascii=False) if args.json else response.message)
                return 1 if args.request_id else 0
            identifier = response.request_id
            print('已请求停止，等待最终结果；请求 ID：' + identifier, file=sys.stderr, flush=True)
            deadline = time.monotonic() + 15.
            while time.monotonic() < deadline:
                response = self.query(identifier)
                if response.found and response.terminal:
                    return self.print_result(response.result, args.json)
                rclpy.spin_once(self.node, timeout_sec=.1)
            raise TimeoutError('停止结果尚未确认；请查询请求 ID：' + identifier)
        response = self.query(args.request_id)
        if response.found and response.terminal:
            return self.print_result(response.result, args.json)
        if args.json:
            print(json.dumps(message_to_ordereddict(response), ensure_ascii=False))
        else:
            print('任务执行中：' + response.feedback.stage if response.found else response.message)
        return 0 if response.found else 1


def main():
    parser = argparse.ArgumentParser(description='G1 统一动作库：默认等待任务收尾后的最终结果')
    commands = parser.add_subparsers(dest='command', required=True)
    run = commands.add_parser('run')
    run.add_argument('action_name')
    run.add_argument('--direction', choices=['up', 'down'])
    run.add_argument('--arm', choices=['auto', 'left', 'right'], default='auto')
    run.add_argument('--timeout-s', type=float, default=60.)
    run.add_argument('--request-id')
    run.add_argument('--parameters')
    run.add_argument('--feedback', action='store_true')
    get = commands.add_parser('get')
    get.add_argument('request_id')
    stop = commands.add_parser('stop')
    stop.add_argument('--request-id')
    for command in [run, get, stop, commands.add_parser('list'), commands.add_parser('status')]:
        command.add_argument('--json', action='store_true')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    client = Client()
    def interrupted(signum, frame):
        client.interrupted = True
    previous_handler = signal.signal(signal.SIGINT, interrupted)
    try:
        return client.command(args)
    except (Exception, KeyboardInterrupt) as error:
        if isinstance(error, KeyboardInterrupt):
            error = RuntimeError('调用已中断；请按请求 ID 查询任务状态')
        print(json.dumps({'success': False, 'message': str(error)}, ensure_ascii=False) if args.json else '调用异常：' + str(error), file=sys.stderr, flush=True)
        return 2
    finally:
        signal.signal(signal.SIGINT, previous_handler)
        client.action.destroy()
        client.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
