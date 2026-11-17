#!/usr/bin/env python3
"""Bounded start/stop/status client; no direct arm commands."""
import argparse,json,os,sys,time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from std_msgs.msg import String
from g1_actions.actions.elevator.log_view import format_result, STAGES

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=('start','stop','status'))
    parser.add_argument('direction',nargs='?',choices=('up','down'))
    parser.add_argument('--timeout',type=float,default=300.,help='start 响应等待秒数；超时不取消任务')
    parser.add_argument('--json',action='store_true',help='输出单份 JSON，供程序解析')
    args=parser.parse_args();rclpy.init();node=Node('ele_elevator_command_'+str(os.getpid()))
    def wait_for(predicate,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.04)
            if predicate():return True
        return False
    def invoke(client,request,timeout=4):
        future=client.call_async(request)
        if not wait_for(future.done,timeout):
            raise RuntimeError('未收到服务响应；请求可能已接收，请用 bash scripts/ele.sh status 查看状态，避免重复发送')
        return future.result()
    code=0
    try:
        if args.action=='status':
            latest=[]
            node.create_subscription(String,'/elevator/service_status',lambda m:latest.append(m.data),1)
            if not wait_for(lambda:bool(latest),3):raise RuntimeError('未收到常驻服务状态；请检查服务终端是否仍在运行')
            status=json.loads(latest[-1])
            if args.json:print(json.dumps(status,ensure_ascii=False),flush=True)
            else:
                print('服务状态：'+STAGES.get(status.get('stage'),status.get('stage','未知')),flush=True)
                print('任务：'+('执行中' if status.get('busy') else '空闲'),flush=True)
                if status.get('reason'):print('说明：'+status['reason'],flush=True)
                if status.get('last_result'):
                    print('最近结果：\n'+format_result(status['last_result']),flush=True)
        else:
            command=node.create_client(Trigger,'/elevator/'+args.action)
            if not command.wait_for_service(timeout_sec=3):
                raise RuntimeError('常驻服务未运行或 ROS 域不一致；请先在另一终端运行 bash scripts/ele.sh')
            if args.action=='start' and args.direction:
                parameters=node.create_client(SetParameters,'/ele_elevator_service/set_parameters')
                if not parameters.wait_for_service(timeout_sec=2):raise RuntimeError('方向参数服务不可用')
                response=invoke(parameters,SetParameters.Request(parameters=[Parameter('direction',value=args.direction).to_parameter_msg()]))
                if not all(x.successful for x in response.results):raise RuntimeError('设置方向失败')
            if args.action=='start' and not args.json:
                print('正在请求按压任务，等待最终结果；可在另一终端调用 stop。',flush=True)
            response=invoke(command,Trigger.Request(),args.timeout if args.action=='start' else 4)
            details=None
            if args.action=='start':
                try:
                    parsed=json.loads(response.message)
                    if isinstance(parsed,dict):details=parsed
                except (ValueError,TypeError):pass
            if args.json:
                payload={'success':response.success,**details} if details is not None else {'success':response.success,'message':response.message}
                print(json.dumps(payload,ensure_ascii=False),flush=True)
            elif details is not None:
                print(format_result(details),flush=True)
            else:
                print(('成功：' if response.success else '失败：')+response.message,flush=True)
            code=0 if response.success else 1
    except Exception as error:
        text=json.dumps({'success':False,'message':str(error)},ensure_ascii=False) if args.json else '调用异常：'+str(error)
        print(text,file=sys.stderr,flush=True);code=2
    finally:
        node.destroy_node();rclpy.shutdown()
    return code

if __name__=='__main__':raise SystemExit(main())
