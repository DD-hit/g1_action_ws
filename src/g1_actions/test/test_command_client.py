import os,sys,json,time,subprocess,threading,unittest
from pathlib import Path
from types import SimpleNamespace
import rclpy
from rclpy.executors import SingleThreadedExecutor
from g1_actions.actions.elevator.service_node import TaskService
from test_task_service import FakeTask
SCRIPT=Path(__file__).resolve().parents[1]/'scripts/service_command.py'

class CommandClientTests(unittest.TestCase):
    def test_missing_service_reports_failure_in_bounded_time(self):
        env=os.environ.copy();env.update(ROS_DOMAIN_ID='231',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        start=time.monotonic();p=subprocess.run([sys.executable,str(SCRIPT),'start','up'],env=env,capture_output=True,text=True,timeout=6)
        self.assertEqual(p.returncode,2);self.assertLess(time.monotonic()-start,6.)
        self.assertIn('常驻服务未运行',p.stderr)

    def test_accept_busy_rejection_and_stop_are_explicit(self):
        rclpy.init();FakeTask.instances=[];node=TaskService({},task_factory=FakeTask)
        node.waist=SimpleNamespace(has_upper_body_authority=False)
        node.dependencies_ready=lambda:True;node.ready=True
        executor=SingleThreadedExecutor();executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
        env=os.environ.copy();env.update(ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        def call(*args):return subprocess.run([sys.executable,str(SCRIPT),*args,'--json'],env=env,capture_output=True,text=True,timeout=6)
        try:
            pending=subprocess.Popen([sys.executable,str(SCRIPT),'start','down','--json'],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            until=time.monotonic()+3
            while not FakeTask.instances and time.monotonic()<until:time.sleep(.01)
            self.assertTrue(FakeTask.instances)
            self.assertIsNone(pending.poll())
            busy=call('start','up');self.assertEqual(busy.returncode,1,busy.stderr);self.assertIn('已有任务',json.loads(busy.stdout)['message'])
            stop=call('stop');self.assertEqual(stop.returncode,0,stop.stderr)
            stdout,stderr=pending.communicate(timeout=5)
            self.assertEqual(pending.returncode,1,stderr)
            result=json.loads(stdout)
            self.assertFalse(result['success']);self.assertEqual(result['result'],'stopped')
            self.assertTrue(result['authority_released'])
            self.assertNotIn('details',result);self.assertNotIn('message',result)
        finally:
            node.close();executor.remove_node(node);executor.shutdown(timeout_sec=2);thread.join(2);node.destroy_node();rclpy.shutdown()
