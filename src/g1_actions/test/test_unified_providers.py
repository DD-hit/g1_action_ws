"""Real ROS Action transport, synthetic providers only, no hardware."""
import json
import threading
import time
from types import SimpleNamespace
import unittest
import test_unified_actions as support
from g1_actions.server import Completion
from g1_action_interfaces.msg import ExecutionResult as Result


class SyntheticHandshake:
    def __init__(self):
        self.worker=self.task=None
        self.ready=True;self.dependency_error='';self.action_event_callback=None
        self.stop_event=threading.Event();self.hold=False;self.fail_cleanup=False
    def refresh_readiness(self):return self.ready
    def start(self,arm,params,completion,request_id=None):
        self.stop_event.clear()
        self.task=SimpleNamespace(arm='left' if arm=='auto' else arm,acquired=True)
        def run():
            self.action_event_callback({'stage':'ARM_SELECTED','arm':self.task.arm})
            self.action_event_callback({'stage':'APPROACH'})
            cancelled=self.stop_event.wait(5. if self.hold else .1)
            # Signal success only after simulated cleanup.
            self.task.acquired=self.fail_cleanup
            raw=dict(arm=self.task.arm,result='stopped' if cancelled else 'completed',
                     authority_acquired=True,authority_released=not self.fail_cleanup,
                     authority_retained=self.fail_cleanup)
            self.worker=None
            completion.set_result(raw)
        self.worker=threading.Thread(target=run);self.worker.start()
        return True,'started'
    def _stop_impl(self,request,response):
        self.stop_event.set();response.success=True;return response


class UnifiedProvidersTests(unittest.TestCase):
    wait=support.UnifiedActionsTests.wait
    goal=support.UnifiedActionsTests.goal
    send=support.UnifiedActionsTests.send
    result=support.UnifiedActionsTests.result
    invoke=support.UnifiedActionsTests.invoke
    def setUp(self):
        support.UnifiedActionsTests.setUp(self)
        self.handshake=SyntheticHandshake()
        self.server.providers['handshake']=self.handshake
        self.handshake.action_event_callback=lambda event:self.server.observe(event,'handshake')
    def tearDown(self):
        self.handshake.stop_event.set()
        self.wait(lambda:self.handshake.worker is None)
        support.UnifiedActionsTests.tearDown(self)

    def test_elevator_and_handshake_share_busy_lock_both_directions(self):
        self.handshake.hold=True
        active=self.send(self.goal(action='handshake',arm=0))
        self.wait(lambda:self.handshake.worker is not None)
        answer=self.result(self.send(self.goal())).result.result
        self.assertEqual(answer.code,Result.BUSY)
        stopped=active.cancel_goal_async();self.wait(stopped.done)
        answer=self.result(active).result.result
        self.assertEqual(answer.code,Result.CANCELLED)
        self.assertEqual(answer.selected_arm,1)
        self.assertEqual(answer.authority_state,Result.AUTHORITY_RELEASED)
        self.backend.cfg['test_mode']='hold'
        active=self.send(self.goal())
        self.wait(lambda:self.backend.worker is not None)
        self.assertEqual(self.result(self.send(self.goal(action='handshake'))).result.result.code,Result.BUSY)
        stopped=active.cancel_goal_async();self.wait(stopped.done)
        self.assertEqual(self.result(active).result.result.code,Result.CANCELLED)

    def test_handshake_result_then_elevator_can_run(self):
        result=self.result(self.send(self.goal(action='handshake',arm=0))).result.result
        self.assertTrue(result.success and result.cleanup_completed)
        self.assertEqual(result.selected_arm,1)
        self.assertEqual(result.authority_state,Result.AUTHORITY_RELEASED)
        self.assertTrue(self.result(self.send(self.goal())).result.result.success)

    def test_handshake_cleanup_failure_blocks_all_new_actions(self):
        self.handshake.fail_cleanup=True
        result=self.result(self.send(self.goal(action='handshake'))).result.result
        self.assertEqual(result.code,Result.CLEANUP_FAILED)
        self.assertEqual(self.result(self.send(self.goal())).result.result.code,Result.NOT_READY)

    def test_handshake_deadline_cancels_and_releases(self):
        self.handshake.hold=True
        result=self.result(self.send(self.goal(action='handshake',timeout_s=1.))).result.result
        self.assertEqual(result.code,Result.DEADLINE_EXCEEDED)
        self.assertEqual(result.authority_state,Result.AUTHORITY_RELEASED)


if __name__=='__main__':unittest.main()
