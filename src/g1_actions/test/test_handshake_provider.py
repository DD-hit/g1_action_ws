"""Persistent ROS feeds and direct worker lifecycle; no hardware or subprocess."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import rclpy
from std_msgs.msg import String
from g1_actions.actions.handshake import provider as module
from g1_actions.server import Completion

PACKAGE=Path(__file__).resolve().parents[1]


class HandshakeProviderTests(unittest.TestCase):
    def run_case(self,result):
        with tempfile.TemporaryDirectory() as temporary:
            rclpy.init()
            provider=module.HandshakeProvider(PACKAGE.parents[1],Path(temporary)/'logs')
            provider.task=SimpleNamespace(arm='auto',acquired=False)
            received=[];events=[];provider.action_event_callback=events.append
            def execute(node,**kwargs):
                node.arm='left';node.event('ARM_SELECTED',arm='left')
                node.owned=bool(result is None or result.get('authority_requested'))
                if node.owned:node.event('ACQUIRE_AUTHORITY',arm='left')
                if result is None:raise RuntimeError('unexpected worker failure')
                return result
            try:
                with patch.object(module,'execute_once',side_effect=execute),patch('subprocess.Popen',side_effect=AssertionError('per-task subprocess must not start')):
                    provider.run('auto',dict(hold_s=4.,subject_handedness='any'),Completion(received.append))
                self.assertEqual(len(received),1)
                self.assertEqual(events[0]['arm'],'left')
                return received[0],provider.task.acquired
            finally:provider.destroy_node();rclpy.shutdown()

    def test_success_preserves_selected_arm_and_release(self):
        raw,owned=self.run_case(dict(success=True,authority_requested=True,authority_released=True,arm='left'))
        self.assertEqual(raw['result'],'completed');self.assertEqual(raw['arm'],'left')
        self.assertTrue(raw['authority_acquired'] and raw['authority_released']);self.assertFalse(owned)

    def test_failure_before_acquisition_reports_not_acquired(self):
        raw,owned=self.run_case(dict(success=False,authority_requested=False,authority_released=False,
            reason='no IK',failure_stage='preflight',target_rejected=True,arm='left'))
        self.assertFalse(raw['authority_acquired']);self.assertTrue(raw['authority_released'])
        self.assertEqual(raw['failed_stage'],'preflight');self.assertFalse(owned)

    def test_worker_exception_after_acquisition_never_claims_release(self):
        raw,owned=self.run_case(None)
        self.assertEqual(raw['result'],'cleanup_failed');self.assertFalse(raw['authority_released'])
        self.assertTrue(owned)

    def test_diagnostic_directory_error_completes_without_acquiring(self):
        with tempfile.TemporaryDirectory() as temporary:
            rclpy.init()
            path=Path(temporary)/'not_a_directory';path.write_text('occupied')
            provider=module.HandshakeProvider(PACKAGE.parents[1],path)
            provider.task=SimpleNamespace(arm='auto',acquired=False)
            received=[]
            try:
                provider.run('auto',dict(hold_s=4.,subject_handedness='any'),Completion(received.append))
                self.assertEqual(len(received),1)
                self.assertEqual(received[0]['result'],'aborted')
                self.assertFalse(received[0]['authority_acquired'])
                self.assertIsNone(provider.worker)
            finally:provider.destroy_node();rclpy.shutdown()

    def test_warm_target_is_fresh_and_frozen_after_lock(self):
        import yaml
        with tempfile.TemporaryDirectory() as temporary:
            rclpy.init();provider=module.HandshakeProvider(PACKAGE.parents[1],Path(temporary)/'logs')
            node=module.Approach(yaml.safe_load((PACKAGE/'config/handshake.yaml').read_text()),PACKAGE,'auto',Path(temporary),feed=provider)
            provider.task=node
            now=node.get_clock().now().nanoseconds*1e-9
            payload=dict(stamp_s=now,frame_id=node.cfg['frames']['camera_optical'],
                         palms=[dict(handedness='Left',geometry_valid=True,stable=True,pose=dict(position_m=[0.,0.,.5]))])
            try:
                provider.on_vision(String(data=json.dumps(payload)))
                self.assertEqual(node.target().tolist(),[0.,0.,.5])
                node.lock_target();self.assertEqual(node.arm,'left')
                provider.on_vision(String(data='{"valid":false,"reason":"camera disconnected"}'))
                node.sync_feed();self.assertIsNone(node.vision)
                self.assertEqual(node.locked.tolist(),[0.,0.,.5])
                provider.stop_event.set()
                with self.assertRaisesRegex(RuntimeError,'停止'):node.check_cancel()
                node.cleaning_up=True;node.check_cancel()
            finally:node.journal.close();node.destroy_node();provider.destroy_node();rclpy.shutdown()
