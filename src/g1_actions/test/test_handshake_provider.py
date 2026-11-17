"""Exercise real provider subprocess/result bridging with a synthetic child."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import rclpy
from g1_actions.actions.handshake import provider as module
from g1_actions.server import Completion


class HandshakeProviderTests(unittest.TestCase):
    def run_case(self, result):
        with tempfile.TemporaryDirectory() as temporary:
            rclpy.init()
            provider=module.HandshakeProvider(Path(temporary),Path(temporary)/'logs')
            provider.task=SimpleNamespace(arm='auto',acquired=False)
            received=[];events=[];provider.action_event_callback=events.append
            popen=subprocess.Popen
            def child(command, **kwargs):
                target=command[-1]
                code=("import pathlib,json; p=pathlib.Path("+repr(target)+"); p.mkdir(parents=True); "
                      "print(json.dumps({'stage':'ARM_SELECTED','arm':'left'}),flush=True); "
                      "print(json.dumps({'stage':'ACQUIRE_AUTHORITY','arm':'left'}),flush=True); "+
                      ("(p/'result.json').write_text("+repr(json.dumps(result))+")" if result is not None else "raise SystemExit(2)"))
                return popen([sys.executable,'-c',code],**kwargs)
            try:
                with patch.object(module.subprocess,'Popen',side_effect=child):
                    provider.run('auto',dict(hold_s=4.,subject_handedness='any'),Completion(received.append))
                self.assertEqual(len(received),1)
                self.assertEqual(events[0]['arm'],'left')
                return received[0],provider.task.acquired
            finally:
                provider.destroy_node();rclpy.shutdown()

    def test_success_preserves_selected_arm_and_release(self):
        raw,owned=self.run_case(dict(success=True,authority_requested=True,authority_released=True,arm='left'))
        self.assertEqual(raw['result'],'completed');self.assertEqual(raw['arm'],'left')
        self.assertTrue(raw['authority_acquired'] and raw['authority_released']);self.assertFalse(owned)

    def test_failure_before_acquisition_reports_not_acquired(self):
        raw,owned=self.run_case(dict(success=False,authority_requested=False,authority_released=False,
            reason='no IK',failure_stage='preflight',target_rejected=True,arm='left'))
        self.assertFalse(raw['authority_acquired']);self.assertTrue(raw['authority_released'])
        self.assertEqual(raw['failed_stage'],'preflight');self.assertFalse(owned)

    def test_missing_final_report_never_claims_release(self):
        raw,owned=self.run_case(None)
        self.assertEqual(raw['result'],'cleanup_failed');self.assertFalse(raw['authority_released'])
        self.assertTrue(owned)


if __name__=='__main__':unittest.main()
