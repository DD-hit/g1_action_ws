import unittest
from g1_actions.actions.elevator.log_view import format_result

class LogViewTests(unittest.TestCase):
    def test_failure_identifies_stage_reason_and_authority(self):
        text=format_result(dict(result='aborted',direction='down',arm='right',failed_stage='PRESS',
                                reason='Pilz LIN failed: PLANNING_FAILED (code -1)',authority_released=True,elapsed_s=2.35))
        for value in ('按压失败｜下行｜右手','失败阶段：直线按压','PLANNING_FAILED','控制权：已归还','2.4 s'):
            self.assertIn(value,text)
    def test_release_error_is_not_shown_as_success(self):
        text=format_result(dict(result='cleanup_failed',authority_retained=True,release_error='RPC timeout'))
        self.assertIn('归还失败，仍保留控制权',text);self.assertIn('RPC timeout',text)
    def test_unknown_authority_is_not_claimed_released(self):
        self.assertIn('未确认，请查询 status',format_result(dict(result='aborted')))
    def test_not_acquired_is_distinguished(self):
        self.assertIn('本次未接管',format_result(dict(result='aborted',authority_released=True,authority_acquired=False)))
