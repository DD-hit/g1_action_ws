"""Clock-controlled readiness regressions; no camera or robot commands."""
import unittest
from g1_actions.actions.elevator.camera_health import FrameReadiness


class CameraReadinessTests(unittest.TestCase):
    def setUp(self):
        self.gate = FrameReadiness()

    def frame(self, t, sequence=None, age=.35):
        self.gate.observe({'sequence': int(round(t*6))+1 if sequence is None else sequence,
                           'stamp_sec': 1000+t-age}, t, 1000+t)

    def healthy(self, start=0.):
        for i in range(14):
            self.frame(start+i/6)
        return start+13/6

    def test_one_frame_does_not_admit_a_task(self):
        self.frame(0.)
        self.assertFalse(self.gate.check(0.,1000.)[0])

    def test_sustained_six_fps_becomes_ready(self):
        t = self.healthy()
        self.assertTrue(self.gate.check(t,1000+t)[0])

    def test_silence_cannot_complete_the_warmup(self):
        self.frame(0.)
        self.assertFalse(self.gate.check(2.1,1002.1)[0])
        self.frame(2.2)
        self.assertEqual(self.gate.frames,1)
        self.assertFalse(self.gate.check(2.2,1002.2)[0])

    def test_normal_interframe_wait_does_not_reset_stability(self):
        t = self.healthy()
        self.assertTrue(self.gate.check(t,1000+t)[0])
        self.assertTrue(self.gate.check(t+.3,1000+t+.3)[0])
        self.assertFalse(self.gate.check(t+.81,1000+t+.81)[0])

    def test_error_and_recovery_require_a_new_stable_window(self):
        t = self.healthy()
        self.gate.observe({'reason':'camera frames stopped','stamp_sec':1000+t},t,1000+t)
        self.assertFalse(self.gate.check(t,1000+t)[0])
        self.frame(t+.2)
        self.assertEqual(self.gate.frames,1)
        self.assertFalse(self.gate.check(t+.2,1000+t+.2)[0])
        t = self.healthy(t+.4)
        self.assertTrue(self.gate.check(t,1000+t)[0])

    def test_duplicate_or_stale_messages_never_refresh_admission(self):
        t = self.healthy()
        self.frame(t)
        self.assertFalse(self.gate.check(t,1000+t)[0])
        self.frame(t+.2,age=1.)
        self.assertFalse(self.gate.check(t+.2,1000+t+.2)[0])

    def test_publisher_restart_requires_requalification(self):
        t = self.healthy()
        self.frame(t+.2,sequence=1)
        self.assertEqual(self.gate.frames,1)
        self.assertFalse(self.gate.check(t+.2,1000+t+.2)[0])

    def test_future_and_nonfinite_timestamps_are_rejected(self):
        for age in (-.1,float('nan'),float('inf')):
            self.frame(0.,age=age)
            self.assertFalse(self.gate.check(0.,1000.)[0])

    def test_frame_count_alone_cannot_replace_stable_duration(self):
        for i in range(20):
            self.frame(i*.01,sequence=i+1,age=.1)
        self.assertFalse(self.gate.check(.2,1000.2)[0])


if __name__=='__main__':
    unittest.main(verbosity=2)
