"""Watchdog and owner isolation tests; never starts a camera or robot driver."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from camera_recovery import CameraRecovery,startup_components
import managed_service as managed
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from g1_actions.actions.elevator.camera_health import CameraReceipts


class RawReceptionTests(unittest.TestCase):
    def setUp(self):self.receipts=CameraReceipts()

    def health(self,color=.1,depth=.1):
        return {'color_receive_age_s':color,'depth_receive_age_s':depth,
                'color_stamp_age_s':.2,'depth_stamp_age_s':.2}

    def test_long_inference_pause_and_task_end_do_not_restart_camera(self):
        now=[0.];stop=Mock();policy=CameraRecovery(stop,clock=lambda:now[0])
        policy.tick(0.,True,True)
        for t in (1.,5.,10.,30.):
            now[0]=t;self.receipts.observe(self.health(),t)
            self.assertIsNone(policy.tick(self.receipts.last_frame(),False,t==30.))
        stop.assert_not_called()
        now[0]=33.1
        self.assertEqual(policy.tick(self.receipts.last_frame(),False,True),'stopping')
        policy.join();stop.assert_called_once()

    def test_one_stalled_stream_is_detected_even_if_other_keeps_receiving(self):
        self.receipts.observe(self.health(),10.)
        self.receipts.observe(self.health(color=.1,depth=4.1),14.)
        self.assertAlmostEqual(self.receipts.last_frame(),9.9)

    def test_no_images_and_invalid_ages_do_not_fake_reception(self):
        self.receipts.observe({'color_receive_age_s':0.,'depth_receive_age_s':0.,'color_stamp_age_s':None,'depth_stamp_age_s':None},10.)
        self.assertIsNone(self.receipts.last_frame())
        self.receipts.observe(self.health(color=float('nan'),depth=float('inf')),20.)
        self.assertIsNone(self.receipts.last_frame())

    def test_reset_requires_both_streams_again(self):
        self.receipts.observe(self.health(),10.);self.receipts.reset()
        self.assertIsNone(self.receipts.last_frame())


class StartupReuseTests(unittest.TestCase):
    def test_healthy_existing_components_are_never_stopped(self):
        now=[0.];stop=Mock();start=Mock()
        policy=CameraRecovery(stop,clock=lambda:now[0])
        for component in startup_components(True,True):start(component)
        for t in (0.,3.,60.,120.):
            now[0]=t
            self.assertIsNone(policy.tick(t,True,True))
        start.assert_not_called();stop.assert_not_called()

    def test_missing_components_start_without_cleanup_or_warmup_wait(self):
        for camera,vision,expected in ((False,False,['perception']),
                                     (True,False,['vision']),
                                     (False,True,['camera'])):
            with self.subTest(camera=camera,vision=vision):
                stop=Mock();start=Mock();policy=CameraRecovery(stop,clock=lambda:0.)
                for component in startup_components(camera,vision):start(component)
                self.assertEqual([c.args[0] for c in start.call_args_list],expected)
                self.assertIsNone(policy.tick(None,False,True));stop.assert_not_called()

    def test_reused_stream_still_recovers_after_sustained_failure(self):
        now=[0.];stop=Mock();policy=CameraRecovery(stop,clock=lambda:now[0])
        self.assertEqual(startup_components(True,True),())
        policy.tick(0.,True,True)
        now[0]=2.99;self.assertIsNone(policy.tick(0.,False,True))
        now[0]=3.;self.assertEqual(policy.tick(0.,False,True),'stopping')
        policy.join();self.assertEqual(policy.tick(0.,False,True),'start')
        stop.assert_called_once()


class RecoveryPolicyTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.stop=Mock()
        self.policy=CameraRecovery(self.stop,clock=lambda:self.now)

    def test_startup_grace_then_recovers_absent_publisher(self):
        self.now=59.;self.assertIsNone(self.policy.tick(None,False,True))
        self.now=60.;self.assertEqual(self.policy.tick(None,False,True),'stopping')
        self.policy.join()
        self.assertEqual(self.policy.tick(None,False,True),'start')
        self.stop.assert_called_once()
        self.now=119.;self.assertIsNone(self.policy.tick(None,False,True))

    def test_live_stream_and_short_gaps_do_not_restart(self):
        self.policy.tick(0.,True,True)
        self.now=2.99;self.assertIsNone(self.policy.tick(0.,False,True))
        self.now=3.;self.assertEqual(self.policy.tick(0.,False,True),'stopping')
        self.policy.join()

    def test_active_task_or_retained_authority_defers_restart(self):
        self.now=100.
        self.assertIsNone(self.policy.tick(None,False,False));self.stop.assert_not_called()
        self.assertEqual(self.policy.tick(None,False,True),'stopping');self.policy.join()

    def test_failed_cleanup_never_starts_duplicate_and_backs_off(self):
        self.stop.side_effect=RuntimeError('unowned camera')
        self.now=60.;self.assertEqual(self.policy.tick(None,False,True),'stopping')
        self.policy.join();self.assertEqual(self.policy.tick(None,False,True),'failed')
        self.assertEqual(self.policy.error,'unowned camera')
        self.now=64.;self.assertIsNone(self.policy.tick(None,False,True))
        self.now=65.;self.assertEqual(self.policy.tick(None,False,True),'stopping')
        self.policy.join();self.assertEqual(self.policy.tick(None,False,True),'failed')
        self.assertEqual(self.policy.until,75.)

    def test_successful_stability_resets_retry_counter(self):
        self.policy.attempts=5;self.policy.error='old failure'
        self.now=100.;self.policy.tick(100.,True,True)
        self.assertEqual(self.policy.attempts,0);self.assertIsNone(self.policy.error)

    def test_startup_forces_exactly_one_restart_even_with_fresh_frames(self):
        self.policy.request_startup()
        self.assertEqual(self.policy.tick(0.,True,True),'stopping')
        self.policy.join();self.assertEqual(self.policy.tick(0.,False,True),'start')
        self.assertFalse(self.policy.startup_pending)
        self.assertIsNone(self.policy.tick(0.,False,True))
        self.stop.assert_called_once()

    def test_startup_waits_until_robot_is_safe(self):
        self.policy.request_startup()
        self.assertIsNone(self.policy.tick(None,False,False))
        self.stop.assert_not_called()
        self.assertEqual(self.policy.tick(None,False,True),'stopping')
        self.policy.join()

    def test_startup_failure_remains_pending_and_respects_backoff(self):
        self.policy.request_startup();self.stop.side_effect=RuntimeError('owned cleanup failed')
        self.assertEqual(self.policy.tick(0.,True,True),'stopping')
        self.policy.join();self.assertEqual(self.policy.tick(0.,False,True),'failed')
        self.assertTrue(self.policy.startup_pending)
        self.assertIsNone(self.policy.tick(0.,True,True))
        self.now=5.;self.stop.side_effect=None
        self.assertEqual(self.policy.tick(5.,True,True),'stopping')
        self.policy.join();self.assertEqual(self.policy.tick(5.,False,True),'start')


class OwnerIsolationTests(unittest.TestCase):
    def test_only_same_project_domain_and_process_identity_can_be_stopped(self):
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory);scripts=Path(managed.__file__).resolve().parent
            table={}
            for pid,role,domain,start,actual,script in (
                (101,'camera','1','good','good',str(scripts/'managed_service.py')),
                (102,'vision','2','other','other',str(scripts/'managed_service.py')),
                (103,'perception','1','old','reused',str(scripts/'perception.py')),
                (104,'camera','1','x','x','/other/project/managed_service.py')):
                (directory/f'{pid}.lock').write_text(json.dumps(dict(pid=pid,role=role,domain=domain,start=start)))
                table[pid]={'start':actual,'argv':['python3',script]}
            with patch.dict('os.environ',{'ROS_DOMAIN_ID':'1'}),patch.object(managed,'state_directory',return_value=directory),patch.object(managed,'processes',return_value=table),patch.object(managed,'same_domain',return_value=True),patch.object(managed,'terminate',return_value=[]) as terminate,patch.object(managed,'legacy',return_value=[]):
                managed.stop_owned_perception()
                terminate.assert_called_once_with({101})

    def test_unowned_camera_prevents_restart_instead_of_being_killed(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(managed,'state_directory',return_value=Path(directory)),patch.object(managed,'processes',return_value={}),patch.object(managed,'terminate',return_value=[]) as terminate,patch.object(managed,'legacy',return_value=[999]):
            with self.assertRaisesRegex(RuntimeError,'999'):managed.stop_owned_perception()
            terminate.assert_called_once_with(set())


if __name__=='__main__':unittest.main()
