"""Snapshot execution, driver monitoring and ownership cleanup without hardware."""
import json
import os
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch, call
from types import SimpleNamespace
import uuid
import numpy as np
try:
    import rclpy
except ImportError:
    module = None
else:
    from g1_actions.actions.handshake import approach_once as module


@unittest.skipIf(module is None, 'ROS 2 interfaces required; run inside the robot container')
class ApproachCleanupTests(unittest.TestCase):
    def run_case(self, execute, start_error=False, move_error=False):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'result'
            node=MagicMock();node.owned=False;node.command_dispatched=False;node.motion_sent=False;node.planning_pending=False
            node.arm="right";node.used_fallback=False;node.selected_handedness="right";node.cancel_requested=False
            node.target.return_value=np.array([0.,0.,.5]);node.lock_target.return_value=np.array([0.,0.,.5])
            node.waist.has_upper_body_authority=False
            node.status.current_joint.left_velocities=[0.]*7;node.status.current_joint.right_velocities=[0.]*7
            node.pose.return_value=[.2,-.2,.1,0.,0.,0.]
            node.prepared_target=[.3,-.2,.1,0.,-.35,0.]
            def camera_gone(*args):
                node.vision=None;node.target.side_effect=AssertionError('Vision accessed after snapshot lock')
            node.preflight.side_effect=camera_gone
            if start_error:
                node.trigger.side_effect=lambda name: (_ for _ in ()).throw(RuntimeError('acquire failed')) if name=='/dual_arm/start' else None
            if move_error:
                def fail_move(*args):
                    node.command_dispatched=True;node.motion_sent=True
                    raise RuntimeError('motion failed')
                node.move.side_effect=fail_move
            node.prepare_target.side_effect=lambda: module.precontact_preview(
                module.yaml.safe_load((Path(module.__file__).resolve().parents[3]/'config/handshake.yaml').read_text()),
                Path(module.__file__).resolve().parents[3],node.lock_target(),'right',source_frame='camera_color_optical_frame')
            node.checked_target.side_effect=lambda target: (node.preflight(target) or target)
            argv=['approach','--managed','--output',str(output)]+(['--execute'] if execute else [])
            with patch.object(module,'Approach',return_value=node),patch.object(module.rclpy,'init'),patch.object(module.rclpy,'ok',return_value=False),patch('sys.argv',argv):
                result=module.main()
            return result,json.loads((output/'result.json').read_text()),node

    def test_preview_never_acquires_or_dispatches(self):
        code,result,node=self.run_case(False)
        self.assertEqual(code,0);self.assertTrue(result['success'])
        node.trigger.assert_not_called();node.move.assert_not_called();node.preflight.assert_not_called()
        node.lock_target.assert_called_once()
        self.assertFalse(result['hardware_motion_commands_sent'])

    def test_failed_acquisition_still_attempts_release(self):
        code,result,node=self.run_case(True,start_error=True)
        self.assertEqual(code,1);self.assertFalse(result['success'])
        self.assertEqual(node.trigger.call_args_list,[call('/dual_arm/start'),call('/dual_arm/end')])
        self.assertTrue(result['authority_released']);node.move.assert_not_called()

    def test_motion_failure_cancels_then_releases(self):
        code,result,node=self.run_case(True,move_error=True)
        self.assertEqual(code,1);node.cancel.assert_called_once()
        node.trigger.assert_any_call('/dual_arm/end');self.assertTrue(result['authority_released'])

    def test_successful_snapshot_flow_holds_two_seconds_then_returns(self):
        code,result,node=self.run_case(True)
        self.assertEqual(code,0);self.assertTrue(result['success'])
        node.lock_target.assert_called_once();node.target.assert_not_called()
        node.hold.assert_called_once_with(2.)
        self.assertEqual([c.args[1] for c in node.move.call_args_list],['APPROACH','RETURN'])
        self.assertEqual(node.trigger.call_args_list,[call('/dual_arm/start'),call('/dual_arm/end')])

    def test_initial_lock_requires_fresh_stable_frame_and_ignores_later_updates(self):
        receiver=SimpleNamespace(sync_feed=lambda:None,arm="auto",subject_handedness="any",selected_handedness=None,locked=None,locked_stamp=-1.,vision_at=10.,vision_subscription=object(),
            cfg={'frames':{'camera_optical':'optical'}},destroy_subscription=MagicMock(),event=MagicMock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000)),
            vision=dict(stamp_s=99.,frame_id='optical',palms=[dict(handedness="Right",stable=True,geometry_valid=True,pose=dict(position_m=[0.,0.,.5]))]))
        receiver.target=lambda:module.Approach.target(receiver)
        with patch.object(module.time,'monotonic',return_value=10.):
            with self.assertRaises(module.ObservationUnavailable):module.Approach.lock_target(receiver)
            receiver.vision['stamp_s']=99.9;receiver.vision['palms'][0]['stable']=False
            with self.assertRaises(module.ObservationUnavailable):module.Approach.lock_target(receiver)
            receiver.vision['palms'][0]['stable']=True
            snapshot=module.Approach.lock_target(receiver)
        self.assertIsNone(receiver.vision_subscription);self.assertIsNone(receiver.vision)
        module.Approach.on_vision(receiver,module.String(data='{"valid":false}'))
        self.assertIsNone(receiver.vision)
        np.testing.assert_allclose(snapshot,[0.,0.,.5])
        with self.assertRaises(ValueError):snapshot[0]=1.
        with self.assertRaises(RuntimeError):module.Approach.lock_target(receiver)

    def test_camera_disconnected_after_lock_does_not_interrupt_driver_monitoring(self):
        if os.environ.get('ROS_DOMAIN_ID')!='214' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            self.skipTest('Use isolated localhost ROS domain 214')
        package=Path(__file__).resolve().parents[1]
        config=module.yaml.safe_load((package/'config/handshake.yaml').read_text())
        with tempfile.TemporaryDirectory() as folder:
            module.rclpy.init();node=module.Approach(config,package,'right',Path(folder))
            try:
                node.count_publishers=lambda topic:1
                arm=module.DualArmStatus();waist=module.WaistStatus()
                waist.sdk_connected=True;waist.has_upper_body_authority=True
                def update_driver():node.on_arm(arm);node.on_waist(waist)
                update_driver();node.create_timer(.05,update_driver)
                node.on_vision(module.String(data=json.dumps(dict(
                    stamp_s=node.get_clock().now().nanoseconds*1e-9,frame_id=config['frames']['camera_optical'],
                    palms=[dict(handedness="Right",stable=True,geometry_valid=True,pose=dict(position_m=[0.,0.,.5]))]))))
                snapshot=node.lock_target().copy()
                node.on_vision(module.String(data='{"valid":false,"reason":"camera stopped"}'))
                node.target=MagicMock(side_effect=AssertionError('Vision accessed after locking'))
                node.run_process([sys.executable,'-c','import time; time.sleep(0.8)'],'helper.log',timeout=3.)
                node.hold(.8)
                np.testing.assert_allclose(node.locked,snapshot);node.target.assert_not_called()
                self.assertFalse(node.owned);self.assertFalse(node.command_dispatched)
                waist.sdk_connected=False
                with self.assertRaisesRegex(RuntimeError,'SDK disconnected'):node.health(owned=True)
                waist.sdk_connected=True;arm.emergency_stop_active=True
                with self.assertRaisesRegex(RuntimeError,'safety check'):node.health(owned=True)
                arm.emergency_stop_active=False;node.arm_at=module.time.monotonic()-1.
                with self.assertRaisesRegex(RuntimeError,'feedback is stale'):node.health(owned=True)
            finally:
                node.journal.close();node.destroy_node()
                if module.rclpy.ok():module.rclpy.shutdown()

    def test_hold_four_seconds_uses_driver_health_without_vision(self):
        clock=[0.];receiver=SimpleNamespace(event=MagicMock(),health=MagicMock())
        receiver.spin_once=lambda **kwargs:spin()
        def spin(*args,**kwargs):clock[0]+=.5
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]),patch.object(module.rclpy,'ok',return_value=True),patch.object(module.rclpy,'spin_once',side_effect=spin):
            module.Approach.hold(receiver,4.)
        self.assertEqual(clock[0],4.);self.assertEqual(receiver.health.call_count,8)
        self.assertEqual(receiver.event.call_args_list,[call('HOLD',duration_s=4.),call('HOLD_COMPLETED')])
        for invalid in (-1.,float('nan'),True,11.):
            with self.assertRaises(ValueError):module.Approach.hold(receiver,invalid)

    def test_protocol_and_schema_accept_two_second_default(self):
        from g1_actions.protocol import validate,InvalidRequest
        package=Path(__file__).resolve().parents[1]
        request=SimpleNamespace(api_version=1,request_id=str(uuid.uuid4()),action_name='handshake',arm=2,timeout_s=30.,parameters_json='{}')
        self.assertEqual(validate(request)['hold_s'],2.)
        request.parameters_json='{"hold_s":4.0}';self.assertEqual(validate(request)['hold_s'],4.)
        request.parameters_json='{"hold_s":11}';self.assertRaises(InvalidRequest,validate,request)
        schema=json.loads((package/'schemas/handshake.schema.json').read_text())
        self.assertEqual(schema['properties']['hold_s']['default'],2)

    def test_waiting_target_reports_driver_and_vision_separately(self):
        receiver=SimpleNamespace(health=MagicMock(side_effect=RuntimeError('driver missing')),
                                 target=MagicMock(side_effect=module.ObservationUnavailable('vision missing')))
        problem=module.Approach.readiness_problem(receiver)
        self.assertIn('驱动：driver missing',problem);self.assertIn('视觉：vision missing',problem)
        receiver=SimpleNamespace(readiness_problem=lambda:problem,event=MagicMock())
        def wait(predicate,*args,**kwargs):
            self.assertFalse(predicate());raise RuntimeError('timeout')
        receiver.wait=wait
        with self.assertRaisesRegex(RuntimeError,'首次目标未就绪.*driver missing.*vision missing'):
            module.Approach.wait_for_target(receiver)

    def test_target_budget_includes_setup_and_preserves_ready_snapshot(self):
        receiver=SimpleNamespace(cfg={'execution':{'target_wait_s':1.}},
                                 started_at=10.,readiness_problem=lambda:'',event=MagicMock())
        def wait(predicate,seconds,label):
            self.assertAlmostEqual(seconds,.65)
            self.assertTrue(predicate())
        receiver.wait=wait
        with patch.object(module.time,'monotonic',return_value=10.35):
            module.Approach.wait_for_target(receiver)
        receiver.event.assert_not_called()

    def test_euler_xyz_pitch_direction_and_orthogonality(self):
        rotation=module.euler_xyz_matrix([0.,np.pi/4,0.])
        np.testing.assert_allclose(rotation[:,0],[2**-.5,0.,-2**-.5],atol=1e-12)


@unittest.skipIf(module is None, 'ROS 2 interfaces required')
class FallbackBoundaryTests(unittest.TestCase):
    target = dict(arm='left', human_palm_xyz_m=[.5,0.,0.],
                  precontact_palm_xyz_m=[.5,0.,0.], wrist_to_palm_center_xyz_m=[.08,0.,0.])

    def context(self, effects):
        package=Path(__file__).resolve().parents[1]
        return SimpleNamespace(cfg=module.yaml.safe_load((package/'config/handshake.yaml').read_text()),
            package=package,arm='left',event=MagicMock(),used_fallback=False,
            preflight=MagicMock(side_effect=effects))

    def test_geometric_rejection_checks_same_arm_fixed_target_before_returning(self):
        receiver=self.context([module.TargetRejected('no IK')]*3+[None])
        checked=module.Approach.checked_target(receiver,self.target)
        self.assertTrue(receiver.used_fallback)
        self.assertEqual(checked['arm'],'left')
        self.assertEqual(checked['target_source'],'fixed_fallback')
        self.assertEqual(receiver.preflight.call_count,4)

    def test_rotated_target_keeps_palm_center(self):
        receiver=self.context([module.TargetRejected('no IK'),None])
        checked=module.Approach.checked_target(receiver,self.target)
        angles=receiver.preflight.call_args.kwargs['rpy']
        np.testing.assert_allclose(
            checked['wrist_xyz_m'] + module.euler_xyz_matrix(angles) @ self.target['wrist_to_palm_center_xyz_m'],
            self.target['precontact_palm_xyz_m'], atol=1e-12)
        self.assertFalse(receiver.used_fallback)

    def test_driver_failure_or_cancel_does_not_try_fixed_motion(self):
        for reason in ('SDK disconnected','用户请求停止','Robot moved during preflight'):
            receiver=self.context([RuntimeError(reason)])
            with self.assertRaisesRegex(RuntimeError,reason):module.Approach.checked_target(receiver,{})
            self.assertEqual(receiver.preflight.call_count,1)
            self.assertFalse(receiver.used_fallback)

    def test_both_targets_rejected_does_not_succeed_or_change_arm(self):
        receiver=self.context([module.TargetRejected('collision')]*4)
        with self.assertRaises(module.TargetRejected):module.Approach.checked_target(receiver,self.target)
        self.assertEqual(receiver.arm,'left')
        self.assertFalse(receiver.used_fallback)


if __name__=='__main__':unittest.main()
