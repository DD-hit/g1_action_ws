"""Hardware-independent panel/contact geometry regression tests."""
import copy
import unittest
from pathlib import Path
import numpy as np
from g1_actions.actions.elevator.config import load, for_direction
from g1_actions.actions.elevator.panel_motion import panel_snapshot
from g1_actions.actions.elevator.hover_guard import contact_geometry_safe

ROOT=Path(__file__).resolve().parents[1]


class PanelGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg=for_direction(load(ROOT/'config/elevator.yaml'),'up')
        self.target={'point':[.5,.2,.1],'normal':[-1.,0.,0.]}

    def test_snapshot_atomic_layout_and_phase(self):
        values=panel_snapshot(self.cfg,'left',self.target,'transit')
        self.assertEqual(len(values),24)
        self.assertEqual(values[6:11],[0.,1.,.05,.035,.020])
        contact=panel_snapshot(self.cfg,'right',self.target,'contact')
        self.assertEqual(contact[6:8],[1.,2.])
        escape=panel_snapshot(self.cfg,'right',self.target,'withdraw')
        self.assertEqual(escape[6:8],[3.,2.])

    def test_nonfinite_and_zero_normals_rejected(self):
        for normal in ([0.,0.,0.],[float('nan'),0.,0.]):
            self.target['normal']=normal
            with self.assertRaises(ValueError):panel_snapshot(self.cfg,'left',self.target,'transit')

    def test_depth_reserve_required(self):
        self.cfg['motion']['press_depth_m']=self.cfg['motion']['max_press_depth_m']
        with self.assertRaises(ValueError):panel_snapshot(self.cfg,'left',self.target,'contact')

    def test_shared_y_offset_for_both_directions_and_arms(self):
        from g1_actions.actions.elevator.config import for_direction
        from g1_actions.actions.elevator.panel_motion import compensated_point
        for direction in ('up', 'down'):
            cfg=for_direction(self.cfg,direction)
            cfg['motion']['target_y_offset_m']=.005
            for arm in ('left','right'):
                target=dict(self.target,direction=direction)
                point=compensated_point(target,cfg['motion'],arm)
                np.testing.assert_allclose(point,[.5,.205 if arm=='left' else .195,.14],atol=1e-12)
                values=panel_snapshot(cfg,arm,target,'contact')
                np.testing.assert_allclose(values[20:23],point)
                self.assertEqual(target['point'],self.target['point'])

    def test_y_offset_missing_zero_and_custom_value(self):
        from g1_actions.actions.elevator.panel_motion import compensated_point
        for value in (None,0.,.012,-.012):
            motion={'target_z_offset_m':.04}
            if value is not None:motion['target_y_offset_m']=value
            for arm,sign in (('left',1),('right',-1)):
                point=compensated_point(self.target,motion,arm)
                np.testing.assert_allclose(point,[.5,.2+sign*abs(value or 0.),.14])

    def test_combined_offsets_preserve_oblique_plane_and_reject_nonfinite(self):
        from g1_actions.actions.elevator.panel_motion import compensated_point
        target=dict(self.target,normal=[-.9,.3,.4])
        normal=np.asarray(target['normal']);normal/=np.linalg.norm(normal)
        point=compensated_point(target,dict(self.cfg['motion'],target_y_offset_m=.005),'left')
        self.assertAlmostEqual(float((point-target['point'])@normal),0.,places=12)
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                compensated_point(target,dict(self.cfg['motion'],target_y_offset_m=value),'right')

    def test_compensation_preserves_tilted_panel_depth(self):
        self.target['normal']=[-.9,0.,.4]
        values=panel_snapshot(self.cfg,'left',self.target,'contact')
        normal=np.asarray(values[3:6])
        centre=np.asarray(values[20:23])
        self.assertAlmostEqual(float((centre-self.target['point'])@normal),0.,places=10)
        self.assertGreater(centre[2],self.target['point'][2])

    def safe(self,triangles):
        return contact_geometry_safe(np.asarray(triangles,float),np.zeros(3),np.array([0.,0.,1.]),.005)

    def test_no_contact_safe(self):
        self.assertTrue(self.safe([[[.1,0,.01],[0,.1,.01],[0,0,.01]]]))

    def test_contact_inside_button_safe(self):
        self.assertTrue(self.safe([[[0,0,-.003],[.003,0,-.003],[0,.003,-.003]]]))

    def test_selected_hand_contact_outside_old_aperture_is_allowed(self):
        self.assertTrue(self.safe([[[.02,0,-.003],[.023,0,-.003],[.02,.003,-.003]]]))

    def test_excessive_depth_rejected(self):
        self.assertFalse(self.safe([[[0,0,-.006],[.003,0,-.006],[0,.003,-.006]]]))

    def test_selected_hand_contact_is_depth_bounded_instead_of_aperture_clipped(self):
        # Only submerged vertex is inside; the crossing edges leave the aperture.
        self.assertTrue(self.safe([[[0,0,-.003],[.1,0,.001],[0,.1,.001]]]))


class ContactModelTests(unittest.TestCase):
    def test_extended_endpoint_is_allowed_for_selected_rubber_hand(self):
        # Detect undersized apertures against the actual shipped collision mesh.
        from g1_actions.actions.elevator.hover_guard import binary_stl_triangles
        from g1_actions.actions.elevator.config import resource
        cfg=for_direction(load(ROOT/'config/elevator.yaml'),'up')
        for arm in ('left','right'):
            triangles=binary_stl_triangles(resource(cfg,'robot/meshes/'+arm+'_rubber_hand_collision.STL'))
            tip=np.asarray(cfg['tools'][arm]['hand_tip_xyz'])
            shifted=triangles-tip+np.array([cfg['motion']['press_depth_m'],0.,0.])
            self.assertTrue(contact_geometry_safe(shifted,np.zeros(3),np.array([-1.,0.,0.]),
                cfg['motion']['max_press_depth_m']),arm)


class RuntimeGuardTests(unittest.TestCase):
    def test_lateral_offset_does_not_stop_withdrawal_before_release_clearance(self):
        guard,hand,link=self.withdrawal(.0199,.011)
        guard.task.cfg['motion']['withdraw_release_clearance_m']=.020
        for clearance in (.0199,.020,.0199):
            hand[2,3]=clearance;guard()
            self.assertAlmostEqual(guard.current_clearance,clearance)

    def test_withdrawal_clearance_configuration_is_bounded(self):
        from g1_actions.actions.elevator.config import withdrawal_clearance
        self.assertEqual(withdrawal_clearance({'motion':{'withdraw_release_clearance_m':.020}}),.020)
        for bad in (.019,.041,float('nan'),float('inf')):
            with self.assertRaises(ValueError):withdrawal_clearance({'motion':{'withdraw_release_clearance_m':bad}})
    def withdrawal(self,clearance,lateral):
        from types import SimpleNamespace
        from g1_actions.actions.elevator.hover_guard import HoverGuard
        hand=np.eye(4);hand[:3,3]=[lateral,0.,clearance]
        link=np.eye(4);link[2,3]=.3
        robot=SimpleNamespace(relative=lambda frame,name,positions:hand if name.endswith('rubber_hand') else link)
        task=SimpleNamespace(cfg={'base_frame':'torso_link','motion':{'max_press_depth_m':.025},'tools':{'right':{'hand_link':'right_rubber_hand','hand_tip_xyz':[0.,0.,0.]}}},
            arm='right',robot=robot,status=SimpleNamespace(current_joint=SimpleNamespace(left_positions=[0.]*7,right_positions=[0.]*7)))
        guard=HoverGuard.__new__(HoverGuard)
        guard.task=task;guard.phase='withdraw';guard.point=np.zeros(3);guard.normal=np.array([0.,0.,1.])
        guard.triangles=np.zeros((1,3,3));guard.minimum_seen=float('inf')
        guard.withdraw_anchor=(np.zeros(3),np.eye(3))
        return guard,hand,link

    def test_reported_10_3mm_lateral_error_is_allowed_during_outward_retrace(self):
        guard,hand,link=self.withdrawal(.0047,.0103)
        guard.withdraw_anchor=(np.array([0.,0.,-.0077]),np.eye(3))
        guard()
        self.assertAlmostEqual(guard.current_clearance,.0047)
        for clearance,lateral in ((.0199,.02),(.0399,.05),(.041,.05)):
            hand[:3,3]=[lateral,0.,clearance];guard()

    def test_clear_hand_still_checks_inward_motion_and_rotation(self):
        guard,hand,link=self.withdrawal(.05,.02)
        guard.withdraw_anchor=(np.array([0.,0.,.06]),np.eye(3))
        with self.assertRaisesRegex(RuntimeError,'progress='):guard()
        guard.withdraw_anchor=(np.zeros(3),np.eye(3))
        theta=.2;hand[:2,:2]=[[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]
        with self.assertRaisesRegex(RuntimeError,'rotation_error='):guard()

    def test_depth_and_arm_link_protection_remain(self):
        guard,hand,link=self.withdrawal(-.026,0.)
        with self.assertRaisesRegex(RuntimeError,'maximum panel penetration'):guard()
        hand[2,3]=.05;link[2,3]=.05
        with self.assertRaisesRegex(RuntimeError,'Arm link'):guard()

    def test_unsafe_initial_hand_is_rejected(self):
        from types import SimpleNamespace
        from g1_actions.actions.elevator.config import resource
        from g1_actions.actions.elevator.robot_model import Robot
        from g1_actions.actions.elevator.hover_guard import HoverGuard
        cfg=for_direction(load(ROOT/'config/elevator.yaml'),'up')
        task=SimpleNamespace(cfg=cfg,arm='left',robot=Robot(resource(cfg,cfg['model_file'])),
            status=SimpleNamespace(current_joint=SimpleNamespace(left_positions=[0.]*7,right_positions=[0.]*7)))
        guard=HoverGuard(task,{'point':[.35,.2,.15],'normal':[-1.,0.,0.]})
        with self.assertRaisesRegex(RuntimeError,'40 mm'):guard()


if __name__=='__main__':unittest.main()

class ArrivalToleranceTests(unittest.TestCase):
    def test_tangent_tracking_error_does_not_change_depth_tolerance(self):
        from types import SimpleNamespace
        from g1_actions.actions.elevator.task_node import PressTask
        from g1_actions.actions.elevator.robot_model import Robot
        from g1_actions.actions.elevator.config import resource
        cfg=for_direction(load(ROOT/'config/elevator.yaml'),'up')
        task=SimpleNamespace(cfg=cfg,arm='right',robot=Robot(resource(cfg,cfg['model_file'])),
            locked_target={'normal':[-1.,0.,0.]})
        target={'xyz':[.2,-.2,.1],'rpy':[0.,0.,0.]}
        task.current_pose=lambda:{'xyz':[.2,-.2,.106],'rpy':[0.,0.,0.]}
        self.assertTrue(PressTask.close_to(task,target,contact=True))
        self.assertTrue(PressTask.close_to(task,target,ready_for_contact=True))
        task.current_pose=lambda:{'xyz':[.2,-.2,.116],'rpy':[0.,0.,0.]}
        self.assertTrue(PressTask.close_to(task,target,contact=True))
        task.current_pose=lambda:{'xyz':[.204,-.2,.1],'rpy':[0.,0.,0.]}
        self.assertTrue(PressTask.close_to(task,target,contact=True))
        task.current_pose=lambda:{'xyz':[.206,-.2,.1],'rpy':[0.,0.,0.]}
        self.assertFalse(PressTask.close_to(task,target,contact=True))

class FeedbackCorrelationTests(unittest.TestCase):
    def test_motion_feedback_is_correlated_by_type_arm_and_exact_timestamp(self):
        import json
        from types import SimpleNamespace
        from std_msgs.msg import String
        from g1_actions.actions.elevator.task_node import PressTask
        task=SimpleNamespace(motion_feedback={})
        sample={'phase':'rejected','command':'MoveL','arm_mask':2,'request_sec':10,'request_nanosec':11,'reason':'collision'}
        PressTask.on_motion_feedback(task,String(data=json.dumps(sample)))
        self.assertEqual(task.motion_feedback[('MoveL',2,10,11)]['reason'],'collision')
        self.assertNotIn(('MovePJ',2,10,11),task.motion_feedback)
        self.assertNotIn(('MoveL',1,10,11),task.motion_feedback)
        PressTask.on_motion_feedback(task,String(data='{}'))
        self.assertEqual(len(task.motion_feedback),1)

    def test_maximum_penetration_still_rejects_the_whole_hand(self):
        triangle=np.array([[[0,0,-.016],[.1,0,-.016],[0,.1,-.016]]])
        self.assertFalse(contact_geometry_safe(triangle,np.zeros(3),np.array([0.,0.,1.]),.015))
