"""Runtime panel protection for transit and bounded button contact."""
import json
import numpy as np
from .config import resource
from .panel_motion import hand_corners, compensated_point


def binary_stl_triangles(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError('Invalid hand collision STL')
    count = int.from_bytes(data[80:84], 'little')
    if len(data) != 84+50*count or count == 0:
        raise ValueError('Expected nonempty binary collision STL')
    dtype = np.dtype([('normal','<f4',(3,)), ('vertices','<f4',(3,3)), ('attribute','<u2')])
    triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
    if not np.all(np.isfinite(triangles)):
        raise ValueError('Invalid collision mesh vertices')
    return triangles


def contact_geometry_safe(triangles, point, normal, max_depth):
    """Selected rubber-hand/target-panel contact, with whole-hand depth bound.

    Fingertip lateral/orientation checks belong to HoverGuard. This function
    does not allow any other robot link or unrelated world collision.
    """
    triangles=np.asarray(triangles,float)
    normal=np.asarray(normal,float)
    point=np.asarray(point,float)
    if (triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not triangles.size
            or point.shape!=(3,) or normal.shape!=(3,)
            or not np.all(np.isfinite(triangles)) or not np.all(np.isfinite(point))
            or not np.all(np.isfinite(normal)) or not np.isfinite(max_depth)
            or not 0 <= max_depth <= .035 or abs(np.linalg.norm(normal)-1.)>1e-4):
        return False
    return bool(np.min((triangles-point)@normal)>=-max_depth)


class HoverGuard:
    def __init__(self, task, target):
        self.task=task
        self.point=np.asarray(target['point'],float)
        self.normal=np.asarray(target['normal'],float)
        self.normal/=np.linalg.norm(self.normal)
        self.centre=compensated_point(target, task.cfg['motion'], task.arm)
        self.phase='transit'
        self.minimum_seen=float('inf')
        measurements=json.loads(resource(task.cfg,'robot/mesh_measurements.json').read_text())
        self.corners=hand_corners(measurements[task.arm])
        self.triangles=binary_stl_triangles(resource(task.cfg,'robot/meshes/'+task.arm+'_rubber_hand_collision.STL'))

    def begin_withdrawal(self):
        self.withdraw_anchor=None
        self.phase='withdraw'
        # First call captures a fresh measured anchor. It still checks full
        # hand depth and arm-link clearance, but never demands inward re-entry
        # into the narrower pressing corridor.
        self()

    def __call__(self):
        task=self.task
        names=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')
        positions={}
        for arm in ('left','right'):
            values=getattr(task.status.current_joint,arm+'_positions')
            if len(values)!=7 or not np.all(np.isfinite(values)):
                raise RuntimeError('Invalid measured arm joints for panel guard')
            positions.update((arm+'_'+name+'_joint',value) for name,value in zip(names,values))
        frame=task.cfg['base_frame'];arm=task.arm
        hand=task.robot.relative(frame,task.cfg['tools'][arm]['hand_link'],positions)
        if self.phase=='transit':
            corners=self.corners@hand[:3,:3].T+hand[:3,3]
            clearance=float(np.min((corners-self.point)@self.normal))
            if clearance<.04:
                raise RuntimeError(f'模型计算的手部间隙 {clearance*1000:.3f} mm，小于 40 mm，停止动作')
        else:
            triangles=self.triangles@hand[:3,:3].T+hand[:3,3]
            clearance=float(np.min((triangles-self.point)@self.normal))
            motion=task.cfg['motion']
            # Contact mode allows the selected rubber hand against the target
            # panel. Whole-hand penetration remains bounded; fingertip lateral
            # error and arm-link clearance are checked separately below.
            if not contact_geometry_safe(triangles,self.point,self.normal,float(motion['max_press_depth_m'])):
                raise RuntimeError('Rubber hand exceeds maximum panel penetration')
            tip=np.asarray(task.cfg['tools'][arm]['hand_tip_xyz'])@hand[:3,:3].T+hand[:3,3]
            if self.phase=='withdraw':
                if self.withdraw_anchor is None:
                    self.withdraw_anchor=(tip.copy(),hand[:3,:3].copy())
                start,rotation=self.withdraw_anchor
                delta=tip-start;outward=float(delta@self.normal)
                drift=np.linalg.norm(delta-self.normal*outward)
                rotation_error=np.linalg.norm(hand[:3,:3]-rotation)
                # Retrace may include tangent motion; only inward progress
                # and excessive rotation stop withdrawal here.
                if outward<-.002 or rotation_error>.15:
                    raise RuntimeError(f'Outward withdrawal tracking error: progress={outward:.4f}, lateral={drift:.4f}, hand_clearance={clearance:.4f}, rotation_error={rotation_error:.4f}')
            else:
                lateral=tip-self.centre;lateral-=self.normal*(lateral@self.normal)
                gap=float((tip-self.point)@self.normal)
                corridor=float(motion.get('button_aperture_half_width_m',.020))+.002+.25*max(0.,gap-.012)
                if np.linalg.norm(lateral)>corridor or np.linalg.norm(hand[:3,0]+self.normal)>.08:
                    raise RuntimeError(f'Contact normal-line/orientation error: lateral={np.linalg.norm(lateral)*1000:.3f} mm, orientation={np.linalg.norm(hand[:3,0]+self.normal):.5f}, tip_clearance={gap*1000:.3f} mm')
        self.minimum_seen=min(self.minimum_seen,clearance)
        for link in (arm+'_elbow_link',arm+'_wrist_roll_link',arm+'_wrist_yaw_link'):
            center=task.robot.relative(frame,link,positions)[:3,3]
            if (center-self.point)@self.normal<.06:
                raise RuntimeError('Arm link too close to panel: '+link)

        self.current_clearance = clearance
