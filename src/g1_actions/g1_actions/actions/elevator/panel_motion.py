"""Atomic elevator scene snapshots shared with the G1 planner."""
import itertools
import json
import numpy as np
from .config import resource


def lateral_offset(motion, arm):
    """Magnitude in metres; left is +Y and right is -Y."""
    if arm not in ('left', 'right'):
        raise ValueError('Lateral compensation requires a selected left/right arm')
    magnitude=abs(float(motion.get('target_y_offset_m',0.)))
    if not np.isfinite(magnitude):
        raise ValueError('Invalid lateral compensation')
    return magnitude if arm=='left' else -magnitude


def compensated_point(target, motion, arm):
    """Apply lateral/height compensation tangentially, preserving the observed panel plane."""
    point=np.asarray(target['point'],float).copy()
    normal=np.asarray(target['normal'],float)
    if point.shape!=(3,) or normal.shape!=(3,) or not np.all(np.isfinite(np.r_[point,normal])) or np.linalg.norm(normal)<.1:
        raise ValueError('Invalid panel point/normal')
    normal=normal/np.linalg.norm(normal)
    delta=np.array([0.,lateral_offset(motion, arm),float(motion.get('target_z_offset_m',0.))])
    if not np.all(np.isfinite(delta)):
        raise ValueError('Invalid target compensation')
    delta-=normal*(delta@normal)
    return point+delta


def panel_snapshot(cfg, arm, target, phase):
    if phase not in ('transit', 'contact', 'return', 'withdraw') or arm not in ('left', 'right'):
        raise ValueError('Invalid panel phase or arm')
    p = np.asarray(target['point'], float)
    n = np.asarray(target['normal'], float)
    if p.shape != (3,) or n.shape != (3,) or not np.all(np.isfinite(np.r_[p, n])) or np.linalg.norm(n) < .1:
        raise ValueError('Invalid panel point/normal')
    n /= np.linalg.norm(n)
    motion = cfg['motion']
    margin = float(motion.get('panel_planning_clearance_m', .05))
    penetration = float(motion['max_press_depth_m'])
    aperture = float(motion.get('button_aperture_half_width_m', .020))
    if not .045 <= margin <= .15 or not 0 <= penetration <= .035 or not .003 <= aperture <= .025:
        raise ValueError('Invalid panel safety limits')
    if not 0 <= motion['press_depth_m'] <= penetration-.001:
        raise ValueError('Press depth needs at least 1 mm reserve below maximum penetration')
    bounds = json.loads(resource(cfg, 'robot/mesh_measurements.json').read_text())[arm]
    tip = np.asarray(cfg['tools'][arm]['hand_tip_xyz'], float)
    centre = compensated_point(target, motion, arm)
    values = np.r_[p, n, float({'transit':0,'contact':1,'return':2,'withdraw':3}[phase]), 1. if arm=='left' else 2.,
                   margin, penetration, aperture, bounds['mesh_aabb_min'],
                   bounds['mesh_aabb_max'], tip, centre, float(motion['press_depth_m'])]
    if values.shape != (24,) or not np.all(np.isfinite(values)):
        raise ValueError('Invalid panel snapshot dimensions or values')
    return values.tolist()


def hand_corners(bounds):
    return np.array(list(itertools.product(*zip(bounds['mesh_aabb_min'], bounds['mesh_aabb_max']))))
