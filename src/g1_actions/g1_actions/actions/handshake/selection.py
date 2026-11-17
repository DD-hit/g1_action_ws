"""Deterministic same-handed selection and fixed torso-frame fallback."""
import numpy as np
from .geometry import fixed_orientation
from .transforms import point3, wrist_to_palm_center


def select_arm(requested, handedness):
    if requested in ('left', 'right'):
        return requested
    if requested != 'auto':
        raise ValueError('Unknown robot arm')
    label = str(handedness).lower()
    if label not in ('left', 'right'):
        raise ValueError('Stable human handedness is required for auto')
    return label


def fixed_fallback(config, package, arm, original):
    fallback = config['execution']['fixed_fallback']
    if not fallback.get('enabled', False):
        raise RuntimeError('目标预检失败，固定握手备用位姿未启用')
    if fallback['frame_id'] != config['frames']['robot_base']:
        raise ValueError('Fixed fallback must use the robot base frame')
    center = point3(fallback[arm]['palm_xyz_m'])
    orientation = fixed_orientation(config, arm)
    offset = wrist_to_palm_center(config, package, arm)
    wrist = center - np.asarray(orientation['rotation']) @ offset
    return dict(original, precontact_palm_xyz_m=center.tolist(), wrist_xyz_m=wrist.tolist(),
                wrist_quaternion_xyzw=orientation['quaternion_xyzw'], target_source='fixed_fallback',
                ik_checked=False, collision_checked=False, executable=False)
