"""Nominal robot handshake orientation; contains no motion APIs."""
import math
import numpy as np


def fixed_orientation(config, arm, *, require_verified=False):
    """Return wrist-yaw orientation in torso, separate from human observations.

    Source model: rubber hand and wrist-yaw frames have identical rotations.
    Fingers use local +X, thumb-side +Z; left/right palm normals are -Y/+Y.
    These are mesh reference axes, not a claim that curved fingers are parallel.
    """
    if arm not in ('left', 'right'):
        raise ValueError('Handshake arm must be left or right')
    execution = config['execution']
    fixed = execution['fixed_orientation']
    if execution['orientation_mode'] != 'fixed' or fixed['frame_id'] != 'torso_link':
        raise ValueError('Expected fixed orientation in torso_link')
    if require_verified and not fixed.get('verified', False):
        raise ValueError('Fixed handshake orientation has not been verified on the robot')
    angle = float(fixed['finger_downward_deg'])
    if not math.isfinite(angle) or not 0 <= angle < 90:
        raise ValueError('Finger downward angle must be in [0, 90) degrees')
    radians = math.radians(angle)
    c, s = math.cos(radians), math.sin(radians)
    rotation = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    quaternion = np.array([0., math.sin(radians/2), 0., math.cos(radians/2)])
    configured = np.asarray(fixed[f'{arm}_quaternion_xyzw'], dtype=float)
    if (configured.shape != (4,) or not np.isfinite(configured).all()
            or not np.isclose(np.linalg.norm(configured), 1., atol=1e-8, rtol=0.)
            or not np.isclose(abs(configured@quaternion), 1., atol=1e-8, rtol=0.)):
        raise ValueError('Configured quaternion does not match finger downward angle')
    link = fixed[f'{arm}_end_effector_link']
    if link != f'{arm}_wrist_yaw_link':
        raise ValueError('Fixed orientation is defined for the wrist-yaw end effector')
    return dict(frame_id=fixed['frame_id'], end_effector_link=link,
                quaternion_xyzw=quaternion.tolist(), rotation=rotation.tolist(),
                finger_direction=(rotation@np.array([1., 0., 0.])).tolist(),
                thumb_side_direction=(rotation@np.array([0., 0., 1.])).tolist(),
                palm_normal=(rotation@np.array([0., -1. if arm == 'left' else 1., 0.])).tolist(),
                verified=bool(fixed.get('verified', False)))
