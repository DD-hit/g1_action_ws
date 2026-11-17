from pathlib import Path
import copy
import math
import os
import hashlib
import json
import yaml


def share():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('g1_actions'))


def load(path=None):
    path = Path(path or os.environ.get('ELE_ELEVATOR_CONFIG', share() / 'config/elevator.yaml'))
    config = yaml.safe_load(path.read_text())
    config['_config_path'] = str(path.resolve())
    # Shared hardware profile is independent of the selected action.
    config['motion']['driver_config'] = str(path.parent / 'robot.yaml')
    config['_share'] = str(share())
    return config


def resource(config, name):
    p = Path(name)
    return p if p.is_absolute() else Path(config['_share']) / p


def for_direction(config, direction):
    config = copy.deepcopy(config)
    motion = config['motion']
    maximum = float(motion['max_press_depth_m'])
    if not math.isfinite(maximum) or not 0 <= maximum <= .035:
        raise ValueError('Maximum penetration must be within 0 to 35 mm')
    depths = motion.get('press_depth_by_direction_m', {})
    if direction not in ('up', 'down') or direction not in depths:
        raise ValueError('Missing press_depth_by_direction_m for direction: '+str(direction))
    # Derived runtime value used by geometry; no shared configuration fallback.
    motion['press_depth_m'] = float(depths[direction])
    if not 0 <= motion['press_depth_m'] <= maximum - .001:
        raise ValueError('Press depth needs at least 1 mm reserve below maximum penetration')
    return config


def camera_signature(config):
    return hashlib.sha256(json.dumps(config['camera'],sort_keys=True).encode()).hexdigest()


def withdrawal_clearance(config):
    value=float(config.get('motion',{}).get('withdraw_release_clearance_m',.04))
    if not math.isfinite(value) or not .020<=value<=.040:
        raise ValueError('Withdrawal release clearance must be within 20 to 40 mm')
    return value


MOTION_STAGES = ('APPROACH', 'PRESS', 'WITHDRAW', 'RETURN',
                 'RECOVERY_WITHDRAW', 'RECOVERY_RETURN',
                 'HOVER_APPROACH', 'HOVER_RETURN', 'HEIGHT_MOVE', 'HEIGHT_RETURN')


def motion_scales(motion, stage):
    """Return driver speed/acceleration scales, not physical velocities."""
    profiles = motion.get('stage_profiles', {})
    if not isinstance(profiles, dict):
        raise ValueError('motion.stage_profiles must be a mapping')
    for name, profile in profiles.items():
        if name not in MOTION_STAGES or not isinstance(profile, dict):
            raise ValueError('Invalid motion stage profile: '+str(name))
        if set(profile) - {'speed', 'acceleration'}:
            raise ValueError('Unknown speed profile fields: '+str(name))
    defaults = {'speed': motion['speed'], 'acceleration': motion['acceleration']}
    if stage.startswith('HOVER_') or stage.startswith('HEIGHT_'):
        defaults = {'speed': motion.get('hover_speed', .20),
                    'acceleration': motion.get('hover_acceleration', .15)}
    values = dict(defaults, **profiles.get(stage, {}))
    result = []
    for key in ('speed', 'acceleration'):
        raw = values[key]
        if isinstance(raw, bool):
            raise ValueError(stage+' '+key+' must be within (0, 1]')
        value = float(raw)
        if not math.isfinite(value) or not 0 < value <= 1:
            raise ValueError(stage+' '+key+' must be within (0, 1]')
        result.append(value)
    return tuple(result)


def validate_motion_profiles(motion):
    for stage in ('DEFAULT',) + MOTION_STAGES:
        motion_scales(motion, stage)
