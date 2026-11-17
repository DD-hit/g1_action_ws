"""Down target acquisition only; motion and perception remain unchanged."""
import copy
import math


def down_target_options(config):
    options = config.get('down_target', {})
    mode = options.get('mode', 'direct')
    if mode not in ('direct', 'from_up'):
        raise ValueError('down_target.mode must be direct or from_up')
    offset = options.get('z_offset_m', 0.06)
    if isinstance(offset, bool):
        raise ValueError('down_target.z_offset_m must be a positive finite number')
    try:
        offset = float(offset)
    except (TypeError, ValueError) as error:
        raise ValueError('down_target.z_offset_m must be a positive finite number') from error
    if not math.isfinite(offset) or offset <= 0:
        raise ValueError('down_target.z_offset_m must be a positive finite number')
    return mode, offset


def source_direction(config, direction):
    mode, _ = down_target_options(config)
    return 'up' if direction == 'down' and mode == 'from_up' else direction


def acquired_target(config, direction, source):
    """Translate in base-frame Z; inherit the measured orientation and freshness."""
    mode, offset = down_target_options(config)
    if direction != 'down' or mode == 'direct':
        return source
    if source.get('direction') != 'up' or source.get('frame_id') != config['base_frame']:
        raise RuntimeError('Expected an up target in the configured base frame')
    point = source.get('point')
    if not isinstance(point, (list, tuple)) or len(point) != 3 or not all(
            isinstance(x, (int, float)) and math.isfinite(x) for x in point):
        raise RuntimeError('Invalid source target point')
    target = copy.deepcopy(source)
    target['direction'] = 'down'
    target['point'] = [float(point[0]), float(point[1]), float(point[2]) - offset]
    # camera_point/box/confidence remain source observations, not down detections.
    target['target_acquisition'] = {
        'mode': 'from_up', 'source_direction': 'up',
        'offset_frame': config['base_frame'], 'offset_xyz_m': [0.0, 0.0, -offset],
        'source_point': list(point), 'image_coordinates_source': 'up'}
    return target
