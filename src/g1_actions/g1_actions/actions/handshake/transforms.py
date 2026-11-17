"""Read-only coordinate conversion and nominal precontact target preview."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import fixed_orientation


def point3(value):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('Expected a finite XYZ point in metres')
    return point


def rigid_transform(xyz, quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1., rtol=0., atol=1e-6):
        raise ValueError('Expected a unit XYZW quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = point3(xyz)
    return result


def fixed_urdf_transform(path, parent, child):
    root = ET.parse(path).getroot()
    links = {link.get('name') for link in root.findall('link')}
    if parent not in links or child not in links:
        raise ValueError('Unknown URDF frame')
    joints = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
    chain, seen = [], set()
    while child != parent:
        if child in seen or child not in joints:
            raise ValueError('No fixed ancestor chain between requested URDF frames')
        seen.add(child); joint = joints[child]
        if joint.get('type') != 'fixed':
            raise ValueError('Measured joint positions are required for a movable URDF chain')
        origin = joint.find('origin')
        xyz = [0., 0., 0.] if origin is None else [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [0., 0., 0.] if origin is None else [float(v) for v in origin.get('rpy', '0 0 0').split()]
        if len(rpy) != 3 or not np.isfinite(rpy).all():
            raise ValueError('Invalid URDF origin RPY')
        r, p, y = rpy
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        t = np.eye(4);t[:3, 3] = point3(xyz)
        t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
        chain.append(t);child = joint.find('parent').get('link')
    result = np.eye(4)
    for t in reversed(chain):result = result@t
    return result


def camera_transform(config, package, *, require_verified=False):
    camera = config['camera_extrinsics']
    if require_verified and not camera.get('verified', False):
        raise ValueError('Camera-to-torso extrinsics have not been physically verified')
    if camera['optical_frame'] != config['frames']['camera_optical']:
        raise ValueError('Camera optical frame mismatch')
    measured = camera.get('measured_torso_to_optical')
    if measured is not None:
        return rigid_transform(measured['xyz_m'], measured['quaternion_xyzw'])
    model = Path(package)/config['model_file']
    mount = fixed_urdf_transform(model, config['frames']['robot_base'], camera['mount_link'])
    alias = camera['mount_to_camera_link']
    factory = camera['camera_link_to_optical']
    if factory['parent_frame'] != 'camera_link' or factory['child_frame'] != camera['optical_frame']:
        raise ValueError('Factory camera TF frame mismatch')
    return mount@rigid_transform(alias['xyz_m'], alias['quaternion_xyzw'])@rigid_transform(factory['xyz_m'], factory['quaternion_xyzw'])


def wrist_to_palm_center(config, package, arm, *, require_verified=False):
    if arm not in ('left', 'right'):raise ValueError('Expected left or right arm')
    tool = config['tools'][arm]
    if require_verified and not tool.get('verified', False):
        raise ValueError('Rubber-hand center has not been physically verified')
    if tool['wrist_link'] != f'{arm}_wrist_yaw_link' or tool['hand_link'] != f'{arm}_rubber_hand':
        raise ValueError('Unexpected rubber-hand tool frames')
    t = fixed_urdf_transform(Path(package)/config['model_file'], tool['wrist_link'], tool['hand_link'])
    return t[:3, :3]@point3(tool['hand_center_xyz_m'])+t[:3, 3]


def precontact_preview(config, package, camera_point, arm, *, source_frame, require_verified=False):
    if source_frame != config['frames']['camera_optical']:
        raise ValueError('Observation does not use the configured camera optical frame')
    point = point3(camera_point)
    if point[2] <= 0:raise ValueError('Camera point must be in front of the imager')
    t = camera_transform(config, package, require_verified=require_verified)
    orientation = fixed_orientation(config, arm, require_verified=require_verified)
    offset = wrist_to_palm_center(config, package, arm, require_verified=require_verified)
    human = t[:3, :3]@point+t[:3, 3]
    clearance = float(config['execution']['precontact_distance_m'])
    if not math.isfinite(clearance) or not 0. <= clearance <= .30:
        raise ValueError('Target offset must be between 0 and 30 cm')
    # Towards the robot along torso -X, never along the tilted finger axis.
    center = human+np.array([-clearance, 0., 0.])
    rotation = np.array(orientation['rotation'])
    wrist = center-rotation@offset
    return dict(arm=arm, frame_id=config['frames']['robot_base'], human_palm_xyz_m=human.tolist(),
                precontact_palm_xyz_m=center.tolist(), wrist_xyz_m=wrist.tolist(),
                wrist_quaternion_xyzw=orientation['quaternion_xyzw'],
                wrist_to_palm_center_xyz_m=offset.tolist(), clearance_m=clearance,
                geometry_verified=bool(orientation['verified'] and config['camera_extrinsics']['verified'] and config['tools'][arm]['verified']),
                ik_checked=False, collision_checked=False, executable=False, robot_motion_enabled=False)
