"""Pure geometry, robust RGB-D localization and physical-button tracking."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation


def arm_from_image(center_xy, image_width):
    """Choose the arm from an unmirrored source image, before motion starts."""
    try:
        u, v = map(float, center_xy)
        width = float(image_width)
    except (TypeError, ValueError):
        raise ValueError('Auto arm selection requires target pixels and source image width') from None
    if not np.all(np.isfinite([u, v, width])) or width <= 0 or not 0 <= u < width:
        raise ValueError('Invalid target pixels or source image width for auto arm selection')
    if u == width/2:
        raise ValueError('Target is on the image midline; explicitly select left or right')
    return 'left' if u < width/2 else 'right'


def transform(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    t[:3, 3] = xyz
    return t


def transform_point(t, p):
    return t[:3, :3] @ np.asarray(p) + t[:3, 3]


def iou(a, b):
    x = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    y = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    overlap = x * y
    area = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - overlap
    return overlap / area if area > 0 else 0


def resolve_conflicts(detections, minimum=0.65, overlap=0.5, margin=0.2):
    """Reject ambiguous class pairs; suppress clear cross-class duplicates."""
    ordered = sorted(detections, key=lambda d: d['confidence'], reverse=True)
    result, used = [], set()
    for i, d in enumerate(ordered):
        if i in used or d['confidence'] < minimum:
            continue
        peers = [(j, other) for j, other in enumerate(ordered[i+1:], i+1)
                 if j not in used and iou(d['xyxy'], other['xyxy']) > overlap]
        ambiguous = any(other['direction'] != d['direction'] and
                        d['confidence']-other['confidence'] < margin for _, other in peers)
        used.update(j for j, _ in peers)
        if not ambiguous:
            result.append(d)
    return result


class StableTargets:
    def __init__(self, frames=8, jitter=4.0):
        self.frames, self.jitter = frames, jitter
        self.history = {}

    def update(self, detections):
        result, present = [], set()
        for direction in ('up', 'down'):
            candidates = [d for d in detections if d['direction'] == direction]
            # Multiple physical panels/direction candidates are ambiguous.
            if len(candidates) != 1:
                self.history.pop(direction, None)
                continue
            d = candidates[0]
            center = np.asarray(d['center_xy'], dtype=float)
            history = self.history.setdefault(direction, deque(maxlen=self.frames))
            if history and np.linalg.norm(center-history[-1]) > self.jitter*3:
                history.clear()
            history.append(center)
            present.add(direction)
            if len(history) == self.frames:
                points = np.asarray(history)
                spread = float(np.max(np.linalg.norm(points-np.median(points, axis=0), axis=1)))
                if spread <= self.jitter:
                    result.append(dict(d, center_xy=np.median(points, axis=0).tolist(),
                                       stable_frames=len(points), jitter_px=spread))
        for direction in list(self.history):
            if direction not in present:
                self.history.pop(direction, None)
        return result


def localize(depth_m, bbox, center, intrinsics, options, distortion=None):
    """Fit a robust panel plane near a button; intersect its centre ray.

    Aligned depth and RGB must have identical intrinsics and pixel coordinates.
    Plane normals face the camera; no zero/invalid depth is substituted.
    """
    h, w = depth_m.shape
    fx, fy, cx, cy = intrinsics
    if fx <= 0 or fy <= 0:
        raise ValueError('invalid camera intrinsics')
    def rays(xs, ys):
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        if distortion and any(abs(x)>1e-12 for x in distortion[1]):
            import cv2
            model, coefficients = distortion
            pixels = np.column_stack((xs.ravel(),ys.ravel())).reshape(-1,1,2)
            matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=float)
            if model == 'equidistant':
                normalized = cv2.fisheye.undistortPoints(pixels,matrix,np.asarray(coefficients,dtype=float))
            elif model in ('plumb_bob','rational_polynomial'):
                normalized = cv2.undistortPoints(pixels,matrix,np.asarray(coefficients,dtype=float))
            else:
                raise ValueError('Unsupported camera distortion model: '+model)
            return normalized.reshape(-1,2)
        return np.column_stack(((xs.ravel()-cx)/fx,(ys.ravel()-cy)/fy))
    u, v = center
    x1, y1, x2, y2 = bbox
    pad = max(12, int(max(x2-x1, y2-y1)*0.65))
    xa, xb = max(0, int(x1)-pad), min(w, int(x2)+pad+1)
    ya, yb = max(0, int(y1)-pad), min(h, int(y2)+pad+1)
    yy, xx = np.mgrid[ya:yb:2, xa:xb:2]
    z = depth_m[ya:yb:2, xa:xb:2]
    valid = np.isfinite(z) & (z > options['min_m']) & (z < options['max_m'])
    # Fit the surrounding panel, avoiding the protruding shiny button itself.
    outside = (xx < x1) | (xx > x2) | (yy < y1) | (yy > y2)
    valid &= outside
    xy = rays(xx[valid],yy[valid])
    points = np.column_stack((xy[:,0]*z[valid],xy[:,1]*z[valid],z[valid]))
    if len(points) < options['min_plane_points']:
        raise ValueError('insufficient valid depth around button')
    rng = np.random.default_rng(17)
    best = None
    limit = options['max_plane_residual_m']
    for _ in range(70):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        inliers = np.abs((points-a) @ n) < limit
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < max(options['min_plane_points'], len(points)*0.60):
        raise ValueError('no consistent button-panel plane')
    cloud = points[best]
    origin = np.median(cloud, axis=0)
    _, _, vt = np.linalg.svd(cloud-origin, full_matrices=False)
    normal = vt[-1]
    if normal @ origin > 0:
        normal = -normal
    ray = np.r_[rays([u],[v])[0],1.0]
    cosine = abs(normal @ (ray/np.linalg.norm(ray)))
    if cosine < math.cos(math.radians(options['max_incidence_deg'])):
        raise ValueError('panel viewed at excessive incidence angle')
    distance = float(normal @ origin / (normal @ ray))
    if not options['min_m'] < distance < options['max_m']:
        raise ValueError('invalid plane-ray intersection')
    panel_point = ray * distance
    # Prefer observed button face depth when it is close to the fitted panel.
    uc, vc = int(round(u)), int(round(v))
    patch = depth_m[max(0,vc-3):min(h,vc+4), max(0,uc-3):min(w,uc+4)]
    good = patch[np.isfinite(patch) & (patch > options['min_m']) & (patch < options['max_m'])]
    face_offset = 0.0
    if good.size >= 9:
        candidate = ray * np.median(good)
        face_offset = float((candidate-panel_point) @ normal)
        if -0.003 <= face_offset <= 0.025:
            panel_point = candidate
        else:
            raise ValueError('button depth inconsistent with panel')
    else:
        raise ValueError('button face depth unavailable; do not press a guessed surface')
    residual = float(np.median(np.abs((cloud-origin) @ normal)))
    return panel_point, normal, {'plane_residual_m': residual,
                                  'plane_points': int(best.sum()), 'face_offset_m': face_offset}


def press_poses(point, outward_normal, wrist_tip, config, palm_normal=None):
    """Convert desired middle-tip poses to wrist-yaw poses expected by G1."""
    point, normal, tip = map(lambda v: np.asarray(v, float), (point, outward_normal, wrist_tip))
    if not np.all(np.isfinite(np.r_[point, normal, tip])) or np.linalg.norm(normal) < 0.9:
        raise ValueError('invalid target geometry')
    normal /= np.linalg.norm(normal)
    if np.any(point < config['workspace_min']) or np.any(point > config['workspace_max']):
        raise ValueError('target outside configured torso workspace')
    if np.linalg.norm(point) > config['max_reach_m']:
        raise ValueError('target exceeds conservative reach bound')
    if not 0 <= config['press_depth_m'] <= config['max_press_depth_m'] <= 0.035:
        raise ValueError('invalid press travel limit')
    x_axis = -normal
    up = np.array([0.0, 0.0, 1.0])
    z_axis = up-x_axis*(up@x_axis)
    if np.linalg.norm(z_axis) < 0.2:
        raise ValueError('panel normal unsuitable for horizontal finger approach')
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    if palm_normal is not None:
        # Tool-local palm normal: left -Y, right +Y. Align it with
        # gravity projected onto the panel, while keeping fingers along -normal.
        palm = np.asarray(palm_normal, float)
        if palm.shape != (3,) or not np.all(np.isfinite(palm)) or abs(palm[0]) > 1e-6 or np.linalg.norm(palm) < .9:
            raise ValueError('Palm normal must be finite and perpendicular to the finger axis')
        palm /= np.linalg.norm(palm)
        finger = np.array([1., 0., 0.])
        local = np.column_stack((finger, palm, np.cross(finger, palm)))
        down = -z_axis
        desired = np.column_stack((x_axis, down, np.cross(x_axis, down)))
        rotation = desired @ local.T
    result = {}
    for name, offset in [('approach', config['approach_m']), ('hover', config['hover_m']),
                         ('press', -config['press_depth_m']), ('retreat', config['approach_m'])]:
        tcp = point+normal*offset
        wrist = tcp-rotation@tip
        result[name] = {'tcp_xyz': tcp.tolist(), 'xyz': wrist.tolist(),
                        # Driver PoseQuaternion uses Rx(rx) @ Ry(ry) @ Rz(rz).
                        'rpy': Rotation.from_matrix(rotation).as_euler('XYZ').tolist()}
    return result
