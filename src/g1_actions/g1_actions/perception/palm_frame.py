"""Right-handed palm axes from model landmarks, not a calibrated metric pose."""
from collections import Counter, deque
import math

import numpy as np

PALM = [0, 5, 9, 13, 17]
MCP = [5, 9, 13, 17]


def rotation_distance(a, b):
    value = (np.trace(a.T @ b) - 1.) / 2.
    return math.degrees(math.acos(float(np.clip(value, -1., 1.))))


def estimate_model_frame(world, handedness, config):
    points = np.asarray(world, dtype=float)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        return {'valid': False, 'reason': 'invalid_landmarks'}
    if handedness not in ('Right', 'Left'):
        return {'valid': False, 'reason': 'unknown_handedness'}
    palm = points[PALM]
    centroid = palm.mean(axis=0)
    _, singular, vt = np.linalg.svd(palm - centroid)
    span = float(np.linalg.norm(points[5] - points[17]))
    if span < config.get('min_palm_span_m', .02) or singular[1] < 1e-6:
        return {'valid': False, 'reason': 'degenerate_palm'}
    residual = float(singular[2] / singular[1])
    if residual > config.get('max_plane_residual_ratio', .5):
        return {'valid': False, 'reason': 'nonplanar_palm'}
    forward = points[MCP].mean(axis=0) - points[0]
    normal_hint = np.cross(points[5] - points[17], forward)
    sign = config.get('normal_sign_by_label', {'Right': 1, 'Left': -1})[handedness]
    if sign not in (-1, 1) or np.linalg.norm(normal_hint) < 1e-8:
        return {'valid': False, 'reason': 'ambiguous_normal'}
    z = vt[-1].copy()
    if np.dot(z, normal_hint * sign) < 0:
        z *= -1
    y = forward - np.dot(forward, z) * z
    if np.linalg.norm(y) < 1e-6:
        return {'valid': False, 'reason': 'ambiguous_finger_direction'}
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    thumb = points[4] - points[2]
    thumb_norm = np.linalg.norm(thumb)
    return dict(valid=True, rotation=np.column_stack((x, y, z)),
                origin=(points[0] + points[MCP].mean(axis=0)) / 2.,
                thumb_direction=thumb / thumb_norm if thumb_norm > 1e-6 else None,
                palm_span_m=span, plane_residual_ratio=residual)


class PalmFrameTracker:
    """Small image-space tracker with locked handedness and SO(3) smoothing.

    A raw label change never reverses a confirmed track's palm normal. A large
    rotation requires several consistent observations before reacquisition.
    """
    def __init__(self, config):
        self.config = config
        self.tracks = {}
        self.next_id = 1
        self.last_timestamp = -1
        self.stats = Counter()
        self.raw_steps = []
        self.smoothed_steps = []

    def update(self, hands, timestamp_ms):
        if timestamp_ms <= self.last_timestamp:
            raise ValueError('Palm frame timestamps must increase')
        self.last_timestamp = timestamp_ms
        now = timestamp_ms / 1000.
        expiry = self.config.get('max_missing_s', .3)
        self.tracks = {i: t for i, t in self.tracks.items() if now - t['seen'] <= expiry}
        centers = []
        for hand in hands:
            points = np.asarray(hand['landmarks_normalized'], dtype=float)
            if points.shape != (21, 3) or not np.isfinite(points).all():
                raise ValueError('Invalid image landmarks for palm tracking')
            centers.append((points[0, :2] + points[MCP, :2].mean(axis=0)) / 2.)
        candidates = sorted((float(np.linalg.norm(center - track['center'])), j, i)
                            for j, center in enumerate(centers)
                            for i, track in self.tracks.items())
        assignments = {}
        used = set()
        for distance, j, i in candidates:
            if distance > self.config.get('match_distance_norm', .2):
                continue
            if j not in assignments and i not in used:
                assignments[j] = i
                used.add(i)
        for j, hand in enumerate(hands):
            if j not in assignments:
                identity = self.next_id
                self.next_id += 1
                self.tracks[identity] = dict(center=centers[j], seen=now,
                    votes=deque(maxlen=self.config.get('confirm_frames', 5)),
                    side=None, rotation=None, raw=None, updated=None,
                    pending_rotation=None, pending_count=0)
                assignments[j] = identity
            identity = assignments[j]
            track = self.tracks[identity]
            track.update(center=centers[j], seen=now)
            label = hand['handedness_raw']
            score = float(hand['handedness_score'])
            track['votes'].append((label, score))
            if track['side'] is None and len(track['votes']) == track['votes'].maxlen:
                weights = Counter()
                for side, confidence in track['votes']:
                    if side in ('Right', 'Left'):
                        weights[side] += confidence
                if weights:
                    side, weight = weights.most_common(1)[0]
                    total = sum(weights.values())
                    if (weight / total >= self.config.get('side_consensus', .8)
                            and total / len(track['votes']) >= self.config.get('min_handedness_score', .6)):
                        track['side'] = side
            result = dict(track_id=identity, handedness_stable=track['side'],
                          center_image=centers[j].tolist(), valid=False,
                          coordinate_frame='mediapipe_estimated_world',
                          metric_camera_pose=False)
            self.stats['observations'] += 1
            if track['side'] is None:
                result['status'] = 'pending_handedness'
                hand['palm_frame'] = result
                self.stats[result['status']] += 1
                continue
            disagree = label != track['side']
            result['raw_label_disagrees'] = disagree
            self.stats['raw_label_disagreements_held'] += int(disagree)
            frame = estimate_model_frame(hand['landmarks_model_world'], track['side'], self.config)
            if not frame['valid']:
                result['status'] = frame['reason']
            else:
                self._orient(track, frame, result, now)
            hand['palm_frame'] = result
            self.stats[result['status']] += 1
        return hands

    def _orient(self, track, frame, result, now):
        raw = frame['rotation']
        result.update(rotation_raw=raw.tolist(), origin_model=frame['origin'].tolist(),
                      thumb_direction_model=(None if frame['thumb_direction'] is None
                                             else frame['thumb_direction'].tolist()),
                      palm_span_model_m=frame['palm_span_m'],
                      plane_residual_ratio=frame['plane_residual_ratio'])
        if track['raw'] is not None:
            self.raw_steps.append(rotation_distance(track['raw'], raw))
        track['raw'] = raw
        previous = track['rotation']
        step = 0. if previous is None else rotation_distance(previous, raw)
        result['raw_to_filtered_angle_deg'] = step
        if previous is not None and step > self.config.get('jump_reject_deg', 70.):
            pending = track['pending_rotation']
            consistent = (pending is not None and rotation_distance(pending, raw)
                          <= self.config.get('reacquire_consistency_deg', 20.))
            track['pending_count'] = track['pending_count'] + 1 if consistent else 1
            track['pending_rotation'] = raw
            if track['pending_count'] < self.config.get('reacquire_frames', 3):
                result['status'] = 'rotation_jump_rejected'
                return
            filtered = raw
            result['status'] = 'orientation_reacquired'
        elif previous is None:
            filtered = raw
            result['status'] = 'tracked'
        else:
            dt = now - track['updated']
            tau = self.config.get('smoothing_tau_s', .08)
            alpha = 1. if tau <= 0 else 1. - math.exp(-dt / tau)
            u, _, vt = np.linalg.svd((1. - alpha) * previous + alpha * raw)
            correction = np.diag([1., 1., float(np.linalg.det(u @ vt))])
            filtered = u @ correction @ vt
            self.smoothed_steps.append(rotation_distance(previous, filtered))
            result['status'] = 'tracked'
        track.update(rotation=filtered, updated=now, pending_rotation=None, pending_count=0)
        result.update(valid=True, rotation=filtered.tolist(),
                      facing=('palm' if filtered[2, 2] < -.2
                              else 'back' if filtered[2, 2] > .2 else 'edge'))

    def summary(self):
        return dict(self.stats, track_count=self.next_id - 1,
                    step_statistics_note='raw includes rejected observations; filtered excludes reacquisition steps',
                    raw_step_p95_deg=(None if not self.raw_steps else float(np.percentile(self.raw_steps, 95))),
                    filtered_step_p95_deg=(None if not self.smoothed_steps else float(np.percentile(self.smoothed_steps, 95))))


def draw_palm_frame(image, hand, config):
    import cv2
    result = hand.get('palm_frame')
    if not result:
        return
    height, width = image.shape[:2]
    origin = tuple(round(v * n) for v, n in zip(result['center_image'], (width, height)))
    if not result['valid']:
        cv2.putText(image, f"#{result['track_id']} {result['status']}",
                    (max(0, origin[0] - 60), min(height - 10, origin[1] + 30)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 150, 255), 1, cv2.LINE_AA)
        return
    rotation = np.asarray(result['rotation'])
    normalized = np.asarray(hand['landmarks_normalized'])
    palm_width = np.linalg.norm((normalized[5, :2] - normalized[17, :2]) * [width, height])
    length = float(np.clip(palm_width * config.get('axis_length_palm_ratio', .7), 30., 95.))
    axes = [('X', rotation[:, 0], (0, 0, 255)),
            ('Y', rotation[:, 1], (0, 210, 0)),
            ('Z', rotation[:, 2], (255, 80, 0))]
    thumb = result['thumb_direction_model']
    if thumb is not None:
        axes.append(('thumb', np.asarray(thumb), (255, 0, 255)))
    for name, axis, color in axes:
        end = (round(origin[0] + length * axis[0]), round(origin[1] + length * axis[1]))
        if np.linalg.norm(axis[:2]) < .25:
            cv2.circle(image, origin, 9, color, 2, cv2.LINE_AA)
            if axis[2] < 0:
                cv2.circle(image, origin, 3, color, -1, cv2.LINE_AA)
            else:
                cv2.line(image, (origin[0]-5, origin[1]-5), (origin[0]+5, origin[1]+5), color, 2)
                cv2.line(image, (origin[0]-5, origin[1]+5), (origin[0]+5, origin[1]-5), color, 2)
            end = (origin[0] + 12, origin[1] + (15 if name == 'Z' else -12))
        else:
            cv2.arrowedLine(image, origin, end, color, 3, cv2.LINE_AA, tipLength=.2)
        cv2.putText(image, name, (end[0]+3, end[1]-3), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, name, (end[0]+3, end[1]-3), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
    cv2.circle(image, origin, 4, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.putText(image, f"#{result['track_id']} {result['handedness_stable']} {result['facing']}",
                (max(0, origin[0]-70), round(min(height-10, origin[1]+length+20))),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (40, 40, 40), 3, cv2.LINE_AA)
    cv2.putText(image, f"#{result['track_id']} {result['handedness_stable']} {result['facing']}",
                (max(0, origin[0]-70), round(min(height-10, origin[1]+length+20))),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)


def draw_palm_legend(image):
    import cv2
    cv2.rectangle(image, (0, 32), (image.shape[1], 83), (25, 25, 25), -1)
    cv2.putText(image, 'PALM: X red | Y green | Z blue | thumb magenta',
                (6, 50), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, 'Z dot: toward view | cross: away | MODEL ESTIMATE',
                (6, 70), cv2.FONT_HERSHEY_SIMPLEX, .38, (255, 255, 255), 1, cv2.LINE_AA)
