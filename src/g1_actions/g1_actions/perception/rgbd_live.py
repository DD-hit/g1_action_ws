"""Read-only RGB-D palm observations. No driver or motion APIs are imported."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PointStamped
from g1_action_interfaces.msg import HandObservation, PalmObservation

from .hand_landmarks import HandLandmarkDetector
from .palm_frame import PalmFrameTracker, rotation_distance
from .palm_pose import estimate_palm_pose, estimate_palm_position, image_array, validate_pair


def stamp_s(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def metadata(message):
    return dict(stamp_s=stamp_s(message), frame_id=message.header.frame_id,
                width=message.width, height=message.height)


class PalmRGBD(Node):
    def __init__(self, config, package, output, record_rgbd=False):
        super().__init__('g1_handshake_rgbd')
        self.cfg, self.output = config, output
        self.camera_verified = bool(config.get('camera_extrinsics', {}).get('verified', False))
        self.record_rgbd = record_rgbd
        if record_rgbd:
            (output/'frames').mkdir()
        self.perception = config['perception']
        self.camera = self.perception['rgbd']
        self.position_only = self.camera.get('position_method', 'plane') == 'wrist_offset'
        if self.camera.get('position_method', 'plane') not in ('plane', 'wrist_offset'):
            raise ValueError('Unknown palm position method')
        self.detector = HandLandmarkDetector(package/self.perception['model_path'], self.perception)
        tracker_config = dict(self.perception['palm_frame'])
        tracker_config['max_missing_s'] = self.camera.get('track_max_missing_s', .5)
        self.tracker = PalmFrameTracker(tracker_config)
        self.colors, self.depths = deque(maxlen=8), deque(maxlen=8)
        self.info = None
        self.last_stamp = -1.
        self.last_pair = self.last_notice = time.monotonic()
        self.last_save = 0.
        self.gates = {}
        self.counts = dict(processed_frames=0, detected_frames=0, geometry_frames=0, stable_frames=0)
        self.file = (output/'observations.jsonl').open('w', buffering=1)
        self.status = self.create_publisher(String, '/handshake/perception_status', 1)
        self.palms = self.create_publisher(PalmObservation, '/handshake/palm_observations', 1)
        self.hands = self.create_publisher(HandObservation, '/handshake/hand_observations', 1)
        self.positions = self.create_publisher(PointStamped, '/handshake/palm_positions', 1)
        self.poses = self.create_publisher(PoseArray, '/handshake/palm_poses', 1)
        self.images = self.create_publisher(Image, '/handshake/image_annotated', 1)
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, self.camera['color_topic'], self.colors.append, qos)
        self.create_subscription(Image, self.camera['depth_topic'], self.depths.append, qos)
        self.create_subscription(CameraInfo, self.camera['info_topic'], self.on_info, qos)
        self.create_timer(.02, self.tick)
        self.get_logger().info(f'Read-only RGB-D, delegate={self.detector.delegate}; outputs use RGB optical frame. Torso coordinates are not published by this perception node.')

    def on_info(self, message):
        self.info = message

    def invalidate(self, reason):
        self.gates.clear()
        now = time.monotonic()
        if now-self.last_notice < .5:
            return
        self.last_notice = now
        data = dict(valid=False, reason=reason, palms=[], robot_motion_enabled=False)
        self.status.publish(String(data=json.dumps(data)))
        msg = PalmObservation(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '' if self.info is None else self.info.header.frame_id
        msg.valid = False; msg.message = reason; self.palms.publish(msg)
        poses = PoseArray(); poses.header = msg.header; self.poses.publish(poses)
        self.get_logger().warning(reason)

    def tick(self):
        try:
            if self.info is None or not self.colors or not self.depths:
                if time.monotonic()-self.last_pair > .6:
                    self.invalidate('Awaiting RGB, aligned depth and CameraInfo')
                return
            pair = None
            for color in reversed(self.colors):
                if stamp_s(color) <= self.last_stamp:
                    break
                depth = min(self.depths, key=lambda value: abs(stamp_s(value)-stamp_s(color)))
                if abs(stamp_s(depth)-stamp_s(color)) <= self.camera['max_sync_skew_s']:
                    pair = color, depth
                    break
            if pair is None:
                if time.monotonic()-self.last_pair > self.camera['max_frame_age_s']:
                    self.invalidate('No fresh synchronized RGB-D pair')
                return
            color, depth = pair
            self.last_stamp = stamp_s(color)
            self.process(color, depth)
            self.last_pair = time.monotonic()
        except (ValueError, RuntimeError, cv2.error) as error:
            self.invalidate(str(error))

    def process(self, color, depth):
        info = self.info
        skew = validate_pair(metadata(color), metadata(depth), metadata(info),
                             self.get_clock().now().nanoseconds*1e-9, self.camera)
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
            raise ValueError('CameraInfo binning/ROI requires explicit calibration handling')
        frame = image_array(color)
        if color.encoding == 'rgb8': frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif color.encoding == 'rgba8': frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif color.encoding == 'bgra8': frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif color.encoding != 'bgr8': raise ValueError('Unsupported color format')
        raw_bgr = frame.copy()
        depth_m = image_array(depth)
        if depth.encoding not in ('16UC1', 'mono16', '32FC1'):
            raise ValueError('Aligned depth must be millimetre 16UC1 or metre 32FC1')
        intrinsic = dict(width=info.width, height=info.height, fx=info.k[0], fy=info.k[4],
                         cx=info.k[2], cy=info.k[5], distortion_model=info.distortion_model, d=list(info.d))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        stamp_ms = color.header.stamp.sec*1000 + color.header.stamp.nanosec//1000000
        hands = self.detector.detect(rgb, stamp_ms)
        self.tracker.update(hands, stamp_ms)
        self.counts['processed_frames'] += 1
        self.counts['detected_frames'] += bool(hands)
        observations = []
        poses = PoseArray(); poses.header = color.header
        for hand in hands:
            tracked = hand['palm_frame']; identity = tracked['track_id']
            if self.position_only:
                metric = estimate_palm_position(hand['landmarks_normalized'], depth_m, intrinsic, config=self.camera)
            else:
                metric = (estimate_palm_pose(hand['landmarks_normalized'], depth_m, intrinsic,
                          handedness=tracked['handedness_stable'], config=self.camera)
                          if tracked['valid'] else dict(valid=False, reason=tracked['status']))
            hand_msg = HandObservation(); hand_msg.header = color.header
            hand_msg.tracking_id = str(identity)
            hand_msg.handedness = (1 if tracked['handedness_stable']=='Left'
                                   else 2 if tracked['handedness_stable']=='Right' else 0)
            hand_msg.confidence = float(hand['handedness_score'])
            for target, point in zip(hand_msg.normalized_landmarks, hand['landmarks_normalized']):
                target.x, target.y, target.z = map(float, point)
            self.hands.publish(hand_msg)
            stable = self.stable(identity, metric, self.last_stamp)
            observation = dict(track_id=identity, handedness=tracked['handedness_stable'],
                               geometry_valid=metric['valid'], stable=stable, pose=metric,
                               position_only=self.position_only, orientation_valid=not self.position_only and metric['valid'],
                               robot_extrinsics_verified=self.camera_verified, torso_pose_available=False)
            observations.append(observation)
            palm = PalmObservation(); palm.header = color.header
            palm.tracking_id = str(identity); palm.handedness = hand_msg.handedness
            palm.valid = bool(metric['valid'] and stable and not self.position_only)
            palm.confidence = float(hand['handedness_score']) if palm.valid else 0.
            palm.rgb_depth_skew_s = float(skew)
            palm.message = metric['reason'] if not metric['valid'] else ('stable camera-frame pose' if stable else 'waiting for stable RGB-D poses')
            if metric['valid']:
                position, orientation = palm.pose.pose.position, palm.pose.pose.orientation
                position.x, position.y, position.z = metric['position_m']
                if not self.position_only:
                    orientation.x, orientation.y, orientation.z, orientation.w = metric['quaternion_xyzw']
                palm.valid_depth_points = metric['valid_depth_points']
                palm.plane_residual_m = metric.get('plane_residual_m', 0.)
                if self.position_only:
                    palm.message = 'Position only; robot fixed orientation not calibrated'
                    if stable:
                        point = PointStamped(); point.header = color.header
                        point.point = palm.pose.pose.position
                        self.positions.publish(point)
                self.draw(frame, metric, intrinsic, identity, stable)
                if palm.valid: poses.poses.append(palm.pose.pose)
            self.palms.publish(palm)
        active_ids = {hand['palm_frame']['track_id'] for hand in hands}
        self.gates = {key: value for key, value in self.gates.items() if key in active_ids}
        self.poses.publish(poses)
        geometry = any(row['geometry_valid'] for row in observations)
        stable = any(row['stable'] for row in observations)
        self.counts['geometry_frames'] += geometry; self.counts['stable_frames'] += stable
        payload = dict(stamp_s=self.last_stamp, frame_id=color.header.frame_id, intrinsics=intrinsic,
                       rgb_depth_skew_s=skew, palms=observations, hands=hands,
                       inference_delegate=self.detector.delegate, robot_motion_enabled=False)
        self.file.write(json.dumps(payload, allow_nan=False)+'\n')
        if self.record_rgbd:
            np.savez_compressed(self.output/'frames'/f"{self.counts['processed_frames']:06d}.npz",
                                color_bgr=raw_bgr, depth_m=depth_m,
                                metadata=json.dumps(payload, allow_nan=False))
        self.status.publish(String(data=json.dumps({k: v for k, v in payload.items() if k != 'hands'})))
        image = Image(); image.header = color.header; image.height, image.width = frame.shape[:2]
        image.encoding = 'bgr8'; image.step = image.width*3; image.data = frame.tobytes(); self.images.publish(image)
        if not hands:
            self.gates.clear()
            palm = PalmObservation(); palm.header = color.header; palm.valid = False
            palm.message = 'No hand detected'; self.palms.publish(palm)
        if time.monotonic()-self.last_save > 1.:
            self.last_save = time.monotonic()
            cv2.imwrite(str(self.output/'latest.jpg'), frame)
            np.savez_compressed(self.output/'rgbd_latest.npz', color_bgr=raw_bgr, depth_m=depth_m,
                                metadata=json.dumps(payload, allow_nan=False))
            if stable:
                cv2.imwrite(str(self.output/'valid.jpg'), frame)
                np.savez_compressed(self.output/'rgbd_valid.npz', color_bgr=raw_bgr, depth_m=depth_m,
                                    metadata=json.dumps(payload, allow_nan=False))
            self.get_logger().info(f'hands={len(hands)} geometry={geometry} stable={stable} frame={color.header.frame_id}')

    def stable(self, identity, metric, stamp):
        if not metric['valid']:
            self.gates.pop(identity, None); return False
        history = self.gates.setdefault(identity, deque(maxlen=self.camera.get('stable_frames', 3)))
        if history and stamp-history[-1][0] > .4: history.clear()
        history.append((stamp, np.array(metric['position_m']),
                        None if 'rotation' not in metric else np.array(metric['rotation'])))
        if len(history) < history.maxlen: return False
        return all(np.linalg.norm(point-history[-1][1]) <= self.camera.get('stable_position_m', .02)
                   and (rotation is None or rotation_distance(rotation, history[-1][2]) <= self.camera.get('stable_angle_deg', 15.))
                   for _, point, rotation in history)

    def draw(self, image, metric, intrinsic, identity, stable):
        if self.position_only:
            origin = np.array(metric['position_m']); wrist = np.array(metric['wrist_position_m'])
            def pixel(point):
                k = np.array([[intrinsic['fx'], 0, intrinsic['cx']], [0, intrinsic['fy'], intrinsic['cy']], [0, 0, 1.]])
                xyz = point.reshape(1, 1, 3)
                if intrinsic['distortion_model'] == 'equidistant':
                    uv, _ = cv2.fisheye.projectPoints(xyz, np.zeros(3), np.zeros(3), k, np.array(intrinsic['d']))
                else:
                    uv, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), k, np.array(intrinsic['d']) if intrinsic['d'] else None)
                return tuple(np.round(uv.reshape(2)).astype(int))
            cv2.arrowedLine(image, pixel(wrist), pixel(origin), (0, 220, 255), 3, tipLength=.2)
            cv2.circle(image, pixel(origin), 8, (255, 0, 255), 2)
            cv2.putText(image, f'#{identity} position only: {origin[0]:+.3f} {origin[1]:+.3f} {origin[2]:.3f} m',
                        (5, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 1)
            return
        rotation = np.array(metric['rotation']); origin = np.array(metric['position_m'])
        points = np.vstack((origin, origin+.04*rotation.T))
        matrix = np.array([[intrinsic['fx'], 0, intrinsic['cx']], [0, intrinsic['fy'], intrinsic['cy']], [0, 0, 1.]])
        dist = np.array(intrinsic['d']) if intrinsic['d'] else None
        if intrinsic['distortion_model']=='equidistant':
            pixels, _ = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), matrix, dist)
        else:
            pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), matrix, dist)
        pixels = np.round(pixels.reshape(-1, 2)).astype(int)
        for i, (name, color) in enumerate([('X', (0, 0, 255)), ('Y', (0, 210, 0)), ('Z', (255, 80, 0))], 1):
            cv2.arrowedLine(image, tuple(pixels[0]), tuple(pixels[i]), color, 3, tipLength=.2)
            cv2.putText(image, name, tuple(pixels[i]+[3, -3]), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
        text = f"#{identity} xyz(m) {origin[0]:+.3f} {origin[1]:+.3f} {origin[2]:.3f} {'stable' if stable else 'waiting'}"
        y = max(18, min(image.shape[0]-10, pixels[0, 1]+45))
        cv2.putText(image, text, (5, int(y)), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3)
        cv2.putText(image, text, (5, int(y)), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)

    def close(self):
        self.detector.close(); self.file.close()
        (self.output/'summary.json').write_text(json.dumps(dict(self.counts,
            output_frame='RGB optical frame', robot_extrinsics_verified=self.camera_verified,
            robot_motion_enabled=False, recorded_rgbd_sequence=self.record_rgbd,
            inference_delegate=self.detector.delegate), indent=2)+'\n')


def main():
    import yaml
    package = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0, help='Seconds; zero means until Ctrl+C')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--record-rgbd', action='store_true',
                        help='Save synchronized RGB, depth, calibration and landmarks for every processed frame')
    parser.add_argument('--config', type=Path, default=package/'config/handshake.yaml')
    args = parser.parse_args()
    output = args.output or package.parents[1]/'data/handshake/rgbd'/time.strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(args.config.read_text())
    rclpy.init()
    node = None
    deadline = time.monotonic()+args.duration if args.duration > 0 else float('inf')
    try:
        node = PalmRGBD(config, package, output, args.record_rgbd)
        while rclpy.ok() and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print('Results: ' + str(output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
