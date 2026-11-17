import argparse
from collections import deque
import json
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from .config import load, resource, camera_signature
from .robot_model import Robot
from .geometry import StableTargets, localize, transform_point
from .inference import Detector


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class Vision(Node):
    def __init__(self, config):
        super().__init__('ele_elevator_vision')
        self.cfg, self.bridge = config, CvBridge()
        self.camera_signature = camera_signature(config)
        self.model = Robot(resource(config, config['model_file']))
        self.camera_t = self.model.camera(config)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detector = Detector(config)
        options = config['detector']
        self.stable = StableTargets(options['stable_frames'], options['stable_jitter_px'])
        self.lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.inference_paused = threading.Event()
        self.color, self.info = None, None
        self.depths = deque(maxlen=12)
        self.colors = deque(maxlen=12)
        self.last_processed = None
        self.last_receive = time.monotonic()
        self.last_depth_receive = time.monotonic()
        self.last_pair_time = time.monotonic()
        self.last_health_warning = ('', 0.)
        self.sequence = 0
        self.stopping = threading.Event()
        self.targets_pub = self.create_publisher(String, '/elevator/targets', 1)
        self.det_pub = self.create_publisher(String, '/elevator_buttons/detections', 1)
        self.image_pub = self.create_publisher(Image, '/elevator_buttons/image_annotated', 1)
        self.poses_pub = self.create_publisher(PoseArray, '/elevator/target_poses', 1)
        camera = config['camera']
        reliability = {
            'reliable': ReliabilityPolicy.RELIABLE,
            'best_effort': ReliabilityPolicy.BEST_EFFORT,
        }[camera.get('image_reliability', 'reliable')]
        # RealSense offers RELIABLE. Retry lost DDS fragments, but keep only the
        # newest complete image instead of processing a backlog after a stall.
        image_qos = QoSProfile(depth=1, reliability=reliability)
        self.create_subscription(Image, camera['color_topic'], self.on_color, image_qos)
        self.create_subscription(Image, camera['depth_topic'], self.on_depth, image_qos)
        self.create_subscription(CameraInfo, camera['info_topic'], self.on_info, image_qos)
        self.create_service(Trigger,'/elevator/vision/freeze',self.freeze)
        self.create_service(Trigger,'/elevator/vision/resume',self.resume)
        self.health_pub=self.create_publisher(String,'/elevator/camera_health',1)
        self.create_timer(.2,self.publish_camera_health)
        self.worker = threading.Thread(target=self.run_worker, daemon=True)
        self.worker.start()
        self.get_logger().info(f"Backend={options['backend']}; awaiting synchronized RGB, aligned depth and CameraInfo")

    def on_color(self, msg):
        with self.lock:
            self.color = msg
            self.colors.append(msg)
            self.last_receive = time.monotonic()

    def freeze(self,request,response):
        with self.processing_lock:
            self.inference_paused.set()
        response.success=True
        response.message='Inference paused; the active task owns its fixed target.'
        return response

    def resume(self,request,response):
        with self.processing_lock:
            self.stable.history.clear()
            # Resume is a freshness barrier, not permission to replay buffered
            # images. Also excludes a pair selected before this RPC took the lock.
            with self.lock:
                latest = None if self.color is None else (
                    self.color.header.stamp.sec, self.color.header.stamp.nanosec)
                if latest is not None and (self.last_processed is None or latest > self.last_processed):
                    self.last_processed = latest
                self.colors.clear()
                self.depths.clear()
            self.last_pair_time=time.monotonic()
            self.inference_paused.clear()
        response.success=True
        response.message='Inference resumed; collecting a fresh stable target.'
        return response

    def on_depth(self, msg):
        with self.lock:
            self.depths.append(msg)
            self.last_depth_receive = time.monotonic()

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def camera_health(self):
        with self.lock:
            now, wall_now = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
            return {
                'color_receive_age_s': now-self.last_receive,
                'depth_receive_age_s': now-self.last_depth_receive,
                'color_stamp_age_s': None if self.color is None else wall_now-seconds(self.color.header.stamp),
                'depth_stamp_age_s': None if not self.depths else wall_now-seconds(self.depths[-1].header.stamp),
                'has_camera_info': self.info is not None,
            }

    def publish_camera_health(self):
        health=self.camera_health()
        health['inference_paused']=self.inference_paused.is_set()
        self.health_pub.publish(String(data=json.dumps(health)))

    def publish_empty(self, reason):
        health = self.camera_health()
        self.targets_pub.publish(String(data=json.dumps({'targets': [], 'reason': reason,
            'stamp_sec': self.get_clock().now().nanoseconds*1e-9,
            'frame_id': self.cfg['base_frame'], 'camera_verified': self.cfg['camera']['verified'],
            'camera_health': health})))
        previous, when = self.last_health_warning
        now = time.monotonic()
        if reason != previous or now-when >= 5.:
            self.get_logger().warning(reason+'; camera_health='+json.dumps(health))
            self.last_health_warning = (reason, now)

    def run_worker(self):
        period = 1/self.cfg['detector']['max_rate']
        while rclpy.ok() and not self.stopping.is_set():
            start = time.monotonic()
            if self.inference_paused.is_set():
                self.stopping.wait(period)
                continue
            try:
                with self.lock:
                    color, depth, info = None, None, self.info
                    # RGB and depth callbacks can arrive in either order. Wait
                    # for a matched pair rather than clearing stable detections
                    # when a new color arrives before its aligned depth.
                    for candidate in reversed(self.colors):
                        tag=(candidate.header.stamp.sec,candidate.header.stamp.nanosec)
                        if self.last_processed is not None and tag<=self.last_processed:
                            break
                        if self.depths:
                            paired=min(self.depths,key=lambda x:abs(seconds(x.header.stamp)-seconds(candidate.header.stamp)))
                            if abs(seconds(paired.header.stamp)-seconds(candidate.header.stamp))<=self.cfg['camera']['max_sync_skew']:
                                color,depth=candidate,paired
                                break
                if self.camera_health()['color_receive_age_s'] > self.cfg['camera']['max_frame_age']:
                    self.stable.update([])
                    self.publish_empty('camera frames stopped')
                elif color is None or info is None or depth is None:
                    if time.monotonic()-self.last_pair_time > self.cfg['camera']['max_frame_age']:
                        self.stable.update([])
                        self.publish_empty('awaiting synchronized RGB-D/camera intrinsics')
                elif (color.header.stamp.sec, color.header.stamp.nanosec) != self.last_processed:
                    with self.processing_lock:
                        tag = (color.header.stamp.sec, color.header.stamp.nanosec)
                        if (not self.inference_paused.is_set() and
                                (self.last_processed is None or tag > self.last_processed)):
                            self.process(color, depth, info)
                            self.last_processed = (color.header.stamp.sec, color.header.stamp.nanosec)
                            self.last_pair_time = time.monotonic()
                            self.last_health_warning = ('', 0.)
            except Exception as error:
                if not rclpy.ok() or self.stopping.is_set():
                    break
                self.stable.update([])
                self.publish_empty(str(error))
            self.stopping.wait(max(0, period-(time.monotonic()-start)))

    def process(self, color, depth, info):
        camera = self.cfg['camera']
        stamp = seconds(color.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if abs(now-stamp) > camera['max_frame_age']:
            raise ValueError('RGB frame timestamp is stale or clocks are unsynchronized')
        if abs(seconds(depth.header.stamp)-stamp) > camera['max_sync_skew']:
            raise ValueError('RGB and depth timestamps do not match')
        if info.header.frame_id != color.header.frame_id or depth.header.frame_id != color.header.frame_id:
            raise ValueError('Color, aligned depth and intrinsics must use the same optical frame')
        if camera.get('use_camera_tf') and camera['measured_xyz'] is None:
            # Preserve RealSense factory color-imager extrinsics, not just an axis rotation.
            sensor = self.tf_buffer.lookup_transform(camera['camera_link_frame'], color.header.frame_id,
                                                     rclpy.time.Time())
            t = sensor.transform
            body_to_optical = np.eye(4)
            body_to_optical[:3,:3] = Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
            body_to_optical[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
            self.camera_t = self.model.relative(self.cfg['base_frame'],camera['mount_link'])@body_to_optical
        frame = self.bridge.imgmsg_to_cv2(color, 'bgr8')
        depth_m = self.bridge.imgmsg_to_cv2(depth, 'passthrough').astype(np.float32)
        if depth.encoding in ('16UC1', 'mono16'):
            depth_m *= 0.001
        elif depth.encoding != '32FC1':
            raise ValueError('Expected depth in millimetre 16UC1 or metre 32FC1')
        if depth_m.shape != frame.shape[:2] or info.width != frame.shape[1] or info.height != frame.shape[0]:
            raise ValueError('Depth must be aligned to RGB at matching resolution')
        detections, inference_ms = self.detector.predict(frame)
        stable = self.stable.update(detections)
        targets = []
        depth_errors = {}
        for detection in stable:
            try:
                point, normal, quality = localize(depth_m, detection['xyxy'], detection['center_xy'],
                    (info.k[0], info.k[4], info.k[2], info.k[5]), self.cfg['depth'],
                    (info.distortion_model,list(info.d)))
                if quality['plane_residual_m'] > 0.003:
                    raise ValueError('plane uncertainty exceeds 3 mm')
                targets.append(dict(detection, point=transform_point(self.camera_t, point).tolist(),
                    normal=(self.camera_t[:3, :3]@normal).tolist(), camera_point=point.tolist(),
                    quality=quality, stamp_sec=stamp, frame_id=self.cfg['base_frame']))
            except ValueError as error:
                detection['depth_rejection'] = str(error)
                depth_errors[detection['direction']] = str(error)
                # StableTargets returns copies; expose rejection on the raw
                # detection too, so observers can see why its box is not usable.
                for raw in detections:
                    if raw['direction'] == detection['direction']:
                        raw['depth_rejection'] = str(error)
        target_status = {}
        for direction in ('up', 'down'):
            found = [d for d in detections if d['direction'] == direction]
            tracked = [d for d in stable if d['direction'] == direction]
            usable = [d for d in targets if d['direction'] == direction]
            if not found:
                reason = '未检测到该方向按钮'
            elif len(found) != 1:
                reason = '检测到多个同方向按钮，目标不唯一'
            elif not tracked:
                reason = '等待按钮连续稳定检测'
            elif not usable:
                reason = depth_errors.get(direction, '按钮深度校验未通过')
            else:
                reason = ''
            target_status[direction] = {
                'ready': len(usable) == 1,
                'reason': reason,
                'detection_count': len(found),
                'stable_frames': tracked[0]['stable_frames'] if tracked else 0,
            }
        self.sequence += 1
        result = {'schema_version': 1, 'sequence': self.sequence, 'stamp_sec': stamp,
                  'frame_id': self.cfg['base_frame'], 'source_frame': color.header.frame_id,
                  'image_width': color.width, 'image_height': color.height,
                  'camera_verified': camera['verified'], 'inference_ms': inference_ms,
                  'camera_calibration_id': self.camera_signature,
                  'detections': detections, 'targets': targets,
                  'target_status': target_status,
                  'camera_health': self.camera_health()}
        self.targets_pub.publish(String(data=json.dumps(result)))
        self.det_pub.publish(String(data=json.dumps({'stamp_sec': stamp, 'frame_id': color.header.frame_id,
            'image_width': frame.shape[1], 'image_height': frame.shape[0], 'detections': detections})))
        poses = PoseArray()
        poses.header.stamp, poses.header.frame_id = color.header.stamp, self.cfg['base_frame']
        for target in targets:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = target['point']
            n = np.array(target['normal'])
            x = -n/np.linalg.norm(n)
            up = np.array([0.,0.,1.])
            z = up-x*(up@x)
            if np.linalg.norm(z) < .1:
                continue
            z /= np.linalg.norm(z)
            q = Rotation.from_matrix(np.column_stack((x,np.cross(z,x),z))).as_quat()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            poses.poses.append(pose)
        self.poses_pub.publish(poses)
        if self.image_pub.get_subscription_count():
            annotated = frame.copy()
            for d in detections:
                a,b,c,e = map(int,d['xyxy'])
                color_value = (0,220,0) if d['direction']=='up' else (0,180,255)
                cv2.rectangle(annotated,(a,b),(c,e),color_value,2)
                cv2.putText(annotated,f"{d['direction']} {d['confidence']:.2f}",(a,max(20,b-5)),
                            cv2.FONT_HERSHEY_SIMPLEX,.6,color_value,2)
            out = self.bridge.cv2_to_imgmsg(annotated,'bgr8')
            out.header = color.header
            self.image_pub.publish(out)

    def close(self):
        self.stopping.set()
        self.worker.join(timeout=30)
        if self.worker.is_alive():
            raise RuntimeError('Inference thread did not stop')
        self.detector.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Vision(load(args.config))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
