"""Synthetic ROS images and a fake detector; no device or robot interfaces."""
import os
os.environ.update(ROS_DOMAIN_ID='196', ROS_LOCALHOST_ONLY='1',
                  RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
import json
import time
import unittest
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from g1_actions.actions.elevator.config import load
from g1_actions.actions.elevator.vision_node import Vision


class FakeDetector:
    def predict(self, frame):
        return [], 1.

    def close(self):
        pass


class VisionHealthTests(unittest.TestCase):
    def test_raw_health_continues_while_inference_is_paused(self):
        from std_srvs.srv import Trigger
        health=[]
        self.sender.create_subscription(String,'/elevator/camera_health',lambda m:health.append(json.loads(m.data)),10)
        self.stream(1.)
        identity=id(self.vision)
        self.vision.freeze(None,Trigger.Response());self.spin(.2)
        before=len([r for r in self.results if 'sequence' in r])
        self.stream(3.5)
        self.assertEqual(before,len([r for r in self.results if 'sequence' in r]))
        paused=[h for h in health if h['inference_paused']]
        self.assertGreater(len(paused),10)
        self.assertLess(paused[-1]['color_receive_age_s'],.6)
        self.assertLess(paused[-1]['depth_receive_age_s'],.6)
        self.vision.resume(None,Trigger.Response());self.stream(1.)
        self.assertEqual(id(self.vision),identity)
        self.assertGreater(len([r for r in self.results if 'sequence' in r]),before)

    def setUp(self):
        rclpy.init()
        cfg=load()
        cfg['camera']['use_camera_tf']=False
        with patch('g1_actions.actions.elevator.vision_node.Detector',return_value=FakeDetector()):
            self.vision=Vision(cfg)
        self.sender=Node('synthetic_camera_health_test')
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.vision);self.executor.add_node(self.sender)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.color=self.sender.create_publisher(Image,cfg['camera']['color_topic'],qos)
        self.depth=self.sender.create_publisher(Image,cfg['camera']['depth_topic'],qos)
        self.info=self.sender.create_publisher(CameraInfo,cfg['camera']['info_topic'],qos)
        self.results=[]
        self.sender.create_subscription(String,'/elevator/targets',
            lambda m:self.results.append(json.loads(m.data)),10)
        self.bridge=CvBridge()

    def tearDown(self):
        self.vision.close()
        self.executor.remove_node(self.vision);self.executor.remove_node(self.sender)
        self.executor.shutdown()
        self.vision.destroy_node();self.sender.destroy_node();rclpy.shutdown()

    def spin(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:self.executor.spin_once(timeout_sec=.01)

    def stream(self,seconds,age=0.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            stamp=rclpy.time.Time(nanoseconds=int((time.time()-age)*1e9)).to_msg()
            color=self.bridge.cv2_to_imgmsg(np.zeros((16,16,3),np.uint8),'bgr8')
            color.header.stamp=stamp;color.header.frame_id='test_optical_frame'
            depth=self.bridge.cv2_to_imgmsg(np.full((16,16),400,np.uint16),'16UC1')
            depth.header=color.header
            info=CameraInfo();info.header=color.header;info.width=16;info.height=16
            info.k=[20.,0.,8.,0.,20.,8.,0.,0.,1.]
            self.color.publish(color);self.depth.publish(depth);self.info.publish(info)
            self.spin(.1)

    def test_fresh_stream_stop_and_recovery_report_receive_ages(self):
        self.stream(2.)
        valid=[r for r in self.results if 'sequence' in r]
        self.assertTrue(valid)
        self.assertLess(valid[-1]['camera_health']['color_receive_age_s'],.6)
        self.results.clear();self.spin(1.)
        stopped=[r for r in self.results if r.get('reason')=='camera frames stopped']
        self.assertTrue(stopped)
        self.assertGreater(stopped[-1]['camera_health']['color_receive_age_s'],.6)
        self.assertGreater(stopped[-1]['camera_health']['depth_receive_age_s'],.6)
        self.results.clear();self.stream(.7)
        self.assertTrue(any('sequence' in r for r in self.results))

    def test_recent_arrival_with_old_capture_timestamp_is_not_validated(self):
        self.stream(2.,age=2.)
        self.assertFalse(any('sequence' in r for r in self.results))
        stale=[r for r in self.results if 'stale' in r.get('reason','')]
        self.assertTrue(stale)
        health=stale[-1]['camera_health']
        self.assertLess(health['color_receive_age_s'],.6)
        self.assertGreater(health['color_stamp_age_s'],1.5)

    def test_depth_rejection_is_exposed_without_creating_a_target(self):
        self.vision.detector.predict=lambda frame: ([{
            'direction':'down','class_id':265,'confidence':.9,
            'xyxy':[4.,4.,12.,12.],'center_xy':[8.,8.],
        }],1.)
        with patch('g1_actions.actions.elevator.vision_node.localize',
                   side_effect=ValueError('button depth inconsistent with panel')):
            self.stream(2.)
        candidates=[r for r in self.results
                    if r.get('target_status',{}).get('down',{}).get('stable_frames',0)>=3]
        self.assertTrue(candidates)
        result=candidates[-1]
        self.assertEqual(result['targets'],[])
        self.assertFalse(result['target_status']['down']['ready'])
        self.assertEqual(result['target_status']['down']['reason'],
                         'button depth inconsistent with panel')
        self.assertEqual(result['detections'][0]['depth_rejection'],
                         'button depth inconsistent with panel')


    def test_resume_does_not_replay_previously_published_or_buffered_frames(self):
        from std_srvs.srv import Trigger
        self.stream(1.)
        valid=[r for r in self.results if 'sequence' in r]
        self.assertTrue(valid)
        before=valid[-1]['stamp_sec']
        self.vision.freeze(None,Trigger.Response())
        # Camera continues while inference is frozen: those queued images
        # must not be admitted as fresh frames after resume.
        self.stream(.3)
        boundary=self.vision.color.header.stamp
        cutoff=boundary.sec+boundary.nanosec*1e-9
        self.results.clear()
        self.vision.resume(None,Trigger.Response())
        self.spin(.2)
        self.assertFalse(any('sequence' in r for r in self.results))
        self.stream(.8)
        for _ in range(4):
            self.vision.resume(None,Trigger.Response())
            self.stream(.3)
        valid=[r for r in self.results if 'sequence' in r]
        stamps=[r['stamp_sec'] for r in valid]
        self.assertTrue(stamps)
        self.assertTrue(all(stamp>max(before,cutoff) for stamp in stamps))
        self.assertTrue(all(b>a for a,b in zip(stamps,stamps[1:])),stamps)


if __name__=='__main__':
    unittest.main(verbosity=2)
