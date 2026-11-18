"""Real image conversion/publishing path; fake detector, no camera or GPU."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch,MagicMock
import numpy as np
import yaml
import rclpy
from sensor_msgs.msg import Image,CameraInfo
from g1_actions.perception import rgbd_live as module

PACKAGE=Path(__file__).resolve().parents[1]


class ProductionPerceptionTests(unittest.TestCase):
    def process_case(self,production,stale=False):
        with tempfile.TemporaryDirectory() as folder:
            rclpy.init();detector=MagicMock();detector.delegate='test';detector.detect.return_value=[]
            with patch.object(module,'HandLandmarkDetector',return_value=detector):
                node=module.PalmRGBD(yaml.safe_load((PACKAGE/'config/handshake.yaml').read_text()),PACKAGE,Path(folder),production=production)
            try:
                color=Image();depth=Image();info=CameraInfo()
                stamp=node.get_clock().now().to_msg()
                if stale:stamp.sec-=2
                for msg in (color,depth,info):
                    msg.header.stamp=stamp;msg.header.frame_id='camera_color_optical_frame'
                    msg.width=640;msg.height=480
                color.encoding='bgr8';color.step=640*3;color.data=np.zeros((480,640,3),np.uint8).tobytes()
                depth.encoding='32FC1';depth.step=640*4;depth.data=np.ones((480,640),np.float32).tobytes()
                info.k=[600.,0.,320.,0.,600.,240.,0.,0.,1.];info.distortion_model='plumb_bob';info.d=[0.]*5
                node.on_info(info)
                with patch.object(module.cv2,'imwrite') as jpg,patch.object(module.np,'savez_compressed') as archive:
                    if stale:
                        with self.assertRaises(ValueError):node.process(color,depth)
                        self.assertEqual(node.counts['processed_frames'],0)
                    else:
                        node.last_stamp=module.stamp_s(color);node.process(color,depth)
                        self.assertEqual(node.counts['processed_frames'],1)
                    if production:jpg.assert_not_called();archive.assert_not_called();self.assertIsNone(node.file)
                    elif not stale:self.assertTrue(jpg.called and archive.called)
            finally:node.close();node.destroy_node();rclpy.shutdown()

    def test_production_does_not_block_on_diagnostic_archives(self):self.process_case(True)
    def test_debug_mode_keeps_existing_artifacts(self):self.process_case(False)
    def test_production_preserves_stale_image_rejection(self):self.process_case(True,stale=True)


if __name__=='__main__':unittest.main()
