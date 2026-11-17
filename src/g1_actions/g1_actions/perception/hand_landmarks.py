"""MediaPipe VIDEO adapter. Model coordinates are not RGB-D measurements."""
from pathlib import Path
import platform


class HandLandmarkDetector:
    def __init__(self, model_path, config):
        import mediapipe as mp
        from mediapipe.tasks.python import vision

        self.mp = mp
        self.last_timestamp_ms = -1
        self.delegate = str(config.get('delegate', 'auto')).lower()
        if self.delegate == 'auto':
            jetson_gpu = (platform.system() == 'Linux'
                          and platform.machine() in ('aarch64', 'arm64')
                          and Path('/sys/devices/gpu.0/load').is_file())
            self.delegate = 'gpu' if jetson_gpu else 'cpu'
        if self.delegate not in ('cpu', 'gpu'):
            raise ValueError('perception.delegate must be auto, cpu or gpu')
        delegate = (mp.tasks.BaseOptions.Delegate.GPU if self.delegate == 'gpu'
                    else mp.tasks.BaseOptions.Delegate.CPU)
        options = vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path), delegate=delegate),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=int(config['num_hands']),
            min_hand_detection_confidence=float(config['min_detection_confidence']),
            min_hand_presence_confidence=float(config['min_presence_confidence']),
            min_tracking_confidence=float(config['min_tracking_confidence']),
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

    def detect(self, rgb_image, timestamp_ms):
        if timestamp_ms <= self.last_timestamp_ms:
            raise ValueError('VIDEO timestamps must be strictly increasing')
        self.last_timestamp_ms = timestamp_ms
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb_image)
        result = self.detector.detect_for_video(image, timestamp_ms)
        hands = []
        for landmarks, world, categories in zip(
            result.hand_landmarks, result.hand_world_landmarks, result.handedness
        ):
            category = max(categories, key=lambda value: value.score)
            hands.append({
                'handedness_raw': category.category_name,
                'handedness_score': float(category.score),
                'landmarks_normalized': [[p.x, p.y, p.z] for p in landmarks],
                'landmarks_model_world': [[p.x, p.y, p.z] for p in world],
            })
        return hands

    def close(self):
        self.detector.close()
