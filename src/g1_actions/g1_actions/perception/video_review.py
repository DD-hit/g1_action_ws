"""Review recorded RGB hand video without importing ROS or motion modules."""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import subprocess
import time

from .hand_landmarks import HandLandmarkDetector
from .palm_frame import PalmFrameTracker, draw_palm_frame, draw_palm_legend

CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


def main(argv=None):
    import cv2
    import imageio_ffmpeg
    import numpy as np
    import yaml

    package = Path(__file__).resolve().parents[2]
    workspace = package.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('--config', type=Path, default=package / 'config/handshake.yaml')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--delegate', choices=('auto', 'cpu', 'gpu'), help='Override handshake.yaml for this run')
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text())['perception']
    palm_config = config.get('palm_frame', {})
    palm_tracker = PalmFrameTracker(palm_config) if palm_config.get('enabled', False) else None
    if args.delegate is not None:
        config['delegate'] = args.delegate
    model = package / config['model_path']
    if not model.is_file():
        parser.error('Model missing; run bash scripts/setup_handshake_video.sh')
    video = args.video.expanduser().resolve()
    if not video.is_file():
        parser.error('Video does not exist: ' + str(video))
    output = args.output or workspace / 'data/handshake/reviews' / (
        video.stem + '_' + time.strftime('%Y%m%d_%H%M%S'))
    if output.exists() and any(output.iterdir()):
        parser.error('Output directory must be empty: ' + str(output))
    output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 1)
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    source_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    orientation = float(capture.get(cv2.CAP_PROP_ORIENTATION_META))
    if not capture.isOpened() or not math.isfinite(fps) or fps <= 0:
        capture.release()
        raise RuntimeError('Unable to decode video or determine frame rate')
    max_side = int(config['max_image_side'])
    max_fps = float(config['max_processing_fps'])
    if max_side < 64 or not math.isfinite(max_fps) or max_fps <= 0:
        capture.release()
        raise ValueError('Invalid image size or processing frame rate')
    stride = max(1, math.ceil(fps / max_fps))
    output_fps = fps / stride
    detector = None
    writer = None
    decoded = processed = detected = multiple = flips = 0
    missing = longest_missing = 0
    latency_ms = []
    timing = dict(decode_s=0., resize_rgb_s=0., inference_s=0.,
                  annotation_json_s=0., encode_wait_s=0., model_creation_s=0.)
    labels = {}
    previous_label = None
    previews = []
    started = time.perf_counter()
    width = height = 0
    try:
        init_started = time.perf_counter()
        detector = HandLandmarkDetector(model, config)
        timing['model_creation_s'] = time.perf_counter() - init_started
        with (output / 'landmarks.jsonl').open('w') as records, (output / 'encoder.log').open('w') as encoder_log:
            while True:
                decode_started = time.perf_counter()
                ok = capture.grab()
                timing['decode_s'] += time.perf_counter() - decode_started
                if not ok:
                    break
                index = decoded
                decoded += 1
                if index % stride:
                    continue
                decode_started = time.perf_counter()
                ok, frame = capture.retrieve()
                timing['decode_s'] += time.perf_counter() - decode_started
                if not ok:
                    raise RuntimeError('Unable to retrieve decoded frame ' + str(index))
                if not processed:
                    original_height, original_width = frame.shape[:2]
                    scale = min(1., max_side / max(original_width, original_height))
                    width = max(2, int(original_width * scale) // 2 * 2)
                    height = max(2, int(original_height * scale) // 2 * 2)
                    writer = subprocess.Popen([
                        imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error',
                        '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}',
                        '-r', str(output_fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                        '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', str(output / 'annotated.mp4'),
                    ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=encoder_log)
                resize_started = time.perf_counter()
                frame = cv2.resize(frame, (width, height))
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                timing['resize_rgb_s'] += time.perf_counter() - resize_started
                source_timestamp_ms = float(capture.get(cv2.CAP_PROP_POS_MSEC))
                if not math.isfinite(source_timestamp_ms) or source_timestamp_ms < 0:
                    raise RuntimeError('Decoder returned an invalid source timestamp')
                timestamp_ms = round(source_timestamp_ms)
                inference_start = time.perf_counter()
                hands = detector.detect(rgb, timestamp_ms)
                inference_ms = (time.perf_counter() - inference_start) * 1000
                latency_ms.append(inference_ms)
                timing['inference_s'] += inference_ms / 1000
                annotation_started = time.perf_counter()
                if palm_tracker is not None:
                    palm_tracker.update(hands, timestamp_ms)
                processed += 1
                detected += bool(hands)
                multiple += len(hands) > 1
                missing = 0 if hands else missing + 1
                longest_missing = max(longest_missing, missing)
                label = hands[0]['handedness_raw'] if len(hands) == 1 else None
                flips += int(label is not None and previous_label is not None and label != previous_label)
                previous_label = label
                for hand in hands:
                    labels[hand['handedness_raw']] = labels.get(hand['handedness_raw'], 0) + 1
                    points = [(round(p[0] * width), round(p[1] * height)) for p in hand['landmarks_normalized']]
                    for a, b in CONNECTIONS:
                        cv2.line(frame, points[a], points[b], (80, 220, 80), 2, cv2.LINE_AA)
                    for point in points:
                        cv2.circle(frame, point, 4, (40, 70, 255), -1, cv2.LINE_AA)
                    text = f"{hand['handedness_raw']} {hand['handedness_score']:.2f}"
                    x = max(0, min(p[0] for p in points))
                    y = max(50, min(p[1] for p in points) - 10)
                    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2, cv2.LINE_AA)
                    draw_palm_frame(frame, hand, palm_config)
                cv2.rectangle(frame, (0, 0), (width, 32), (25, 25, 25), -1)
                cv2.putText(frame, f'{timestamp_ms/1000:.2f}s | hands={len(hands)} | {inference_ms:.1f}ms',
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
                if palm_tracker is not None:
                    draw_palm_legend(frame)
                record = dict(source_frame=index, timestamp_ms=timestamp_ms, inference_ms=inference_ms,
                              image_width=width, image_height=height, hands=hands)
                records.write(json.dumps(record, allow_nan=False) + '\n')
                if len(previews) < 8 and index >= len(previews) * max(1, source_count - 1) / 8:
                    previews.append(frame.copy())
                timing['annotation_json_s'] += time.perf_counter() - annotation_started
                encode_started = time.perf_counter()
                writer.stdin.write(frame.tobytes())
                timing['encode_wait_s'] += time.perf_counter() - encode_started
                if processed % 60 == 0:
                    print(f'Processed {processed} frames; detected {detected}', flush=True)
            if not processed:
                raise RuntimeError('No frames decoded')
            if source_count > 0 and decoded < source_count - 1:
                raise RuntimeError(f'Video truncated: decoded {decoded}/{source_count} frames')
            encode_started = time.perf_counter()
            writer.stdin.close()
            if writer.wait(timeout=60):
                raise RuntimeError('Video encoding failed; see encoder.log')
            timing['encode_wait_s'] += time.perf_counter() - encode_started
    finally:
        capture.release()
        if detector is not None:
            detector.close()
        if writer is not None and writer.poll() is None:
            if not writer.stdin.closed:
                writer.stdin.close()
            try:
                writer.wait(timeout=10)
            except subprocess.TimeoutExpired:
                writer.kill()
                writer.wait()
    summary = dict(
        input_file=str(video), source_fps=fps, source_frames=decoded,
        timestamp_source='decoder CAP_PROP_POS_MSEC',
        source_duration_s=decoded/fps, orientation_metadata_degrees=orientation,
        inference_resolution=[width, height], frame_stride=stride,
        reviewed_frames=processed, detected_frames=detected,
        detection_fraction=detected/processed, multiple_hand_frames=multiple,
        longest_missing_s=longest_missing/output_fps, handedness_raw_counts=labels,
        consecutive_single_hand_label_flips=flips,
        inference_median_ms=float(np.median(latency_ms)),
        inference_p95_ms=float(np.percentile(latency_ms, 95)),
        inference_delegate=detector.delegate, stage_timing_s=timing,
        total_wall_time_s=time.perf_counter()-started,
        versions={name: importlib.metadata.version(name) for name in ['mediapipe', 'numpy']},
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        coordinate_contract={
            'landmarks_normalized': 'image-relative x/y; z is model-relative depth',
            'landmarks_model_world': 'model-estimated local hand coordinates; NOT camera or torso position',
            'handedness_raw': 'unmodified MediaPipe label; anatomical convention not yet verified',
            'palm_frame': 'model-estimated palm axes, X=Y cross Z; not a calibrated camera or torso pose',
        },
        metric_palm_pose_available=False, robot_motion_enabled=False,
        palm_frame_stats=(None if palm_tracker is None else palm_tracker.summary()),
    )
    check = cv2.VideoCapture(str(output / 'annotated.mp4'))
    encoded_count = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
    check.release()
    if encoded_count != processed:
        raise RuntimeError(f'Encoded frame count mismatch: {encoded_count}/{processed}')
    summary['encoded_frames'] = encoded_count
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    thumbnails = [cv2.resize(image, (width//2, height//2)) for image in previews]
    cv2.imwrite(str(output / 'preview.jpg'), np.concatenate(thumbnails, axis=1))
    print(json.dumps(summary, indent=2))
    print('Results: ' + str(output.resolve()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
