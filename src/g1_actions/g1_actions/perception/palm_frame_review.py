"""Replay a landmark review to inspect palm axes without repeating inference."""
import argparse
import json
from pathlib import Path
import subprocess
import time

from .palm_frame import PalmFrameTracker, draw_palm_frame, draw_palm_legend


def main(argv=None):
    import cv2
    import imageio_ffmpeg
    import numpy as np
    import yaml

    package = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('review', type=Path, help='Folder containing annotated.mp4 and landmarks.jsonl')
    parser.add_argument('--config', type=Path, default=package / 'config/handshake.yaml')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    folder = args.review.expanduser().resolve()
    output = args.output or folder.parent / (folder.name + '_palm_axes')
    if output.exists() and any(output.iterdir()):
        parser.error('Output directory must be empty: ' + str(output))
    records = [json.loads(line) for line in (folder / 'landmarks.jsonl').read_text().splitlines()]
    if not records:
        parser.error('No source landmark records')
    config = yaml.safe_load(args.config.read_text())['perception']['palm_frame']
    tracker = PalmFrameTracker(config)
    cap = cv2.VideoCapture(str(folder / 'annotated.mp4'))
    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened() or int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) != len(records):
        cap.release()
        raise RuntimeError('Encoded source frame count does not match landmark records')
    output.mkdir(parents=True, exist_ok=True)
    previews = []
    started = time.perf_counter()
    with (output / 'encoder.log').open('w') as log:
        writer = subprocess.Popen([
            imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}',
            '-r', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
            '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(output / 'palm_axes.mp4'),
        ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
        try:
            with (output / 'palm_frames.jsonl').open('w') as rows:
                for index, record in enumerate(records):
                    ok, image = cap.read()
                    if not ok:
                        raise RuntimeError('Source video truncated at frame ' + str(index))
                    tracker.update(record['hands'], record['timestamp_ms'])
                    for hand in record['hands']:
                        draw_palm_frame(image, hand, config)
                    draw_palm_legend(image)
                    writer.stdin.write(image.tobytes())
                    rows.write(json.dumps(record, allow_nan=False) + '\n')
                    if len(previews) < 8 and index >= len(previews) * len(records) / 8:
                        previews.append(cv2.resize(image, (width//2, height//2)))
                if cap.read()[0]:
                    raise RuntimeError('Source video has more frames than landmark records')
            writer.stdin.close()
            if writer.wait(timeout=60):
                raise RuntimeError('Axis video encoding failed; see encoder.log')
        finally:
            cap.release()
            if writer.poll() is None:
                if not writer.stdin.closed:
                    writer.stdin.close()
                try:
                    writer.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    writer.kill()
                    writer.wait()
    check = cv2.VideoCapture(str(output / 'palm_axes.mp4'))
    count = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
    check.release()
    if count != len(records):
        raise RuntimeError('Axis video output frame count mismatch')
    summary = dict(mode='cached_landmarks_review', source_review=str(folder),
        processed_frames=count, palm_frame_stats=tracker.summary(),
        total_wall_time_s=time.perf_counter()-started,
        axes=dict(z='outward from palm', y='wrist toward finger bases', x='y cross z'),
        projection='orthographic image overlay of model-estimated axes; not camera calibration',
        metric_camera_pose_available=False, robot_motion_enabled=False)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    cv2.imwrite(str(output / 'preview.jpg'), np.concatenate(previews, axis=1))
    print(json.dumps(summary, indent=2))
    print('Results: ' + str(output.resolve()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
