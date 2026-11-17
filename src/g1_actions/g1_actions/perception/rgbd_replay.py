"""Replay saved RGB-D geometry without ROS, a camera or any motion interface."""
import argparse
from collections import Counter
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .palm_pose import estimate_palm_pose, estimate_palm_position


def main():
    package = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='One .npz snapshot or a recorded RGB-D output directory')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=package/'config/handshake.yaml')
    args = parser.parse_args()
    if args.input.is_file():
        files = [args.input]
    elif (args.input/'frames').is_dir():
        files = sorted((args.input/'frames').glob('*.npz'))
    else:
        files = sorted(args.input.glob('rgbd_*.npz'))
    if not files:
        parser.error('No saved RGB-D frames found; an ordinary color video contains no measured depth')
    config = yaml.safe_load(args.config.read_text())['perception']['rgbd']
    args.output.mkdir(parents=True, exist_ok=False)
    counts = Counter()
    with (args.output/'replayed.jsonl').open('w') as stream:
        for source in files:
            with np.load(source, allow_pickle=False) as saved:
                metadata = json.loads(str(saved['metadata']))
                image, depth = saved['color_bgr'].copy(), saved['depth_m']
                observations = []
                for hand in metadata['hands']:
                    side = hand['palm_frame']['handedness_stable']
                    if config.get('position_method', 'plane') == 'wrist_offset':
                        pose = estimate_palm_position(hand['landmarks_normalized'], depth, metadata['intrinsics'], config=config)
                    else:
                        pose = estimate_palm_pose(hand['landmarks_normalized'], depth,
                                                  metadata['intrinsics'], handedness=side, config=config)
                    observations.append(dict(track_id=hand['palm_frame']['track_id'], handedness=side, pose=pose))
                    counts[pose['reason']] += 1
                    for point in hand['landmarks_normalized']:
                        pixel = tuple(round(float(v)*n) for v, n in zip(point[:2], (image.shape[1], image.shape[0])))
                        cv2.circle(image, pixel, 2, (0, 220, 255), -1)
                    if pose['valid']:
                        text = 'xyz(m) ' + ' '.join(f'{v:+.3f}' for v in pose['position_m'])
                        cv2.putText(image, text, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 0), 2)
                result = dict(source=str(source), stamp_s=metadata['stamp_s'], frame_id=metadata['frame_id'],
                              observations=observations, replay_mode='cached landmarks and measured depth',
                              temporal_stability_rechecked=False, robot_motion_enabled=False)
                stream.write(json.dumps(result, allow_nan=False)+'\n')
                cv2.imwrite(str(args.output/'latest.jpg'), image)
    summary = dict(frames=len(files), reasons=dict(counts),
                   note='Geometry replay only; cached landmarks are reused, no new MediaPipe inference or temporal stability certification.')
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
