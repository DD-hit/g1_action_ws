"""Compute nominal precontact targets from saved RGB-D. Never connects to a driver."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
import yaml
from .transforms import precontact_preview, camera_transform, wrist_to_palm_center
from ...perception.palm_pose import estimate_palm_position


def main():
    package = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--arm', choices=('left', 'right'), default='right')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=package/'config/handshake.yaml')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    files = [args.input] if args.input.is_file() else sorted((args.input/'frames').glob('*.npz'))
    if not files:parser.error('No saved RGB-D frames found')
    args.output.mkdir(parents=True, exist_ok=False)
    counts, targets = Counter(), []
    with (args.output/'targets.jsonl').open('w') as output:
        for source in files:
            with np.load(source, allow_pickle=False) as data:
                metadata = json.loads(str(data['metadata']))
                for hand in metadata['hands']:
                    position = estimate_palm_position(hand['landmarks_normalized'], data['depth_m'],
                                                     metadata['intrinsics'], config=config['perception']['rgbd'])
                    if not position['valid']:
                        counts[position['reason']] += 1;continue
                    result = precontact_preview(config, package, position['position_m'], args.arm,
                                                source_frame=metadata['frame_id'])
                    result.update(stamp_s=metadata['stamp_s'], track_id=hand['palm_frame']['track_id'],
                                  position_stability_checked=False, source=str(source))
                    targets.append(result);output.write(json.dumps(result, allow_nan=False)+'\n')
    summary = dict(frames=len(files), preview_targets=len(targets), rejected=dict(counts),
                   T_torso_color_optical=camera_transform(config, package).tolist(),
                   wrist_to_palm_center_xyz_m=wrist_to_palm_center(config, package, args.arm).tolist(),
                   geometry_verified=False, ik_checked=False, collision_checked=False,
                   executable=False, robot_motion_enabled=False)
    if targets:
        xyz = np.array([t['human_palm_xyz_m'] for t in targets])
        summary['human_palm_range_torso_m'] = dict(min=xyz.min(axis=0).tolist(), max=xyz.max(axis=0).tolist())
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':raise SystemExit(main())
