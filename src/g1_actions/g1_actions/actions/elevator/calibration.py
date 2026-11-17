"""Record measured calibration, with explicit provenance and no inferred acceptance."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import math
import shutil
import yaml
from .config import load


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config')
    sub=p.add_subparsers(dest='kind',required=True)
    camera=sub.add_parser('camera',help='Measured torso -> color optical transform in metres/radians')
    camera.add_argument('--xyz',type=float,nargs=3,required=True)
    camera.add_argument('--rpy',type=float,nargs=3,required=True)
    camera.add_argument('--note',required=True,help='Measurement method/reference and date')
    tool=sub.add_parser('tool',help='Measured contact point expressed in the rubber-hand link')
    tool.add_argument('--arm',choices=['left','right'],required=True)
    tool.add_argument('--xyz',type=float,nargs=3,required=True)
    tool.add_argument('--note',required=True)
    a=p.parse_args()
    if not all(math.isfinite(v) for v in a.xyz):
        p.error('Calibration values must be finite')
    cfg=load(a.config)
    if a.kind=='camera':
        if not all(math.isfinite(v) for v in a.rpy): p.error('RPY must be finite')
        cfg['camera'].update(measured_xyz=a.xyz,measured_rpy=a.rpy,verified=True)
        section=cfg['camera']
    else:
        if max(abs(v) for v in a.xyz)>.3: p.error('Tool offset exceeds 0.3 m; check units')
        section=cfg['tools'][a.arm]
        section.update(hand_tip_xyz=a.xyz,verified=True)
    section['verification_note']=a.note
    section['verified_at']=datetime.now(timezone.utc).isoformat()
    path=Path(cfg.pop('_config_path'))
    cfg.pop('_share',None)
    backup=path.with_suffix(path.suffix+'.before_calibration')
    shutil.copy2(path,backup)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True))
    temp.replace(path)
    print('Recorded measured calibration in',path)
    print('Restart the vision process to load the updated transform.')
