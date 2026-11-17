"""Capture full G1 feedback using a subscriber-only SDK helper; no driver."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml

# Unitree G1_29 joint indices, same mapping as SDK examples and upper-body driver.
SDK_JOINT_NAMES = tuple(
    [f'{arm}_{name}_joint' for arm in ('left','right')
     for name in ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')]
    + [f'waist_{name}_joint' for name in ('yaw','roll','pitch')]
    + [f'{arm}_{name}_joint' for arm in ('left','right')
       for name in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--interface', default='eth0')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[3]
    project, workspace = package.parents[1], package.parents[3]
    args.output.mkdir(parents=True, exist_ok=False)
    joints = ET.parse(package/'robot/g1_29dof_mode_15.urdf').getroot().findall('joint')
    expected = {j.get('name') for j in joints if j.get('type')!='fixed'}
    if expected != set(SDK_JOINT_NAMES):raise RuntimeError('SDK mapping and robot model joints differ')
    build = workspace/'build/handshake_preflight'
    with (args.output/'build.log').open('w') as log:
        subprocess.run(['cmake','-S',str(project/'tools/handshake_preflight'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],check=True,stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['cmake','--build',str(build),'--target','handshake_full_state_reader','-j2'],check=True,stdout=log,stderr=subprocess.STDOUT)
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = '/opt/unitree_sdk2/lib:'+env.get('LD_LIBRARY_PATH','')
    with (args.output/'reader.log').open('w') as log:
        subprocess.run([str(build/'handshake_full_state_reader'),args.interface,str(args.output/'motors.yaml')],
                       check=True,stdout=log,stderr=subprocess.STDOUT,env=env,timeout=15)
    report = yaml.safe_load((args.output/'motors.yaml').read_text())
    values = report.pop('motor_q')
    if len(values)!=29:raise RuntimeError('Incomplete motor snapshot')
    report['joint_positions'] = dict(zip(SDK_JOINT_NAMES, values))
    (args.output/'state.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0


if __name__ == '__main__':raise SystemExit(main())
