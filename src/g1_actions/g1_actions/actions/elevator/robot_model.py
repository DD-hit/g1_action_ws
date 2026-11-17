import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import transform, transform_point


class Robot:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {}
        self.links = {x.attrib['name'] for x in root.findall('link')}
        for j in root.findall('joint'):
            origin = j.find('origin')
            xyz = [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            axis = j.find('axis')
            limit = j.find('limit')
            self.joints[j.find('child').attrib['link']] = {
                'name': j.attrib['name'], 'type': j.attrib['type'],
                'parent': j.find('parent').attrib['link'], 'origin': transform(xyz, rpy),
                'axis': np.array([float(v) for v in axis.attrib['xyz'].split()]) if axis is not None else np.array([1.,0,0]),
                'limit': dict(limit.attrib) if limit is not None else {}}

    def relative(self, parent, child, positions=None):
        positions = positions or {}
        chain, current = [], child
        while current != parent:
            if current not in self.joints:
                raise ValueError(f'{parent} is not an ancestor of {child}')
            j = self.joints[current]
            local = j['origin'].copy()
            if j['type'] != 'fixed':
                if j['name'] not in positions:
                    raise ValueError(f'Missing measured joint {j["name"]}')
                q = positions[j['name']]
                axis = j['axis']/np.linalg.norm(j['axis'])
                motion = np.eye(4)
                if j['type'] in ('revolute', 'continuous'):
                    motion[:3, :3] = Rotation.from_rotvec(axis*q).as_matrix()
                elif j['type'] == 'prismatic':
                    motion[:3, 3] = axis*q
                else:
                    raise ValueError('Unsupported non-fixed joint')
                local = local@motion
            chain.append(local)
            current = j['parent']
        result = np.eye(4)
        for local in reversed(chain):
            result = result@local
        return result

    def wrist_tip(self, tool):
        t = self.relative(tool['wrist_link'], tool['hand_link'])
        return transform_point(t, tool['hand_tip_xyz'])

    def camera(self, config):
        c = config['camera']
        if c['measured_xyz'] is not None:
            return transform(c['measured_xyz'], c['measured_rpy'])
        return self.relative(config['base_frame'], c['mount_link']) @ transform(c['optical_xyz'], c['optical_rpy'])


def main():
    from .config import load, resource
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    args = parser.parse_args()
    cfg = load(args.config)
    robot = Robot(resource(cfg, cfg['model_file']))
    print(json.dumps({'base_frame': cfg['base_frame'], 'T_torso_color_optical': robot.camera(cfg).tolist(),
                      'camera_verified': cfg['camera']['verified'],
                      'wrist_to_middle_tip': {arm: robot.wrist_tip(tool).tolist() for arm, tool in cfg['tools'].items()},
                      'tools_verified': {arm: tool['verified'] for arm, tool in cfg['tools'].items()}}, indent=2))
