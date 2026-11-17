#!/usr/bin/env python3
import argparse
import json
import time
import cv2
import numpy as np
from g1_actions.actions.elevator.config import load
from g1_actions.actions.elevator.inference import Detector
from g1_actions.actions.elevator.robot_model import Robot
from g1_actions.actions.elevator.config import resource

p=argparse.ArgumentParser()
p.add_argument('--image')
p.add_argument('--video')
p.add_argument('--frames',type=int,default=30)
a=p.parse_args()
config=load()
robot=Robot(resource(config,config['model_file']))
detector=Detector(config)
capture=cv2.VideoCapture(a.video) if a.video else None
image=cv2.imread(a.image) if a.image else np.zeros((480,640,3),np.uint8)
if image is None: raise SystemExit('Cannot read requested image')
times=[];examples=[]
try:
    for index in range(a.frames if capture else 3):
        if capture:
            ok,image=capture.read()
            if not ok: break
        detections,ms=detector.predict(image)
        times.append(ms)
        if detections and len(examples)<3:
            examples.append({'frame':index,'detections':detections})
    assert times, 'No frames processed'
    result={'backend':config['detector']['backend'],'frames':len(times),
        'mean_inference_ms':float(np.mean(times)),
        'steady_inference_ms':float(np.mean(times[1:])) if len(times)>1 else times[0],
        'input_size':config['detector']['imgsz'],'examples':examples,
        'camera_transform':robot.camera(config).tolist(),
        'wrist_to_candidate_middle_tip':{side:robot.wrist_tip(t).tolist() for side,t in config['tools'].items()},
        'calibration_verified':config['camera']['verified'] and all(t['verified'] for t in config['tools'].values())}
    print(json.dumps(result,ensure_ascii=False,indent=2))
finally:
    detector.close()
    if capture: capture.release()
