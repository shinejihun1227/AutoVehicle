#!/usr/bin/env python3
"""Real six-class model load/inference; does not open sensors or command UDP."""
from pathlib import Path
import sys
import numpy as np

ws=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ws/'src/detection/camera_perception/post_processing'))
import real_lane

seg=real_lane.Segmenter(checkpoint=str(ws/'src/detection/camera_perception/models/highway_best.pt'),
    cam_set=str(ws/'src/detection/camera_perception/lane/cam_set.json'),device='cpu')
assert seg.info['num_classes']==6 and seg.class_names==real_lane.CLASS_NAMES
frame=np.zeros((seg.cam.height+seg.crop_top,seg.cam.width,3),dtype=np.uint8)
mask,_=seg.apply(frame)
assert mask.ndim==2 and np.isfinite(mask).all() and mask.min()>=0 and mask.max()<6
print('HIGHWAY_MODEL_INFERENCE_OK',seg.info)
