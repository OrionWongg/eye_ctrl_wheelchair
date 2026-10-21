#!/usr/bin/env python3
"""Benchmark deployed TensorRT on an image; never interprets detections as accuracy."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from ritnet_backend import TensorRTModel,AdaptiveRitnet,segmentation_metrics,draw_segmentation

p=argparse.ArgumentParser()
p.add_argument('image')
p.add_argument('--engine',default='models/ritnet/ritnet_640x480.engine')
p.add_argument('--full-frame',action='store_true')
p.add_argument('--crop-engine',default='models/ritnet/ritnet_320x240.engine')
p.add_argument('--output',default='model-results/image')
a=p.parse_args()
cv2.setNumThreads(2)
f=cv2.imread(a.image)
if f is None:raise RuntimeError('Cannot load '+a.image)
m=TensorRTModel(a.engine) if a.full_frame else AdaptiveRitnet(a.engine,a.crop_engine)
try:
 for _ in range(20):m.infer(f)
 timings=[]
 for _ in range(100):
  labels,confidence,ms=m.infer(f);timings.append(ms)
 metrics=segmentation_metrics(labels,confidence)
 cv2.imwrite(a.output+'-overlay.jpg',draw_segmentation(f,labels,metrics))
 np.savez(a.output+'-prediction.npz',labels=labels,confidence=confidence)
 report=dict(engine=a.engine,adaptive_crop=not a.full_frame,image=a.image,inference_ms_p50=float(np.median(timings)),
             inference_ms_p95=float(np.percentile(timings,95)),metrics=metrics,
             note='Inference includes preprocessing and CPU/GPU transfers; no camera wait or GUI. No ground-truth accuracy measured.')
 Path(a.output+'-report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report))
finally:m.close()
