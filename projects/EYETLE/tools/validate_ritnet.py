#!/usr/bin/env python3
"""Compare deployed FP16 TensorRT against original FP32 checkpoint on an image."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import torch
from ritnet_backend import TensorRTModel,segmentation_metrics,draw_segmentation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'third_party/RITnet'))
from densenet import DenseNet2D

p=argparse.ArgumentParser();p.add_argument('image');p.add_argument('--engine',default='models/ritnet/ritnet_640x480.engine');p.add_argument('--output',default='model-results/parity.json');a=p.parse_args()
cv2.setNumThreads(2);torch.set_num_threads(4)
f=cv2.imread(a.image)
if f is None:raise RuntimeError('Cannot load image')
net=DenseNet2D(dropout=True,prob=.2)
net.load_state_dict(torch.load(ROOT/'third_party/RITnet/best_model.pkl',map_location='cpu',weights_only=True));net.eval()
model=TensorRTModel(a.engine)
try:
 inp=model.preprocess(f)
 with torch.no_grad():
  conf,labels=torch.softmax(net(torch.from_numpy(inp)),dim=1).max(dim=1)
 labels=labels[0].numpy().astype(np.uint8);conf=conf[0].numpy()
 batch=model.buffers['eye'][0].shape[0]
 outputs,_=model.infer_batch([f]*batch)
 actual,actual_conf=outputs[0]
 assert all(np.array_equal(actual,l) for l,c in outputs),'Batch samples diverged'
 result=dict(label_agreement=float(np.mean(labels==actual)),confidence_mae=float(np.mean(abs(conf-actual_conf))),
             cpu_metrics=segmentation_metrics(labels,conf),trt_metrics=segmentation_metrics(actual,actual_conf),
             note='Numerical conversion check on one image, not ground-truth model accuracy.')
 for name,l,c in [('cpu',labels,conf),('trt',actual,actual_conf)]:
  cv2.imwrite(str(Path(a.output).with_suffix(''))+'-'+name+'.jpg',draw_segmentation(f,l,segmentation_metrics(l,c)))
 Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
 assert result['label_agreement']>.99, 'Accelerated segmentation differs from original checkpoint'
finally:model.close()
