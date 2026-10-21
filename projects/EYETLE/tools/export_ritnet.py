#!/usr/bin/env python3
"""Export the unmodified official RITnet checkpoint into a static ONNX model."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'third_party/RITnet'))
from densenet import DenseNet2D


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--width',type=int,default=320)
    p.add_argument('--height',type=int,default=240)
    p.add_argument('--batch',type=int,choices=(1,2),default=1)
    args=p.parse_args()
    if args.width%16 or args.height%16:p.error('Dimensions must be multiples of 16')
    torch.set_num_threads(4)
    weights=ROOT/'third_party/RITnet/best_model.pkl'
    net=DenseNet2D(dropout=True,prob=.2)
    net.load_state_dict(torch.load(weights,map_location='cpu',weights_only=True))
    net.eval()
    class Output(torch.nn.Module):
        def __init__(self):
            super().__init__();self.net=net
        def forward(self,x):
            probability=torch.softmax(self.net(x),dim=1)
            confidence,labels=probability.max(dim=1)
            return torch.stack((labels.float(),confidence),dim=1)
    suffix='' if args.batch==1 else '_b2'
    out=ROOT/'models/ritnet'/f'ritnet_{args.width}x{args.height}{suffix}.onnx'
    model=Output().eval()
    torch.onnx.export(model,torch.zeros(args.batch,1,args.height,args.width),str(out),
                      input_names=['eye'],output_names=['segmentation'],opset_version=17)
    import onnx
    onnx.checker.check_model(onnx.load(str(out)))
    info={'source':'https://github.com/AayushKrChaudhary/RITnet',
          'commit':subprocess.check_output(['git','-C',str(weights.parent),'rev-parse','HEAD'],text=True).strip(),
          'weights_sha256':hashlib.sha256(weights.read_bytes()).hexdigest(),
          'onnx_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),
          'batch':args.batch,'width':args.width,'height':args.height,'torch_version':torch.__version__}
    out.with_suffix('.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps(info))

if __name__=='__main__':main()
