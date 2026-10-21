"""Synthetic segmentation -> actual RITnet closure adapter, no GPU."""
import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from ritnet_blink import RitnetBlink


class SyntheticRitnet:
    def __init__(self):
        self.labels=np.zeros((240,320),np.uint8)
        cv2.ellipse(self.labels,(160,120),(70,30),0,0,360,1,-1)
        cv2.circle(self.labels,(160,120),26,2,-1)
        cv2.circle(self.labels,(160,120),12,3,-1)
        self.frame=cv2.cvtColor(np.tile(np.linspace(80,180,320,dtype=np.uint8),(240,1)),cv2.COLOR_GRAY2BGR)
        self.closed=[False,False];self.roi=(0,0,320,240);self.feedback_calls=[]
    def infer_views(self,frames):
        return [(f,np.zeros_like(self.labels) if self.closed[i] else self.labels,
                 np.full(self.labels.shape,.99,np.float32),1.,self.roi) for i,f in enumerate(frames)]
    def feedback(self,*args):self.feedback_calls.append(args)
    def close(self):pass


class RitnetBlinkTest(unittest.TestCase):
    def setUp(self):
        self.model=SyntheticRitnet()
        self.blink=RitnetBlink(None,None,None,2,model=self.model)
    def test_open_then_closed_and_long_loss(self):
        frames=[self.model.frame]*2
        self.assertEqual(self.blink.infer(frames,[0.,0.])[0],['open']*2)
        self.model.closed=[True,True]
        self.assertEqual(self.blink.infer(frames,[.12,.12])[0],['closed']*2)
        self.assertEqual(self.blink.infer(frames,[.72,.72])[0],['closed']*2)
        self.assertEqual(self.blink.infer(frames,[1.7,1.7])[0],['unknown']*2)
    def test_crop_change_cannot_reuse_closed_reference(self):
        frames=[self.model.frame]*2
        self.blink.infer(frames,[0.,0.])
        self.model.closed=[True,True];self.model.roi=(10,20,320,240)
        self.assertEqual(self.blink.infer(frames,[.12,.12])[0],['unknown']*2)
    def test_full_frame_feedback_coordinates_and_black_occlusion(self):
        self.model.roi=(100,50,320,240)
        self.blink.infer([self.model.frame]*2,[0.,0.])
        center=self.model.feedback_calls[0][2]['pupil'][0]
        np.testing.assert_allclose(center,(260,170),atol=1)
        self.model.closed=[True,True]
        states,_=self.blink.infer([np.zeros_like(self.model.frame)]*2,[.2,.2])
        self.assertEqual(states,['unknown']*2)

if __name__=='__main__':unittest.main()
