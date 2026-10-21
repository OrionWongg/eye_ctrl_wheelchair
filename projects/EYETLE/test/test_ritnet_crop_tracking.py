"""Crop escape and blink regression, using actual adaptive/closure code without GPU."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from ritnet_backend import AdaptiveRitnet,segmentation_metrics
from ritnet_blink import RitnetBlink


def eye(shape,center):
    labels=np.zeros(shape,np.uint8)
    cv2.ellipse(labels,center,(65,30),0,0,360,1,-1)
    cv2.circle(labels,center,24,2,-1)
    cv2.circle(labels,center,12,3,-1)
    return labels


class CropTrackingTest(unittest.TestCase):
    def setUp(self):
        self.frame=cv2.cvtColor(np.tile(np.linspace(80,180,640,dtype=np.uint8),(480,1)),cv2.COLOR_GRAY2BGR)
        self.full_labels=eye((480,640),(400,210))
        self.local_labels=eye((240,320),(144,104))
        self.model=AdaptiveRitnet.__new__(AdaptiveRitnet)
        self.model.rois={0:(256,106,320,240)}
        self.model.unknown_since={};self.model.frame_sizes={0:(640,480)};self.model.view_diagnostics={}
        self.calls=0
        def full(frame):
            self.calls+=1
            return self.full_labels,np.full((480,640),.99,np.float32),10.
        self.model.full=SimpleNamespace(infer=full)
        self.model.crop=SimpleNamespace(width=320,height=240,
            infer=lambda frame:(self.local_labels,np.full((240,320),.99,np.float32),2.))
        self.model.batch=None

    def test_preemptive_recenter_with_image_bounds(self):
        metrics=segmentation_metrics(eye((480,640),(340,210)),np.ones((480,640),np.float32))
        self.model.feedback(0,'open',metrics,0.)
        self.assertEqual(self.model.rois[0],(180,90,320,240))
        metrics['pupil']=((10,10),(20,20),0)
        self.model.feedback(0,'open',metrics,.1)
        self.assertEqual(self.model.rois[0],(0,0,320,240))

    def test_fast_right_gaze_recovers_in_same_frame(self):
        blink=RitnetBlink(None,None,None,1,model=self.model)
        self.assertEqual(blink.infer([self.frame],[0.])[0],['open'])
        # Log failure: left crop starts at 256, actual pupil moves to x=260.
        self.local_labels[:]=0
        self.full_labels=eye((480,640),(260,240))
        states,details=blink.infer([self.frame],[.068])
        self.assertEqual(states,['open'])
        self.assertAlmostEqual(details[0]['pupil_center'][0],260,delta=1)
        self.assertEqual(details[0]['crop_tracking']['recovery'],'FULL_FRAME_PUPIL')
        self.assertEqual(self.model.rois[0],(100,120,320,240))
        self.assertEqual(self.calls,1)

    def test_true_blink_preserves_crop_reference(self):
        blink=RitnetBlink(None,None,None,1,model=self.model)
        blink.infer([self.frame],[0.])
        self.local_labels[:]=0;self.full_labels[:]=0
        states,details=blink.infer([self.frame],[.1])
        self.assertEqual(states,['closed'])
        self.assertEqual(details[0]['crop_tracking']['recovery'],'FULL_FRAME_NO_PUPIL')
        self.assertEqual(details[0]['roi'],[256,106,320,240])
        self.local_labels=eye((240,320),(144,104))
        self.assertEqual(blink.infer([self.frame],[.2])[0],['open'])

    def test_visible_eye_outside_crop_is_not_closure(self):
        blink=RitnetBlink(None,None,None,1,model=self.model)
        blink.infer([self.frame],[0.])
        self.local_labels[:]=0
        self.full_labels=eye((480,640),(180,240))
        self.full_labels[self.full_labels==3]=2  # Iris visible; no usable pupil.
        states,details=blink.infer([self.frame],[.1])
        self.assertEqual(states,['unknown'])
        self.assertEqual(details[0]['closure_diagnostics']['reason'],'EYE_OUTSIDE_CROP')
        self.assertNotIn(0,self.model.rois)

    def test_batch_recovery_is_per_eye(self):
        self.model.rois[1]=(256,106,320,240)
        good=self.local_labels.copy();bad=np.zeros_like(good);conf=np.ones_like(good,dtype=np.float32)
        self.model.batch=SimpleNamespace(infer_batch=lambda frames:([(bad,conf),(good,conf)],4.))
        self.full_labels=eye((480,640),(260,240))
        views=self.model.infer_views([self.frame,self.frame])
        self.assertEqual(views[0][-1],(0,0,640,480))
        self.assertEqual(views[1][-1],(256,106,320,240))
        self.assertEqual(self.calls,1)

if __name__=='__main__':unittest.main()
