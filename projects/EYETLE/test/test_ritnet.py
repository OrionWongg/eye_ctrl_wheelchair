"""Postprocessing and blink gating checks; no camera or GPU needed."""
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from ritnet_backend import segmentation_metrics
from ritnet_tracker import EyeState


class RitnetTest(unittest.TestCase):
    def setUp(self):
        self.labels=np.zeros((240,320),np.uint8)
        cv2.ellipse(self.labels,(160,120),(70,30),0,0,360,1,-1)
        cv2.circle(self.labels,(160,120),26,2,-1)
        cv2.circle(self.labels,(160,120),12,3,-1)
        self.conf=np.full(self.labels.shape,.99,np.float32)
        # Textured surrounding image, with an independently changing eye area.
        gray=np.tile(np.linspace(80,180,320,dtype=np.uint8),(240,1))
        self.frame=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)

    def test_pupil_rejects_larger_border_artifact(self):
        cv2.ellipse(self.labels,(319,100),(28,50),0,0,360,3,-1)
        m=segmentation_metrics(self.labels,self.conf)
        self.assertIsNotNone(m['pupil'])
        self.assertAlmostEqual(m['pupil'][0][0],160,delta=1)

    def test_iris_context_and_confidence_required(self):
        self.labels[self.labels==2]=0
        self.assertIsNone(segmentation_metrics(self.labels,self.conf)['pupil'])
        self.assertIsNone(segmentation_metrics(self.labels,self.conf*.2)['pupil'])

    def test_closure_requires_recent_open_and_stable_surroundings(self):
        state=EyeState();empty=np.zeros_like(self.labels)
        open_metrics=segmentation_metrics(self.labels,self.conf)
        closed_metrics=segmentation_metrics(empty,self.conf)
        self.assertEqual(state.update(self.frame,empty,self.conf,closed_metrics,0),'unknown')
        self.assertEqual(state.update(self.frame,self.labels,self.conf,open_metrics,.1),'open')
        self.assertEqual(state.update(self.frame,empty,self.conf,closed_metrics,.2),'closed')
        self.assertEqual(state.update(self.frame,empty,self.conf,closed_metrics,.9),'closed')
        self.assertEqual(state.update(self.frame,empty,self.conf,closed_metrics,2),'unknown')

    def test_unknown_does_not_repeat_open_or_closed_within_80ms(self):
        state=EyeState();empty=np.zeros_like(self.labels)
        visible=segmentation_metrics(self.labels,self.conf)
        missing=segmentation_metrics(empty,self.conf)
        self.assertEqual(state.update(self.frame,self.labels,self.conf,visible,0.),'open')
        black=np.zeros_like(self.frame)
        self.assertEqual(state.update(black,empty,self.conf,missing,.04),'unknown')
        self.assertEqual(state.diagnostics['reason'],'SURROUNDINGS_UNSTABLE')
        self.assertEqual(state.update(self.frame,empty,self.conf,missing,.07),'closed')
        self.assertEqual(state.update(black,empty,self.conf,missing,.10),'unknown')

    def test_tight_crop_uses_full_camera_surroundings(self):
        # A large segmented eye leaves <20% background in the model crop.
        labels=self.labels.copy();labels[labels==0]=1
        visible=segmentation_metrics(labels,self.conf)
        full=cv2.resize(self.frame,(640,480))
        state=EyeState()
        state.update(self.frame,labels,self.conf,visible,0.,full,(160,120,320,240))
        empty=np.zeros_like(labels);missing=segmentation_metrics(empty,self.conf)
        self.assertEqual(state.update(self.frame,empty,self.conf,missing,.1,
                                     full,(160,120,320,240)),'closed')
        self.assertGreater(state.diagnostics['outer_fraction'],.2)
        self.assertEqual(state.update(self.frame,empty,self.conf,missing,.2,
                                     np.zeros_like(full),(160,120,320,240)),'unknown')
        self.assertEqual(state.diagnostics['reason'],'SURROUNDINGS_UNSTABLE')

    def test_insufficient_surroundings_has_distinct_reason(self):
        labels=self.labels.copy();labels[labels==0]=1
        state=EyeState()
        state.update(self.frame,labels,self.conf,segmentation_metrics(labels,self.conf),0.)
        empty=np.zeros_like(labels)
        self.assertEqual(state.update(self.frame,empty,self.conf,
                         segmentation_metrics(empty,self.conf),.1),'unknown')
        self.assertEqual(state.diagnostics['reason'],'INSUFFICIENT_SURROUNDINGS')

    def test_black_occlusion_and_lost_pupil_are_not_closed(self):
        state=EyeState();m=segmentation_metrics(self.labels,self.conf)
        state.update(self.frame,self.labels,self.conf,m,0)
        m=dict(m,pupil=None)
        self.assertEqual(state.update(self.frame,self.labels,self.conf,m,.2),'unknown')
        empty=np.zeros_like(self.labels)
        self.assertEqual(state.update(np.zeros_like(self.frame),empty,self.conf,segmentation_metrics(empty,self.conf),.4),'unknown')

if __name__=='__main__':unittest.main()
