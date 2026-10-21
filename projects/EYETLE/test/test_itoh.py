"""Native original fitter: projected-eye ground truth and input failures."""
from pathlib import Path
import sys
import unittest
import json
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from itoh_backend import ItohEye, ROOT


def projected_ellipse(yaw,pitch,focal=476.70143703768406):
    yaw,pitch=np.deg2rad([yaw,pitch])
    normal=np.array([np.sin(yaw)*np.cos(pitch),np.sin(pitch),-np.cos(yaw)*np.cos(pitch)])
    center=np.array([0.,0.,40.])+12*normal
    u=np.cross(normal,[0.,1.,0.]);u/=np.linalg.norm(u)
    v=np.cross(normal,u)
    theta=np.linspace(0,2*np.pi,180,endpoint=False)
    pts=center+2*(np.cos(theta)[:,None]*u+np.sin(theta)[:,None]*v)
    image=np.column_stack((320+focal*pts[:,0]/pts[:,2],240+focal*pts[:,1]/pts[:,2]))
    (cx,cy),(w,h),angle=cv2.fitEllipse(image.astype(np.float32))
    return [cx,cy,w,h,angle],normal


@unittest.skipUnless((ROOT/'build/itoh/libitoh.so').exists(),'Run tools/build_itoh.sh for native tests')
class ItohTest(unittest.TestCase):
    def setUp(self):
        self.eye=ItohEye();self.addCleanup(self.eye.close)
        self.gray=np.full((480,640),170,np.uint8)

    def test_blank_image_and_resolution_rejected(self):
        self.assertTrue(np.isnan(self.eye.infer(self.gray)).all())
        self.assertFalse(self.eye.detail['detected'])
        with self.assertRaises(ValueError):self.eye.infer(self.gray[:240,:320])

    def test_fixed_gaze_cannot_build_and_reset_clears_observations(self):
        ellipse,_=projected_ellipse(15,8)
        for _ in range(40):self.eye.infer(self.gray,ellipse)
        self.assertEqual(self.eye.detail['observations'],1)
        self.assertFalse(self.eye.detail['model_built'])
        self.eye.reset();self.eye.infer(self.gray,[0,0,0,0,0])
        self.assertEqual(self.eye.detail['observations'],0)

    def test_projected_eye_builds_and_recovers_gaze(self):
        dots=[]
        for i in range(180):
            ellipse,normal=projected_ellipse(30*np.sin(i*.13),20*np.sin(i*.21))
            self.eye.infer(self.gray,ellipse)
            # The first valid result used to expose np.bool_ and crash both
            # the live JSON logger and its finally/shutdown report.
            self.assertIs(type(self.eye.detail['valid']),bool)
            payload=dict(gaze_details=[self.eye.detail],direction_method='itoh')
            self.assertEqual(json.loads(json.dumps(payload,allow_nan=False)),payload)
            if self.eye.detail['valid']:dots.append(float(np.dot(normal,self.eye.detail['normal'])))
        self.assertTrue(self.eye.detail['model_built'])
        self.assertGreater(len(dots),50)
        self.assertGreater(np.percentile(dots,5),.99)

if __name__=='__main__':unittest.main()
