"""Algorithm checks independent of the connected cameras."""
import importlib.util
from pathlib import Path
import unittest
import cv2
import numpy as np

spec = importlib.util.spec_from_file_location('dual', Path(__file__).parents[1]/'tools/dual_eye_tracker.py')
dual = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dual)

class DualEyeTest(unittest.TestCase):
    def test_pupil_and_blank(self):
        frame = np.full((240,320,3),190,dtype=np.uint8)
        self.assertIsNone(dual.find_pupil(frame,(0,0,320,240)))
        cv2.ellipse(frame,(160,120),(22,18),15,0,360,(15,15,15),-1)
        pupil = dual.find_pupil(frame,(0,0,320,240))
        self.assertIsNotNone(pupil)
        self.assertAlmostEqual(pupil[0][0],160,delta=2)
        self.assertAlmostEqual(pupil[0][1],120,delta=2)
    def test_mirrored_calibration(self):
        for c in [dict(left=.3,center=.5,right=.7),dict(left=.7,center=.5,right=.3)]:
            self.assertAlmostEqual(dual.gaze(c['left'],c),-1)
            self.assertAlmostEqual(dual.gaze(c['center'],c),0)
            self.assertAlmostEqual(dual.gaze(c['right'],c),1)

if __name__ == '__main__':unittest.main()
