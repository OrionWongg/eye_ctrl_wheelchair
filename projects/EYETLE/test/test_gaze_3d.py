"""Coordinate, fitting, center persistence and fail-closed tests for 3D gaze."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from gaze_3d import EyeGeometry, CenterCalibration, full_frame_ellipse, FrameMotionGuard


class CenterTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'center.json'
        self.bindings=[dict(camera='left',focal=477),dict(camera='right',focal=477)]
        self.c=CenterCalibration(self.path,self.bindings,10,.1)

    def capture(self):
        self.c.begin('center',1.)
        for i in range(10):self.c.observe([12.,-8.],['open']*2,1.2+i*.04,True)

    def test_only_center_and_post_key_valid_samples_save(self):
        self.assertFalse(self.c.begin('left',0.))
        self.c.begin('center',1.)
        for i in range(10):self.c.observe([12.,-8.],['open']*2,.5+i*.04,True)
        self.assertFalse(self.path.exists())
        self.capture();self.assertTrue(self.c.ready)
        loaded=CenterCalibration(self.path,self.bindings)
        self.assertEqual(loaded.loaded,2)
        self.assertEqual(loaded.horizontal([7.,-13.],[-1,-1]),(.2,True))
        self.assertEqual(loaded.horizontal([12.,-8.],[-1,-1]),(0.,True))
        self.assertEqual(loaded.horizontal([22.,-18.],[-1,-1]),(0.,False))

    def test_binding_mismatch_and_corrupt_file_fail_closed(self):
        self.capture()
        self.assertFalse(CenterCalibration(self.path,[dict(camera='different')]).ready)
        self.path.write_text('{')
        self.assertFalse(CenterCalibration(self.path,self.bindings).ready)

    def test_single_eye_profile_cannot_enable_binocular_control(self):
        single=CenterCalibration(self.path,self.bindings[:1],10,.1)
        single.begin('center',0.)
        for i in range(10):single.observe([12.],['open'],1+i*.04,True)
        both=CenterCalibration(self.path,self.bindings,10,.1)
        self.assertEqual(both.loaded,1)
        self.assertFalse(both.ready)
        self.assertEqual(both.horizontal([12.,-8.],[-1,-1]),(0.,False))
        self.capture()
        self.assertTrue(CenterCalibration(self.path,self.bindings).ready)

    def test_binocular_direction_conflict_cancels_closure_history(self):
        from dual_eye_tracker import BinocularState
        self.capture()
        binocular=BinocularState()
        _,ready=self.c.horizontal([12.,-8.],[-1,-1])
        self.assertEqual(binocular.update(['open']*2,1.,ready),(False,True))
        _,ready=self.c.horizontal([22.,-18.],[-1,-1])
        self.assertEqual(binocular.update(['open']*2,1.04,ready),(False,False))
        _,ready=self.c.horizontal([12.,-8.],[-1,-1])
        self.assertEqual(binocular.update(['closed']*2,1.08,ready),(False,False))
        self.assertEqual(binocular.update(['open']*2,1.12,ready),(False,True))

    def test_invalid_and_closed_cancel_incomplete_capture(self):
        self.c.begin('center',0.)
        for i in range(9):self.c.observe([12.,-8.],['open']*2,1+i*.04,True)
        self.c.observe([None,None],['closed']*2,2.,False)
        self.assertEqual([len(b) for b in self.c.buffers],[0,0])
        self.assertFalse(self.c.ready)

    def test_moving_center_and_replayed_frames_not_saved(self):
        self.c.begin('center',0.)
        for i in range(25):self.c.observe([float(i%2)*10.,0.],['open']*2,1+i*.04,True)
        self.assertFalse(self.c.ready)
        self.c.restart();self.c.begin('center',3.)
        for i in range(20):self.c.observe([0.,0.],['open']*2,4.,True)
        self.assertFalse(self.c.ready)

    def test_failed_write_preserves_previous_profile(self):
        self.capture();old=self.path.read_bytes()
        self.c.begin('center',3.)
        with patch('gaze_3d.os.replace',side_effect=OSError('disk full')):
            for i in range(10):self.c.observe([13.,-9.],['open']*2,3.2+i*.04,True)
        self.assertFalse(self.c.ready);self.assertTrue(self.c.pending_save)
        self.assertEqual(self.path.read_bytes(),old)
        self.c.save_pending();self.assertTrue(self.c.ready)

    def test_rotation_invalidation_survives_restart(self):
        self.capture();self.c.invalidate_rotation()
        self.assertFalse(CenterCalibration(self.path,self.bindings).ready)
        self.c.last_stamp=None
        self.capture()
        self.assertTrue(CenterCalibration(self.path,self.bindings).ready)

    def test_legacy_file_never_overwritten(self):
        self.path.write_text(json.dumps({'schema':1,'profiles':{},'drafts':{}}))
        self.capture();self.assertFalse(self.c.ready)
        self.assertEqual(json.loads(self.path.read_text())['schema'],1)


class GeometryTests(unittest.TestCase):
    def test_roi_and_anisotropic_ellipse_mapping(self):
        e=full_frame_ellipse(((40.,30.),(10.,20.),0.),(100,50,640,240),(240,320))
        np.testing.assert_allclose(e['center'],[180,80])
        np.testing.assert_allclose(e['axes'],[20,20])
        full=full_frame_ellipse(((180.,80.),(10.,20.),30.),(0,0,640,480),(480,640))
        crop=full_frame_ellipse(((80.,30.),(10.,20.),30.),(100,50,320,240),(240,320))
        np.testing.assert_allclose(full['center'],crop['center'])
        np.testing.assert_allclose(full['axes'],crop['axes'])
        self.assertAlmostEqual(full['angle'],crop['angle'])

    def test_surrounding_image_shift_but_not_pupil_motion(self):
        rng=np.random.default_rng(14)
        frame=rng.integers(0,255,(480,640,3),dtype=np.uint8)
        guard=FrameMotionGuard();bbox=(220,150,200,160)
        self.assertEqual(guard.update(frame,bbox,1.),(False,False))
        changed=frame.copy();changed[150:310,220:420]=0
        self.assertEqual(guard.update(changed,bbox,1.04),(False,False))
        moved=cv2.warpAffine(changed,np.float32([[1,0,10],[0,1,0]]),(640,480))
        shifted,rotated=guard.update(moved,bbox,1.08)
        self.assertTrue(shifted);self.assertFalse(rotated)

    def test_real_pye3d_fit_and_failure_gating(self):
        from pye3d.camera import CameraModel
        from pye3d.detector_3d import Detector3D
        from pye3d.geometry.primitives import Circle
        from pye3d.geometry.projections import project_circle_into_image_plane
        from pye3d.constants import _EYE_RADIUS_DEFAULT
        # Synthetic pinhole geometry has no cornea: disable optical correction
        # only for this test. Production uses pye3d's refraction correction.
        class PinholeDetector(Detector3D):
            def update_and_detect(self,data,frame):
                return super().update_and_detect(data,frame,apply_refraction_correction=False)
        g=EyeGeometry((640,480),477.,PinholeDetector(CameraModel(477.,(640,480))))
        self.addCleanup(g.close)
        frame=np.random.default_rng(18).integers(0,255,(480,640,3),dtype=np.uint8);errors=[]
        initial_frame=frame.copy()
        for i in range(360):
            yaw=np.deg2rad(20*np.sin(i*.07));pitch=np.deg2rad(12*np.sin(i*.11))
            normal=np.array([np.sin(yaw)*np.cos(pitch),np.sin(pitch),-np.cos(yaw)*np.cos(pitch)])
            if i==180:frame=cv2.warpAffine(initial_frame,np.float32([[1,0,10],[0,1,0]]),(640,480))
            sphere=np.array([2. if i<180 else 4.,0.,40.])
            e=project_circle_into_image_plane(Circle(sphere+_EYE_RADIUS_DEFAULT*normal,normal,2.),477.,True,640,480)
            ellipse=(tuple(e.center),(e.minor_radius,e.major_radius),e.angle)
            angle,ok=g.update(frame,ellipse,(0,0,640,480),(480,640),.99,'open',1+i/30)
            if i<45 or i==180:self.assertFalse(ok)
            if ok and (i<180 or i>=330):errors.append(abs(angle-np.degrees(yaw)))
        self.assertGreater(len(errors),60)
        self.assertEqual(g.shift_events,1)
        self.assertLess(np.percentile(errors,95),1.)
        self.assertTrue(g.update(frame,None,(0,0,640,480),(480,640),0.,'closed',13.01)[1])
        self.assertFalse(g.update(frame,None,(0,0,640,480),(480,640),0.,'unknown',13.04)[1])
        self.assertFalse(g.update(frame,None,(0,0,640,480),(480,640),0.,'closed',13.07)[1])

if __name__=='__main__':unittest.main()

class FixationRegressionTests(unittest.TestCase):
    def test_learned_eye_model_stays_ready_during_long_center_fixation(self):
        class Model:
            sphere_center=np.array([0.,0.,40.])
        class Detector:
            short_term_model=Model()
            def update_and_detect(self,data,frame):
                return {'circle_3d':{'normal':[0.,0.,-1.]},'sphere':{'center':[0.,0.,40.]},
                        'ellipse':data['ellipse'],'diameter_3d':4.,'model_confidence':1.}
            def _cleanup_models(self):pass
        g=EyeGeometry((640,480),detector=Detector())
        frame=np.zeros((480,640,3),np.uint8)
        for i in range(300):
            x=320.+20*np.sin(i*.1) if i<100 else 320.
            _,ok=g.update(frame,((x,240.),(30.,40.),0.),(0,0,640,480),(480,640),.99,'open',1+i/30)
            if i>=100:self.assertTrue(ok,(i,g.reason))
        self.assertTrue(g.has_diversity)
        self.assertAlmostEqual(g.diagnostics['input_span_px'],0.)
