import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from fusion_calibration import GuidedCalibration
from fusion_control import FusionControl,combine_eye_state
from purest_backend import PuReST,ROOT
import cv2


class FusionTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'profile.json';self.binding={'cameras':['left','right']}
        self.cal=GuidedCalibration(self.path,self.binding,2,hold=.6,settle=.1,min_samples=8)
        self.now=0.
        self.values={'center':np.array([[.5,.51],[.5,.49]]),
                     'left':np.array([[.35,.36],[.65,.64]]),
                     'right':np.array([[.65,.66],[.35,.34]])}
    def advance(self,value,states=None,fresh=True):
        self.now+=.05
        self.cal.observe(value,np.ones((2,2)),states or ['open']*2,self.now,fresh)
    def complete(self):
        for _ in range(200):
            if self.cal.ready or self.cal.pending_save:break
            self.advance(self.values.get(self.cal.phase,self.values['center']))
        self.assertTrue(self.cal.ready or self.cal.pending_save,self.cal.message)
    def test_guided_mirrored_mapping_and_saved_center_only_start(self):
        self.complete()
        for name,wanted in [('left',-1),('center',0),('right',1)]:
            h,ok,_,_=self.cal.horizontal(self.values[name],np.ones((2,2)))
            self.assertTrue(ok);self.assertAlmostEqual(h,wanted)
        other=GuidedCalibration(self.path,self.binding,2)
        self.assertTrue(other.loaded);self.assertEqual(other.steps,['confirm']);self.assertFalse(other.ready)
        self.assertEqual(other.horizontal(self.values['center'],np.ones((2,2)))[1],False)
        changed=GuidedCalibration(self.path,{'cameras':['changed']},2)
        self.assertFalse(changed.loaded);self.assertEqual(changed.phase,'center')
    def test_tracking_loss_clears_capture_and_cannot_authorize(self):
        for _ in range(10):self.advance(self.values['center'])
        self.advance(self.values['center'],fresh=False)
        self.assertEqual(self.cal.progress,0)
        self.assertFalse(self.cal.ready)
        for _ in range(50):self.advance(self.values['center'],states=['closed']*2)
        self.assertFalse(self.path.exists())
    def test_indistinguishable_directions_fail_without_overwriting(self):
        for _ in range(200):self.advance(self.values['center'])
        self.assertEqual(self.cal.phase,'failed');self.assertFalse(self.path.exists())
    def test_save_failure_disarms_and_retry_saves(self):
        with patch('fusion_calibration.os.replace',side_effect=OSError('test disk full')):self.complete()
        self.assertTrue(self.cal.pending_save);self.assertFalse(self.cal.ready)
        self.cal.save_pending();self.assertTrue(self.cal.ready)
        json.loads(self.path.read_text())
    def test_detector_conflict_and_one_eye_missing_stop(self):
        self.complete();q=np.ones((2,2))
        bad=self.values['center'].copy();bad[0]=[.35,.66]
        self.assertEqual(self.cal.horizontal(bad,q)[3],'DETECTORS_DISAGREE')
        q[0]=0
        self.assertEqual(self.cal.horizontal(self.values['center'],q)[3],'NO_DIRECTION_FEATURE')
        q=np.ones((2,2));q[:,1]=0
        self.assertTrue(self.cal.horizontal(self.values['left'],q)[1])
    def test_blink_stays_independent_of_closed_pupil_coordinates(self):
        self.complete();f=FusionControl(self.cal)
        self.now+=.05
        self.assertTrue(f.update(self.values['center'],np.ones((2,2)),['open']*2,self.now,True)[2])
        for i in range(1,15):
            h,c,v,_=f.update(np.full((2,2),np.nan),np.zeros((2,2)),['closed']*2,self.now+i*.05,True)
            self.assertTrue(c and v)
    def test_second_detector_recovers_open_but_never_turns_unknown_into_closed(self):
        self.assertEqual(combine_eye_state('unknown',True),'open')
        self.assertEqual(combine_eye_state('unknown',False),'unknown')
        self.assertEqual(combine_eye_state('closed',True),'closed')
        self.assertEqual(combine_eye_state('closed',False),'closed')

    def test_async_closure_survives_transition_but_persistent_loss_stops(self):
        self.complete();f=FusionControl(self.cal);start=self.now+.1
        good=self.values['center'];q=np.ones((2,2))
        self.assertTrue(f.update(good,q,['open']*2,start,True)[2])
        self.assertEqual(f.update(good,q,['closed','open'],start+.05,True)[1:3],(False,True))
        self.assertEqual(f.update(good,q,['unknown']*2,start+.12,True)[1:3],(False,True))
        self.assertEqual(f.update(good,q,['closed']*2,start+.17,True)[1:3],(True,True))
        self.assertEqual(f.update(good,q,['open']*2,start+.3,True)[1:3],(False,True))
        self.assertFalse(f.update(good,q,['unknown']*2,start+.51,True)[2])
        self.assertFalse(f.update(good,q,['closed']*2,start+.56,True)[2])

    def test_stale_and_direction_conflict_do_not_get_transition_grace(self):
        self.complete();f=FusionControl(self.cal);start=self.now+.1
        good=self.values['center'];q=np.ones((2,2))
        f.update(good,q,['open']*2,start,True)
        self.assertFalse(f.update(good,q,['closed']*2,start+.05,False)[2])
        f.update(good,q,['open']*2,start+.1,True)
        bad=good.copy();bad[0]=[.35,.66]
        self.assertFalse(f.update(bad,q,['open']*2,start+.15,True)[2])

    def test_blink_event_survives_direction_loss_and_retries_are_bounded(self):
        self.complete();f=FusionControl(self.cal);start=self.now+.1
        good=self.values['center'];q=np.ones((2,2));missing=np.full((2,2),np.nan)
        f.update(good,q,['open']*2,start,True)
        f.update(missing,np.zeros((2,2)),['open']*2,start+.04,True)
        self.assertFalse(f.motion_valid)
        f.update(missing,q,['closed','open'],start+.08,True)
        f.update(missing,q,['closed','closed'],start+.16,True)
        f.update(missing,q,['open','closed'],start+.21,True)
        f.update(good,q,['open']*2,start+.28,True)
        self.assertIsNotNone(f.event);self.assertTrue(f.motion_valid)
        event_id=f.event['id']
        f.update(good,q,['open']*2,start+.32,True)
        self.assertIsNone(f.event);self.assertEqual(f.outgoing_event['id'],event_id)
        f.update(good,q,['open']*2,start+.50,True)
        self.assertIsNone(f.outgoing_event)
        f.reset_gesture();self.assertEqual(f.gestures.serial,event_id)


@unittest.skipUnless((ROOT/'build/purest/libpurest.so').exists(),'Build PuReST first')
class PuReSTTest(unittest.TestCase):
    def test_original_tracker_detects_ellipse_and_rejects_black_occlusion(self):
        p=PuReST(1);self.addCleanup(p.close)
        frame=np.full((240,320,3),180,np.uint8)
        cv2.ellipse(frame,(160,120),(25,20),15,0,360,(10,10,10),-1)
        result=p.infer([frame],[(0,0,320,240)],[1.])[0]
        self.assertTrue(result['valid']);np.testing.assert_allclose(result['center'],[160,120],atol=2)
        self.assertFalse(p.infer([np.zeros_like(frame)],[(0,0,320,240)],[1.1])[0]['valid'])
    def test_no_eye_roi_disables_fallback(self):
        p=PuReST(1);self.addCleanup(p.close)
        frame=np.full((240,320,3),180,np.uint8)
        cv2.ellipse(frame,(160,120),(25,20),15,0,360,(10,10,10),-1)
        self.assertFalse(p.infer([frame],[None],[1.])[0]['valid'])

if __name__=='__main__':unittest.main()
