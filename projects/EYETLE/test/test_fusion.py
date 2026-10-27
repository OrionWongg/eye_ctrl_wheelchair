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
    def test_detector_conflict_stops_and_one_missing_eye_falls_back(self):
        self.complete();q=np.ones((2,2))
        bad=self.values['center'].copy();bad[0]=[.35,.66]
        self.assertEqual(self.cal.horizontal(bad,q)[3],'DETECTORS_DISAGREE')
        q[0]=0
        h,ok,eyes,_=self.cal.horizontal(self.values['left'],q)
        self.assertTrue(ok);self.assertEqual(len(eyes),1);self.assertAlmostEqual(h,-1)
        self.assertEqual(self.cal.horizontal(self.values['center'],np.zeros((2,2)))[3],'NO_DIRECTION_FEATURE')
    def test_opposite_turned_eyes_disagree_but_amplitude_mismatch_passes(self):
        self.complete();q=np.ones((2,2))
        mixed=np.array([self.values['left'][0],self.values['right'][1]])
        self.assertEqual(self.cal.horizontal(mixed,q)[3],'EYES_DISAGREE')
        weak=np.array([self.values['left'][0],(self.values['left'][1]+self.values['center'][1])/2])
        h,ok,_,_=self.cal.horizontal(weak,q)
        self.assertTrue(ok);self.assertLess(h,-.6)
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


class CenterOnlyTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'unused.json'
        self.cal=GuidedCalibration(self.path,{'cameras':['l','r']},2,hold=.6,settle=.1,
                                   min_samples=8,center_only=True,span=.2)
        self.now=0.
    def look(self,value,n=40):
        for _ in range(n):
            self.now+=.05;self.cal.observe(np.asarray(value,float),np.ones((2,2)),['open']*2,self.now,True)
    def test_single_center_fixation_is_ready_and_writes_nothing(self):
        self.assertEqual(self.cal.steps,['center'])
        center=[[.55,.55],[.45,.45]];self.look(center)
        self.assertTrue(self.cal.ready);self.assertFalse(self.path.exists())
        q=np.ones((2,2))
        self.assertAlmostEqual(self.cal.horizontal(np.array(center),q)[0],0)
        # Looking left raises full-frame pupil x on both mounted cameras.
        self.assertAlmostEqual(self.cal.horizontal(np.array(center)+.2,q)[0],-1)
        self.assertAlmostEqual(self.cal.horizontal(np.array(center)-.1,q)[0],.5)
        self.cal.restart(full=True)
        self.assertFalse(self.cal.ready);self.assertEqual(self.cal.phase,'center')
    def test_unstable_center_is_not_accepted(self):
        for i in range(60):
            self.now+=.05;v=.5+(.05 if i%2 else -.05)
            self.cal.observe(np.full((2,2),v),np.ones((2,2)),['open']*2,self.now,True)
        self.assertFalse(self.cal.ready)
    def test_invalid_span_rejected(self):
        with self.assertRaises(ValueError):
            GuidedCalibration(self.path,{},2,center_only=True,span=0)


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


class SavedSimulationProfileTest(unittest.TestCase):
    def test_known_hub_move_reuses_exact_points_without_writing(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'profile.json'
            old = {'cameras': [f'/dev/v4l/by-path/usb-0:4.4.{i}:1.0-video-index0'
                               for i in (1, 2)], 'rotations': [180, 0]}
            current = dict(old, cameras=[p.replace('4.4.', '4.1.') for p in old['cameras']])
            data = dict(schema='eye-fusion-v1', binding=old,
                        points=[[[.6, .6], [.6, .6]], [[.5, .5], [.5, .5]],
                                [[.4, .4], [.4, .4]]], selected=[[True, True], [True, True]])
            original = json.dumps(data).encode()
            path.write_bytes(original)
            cal = GuidedCalibration(path, current, 2, saved_only=True, allow_hub_move=True)
            self.assertTrue(cal.loaded)
            self.assertTrue(cal.binding_remapped)
            self.assertEqual(cal.steps, ['confirm'])
            self.assertEqual(cal.profile['points'], data['points'])
            cal.restart(full=True)
            self.assertEqual(cal.steps, ['confirm'])
            cal.pending_save = True
            cal.save_pending()
            self.assertEqual(path.read_bytes(), original)
            for binding in (dict(current, rotations=[0, 0]),
                            dict(current, cameras=current['cameras'][::-1])):
                with self.assertRaises(ValueError):
                    GuidedCalibration(path, binding, 2, saved_only=True, allow_hub_move=True)
            with self.assertRaises(ValueError):
                GuidedCalibration(path, current, 2, saved_only=True)
