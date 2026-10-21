import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from teyed_backend import AutoCenter, BinocularInput, eye_state

class TeyedTest(unittest.TestCase):
    def ready(self):
        f=BinocularInput(2,[-1,-1])
        for i in range(61):f.update([[.1,0],[-.1,0]],['open']*2,i*.05,True)
        self.assertTrue(f.center.ready)
        return f

    def test_auto_center_needs_stable_open_interval(self):
        c=AutoCenter(2)
        for i in range(60):c.update([[i%2*.2,0],[0,0]],['open']*2,i*.05,True)
        self.assertFalse(c.ready)
        for i in range(40):c.update([[0,0],[0,0]],['closed']*2,3+i*.05,True)
        self.assertFalse(c.ready)
        for i in range(42):c.update([[0,0],[0,0]],['open']*2,5+i*.05,True)
        self.assertTrue(c.ready)

    def test_stale_nonfinite_and_gap_cannot_complete_center(self):
        for kind in ('stale','nan','gap'):
            c=AutoCenter(2)
            for i in range(40):c.update([[0,0],[0,0]],['open']*2,i*.05,True)
            c.update([[np.nan if kind=='nan' else 0,0],[0,0]],['open']*2,3 if kind=='gap' else 2,kind!='stale')
            self.assertFalse(c.ready);self.assertEqual(c.progress,0.)

    def test_center_does_not_drift_while_looking_left(self):
        f=self.ready();center=f.center.center.copy()
        for i in range(100):
            h,closed,valid,_=f.update([[.3,0],[.1,0]],['open']*2,3.05+i*.05,True)
            self.assertTrue(valid);self.assertAlmostEqual(h,-.8)
        np.testing.assert_array_equal(f.center.center,center)

    def test_closed_gaze_not_needed_but_recent_valid_open_is(self):
        f=self.ready()
        for i in range(1,16):
            h,closed,valid,_=f.update([[np.nan,np.nan]]*2,['closed']*2,3+i*.05,True)
            self.assertTrue(closed);self.assertTrue(valid)
        self.assertFalse(f.update([[0,0]]*2,['closed']*2,4.3,True)[2])

    def test_conflict_and_stale_stop_and_cancel_blink(self):
        for values,fresh in [([[.8,0],[-.8,0]],True),([[.1,0],[-.1,0]],False)]:
            f=self.ready()
            self.assertFalse(f.update(values,['open']*2,3.05,fresh)[2])
            self.assertFalse(f.update([[0,0]]*2,['closed']*2,3.1,True)[2])

    def test_reset_requires_new_center(self):
        f=self.ready();f.reset()
        self.assertFalse(f.update([[0,0]]*2,['closed']*2,4,True)[2])
        self.assertFalse(f.center.ready)

    def test_eye_classification_rejects_missing_and_ambiguous(self):
        self.assertEqual(eye_state([.99,.01]),'open')
        self.assertEqual(eye_state([.99,.99]),'closed')
        for tags in ([.2,.99],[.99,.275],[np.nan,0],[1.1,0]):self.assertEqual(eye_state(tags),'unknown')

    def test_lower_threshold_accepts_confident_eye_without_accepting_tracking_loss(self):
        self.assertEqual(eye_state([.913574,.407714]),'closed')
        self.assertEqual(eye_state([.913574,.407714],.8),'unknown')
        self.assertEqual(eye_state([.136596,.050231]),'unknown')
        self.assertEqual(eye_state([.1,.9]),'unknown')
        self.assertEqual(eye_state([.99,.35]),'closed')
        self.assertEqual(eye_state([.99,.2]),'open')
        for threshold in (.2,0.,1.1,float('nan')):
            with self.assertRaises(ValueError):eye_state([.99,.4],threshold)

if __name__=='__main__':unittest.main()
