"""Exercise capture timing, profile reuse, rejected data and atomic persistence."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from gaze_calibration import CalibrationStore, GazeCalibration


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'calibration.json'
        self.store = CalibrationStore(self.path)
        self.bindings = [dict(camera='usb-A', rotation=180, model=['hash'], width=640),
                         dict(camera='usb-B', rotation=0, model=['hash'], width=640)]
        self.points = [dict(left=.3, center=.5, right=.7),
                       dict(left=.7, center=.5, right=.3)]
        self.now = 1.

    def session(self):
        return GazeCalibration(self.store, self.bindings, count=10, settle=.2)

    def feed(self, s, positions, count=10, states=None, valid=True):
        for _ in range(count):
            self.now += .04
            s.observe(positions, states or ['open','open'], self.now, valid)

    def capture(self, s, name):
        self.assertTrue(s.begin(name, self.now))
        self.now += .21
        self.feed(s, [p[name] for p in self.points])

    def seed(self):
        self.store.save(self.bindings, self.points, [{},{}], complete=True)

    def test_only_frames_after_key_and_settling_are_counted(self):
        s = self.session()
        self.feed(s, [.9,.9], 20)
        self.assertEqual(s.values, [{},{}])
        self.assertTrue(s.begin('left', self.now))
        self.feed(s, [.9,.9], 4)  # Before the 0.2s settling deadline.
        self.now += .1
        self.feed(s, [.3,.7], 9)
        self.assertEqual(s.active, 'left')
        self.feed(s, [.3,.7], 1)
        self.assertEqual(s.values, [dict(left=.3),dict(left=.7)])
        self.assertEqual(s.expected, 'center')
        self.assertTrue(self.path.exists())

    def test_restart_loads_finished_profiles_and_mirrored_eye(self):
        s = self.session()
        for name in ('left','center','right'):
            self.capture(s, name)
        self.assertTrue(s.ready)
        reloaded = self.session()
        self.assertTrue(reloaded.ready)
        self.assertEqual(reloaded.loaded, 2)
        self.assertEqual(reloaded.values, self.points)
        self.assertEqual(json.loads(self.path.read_text())['drafts'], {})

    def test_draft_resumes_without_publishing_as_complete(self):
        s = self.session(); self.capture(s, 'left')
        resumed = self.session()
        self.assertFalse(resumed.ready)
        self.assertEqual(resumed.expected, 'center')
        self.assertEqual(resumed.values[0], dict(left=.3))

    def test_invalid_closed_and_unsynced_frames_do_not_count(self):
        s = self.session(); s.begin('left', self.now); self.now += .21
        self.feed(s, [.3,None], states=['open','closed'], count=20)
        self.feed(s, [.3,.7], valid=False, count=20)
        self.feed(s, [.3,float('nan')], count=20)
        self.assertEqual(len(s.buffers[0]), 0)
        self.feed(s, [.3,.7])
        self.assertEqual(s.expected, 'center')

    def test_moving_gaze_waits_for_stability(self):
        s = self.session(); s.begin('left', self.now); self.now += .21
        for i in range(20):self.feed(s, [.2 if i%2 else .7,.7],count=1)
        self.assertEqual(s.active, 'left')
        self.feed(s, [.3,.7])
        self.assertEqual(s.expected, 'center')

    def test_wrong_key_and_repeat_do_not_restart_sampling(self):
        s = self.session()
        self.assertFalse(s.begin('right', self.now))
        s.begin('left', self.now); self.now += .21
        self.feed(s, [.3,.7], count=5)
        self.assertFalse(s.begin('left', self.now))
        self.assertFalse(s.begin('center', self.now))
        self.feed(s, [.3,.7], count=5)
        self.assertEqual(s.expected, 'center')

    def test_invalid_range_does_not_overwrite_previous_good_profile(self):
        self.seed(); original=self.path.read_text()
        s=self.session(); s.restart()
        self.capture(s,'left');self.capture(s,'center')
        s.begin('right',self.now);self.now+=.21;self.feed(s,[.4,.6])
        self.assertFalse(s.ready)
        self.assertEqual(s.expected,'right')
        self.assertEqual(self.session().values,self.points)
        self.assertEqual(json.loads(original)['profiles'],json.loads(self.path.read_text())['profiles'])

    def test_changed_camera_rotation_or_model_requires_new_calibration(self):
        self.seed()
        for field,new in [('camera','usb-C'),('rotation',90),('model',['new-hash']),('width',320)]:
            bindings=[dict(self.bindings[0],**{field:new}),self.bindings[1]]
            changed=GazeCalibration(self.store,bindings,count=10)
            self.assertFalse(changed.ready)
            self.assertEqual(changed.loaded,1)

    def test_single_and_dual_mode_share_camera_profiles(self):
        self.store.save(self.bindings[:1],self.points[:1],[{}],complete=True)
        single=GazeCalibration(self.store,self.bindings[:1])
        self.assertTrue(single.ready)
        dual=self.session()
        for name in ('left','center','right'):
            dual.begin(name,self.now);self.now+=.21
            self.assertEqual(dual.targets,[1])
            self.feed(dual,[None,self.points[1][name]],states=['closed','open'])
        self.assertTrue(dual.ready)
        self.assertEqual(dual.values,self.points)

    def test_atomic_write_failure_disarms_until_retry(self):
        self.seed(); old=self.path.read_bytes()
        s=self.session();s.restart()
        s.values=[dict(left=.25,center=.5,right=.75),self.points[1]]
        s.pending_save=True
        with patch('gaze_calibration.os.replace',side_effect=OSError('disk failure')):
            self.assertFalse(s.save_pending())
        self.assertEqual(self.path.read_bytes(),old)
        self.assertFalse(s.ready)
        self.assertTrue(s.save_pending())
        self.assertTrue(s.ready)
        self.assertEqual(self.session().values[0]['left'],.25)

    def test_corrupt_file_is_not_loaded(self):
        self.path.write_text('{broken')
        s=self.session()
        self.assertFalse(s.ready)
        self.assertTrue(self.store.warning)

if __name__=='__main__':unittest.main()
