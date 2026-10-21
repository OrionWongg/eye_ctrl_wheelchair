"""Synthetic eye images -> binocular state -> real controller callbacks, no cameras."""
import importlib.util
from pathlib import Path
import unittest
import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray
from eye_turtle_control.controller import EyeTurtleController, Motion

spec = importlib.util.spec_from_file_location('dual', Path(__file__).parents[1]/'tools/dual_eye_tracker.py')
dual = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dual)


def picture(closed=False):
    f = np.full((120,240,3),150,np.uint8)
    if closed:
        cv2.line(f,(30,60),(210,60),(55,55,55),5)
    else:
        cv2.ellipse(f,(120,60),(90,32),0,0,360,(230,230,230),-1)
        cv2.circle(f,(120,60),19,(10,10,10),-1)
    return f


class BlinkIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.frames = {'open':picture(), 'closed':picture(True), 'unknown':np.zeros((120,240,3),np.uint8)}
        self.roi = (0,0,240,120)
        self.model = dual.EyeAppearance()
        opened = dual.eye_patch(self.frames['open'],self.roi)
        self.model.open = {k:opened for k in ('left','center','right')}
        self.assertTrue(self.model.set_closed([dual.eye_patch(self.frames['closed'],self.roi)]*20))
        self.binocular = dual.BinocularState()
        self.node = EyeTurtleController()
        self.now = 10.
        class Clock:
            def now(clock):return Time(nanoseconds=int(self.now*1e9))
        self.node.get_clock = lambda:Clock()
        self.node.last_signal = self.node.get_clock().now()
        self.feed(.2)

    def tearDown(self):self.node.destroy_node()

    def feed(self, duration, left='open', right=None):
        states=[]
        for name in (left,right or left):
            f=self.frames[name]
            states.append(self.model.classify(dual.eye_patch(f,self.roi),dual.find_pupil(f,self.roi)))
        for _ in range(round(duration/.02)):
            self.now += .02
            closed, valid = self.binocular.update(states,self.now,True)
            self.node.signal_cb(Float32MultiArray(data=[0.,float(closed),float(valid)]))
            self.node.control_loop()

    def blinks(self,count):
        for _ in range(count):
            self.feed(.12,'closed'); self.feed(.28)

    def test_double_blink_forward_long_close_stop(self):
        self.blinks(2)
        self.assertEqual(self.node.motion,Motion.FORWARD)
        self.feed(.72,'closed')
        self.assertEqual(self.node.motion,Motion.STOPPED)

    def test_teyed_auto_center_double_blink_pause_and_resume(self):
        import sys
        sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
        from teyed_backend import BinocularInput
        from ritnet_blink import RitnetBlink
        from test_ritnet_blink import SyntheticRitnet
        segmentation=SyntheticRitnet()
        detector=RitnetBlink(None,None,None,2,model=segmentation)
        fusion=BinocularInput(2,[-1,-1])
        def feed(seconds,closed=False):
            for _ in range(round(seconds/.02)):
                self.now+=.02
                values=[[float('nan')]*2]*2 if closed else [[.1,0],[-.1,0]]
                segmentation.closed=[closed]*2
                states,_=detector.infer([segmentation.frame]*2,[self.now]*2)
                h,c,v,_=fusion.update(values,states,self.now,True)
                self.node.signal_cb(Float32MultiArray(data=[h,float(c),float(v)]))
                self.node.control_loop()
        feed(2.2)
        self.assertTrue(fusion.center.ready)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        feed(.12,True);feed(.28);feed(.12,True);feed(.1)
        self.assertEqual(self.node.motion,Motion.FORWARD)
        feed(.72,True);feed(.2)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        feed(.12,True);feed(.1)
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def signals(self, seconds, horizontal=0., closed=False, valid=True):
        for _ in range(round(seconds/.02)):
            self.now += .02
            self.binocular.update(['closed' if closed else 'open']*2,self.now,valid)
            self.node.signal_cb(Float32MultiArray(data=[float(horizontal),float(closed),float(valid)]))
            self.node.control_loop()

    def test_guided_fusion_drives_real_controller_and_stops_on_loss(self):
        import sys,tempfile
        sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
        from fusion_calibration import GuidedCalibration
        from fusion_control import FusionControl
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        cal=GuidedCalibration(Path(tmp.name)/'profile.json',{},2,hold=.6,settle=.1,min_samples=8)
        f=FusionControl(cal)
        self.node.deadzone=.55;self.node.turn_release=.25
        samples={'center':[[.5,.5],[.5,.5]],'left':[[.35,.35],[.65,.65]],'right':[[.65,.65],[.35,.35]]}
        samples.update(slight_left=[[.44,.44],[.56,.56]],slight_right=[[.56,.56],[.44,.44]])
        def feed(seconds,closed=False,lost=False,direction='center',states=None):
            for _ in range(round(seconds/.02)):
                self.now+=.02
                h,c,v,_=f.update(samples[direction],np.ones((2,2)),states or ['closed' if closed else 'open']*2,self.now,not lost)
                self.node.signal_cb(Float32MultiArray(data=[h,float(c),float(v)]));self.node.control_loop()
        for _ in range(500):
            if cal.ready:break
            feed(.02,direction=cal.phase if cal.phase in samples else 'center')
        self.assertTrue(cal.ready);self.assertEqual(self.node.motion,Motion.STOPPED)
        for _ in range(2):
            feed(.06,states=['closed','open'])
            feed(.06,states=['unknown','unknown'])
            feed(.12,True);feed(.2)
        self.assertEqual(self.node.motion,Motion.FORWARD)
        feed(.2,direction='slight_left');self.assertEqual(self.node.turn,0)
        feed(.2,direction='slight_right');self.assertEqual(self.node.turn,0)
        feed(.1,direction='left');self.assertEqual(self.node.turn,-1)
        feed(.02,lost=True);feed(.2)
        self.assertEqual(self.node.motion,Motion.STOPPED)

    def resume(self):
        self.signals(.1)
        self.signals(.12,closed=True)
        self.signals(.1)

    def test_single_blink_resume_only_after_session_enabled(self):
        self.resume()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.blinks(2);self.signals(.72,closed=True)
        self.assertTrue(self.node.session_enabled)
        self.signals(2.)
        self.signals(.3,-.7);self.signals(.8)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.resume()
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_healthy_motion_persists_beyond_five_seconds(self):
        self.blinks(2);self.signals(12.)
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_resume_progress_and_invalid_cancellation(self):
        self.blinks(2);self.signals(.72,closed=True);self.signals(.1)
        self.signals(.12,closed=True)
        self.assertEqual(self.node.resume.phase,"REOPEN_TO_RESUME")
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.signals(.02,valid=False);self.signals(1.)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.resume()
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_watchdog_and_emergency_preserved(self):
        self.blinks(2);self.now+=.5;self.node.control_loop()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.signals(1.)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.resume()
        self.assertEqual(self.node.motion,Motion.FORWARD)
        self.node.stop_cb(Bool(data=True));self.resume()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.blinks(2)
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_fast_double_blink_dual_mode(self):
        self.node.blink_cooldown = .12
        for _ in range(2):
            self.feed(.1,'closed'); self.feed(.12)
        self.feed(.9)
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_no_closure_before_first_open(self):
        b=dual.BinocularState()
        self.assertEqual(b.update(['closed','closed'],0,True),(False,False))
        self.assertEqual(b.update(['open','open'],.1,True),(False,True))
        self.assertEqual(b.update(['closed','closed'],.2,True),(True,True))

    def test_third_blink_keeps_forward(self):
        self.blinks(3)
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_single_blink_and_brief_noise_do_not_start(self):
        self.blinks(1);self.feed(1.)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.feed(.02,'closed');self.feed(1.)
        self.assertEqual(self.node.motion,Motion.STOPPED)

    def test_occlusion_interrupts_pending_double_blink(self):
        self.blinks(1); self.feed(.2,'unknown'); self.blinks(1); self.feed(1.)
        self.assertEqual(self.node.motion,Motion.STOPPED)

    def test_winks_do_not_start(self):
        for _ in range(2):self.feed(.14,'closed','open');self.feed(.28)
        self.feed(1.)
        self.assertEqual(self.node.motion,Motion.STOPPED)

    def test_loss_stops_motion(self):
        self.blinks(2);self.feed(.7)
        self.assertEqual(self.node.motion,Motion.FORWARD)
        self.feed(.2,'unknown')
        self.assertEqual(self.node.motion,Motion.STOPPED)

    def test_bad_calibration_and_unclassified_missing_pupil(self):
        opened=dual.eye_patch(self.frames['open'],self.roi)
        self.assertEqual(self.model.classify(opened,None),'unknown')
        self.assertFalse(self.model.set_closed([opened]*20))
        self.assertFalse(self.model.ready)

    def test_transition_grace_expires_and_calibration_disarms(self):
        b=dual.BinocularState()
        self.assertEqual(b.update(['open','open'],0,True),(False,True))
        self.assertEqual(b.update(['closed','open'],.04,True),(False,True))
        self.assertEqual(b.update(['closed','open'],.12,True),(False,False))
        self.assertEqual(b.update(['closed','closed'],.14,False),(False,False))

if __name__ == '__main__':unittest.main()
