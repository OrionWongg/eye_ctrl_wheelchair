"""Actual per-eye matcher -> new ROS sample callback -> turtle state machine."""
import json
from pathlib import Path
import sys
import unittest
import rclpy
from rclpy.time import Time
from std_msgs.msg import String,Bool
from eye_turtle_control.controller import EyeTurtleController,Motion
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from blink_events import BlinkEvents


class FusionEventsControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args','-p','blink_input_mode:=events','-p','blink_cooldown_ms:=120.0'])
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.node=EyeTurtleController();self.now=10.;self.seq=0;self.detector=BlinkEvents(2)
        class Clock:
            def now(clock):return Time(nanoseconds=round(self.now*1e9))
        self.node.get_clock=lambda:Clock();self.node.last_signal=self.node.get_clock().now()
        self.last=None
        self.send()
    def tearDown(self):self.node.destroy_node()
    def send(self,states=None,motion=True,step=.04,fresh=True,alter=None):
        self.now+=step;self.seq+=1;states=states or ['open']*2
        event,long=self.detector.update(states,self.now,fresh)
        d=dict(schema=1,session='test-session',sequence=self.seq,stamp_ns=round(self.now*1e9),source_t=self.now,
            horizontal=0.,motion_valid=motion and states==['open']*2,fresh=fresh,calibrated=True,
            open=states==['open']*2,gesture_continuity=self.detector.continuity,long_close=long,blink_event=event)
        if alter:d.update(alter)
        self.last=d
        self.node.fusion_cb(String(data=json.dumps(d)));self.node.control_loop()
    def blink(self,end_motion=True):
        # Closure intersection is only 50 ms; each eye is closed >=120 ms.
        self.send(['closed','open'],step=.04)
        self.send(['closed','closed'],step=.08)
        self.send(['open','closed'],step=.05)
        self.send(step=.07,motion=end_motion)
    def arm(self):
        self.blink();self.send(step=.12);self.blink()
        self.assertEqual(self.node.motion,Motion.FORWARD)

    def test_short_overlap_double_blink_arms(self):
        self.assertEqual(self.node.sub.topic_name,'/eye_tracking/fusion_samples')
        self.arm()
        self.assertEqual(self.node.pause_reason,'DOUBLE_BLINK_CONFIRMED')
    def test_direction_loss_does_not_erase_first_blink(self):
        self.blink();self.assertEqual(len(self.node.blinks),1)
        self.send(motion=False,step=.08)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.assertEqual(len(self.node.blinks),1)
        self.blink();self.assertEqual(self.node.motion,Motion.FORWARD)
    def test_second_event_waits_briefly_for_current_valid_open_direction(self):
        self.blink();self.send(step=.12);self.blink(end_motion=False)
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.send(step=.08)
        self.assertEqual(self.node.motion,Motion.FORWARD)
    def test_pending_authorization_expires_instead_of_late_restart(self):
        self.blink();self.send(step=.12);self.blink(end_motion=False)
        for _ in range(4):self.send(motion=False,step=.08)
        self.send()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.assertFalse(self.node.session_enabled)
    def test_stale_and_persistent_unknown_cancel_count(self):
        self.blink();self.send(fresh=False)
        self.assertEqual(len(self.node.blinks),0)
        self.send();self.blink()
        self.send(['unknown']*2);self.send(['unknown']*2,step=.16)
        self.assertEqual(len(self.node.blinks),0)
        self.send();self.blink()
        self.assertEqual(self.node.motion,Motion.STOPPED)
    def test_duplicate_event_and_replayed_sample_cannot_arm(self):
        self.blink();saved=dict(self.last)
        self.node.fusion_cb(String(data=json.dumps(saved)))
        self.assertEqual(len(self.node.blinks),1)
        self.send(alter={'blink_event':saved['blink_event']})
        self.assertEqual(len(self.node.blinks),1)
        self.assertEqual(self.node.motion,Motion.STOPPED)
    def test_long_close_pauses_and_single_complete_blink_resumes(self):
        self.arm()
        for _ in range(20):self.send(['closed']*2)
        self.send()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.assertTrue(self.node.session_enabled)
        self.blink();self.assertEqual(self.node.motion,Motion.FORWARD)
    def test_emergency_stop_discards_old_gesture_and_requires_new_double(self):
        self.arm();self.node.stop_cb(Bool(data=True))
        self.send();self.blink()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.send(step=.12);self.blink()
        self.assertEqual(self.node.motion,Motion.FORWARD)
    def test_timeout_stops_motion_and_requires_fresh_blink(self):
        self.arm();self.now+=.5;self.node.control_loop();self.send()
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.blink();self.assertEqual(self.node.motion,Motion.FORWARD)
    def delayed_recovery_event(self,emergency=False):
        self.arm()
        # Loss captured 180 ms ago arrives after the next blink has begun.
        loss_capture=self.now-.18
        self.send(alter={'stamp_ns':round(loss_capture*1e9),
                         'gesture_continuity':False})
        if emergency:self.node.stop_cb(Bool(data=True))
        event_start=loss_capture+.05
        event_end=event_start+.12
        event=dict(id=50,start=event_start,end=event_end,durations=[.12,.12])
        self.send(alter={'blink_event':event})

    def test_delayed_loss_uses_capture_time_for_new_blink(self):
        self.delayed_recovery_event()
        self.assertEqual(self.node.motion,Motion.FORWARD)
        self.assertEqual(self.node.event_decision,'ACCEPTED')

    def test_delayed_loss_cannot_weaken_emergency_barrier(self):
        self.arm();self.node.stop_cb(Bool(data=True))
        barrier=self.node.gesture_barrier
        self.send(alter={'stamp_ns':round((self.now-.18)*1e9),
                         'gesture_continuity':False})
        self.assertGreaterEqual(self.node.gesture_barrier,barrier)
        event=dict(id=50,start=barrier-.14,end=barrier-.02,durations=[.12,.12])
        self.send(alter={'blink_event':event})
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.assertEqual(self.node.event_decision,'REJECT_BEFORE_STOP')

    def test_malformed_sample_stops_without_crashing(self):
        self.arm()
        self.node.fusion_cb(String(data='[]'))
        self.assertEqual(self.node.motion,Motion.STOPPED)
        self.assertEqual(self.node.pause_reason,'INVALID_FUSION_SAMPLE')

if __name__=='__main__':unittest.main()
