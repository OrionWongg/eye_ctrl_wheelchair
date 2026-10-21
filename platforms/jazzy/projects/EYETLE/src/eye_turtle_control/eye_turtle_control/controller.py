#!/usr/bin/env python3
"""Safety-oriented eye signal to turtlesim velocity controller."""
from enum import Enum
import math
import json
from .resume import ResumeGesture

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, Float32MultiArray, String
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute


class Motion(Enum):
    STOPPED = 0
    FORWARD = 1


class EyeTurtleController(Node):
    """Input layout: [horizontal, eye_closed, valid].

    horizontal is normalized to [-1, 1] (left negative, right positive),
    eye_closed and valid are 0/1. Short blinks resume an enabled paused session.
    """
    def __init__(self):
        super().__init__('eye_turtle_controller')
        self.declare_parameter('input_topic', '/eye_tracking/signals')
        self.declare_parameter('cmd_vel_topic', '/turtle1/cmd_vel')
        self.declare_parameter('horizontal_deadzone', 0.25)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 2.0)
        self.declare_parameter('long_blink_ms', 650.0)
        self.declare_parameter('multi_blink_window_ms', 900.0)
        self.declare_parameter('blink_min_ms', 60.0)
        self.declare_parameter('signal_timeout_ms', 400.0)
        self.declare_parameter('blink_max_ms', 400.0)
        self.declare_parameter('turn_release_threshold', 0.15)
        self.declare_parameter('blink_cooldown_ms', 300.0)
        self.declare_parameter('turn_pulse_duration_s', 1.0)
        self.declare_parameter('wall_margin', 0.35)
        self.declare_parameter('arena_min', 0.0)
        self.declare_parameter('arena_max', 11.0)
        self.declare_parameter('recenter_cooldown_s', 2.0)
        self.declare_parameter('control_mode', 'twist')
        self.declare_parameter('blink_input_mode', 'legacy')
        p = self.get_parameter
        self.deadzone = float(p('horizontal_deadzone').value)
        self.linear_speed = float(p('linear_speed').value)
        self.angular_speed = float(p('angular_speed').value)
        self.long_blink = float(p('long_blink_ms').value) / 1000.0
        self.blink_window = float(p('multi_blink_window_ms').value) / 1000.0
        self.blink_min = float(p('blink_min_ms').value) / 1000.0
        self.timeout = float(p('signal_timeout_ms').value) / 1000.0
        self.resume = ResumeGesture(self.blink_min, float(p('blink_max_ms').value)/1000., self.timeout)
        self.session_enabled = False
        self.pause_reason = 'STARTUP'
        self.turn_release = float(p('turn_release_threshold').value)
        self.blink_cooldown = float(p('blink_cooldown_ms').value) / 1000.0
        self.turn_pulse_duration = float(p('turn_pulse_duration_s').value)
        self.wall_margin = float(p('wall_margin').value)
        self.arena_min = float(p('arena_min').value)
        self.arena_max = float(p('arena_max').value)
        self.recenter_cooldown = float(p('recenter_cooldown_s').value)
        self.control_mode = str(p('control_mode').value).lower()
        self.blink_input_mode = str(p('blink_input_mode').value)
        if self.blink_input_mode not in ('legacy','events'):raise ValueError('Invalid blink_input_mode')
        self.fusion_session=None;self.fusion_sequence=0;self.fusion_event_id=0
        self.pending_blink_authorization=None;self.gesture_barrier=-float('inf')
        self.event_decision='NONE'
        self.keyboard = None
        self.last_key = None
        if self.control_mode == 'keyboard':
            try:
                from pynput.keyboard import Controller
                self.keyboard = Controller()
            except ImportError as exc:
                raise RuntimeError('control_mode=keyboard requires pynput: python3 -m pip install pynput') from exc
        self.motion = Motion.STOPPED
        self.horizontal = 0.0
        self.closed_since = None
        self.blinks = []
        self.last_signal = self.get_clock().now()
        self.was_closed = False
        self.motion_started = None
        self.last_blink = -float('inf')
        self.turn = 0
        self.turn_until = 0.0
        self.turn_armed = True
        self.last_recenter = -float('inf')
        self.pub = self.create_publisher(Twist, str(p('cmd_vel_topic').value), 10)
        if self.blink_input_mode=='events':
            self.sub=self.create_subscription(String,'/eye_tracking/fusion_samples',self.fusion_cb,1)
        else:self.sub = self.create_subscription(Float32MultiArray, str(p('input_topic').value), self.signal_cb, 10)
        self.stop_sub = self.create_subscription(Bool, '/eye_turtle/stop', self.stop_cb, 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.feedback = self.create_publisher(String, '/eye_control/feedback', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def signal_cb(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warning('Expected [horizontal, eye_closed, valid]')
            self.pause('INVALID_INPUT')
            return
        values = [float(value) for value in msg.data[:3]]
        if not all(math.isfinite(value) for value in values):
            self.pause('INVALID_INPUT')
            return
        horizontal, closed, valid = values[0], bool(values[1]), bool(values[2])
        if (self.get_clock().now()-self.last_signal).nanoseconds/1e9 > self.timeout:
            self.pause('SIGNAL_TIMEOUT')
        self.last_signal = self.get_clock().now()
        if not valid:
            self.pause('INVALID_INPUT')
            return
        self.horizontal = max(-1.0, min(1.0, horizontal))
        now = self.get_clock().now().nanoseconds / 1e9
        if closed and not self.was_closed:
            self.closed_since = now
        elif closed and self.closed_since is not None and now - self.closed_since >= self.long_blink:
            # Stop as soon as an intentional sustained close is confirmed.
            self.pause('LONG_EYE_CLOSE', keep_closure=True)
        elif not closed and self.was_closed and self.closed_since is not None:
            duration = now - self.closed_since
            if duration >= self.long_blink:
                self.pause('LONG_EYE_CLOSE', keep_closure=True)
            elif duration >= self.blink_min:
                if not self.session_enabled and now - self.last_blink < self.blink_cooldown:
                    self.was_closed = closed
                    return
                self.last_blink = now
                self.blinks = [t for t in self.blinks if now - t <= self.blink_window]
                self.blinks.append(now)
                if not self.session_enabled and len(self.blinks) >= 2:
                    self.session_enabled = True
                    self.pause_reason = 'DOUBLE_BLINK_CONFIRMED'
                    self.resume.reset()
                    self.motion = Motion.FORWARD
                    self.motion_started = now
                    self.blinks.clear()
        self.was_closed = closed
        if self.session_enabled and self.motion == Motion.STOPPED:
            if self.resume.update(now, self.horizontal, closed):
                self.motion = Motion.FORWARD
                self.motion_started = now
                self.pause_reason = 'SINGLE_BLINK_RESUME_CONFIRMED'
                self.turn = 0; self.turn_armed = True
                self.resume.reset(); self.blinks.clear()

    def fusion_cb(self,msg):
        """One atomic sample carries independent motion quality and blink events."""
        if self.blink_input_mode!='events':return
        now=self.get_clock().now().nanoseconds/1e9
        try:
            d=json.loads(msg.data)
            if not isinstance(d,dict) or d.get('schema')!=1:raise ValueError('schema')
            session=d['session'];seq=d['sequence'];stamp=d['stamp_ns']/1e9
            source_t=float(d['source_t']);h=float(d['horizontal'])
            if not isinstance(session,str) or not session or len(session)>128 or type(seq) is not int or seq<1:raise ValueError('source')
            if not all(math.isfinite(x) for x in (stamp,source_t,h)) or abs(h)>1:raise ValueError('nonfinite')
            for name in ('fresh','calibrated','motion_valid','open','gesture_continuity','long_close'):
                if type(d[name]) is not bool:raise ValueError('boolean')
            event=d.get('blink_event')
            if event is not None:
                durations=event['durations'];start=float(event['start']);end=float(event['end'])
                if (type(event['id']) is not int or event['id']<1 or not isinstance(durations,list) or len(durations) not in (1,2)
                    or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and .06<=v<=.4 for v in durations)
                    or not math.isfinite(start) or not math.isfinite(end) or not 0<end-start<=.58
                    or not 0<=source_t-end<=.25):raise ValueError('blink event')
        except (ValueError,KeyError,TypeError,OverflowError):
            self.pause('INVALID_FUSION_SAMPLE');return
        if session!=self.fusion_session:
            self.pause('SOURCE_CHANGED');self.session_enabled=False
            self.fusion_session=session;self.fusion_sequence=0;self.fusion_event_id=0
        if seq<=self.fusion_sequence:return  # Replays never refresh the watchdog.
        self.fusion_sequence=seq
        if now-self.last_signal.nanoseconds/1e9>self.timeout:self.pause('SIGNAL_TIMEOUT')
        self.last_signal=self.get_clock().now()
        age=now-stamp
        if not -.05<=age<=.25 or not d['fresh'] or not d['calibrated']:
            self.pause('STALE_OR_UNCALIBRATED');return
        if not d['gesture_continuity']:
            self.pause('GESTURE_TRACKING_LOST',barrier_at=stamp)
        if d['long_close']:
            self.pause('LONG_EYE_CLOSE');return
        motion_ready=d['motion_valid'] and d['open']
        self.horizontal=h
        if not motion_ready:
            # Stop immediately, while preserving only bounded, completed events.
            self.pause('INVALID_INPUT',keep_event_history=True)
        self.blinks=[t for t in self.blinks if 0<=now-t<=self.blink_window]
        if self.pending_blink_authorization is not None and now-self.pending_blink_authorization[0]>.25:
            self.pending_blink_authorization=None;self.blinks.clear()
            self.event_decision='AUTHORIZATION_EXPIRED'
        if event is not None and event['id']>self.fusion_event_id:
            self.fusion_event_id=event['id']
            event_start=stamp-(source_t-start);event_end=stamp-(source_t-end)
            if not d['gesture_continuity']:
                self.event_decision='REJECT_CONTINUITY'
            elif event_start<self.gesture_barrier:
                self.event_decision='REJECT_BEFORE_STOP'
            elif not 0<=now-event_end<=.25:
                self.event_decision='REJECT_EVENT_AGE'
            elif self.blinks and event_end-self.blinks[-1]<self.blink_cooldown:
                self.event_decision='REJECT_COOLDOWN'
            else:
                self.event_decision='ACCEPTED'
                self.blinks.append(event_end)
                if not self.session_enabled and len(self.blinks)>=2:
                    self.pending_blink_authorization=(event_end,'DOUBLE_BLINK_CONFIRMED')
                elif self.session_enabled and self.motion==Motion.STOPPED:
                    self.pending_blink_authorization=(event_end,'SINGLE_BLINK_RESUME_CONFIRMED')
            self.get_logger().info('BLINK_DECISION '+json.dumps(dict(
                id=event['id'],decision=self.event_decision,event_start=event_start,
                event_end=event_end,barrier=self.gesture_barrier,age=now-event_end,
                motion_ready=motion_ready)))
        if motion_ready and self.pending_blink_authorization is not None:
            self.session_enabled=True;self.motion=Motion.FORWARD;self.motion_started=now
            self.pause_reason=self.pending_blink_authorization[1]
            self.turn=0;self.turn_armed=True;self.blinks.clear();self.resume.reset()
            self.pending_blink_authorization=None

    def pause(self, reason, keep_closure=False,keep_event_history=False,barrier_at=None):
        self.motion = Motion.STOPPED
        self.motion_started = None
        self.turn = 0
        if not keep_event_history:
            self.blinks.clear();self.pending_blink_authorization=None
            # Input loss belongs to acquisition time. Local stops (emergency,
            # watchdog, boundary) still use receipt time and cannot be weakened.
            barrier=self.get_clock().now().nanoseconds/1e9 if barrier_at is None else barrier_at
            self.gesture_barrier=max(self.gesture_barrier,barrier)
        self.resume.reset()
        self.pause_reason = reason
        if not keep_closure:
            self.was_closed = False; self.closed_since = None
        self.send_key('space')

    def stop_cb(self, msg):
        if msg.data:
            # Explicit emergency stop revokes the session; gaze cannot unlock it.
            self.session_enabled = False
            self.pause('EMERGENCY_STOP')

    def send_key(self, key):
        if self.keyboard is not None and key != self.last_key:
            self.keyboard.press(key)
            self.keyboard.release(key)
            self.last_key = key

    def pose_cb(self, pose):
        now = self.get_clock().now().nanoseconds / 1e9
        near_wall = (pose.x <= self.arena_min + self.wall_margin or
                     pose.x >= self.arena_max - self.wall_margin or
                     pose.y <= self.arena_min + self.wall_margin or
                     pose.y >= self.arena_max - self.wall_margin)
        if near_wall and now - self.last_recenter >= self.recenter_cooldown:
            self.last_recenter = now
            self.pause('BOUNDARY_STOP')
            if self.teleport_client.service_is_ready():
                request = TeleportAbsolute.Request()
                request.x, request.y, request.theta = 5.544445, 5.544445, 0.0
                self.teleport_client.call_async(request)
            self.get_logger().warn('Wall detected; turtle stopped and recentered.')

    def control_loop(self):
        now = self.get_clock().now()
        age = (now - self.last_signal).nanoseconds / 1e9
        if age > self.timeout:
            self.pause('SIGNAL_TIMEOUT')
        if self.control_mode == 'keyboard':
            if self.motion == Motion.STOPPED:
                key = 'space'
            elif self.turn < 0:
                key = 'a'
            elif self.turn > 0:
                key = 'd'
            else:
                key = 'w'
            self.send_key(key)
        cmd = Twist()
        now_s = now.nanoseconds / 1e9
        if self.motion != Motion.STOPPED:
            cmd.linear.x = self.linear_speed
            if abs(self.horizontal) < self.turn_release:
                self.turn_armed = True
            if self.turn != 0 and now_s >= self.turn_until:
                self.turn = 0
            if self.turn == 0 and self.turn_armed:
                if self.horizontal < -self.deadzone:
                    self.turn = -1
                    self.turn_until = now_s + self.turn_pulse_duration
                    self.turn_armed = False
                elif self.horizontal > self.deadzone:
                    self.turn = 1
                    self.turn_until = now_s + self.turn_pulse_duration
                    self.turn_armed = False
            cmd.angular.z = -self.angular_speed * self.turn
        else:
            self.turn = 0
            self.turn_armed = True
        self.pub.publish(cmd)
        state = 'ACTIVE' if self.motion == Motion.FORWARD else 'PAUSED' if self.session_enabled else 'DISARMED'
        self.feedback.publish(String(data=json.dumps(dict(state=state, reason=self.pause_reason,
            resume_phase=self.resume.phase, resume_progress=self.resume.progress,
            blink_count=min(2,sum(now_s-t<=self.blink_window for t in self.blinks)) if not self.session_enabled else 0,
            turn=self.turn,turn_armed=self.turn_armed,horizontal=self.horizontal,
            turn_threshold=self.deadzone,last_blink_event_id=self.fusion_event_id,event_decision=self.event_decision))))


def main(args=None):
    rclpy.init(args=args)
    node = EyeTurtleController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.motion = Motion.STOPPED
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
