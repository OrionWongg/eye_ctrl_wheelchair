"""ROS transport; watchdogs run on steady time even when Gazebo is paused."""
import json
import math
import time
import uuid
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray, String
from wheelchair_interfaces.msg import ControlIntent, ControlStatus
from .core import Config, EyeMachine

class EyeIntentNode(Node):
    def __init__(self):
        super().__init__('eye_intent')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        mode = self.declare_parameter('interaction_mode', 'gaze_joystick').value
        self.confirm_transport = None
        if mode in ('intent_confirm','directional_intent'):
            from .confirm_node import ConfirmTransport
            self.confirm_transport = ConfirmTransport(self,directional=mode=='directional_intent')
            self.add_on_set_parameters_callback(self.reject_runtime_parameters)
            return
        if mode != 'gaze_joystick':
            raise ValueError('unknown interaction_mode: ' + str(mode))
        cfg = Config()
        for key, value in vars(cfg).items():
            setattr(cfg, key, float(self.declare_parameter(key, value).value))
        self.machine = EyeMachine(cfg)
        self.session, self.sequence = str(uuid.uuid4()), 0
        self.last_ros = None
        self.last_ros_change = time.monotonic()
        self.clock_valid = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(ControlIntent, '/wheelchair/eye_intent', qos)
        self.feedback = self.create_publisher(String, '/wheelchair/eye_feedback', qos)
        self.common_feedback = self.create_publisher(String, '/eye_control/feedback', qos)
        self.create_subscription(Float32MultiArray, '/eye_tracking/signals', self.signal, sensor_qos)
        self.create_subscription(ControlStatus, '/wheelchair/control_status', self.status, qos)
        self.timer = self.create_timer(.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.reject_runtime_parameters)

    def reject_runtime_parameters(self, parameters):
        # Config is copied into a pure state machine during startup. Accepting
        # later parameter writes would falsely advertise a live mode/config
        # change while leaving its old authorization semantics in force.
        changed = [p.name for p in parameters
                   if not self.has_parameter(p.name) or self.get_parameter(p.name).value != p.value]
        if changed:
            return SetParametersResult(successful=False, reason=(
                'Eye input configuration is fixed at startup; stop the wheelchair, '
                'wait for stationary feedback, then restart to change: ' + ', '.join(changed)))
        return SetParametersResult(successful=True)

    def signal(self, msg):
        if not self.clock_valid:
            self.machine.invalidate('ROS_CLOCK_UNAVAILABLE')
            return
        if len(msg.data) != 3 or not all(math.isfinite(x) for x in msg.data):
            self.machine.invalidate('MALFORMED_EYE_SIGNAL')
            return
        h, closed, valid = msg.data
        if closed not in (0., 1.) or valid not in (0., 1.):
            self.machine.invalidate('MALFORMED_EYE_FLAGS')
            return
        self.machine.signal(time.monotonic(), h, bool(closed), bool(valid))

    def status(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if not self.clock_valid or not (0 <= now_ns - stamp_ns <= int(self.machine.cfg.status_timeout * 1e9)):
            return
        self.machine.status(time.monotonic(), msg.session_id, msg.sequence,
                            msg.requires_rearm, msg.ready_to_rearm, msg.reason)

    def publish(self):
        if self.confirm_transport is not None:
            self.confirm_transport.publish()
            return
        mono = time.monotonic()
        ros = self.get_clock().now()
        ns = ros.nanoseconds
        jumped = self.last_ros is not None and (ns < self.last_ros or ns - self.last_ros > 1000000000)
        if ns != self.last_ros:
            self.last_ros_change = mono
        self.clock_valid = ns > 0 and mono - self.last_ros_change <= .30 and not jumped
        self.last_ros = ns
        if not self.clock_valid:
            self.machine.invalidate('ROS_CLOCK_JUMP_OR_PAUSE')
            self.machine.ready = False
        intent = self.machine.tick(mono)
        msg = ControlIntent()
        msg.header.stamp, msg.header.frame_id = ros.to_msg(), 'base_link'
        msg.session_id, msg.sequence = self.session, self.sequence
        self.sequence += 1
        msg.command, msg.enabled, msg.valid = int(intent.command), intent.enabled, intent.valid
        msg.ttl_ms, msg.reason = 250, intent.reason
        self.pub.publish(msg)
        feedback = String()
        feedback.data = json.dumps(dict(state=intent.state.value, command=intent.command.name,
                                       enabled=intent.enabled, valid=intent.valid, reason=intent.reason,
                                       resume_phase=self.machine.resume.phase, resume_progress=self.machine.resume.progress))
        self.feedback.publish(feedback)
        self.common_feedback.publish(feedback)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = EyeIntentNode()
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.machine.invalidate('NODE_SHUTDOWN')
        if rclpy.ok():
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
