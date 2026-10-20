"""Synthetic eyes, never a camera or a hardware controller."""
import argparse
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish synthetic eye samples; Ctrl+C stops source.')
    parser.add_argument('--direction', choices=['forward', 'left', 'right'], default='forward')
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--arm-after', type=float, default=2., help='seconds before fresh double blink')
    parser.add_argument('--no-arm', action='store_true')
    parser.add_argument('--invalid-after', type=float, default=-1.)
    parser.add_argument('--long-close-after', type=float, default=-1.)
    opts, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('mock_eye_signals')
    pub = node.create_publisher(Float32MultiArray, '/eye_tracking/signals', 1)
    start = time.monotonic()
    h = {'forward': 0., 'left': -.6, 'right': .6}[opts.direction]
    try:
        while rclpy.ok() and time.monotonic() - start < opts.duration:
            elapsed = time.monotonic() - start
            blink = elapsed - opts.arm_after
            closed = not opts.no_arm and (0 <= blink < .12 or .25 <= blink < .37)
            if opts.long_close_after >= 0 and elapsed >= opts.long_close_after:
                closed = True
            valid = not (opts.invalid_after >= 0 and elapsed >= opts.invalid_after)
            pub.publish(Float32MultiArray(data=[h, float(closed), float(valid)]))
            rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pub.publish(Float32MultiArray(data=[0., 0., 0.]))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
