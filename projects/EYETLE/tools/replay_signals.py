#!/usr/bin/env python3
"""Replay labelled synthetic gestures; no camera or inference is used."""
import argparse
import time
import rclpy
from std_msgs.msg import Float32MultiArray


def main():
    p = argparse.ArgumentParser(); p.add_argument('--loop', action='store_true'); args = p.parse_args()
    rclpy.init(); node = rclpy.create_node('synthetic_eye_signal_replay')
    pub = node.create_publisher(Float32MultiArray, '/eye_tracking/signals', 10)
    def emit(seconds, horizontal=0., closed=0., valid=1.):
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            pub.publish(Float32MultiArray(data=[horizontal, closed, valid]))
            rclpy.spin_once(node, timeout_sec=0.); time.sleep(1/30)
    def blink(count):
        for _ in range(count): emit(.12, closed=1.); emit(.28)
        emit(.7)
    try:
        while rclpy.ok():
            node.get_logger().info('SYNTHETIC INPUT: center, double blink, forward, left, right, stop')
            emit(1.); blink(2); emit(.6); emit(1.2, horizontal=-.7)
            emit(.2); emit(.8, horizontal=.7); emit(.8, closed=1.)
            emit(.5, valid=0.)
            if not args.loop: break
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): pub.publish(Float32MultiArray(data=[0., 0., 0.]))
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()
