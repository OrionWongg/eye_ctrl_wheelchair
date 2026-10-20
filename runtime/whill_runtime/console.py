"""First person simulation display and keyboard / eye input."""
import argparse
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Empty
from wheelchair_sim.first_person_view import decode_image
from .drive import EyeDrive


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['keyboard', 'eye'], default='keyboard')
    parser.add_argument('--no-display', action='store_true')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('whill_simulation_console')
    pub = node.create_publisher(Twist, '/wheelchair/sim/drive', 1)
    feedback = node.create_publisher(String, '/eye_control/feedback', 1)
    gate = EyeDrive()
    image = None
    image_at = scan_at = -math.inf
    front = 0.
    target = (0., 0.)
    key_at = -math.inf
    emergency = False

    def receive_image(msg):
        nonlocal image, image_at
        image = decode_image(msg)
        image_at = time.monotonic()

    def scan(msg):
        nonlocal scan_at, front
        values = [v for i, v in enumerate(msg.ranges)
                  if abs(msg.angle_min + i * msg.angle_increment) < .55
                  and not math.isnan(v) and v >= msg.range_min]
        front = min(values, default=0.)
        scan_at = time.monotonic()

    def eye(msg):
        gate.receive(msg.data, time.monotonic(), time.time_ns())

    def stop(msg):
        nonlocal emergency
        emergency = True
        gate.stop('EMERGENCY STOP', True)

    node.create_subscription(Image, '/wheelchair/first_person/image_raw', receive_image, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/wheelchair/scan', scan, qos_profile_sensor_data)
    node.create_subscription(Empty, '/wheelchair/emergency_stop', stop, 1)
    node.create_subscription(String, '/eye_tracking/fusion_samples', eye, 1)
    title = 'WHILL Simulation - W/A/S/D hold | Space STOP | E reset | Q quit'
    if not args.no_display:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, 960, 580)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.005)
            now = time.monotonic()
            frame = image.copy() if image is not None and now - image_at < 1. else np.zeros((540, 960, 3), np.uint8)
            if args.no_display:
                time.sleep(.015)
            key = (cv2.waitKey(15) & 0xff) if not args.no_display else -1
            if key in (27, ord('q')) or (not args.no_display and cv2.getWindowProperty(title, cv2.WND_PROP_AUTOSIZE) < 0):
                break
            if key == ord(' '):
                emergency = True
                gate.stop('EMERGENCY STOP', True)
            if key == ord('e'):
                emergency = False
            commands = {ord('w'): (.25, 0.), ord('s'): (-.15, 0.),
                        ord('a'): (0., .5), ord('d'): (0., -.5)}
            if args.mode == 'keyboard' and key in commands:
                target, key_at = commands[key], now
            velocity = gate.velocity(now) if args.mode == 'eye' else (target if now - key_at < .25 else (0., 0.))
            status = gate.reason if args.mode == 'eye' else 'KEYBOARD'
            if emergency:
                velocity, status = (0., 0.), 'EMERGENCY STOP - E TO RESET'
            elif now - scan_at > .5:
                velocity, status = (0., 0.), 'WAITING FOR FRESH LIDAR'
            elif velocity[0] > 0 and front < .45:
                velocity, status = (0., 0.), 'OBSTACLE AHEAD'
                if args.mode == 'eye':
                    gate.stop(status)
            msg = Twist()
            msg.linear.x, msg.angular.z = velocity
            pub.publish(msg)
            feedback.publish(String(data=json.dumps({'state': gate.reason, 'mode': args.mode,
                'status': status, 'linear': velocity[0], 'angular': velocity[1]})))
            cv2.putText(frame, f'{args.mode.upper()} | {status}', (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            cv2.putText(frame, f'v={velocity[0]:.2f} m/s  yaw={velocity[1]:.2f} rad/s  front={front:.2f} m', (15, 58), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
            if not args.no_display:
                cv2.imshow(title, frame)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
