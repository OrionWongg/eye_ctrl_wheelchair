#!/usr/bin/env python3
"""Exercise Tk, ROS topics and optional PTY-emulated ESP32 without hardware."""
import argparse
import json
import os
from pathlib import Path
import select
import threading
import time

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from whill_controller.controller_node import WhillControllerNode


class Esp32Emulator:
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.port = os.ttyname(self.slave)
        assert self.port.startswith('/dev/pts/')
        self.connected = False
        self.received = bytearray()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            if not select.select([self.master], [], [], .05)[0]:
                continue
            try:
                data = os.read(self.master, 4096)
                self.received.extend(data)
                for byte in data:
                    if byte == ord('?'):
                        self.feedback()
            except OSError:
                time.sleep(.02)

    def feedback(self):
        os.write(self.master, ('BLE_CONNECTED=%d\n' % self.connected).encode())

    def close(self):
        self.stop.set(); self.thread.join(timeout=1)
        os.close(self.master); os.close(self.slave)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock-serial', action='store_true')
    parser.add_argument('--output', default='offline-results')
    args = parser.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    emulator = Esp32Emulator() if args.mock_serial else None
    ros_args = ['--ros-args', '-p', 'hardware_enabled:=' + str(args.mock_serial).lower(),
                '-p', 'serial_port:=' + (emulator.port if emulator else '/dev/DO_NOT_OPEN')]
    rclpy.init(args=ros_args)
    controller = None
    observer = rclpy.create_node('whill_pipeline_verifier')
    state = {'status': None, 'odom': None, 'path': None}
    commands, cases = [], []
    transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        observer.create_subscription(String, '/whill/controller/status', lambda m: state.update(status=json.loads(m.data)), transient),
        observer.create_subscription(String, '/whill/controller/command', lambda m: commands.append(json.loads(m.data)), 100),
        observer.create_subscription(Odometry, '/whill/controller/odometry', lambda m: state.update(odom=m), 20),
        observer.create_subscription(RosPath, '/whill/controller/path', lambda m: state.update(path=m), transient)]
    pub = observer.create_publisher(String, '/whill/controller/input', 10)
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            controller.root.update()
            rclpy.spin_once(observer, timeout_sec=0.)
            time.sleep(.005)
    def wait_for(predicate, timeout=6):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            pump(.05)
        return predicate()
    def check(name, condition):
        cases.append({'test': name, 'passed': bool(condition)})
        print(name, 'PASS' if condition else 'FAIL', flush=True)
        assert condition, name
    def key(name, duration=.1):
        controller.root.focus_force()
        controller.root.event_generate('<KeyPress>', keysym=name)
        pump(duration)
        controller.root.event_generate('<KeyRelease>', keysym=name)
        pump(.08)
    try:
        controller = WhillControllerNode()
        check('GUI_ROS_status_odometry_path', wait_for(lambda: all(state.values())))
        check('hardware_mode_matches_test', state['status']['hardware_enabled'] == args.mock_serial)
        if not emulator:
            check('dry_run_opens_no_serial_port', controller.serial_device is None)
        pub.publish(String(data='tap:w')); pump(.4)
        check('ROS_input_to_command_topic', any(c['action'] == 'ros_movement' and c['payload_ascii'] == 'w' for c in commands))
        if emulator:
            check('serial_bytes_reach_PTY', b'w' in emulator.received)
        pub.publish(String(data='mode:toggle')); pump(.8)
        if emulator:
            check('BLE_disconnected_blocks_homing', b'j' not in emulator.received and controller.calibrating)
            emulator.connected = True; emulator.feedback()
        check('app_mode_calibration_completes', wait_for(lambda: controller.control_mode == 'app_locked' and not controller.calibrating and controller.locked_current_coordinates is not None))
        initialization = [c for c in commands if c['action'] == 'locked_mode_initialize']
        check('locked_homing_sequence', bool(initialization) and initialization[-1]['approach_a'] == 25 and initialization[-1]['approach_w'] == 14)
        controller.arm_button.invoke(); pump(.1)
        key('w', .3)
        check('motion_requires_button_hold', abs(state['odom'].pose.pose.position.x) < .001)
        key('f'); key('w', .8); key('a', .4)
        check('GUI_keys_drive_ROS_odometry', state['odom'].pose.pose.position.x > .15 and abs(state['odom'].pose.pose.orientation.z) > .02)
        check('ROS_path_accumulates', len(state['path'].poses) > 3)
        check('key_duration_telemetry', controller.key_duration_totals.get('w', 0) > .7)
        key('f'); controller.arm_button.invoke(); pump(.2)
        check('disarm_releases_button', not controller.click_held and not controller.keyboard_armed)
        if emulator:
            count = len([c for c in commands if c['action'] == 'locked_mode_initialize'])
            emulator.connected = False; emulator.feedback(); pump(.4)
            check('BLE_disconnect_stops_tracking', controller.calibrating and not controller.motion_tracking_active)
            emulator.connected = True; emulator.feedback()
            check('BLE_reconnect_rehomes', wait_for(lambda: not controller.calibrating and len([c for c in commands if c['action'] == 'locked_mode_initialize']) > count))
        pub.publish(String(data='trajectory:reset')); pump(.3)
        check('trajectory_reset', abs(state['odom'].pose.pose.position.x) < .001 and len(state['path'].poses) == 1)
        controller.arm_button.invoke(); key('f')
        controller.close(); time.sleep(.15)
        if emulator:
            check('shutdown_releases_mock_button', emulator.received.endswith(b']'))
        else:
            check('dry_run_remains_hardware_free', all(not c['hardware_enabled'] for c in commands))
    finally:
        if controller is not None:
            if not controller.closing: controller.close()
            controller.destroy_node()
        observer.destroy_node(); rclpy.try_shutdown()
        if emulator:
            (out/'serial-bytes.txt').write_text(bytes(emulator.received).decode('ascii', errors='replace'))
            emulator.close()
        (out/'report.json').write_text(json.dumps(cases, indent=2)+'\n')
        (out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')

if __name__ == '__main__':
    main()
