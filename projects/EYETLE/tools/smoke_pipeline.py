#!/usr/bin/env python3
"""Exercise real ROS controller and turtlesim processes without a camera."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='offline-results')
    args = parser.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    children, logs, cases = [], [], []
    rclpy.init()
    node = rclpy.create_node('eye_pipeline_verifier')
    state = {'cmd': None, 'pose': None}
    pub = node.create_publisher(Float32MultiArray, '/eye_tracking/signals', 10)
    stop = node.create_publisher(Bool, '/eye_turtle/stop', 10)
    subscriptions = [
        node.create_subscription(Twist, '/turtle1/cmd_vel', lambda m: state.update(cmd=m), 20),
        node.create_subscription(Pose, '/turtle1/pose', lambda m: state.update(pose=m), 20)]
    def spin(duration, values=(0., 0., 1.)):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if values is not None:
                pub.publish(Float32MultiArray(data=list(values)))
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.01)
    def check(name, condition):
        cases.append({'test': name, 'passed': bool(condition)})
        print(name, 'PASS' if condition else 'FAIL', flush=True)
        assert condition, name
    def stopped():
        return state['cmd'] is not None and abs(state['cmd'].linear.x) < 1e-6 and abs(state['cmd'].angular.z) < 1e-6
    def reset():
        stop.publish(Bool(data=True)); spin(.5)
    def blinks(count):
        for _ in range(count):
            spin(.12, (0., 1., 1.)); spin(.28)
        spin(.65 if count == 2 else .2)
    try:
        for package, executable in [('turtlesim', 'turtlesim_node'), ('eye_turtle_control', 'eye_turtle_controller')]:
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            log = (out / (executable + '.log')).open('w'); logs.append(log)
            command=[str(binary)]
            if executable=='eye_turtle_controller':command += ['--ros-args','-p','linear_speed:=0.2']
            children.append(subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True))
        deadline = time.monotonic() + 12
        while (state['pose'] is None or pub.get_subscription_count() == 0) and time.monotonic() < deadline:
            spin(.1)
        check('ROS_discovery_and_turtlesim_pose', state['pose'] is not None and pub.get_subscription_count() > 0)
        reset(); blinks(3)
        check('third_blink_keeps_forward', state['cmd'].linear.x > 0)
        reset(); start_x = state['pose'].x; spin(.6)
        check('center_gaze_does_not_arm', stopped() and abs(state['pose'].x - start_x) < .04)
        blinks(1); spin(.9); check('single_blink_does_not_arm', stopped())
        reset(); start_x = state['pose'].x; blinks(2)
        check('two_blinks_forward_and_pose_changes', state['cmd'].linear.x > 0 and state['pose'].x > start_x + .03)
        theta = state['pose'].theta; spin(.35, (-.7, 0., 1.))
        check('left_gaze_turns_left', state['cmd'].angular.z > 0 and state['pose'].theta > theta)
        spin(1.0, (-.7, 0., 1.)); check('turn_pulse_ends', abs(state['cmd'].angular.z) < 1e-6)
        spin(.2); spin(.3, (.7, 0., 1.)); check('recenter_then_right_turn', state['cmd'].angular.z < 0)
        spin(.78, (0., 1., 1.)); check('long_blink_stops', stopped())
        spin(1.); check('reopening_does_not_resume', stopped())
        spin(.12,(0.,1.,1.));check('closed_eye_still_paused',stopped())
        spin(.2);check('single_blink_resumes',state['cmd'].linear.x>0)
        reset(); blinks(2); spin(.2, (0., 0., 0.)); check('invalid_signal_stops', stopped())
        reset(); blinks(2); spin(.2, (0., 0., float('nan'))); check('nonfinite_valid_flag_stops', stopped())
        reset(); blinks(2); spin(.2, (0., float('nan'), 1.)); check('nonfinite_closed_flag_stops', stopped())
        reset(); blinks(2); pub.publish(Float32MultiArray(data=[0.])); spin(.12, None)
        check('malformed_signal_stops', stopped())
        reset(); blinks(2); spin(.7, None); check('publisher_timeout_stops', stopped())
        reset(); blinks(2); stop.publish(Bool(data=True)); spin(.2); check('emergency_stop', stopped())
        reset(); blinks(2); spin(5.2); check('motion_continues_beyond_five_seconds', state['cmd'].linear.x>0)
        client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        check('teleport_service_ready', client.wait_for_service(timeout_sec=3))
        future = client.call_async(TeleportAbsolute.Request(x=10.9, y=5.5, theta=0.0))
        spin(.8)
        check('boundary_stops_and_recenters', future.done() and stopped() and abs(state['pose'].x - 5.544445) < .08)
    finally:
        stop.publish(Bool(data=True))
        for proc in children:
            if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
        for proc in children:
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        for log in logs: log.close()
        (out/'report.json').write_text(json.dumps(cases, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
