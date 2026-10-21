#!/usr/bin/env python3
"""Verify MediaPipe video inference -> ROS eye signals -> controller -> turtlesim."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from turtlesim.msg import Pose


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', default='video-results'); args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]; out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = rclpy.create_node('offline_video_pipeline_verifier')
    stats = {'signals': 0, 'valid_signals': 0, 'velocity_messages': 0, 'pose_messages': 0, 'unexpected_motion': 0}
    def on_signal(msg):
        stats['signals'] += 1
        stats['valid_signals'] += int(len(msg.data) == 3 and msg.data[2] == 1.)
    def on_cmd(msg):
        stats['velocity_messages'] += 1
        stats['unexpected_motion'] += int(abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6)
    def on_pose(msg):
        stats['pose_messages'] += 1
        stats['last_pose'] = [msg.x, msg.y, msg.theta]
    subs = [node.create_subscription(Float32MultiArray, '/eye_tracking/signals', on_signal, 20),
            node.create_subscription(Twist, '/turtle1/cmd_vel', on_cmd, 20),
            node.create_subscription(Pose, '/turtle1/pose', on_pose, 20)]
    proc = None
    try:
        with (out/'launch.log').open('w') as log:
            proc = subprocess.Popen(['bash', str(repo/'start_eye_turtle.sh'), '--video',
                str(repo/'test/fixtures/portrait.avi'), '--no-display', '--max-frames', '100',
                '--report', str(out/'model.json')], stdout=log, stderr=log, start_new_session=True)
            end = time.monotonic() + 90
            while proc.poll() is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)
            assert proc.poll() == 0, 'Video pipeline failed or timed out; inspect launch.log'
        model = json.loads((out/'model.json').read_text()); stats['model'] = model
        assert model['frames'] == 100 and model['face_frames'] > 80 and model['valid_frames'] > 50
        assert stats['valid_signals'] > 40 and stats['velocity_messages'] > 50 and stats['pose_messages'] > 50
        assert stats['unexpected_motion'] == 0, 'Stationary face must not arm motion'
        stats['passed'] = True
        print('MEDIAPIPE_VIDEO_ROS_TURTLESIM_PASS', json.dumps(stats), flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        (out/'report.json').write_text(json.dumps(stats, indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()
