#!/usr/bin/env python3
"""MediaPipe FaceMesh iris tracker and ROS2 signal publisher.

Run: python3 tools/eye_tracking_test.py --camera 0
Input to the controller: [horizontal, eye_closed, valid].
"""
import argparse
from collections import deque
import sys
import time

import cv2
import numpy as np

try:
    import mediapipe as mp
except ImportError:
    print('MediaPipe is required: python3 -m pip install mediapipe==0.10.21', file=sys.stderr)
    raise


LEFT_CORNERS = (33, 133)
RIGHT_CORNERS = (362, 263)
LEFT_IRIS = range(468, 473)
RIGHT_IRIS = range(473, 478)


def point(lm, width, height):
    return np.array([lm.x * width, lm.y * height], dtype=np.float32)


def eye_metrics(landmarks, corners, upper_id, lower_id, iris_ids, width, height):
    a, b = (point(landmarks[i], width, height) for i in corners)
    iris = np.mean([point(landmarks[i], width, height) for i in iris_ids], axis=0)
    # Vertical lid opening around the iris, normalized by eye width.
    upper = point(landmarks[upper_id], width, height)
    lower = point(landmarks[lower_id], width, height)
    eye_width = max(float(np.linalg.norm(a - b)), 1.0)
    openness = float(np.linalg.norm(upper - lower) / eye_width)
    horizontal = float(np.dot(iris - a, b - a) / max(np.dot(b - a, b - a), 1.0))
    return horizontal, openness, iris


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--calibration-frames', type=int, default=30)
    parser.add_argument('--threshold', type=float, default=0.12)
    parser.add_argument('--closed-ratio', type=float, default=0.16)
    parser.add_argument('--publish-ros', action='store_true')
    args = parser.parse_args()

    ros = None
    pub = None
    if args.publish_ros:
        import rclpy
        from std_msgs.msg import Float32MultiArray
        rclpy.init()
        ros = rclpy.create_node('mediapipe_eye_tracker')
        pub = ros.create_publisher(Float32MultiArray, '/eye_tracking/signals', 10)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError('Cannot open camera {}. Try --camera 1/2/3'.format(args.camera))
    history = deque(maxlen=7)
    calibration = []
    baseline = 0.5
    last_label = 'CALIBRATING'
    with mp.solutions.face_mesh.FaceMesh(
        max_num_faces=1, refine_landmarks=True,
        min_detection_confidence=0.5, min_tracking_confidence=0.5) as mesh:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            result = mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            horizontal, closed, valid = 0.0, 0.0, 0.0
            if result.multi_face_landmarks:
                lm = result.multi_face_landmarks[0].landmark
                lpos, lopen, lip = eye_metrics(lm, LEFT_CORNERS, 159, 145, LEFT_IRIS, w, h)
                rpos, ropen, rip = eye_metrics(lm, RIGHT_CORNERS, 386, 374, RIGHT_IRIS, w, h)
                pos = (lpos + rpos) / 2.0
                openness = (lopen + ropen) / 2.0
                calibration.append(pos) if len(calibration) < args.calibration_frames else None
                if len(calibration) == args.calibration_frames:
                    baseline = float(np.mean(calibration))
                    calibration.append(baseline)  # prevent recalibration
                history.append(pos)
                horizontal = float(np.clip((np.mean(history) - baseline) / 0.25, -1, 1))
                closed = float(openness < args.closed_ratio)
                valid = 1.0
                if len(calibration) <= args.calibration_frames:
                    last_label = 'CALIBRATING {}/{}'.format(len(calibration), args.calibration_frames)
                elif closed:
                    last_label = 'BLINK/CLOSED'
                elif horizontal < -args.threshold:
                    last_label = 'A (LEFT)'
                elif horizontal > args.threshold:
                    last_label = 'D (RIGHT)'
                else:
                    last_label = 'CENTER (2 BLINKS = FORWARD)'
                for p in (lip, rip):
                    cv2.circle(frame, tuple(p.astype(int)), 3, (0, 255, 0), -1)
            else:
                last_label, valid = 'NO FACE', 0.0
            if pub:
                pub.publish(Float32MultiArray(data=[horizontal, closed, valid]))
                rclpy.spin_once(ros, timeout_sec=0.0)
            cv2.putText(frame, last_label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)
            cv2.imshow('eye_tracking_debug', frame)
            if cv2.waitKey(1) & 0xff == 27:
                break
    cap.release()
    cv2.destroyAllWindows()
    if ros:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
