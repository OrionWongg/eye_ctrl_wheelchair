#!/usr/bin/env python3
"""Minimal webcam eye-signal publisher using OpenCV Haar cascades."""
import cv2
import numpy as np
import rclpy
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CameraEyeTracker(Node):
    def __init__(self):
        super().__init__('camera_eye_tracker')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('show_debug', True)
        p = self.get_parameter
        device = str(p('camera_device').value).strip()
        index = int(p('camera_index').value)
        # V4L2 avoids GStreamer format negotiation failures on multi-node cameras.
        self.cap = cv2.VideoCapture(device if device else index, cv2.CAP_V4L2)
        if not self.cap.isOpened() and not device:
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            self.get_logger().error(
                'Cannot open camera index %s. Check /dev/videoN permissions and camera_index.',
                device or str(p('camera_index').value))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(p('camera_width').value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(p('camera_height').value))
        self.show_debug = bool(p('show_debug').value)
        cascade_dirs = []
        if hasattr(cv2, 'data') and hasattr(cv2.data, 'haarcascades'):
            cascade_dirs.append(Path(cv2.data.haarcascades))
        cascade_dirs.extend([
            Path('/usr/share/opencv4/haarcascades'),
            Path('/usr/share/opencv/haarcascades'),
        ])
        cascade_dir = next((d for d in cascade_dirs if d.exists()), None)
        if cascade_dir is None:
            raise RuntimeError('OpenCV Haar cascade files not found; install opencv-data')
        self.face = cv2.CascadeClassifier(str(cascade_dir / 'haarcascade_frontalface_default.xml'))
        self.eye = cv2.CascadeClassifier(str(cascade_dir / 'haarcascade_eye_tree_eyeglasses.xml'))
        if self.face.empty() or self.eye.empty():
            raise RuntimeError('Could not load Haar cascade classifiers from ' + str(cascade_dir))
        self.pub = self.create_publisher(Float32MultiArray, '/eye_tracking/signals', 10)
        self.timer = self.create_timer(1.0 / float(p('publish_hz').value), self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            return
        debug_text = 'NO FACE'
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face.detectMultiScale(gray, 1.2, 5, minSize=(100, 100))
        if len(faces) == 0:
            if self.show_debug:
                cv2.putText(frame, debug_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('eye_tracking_debug', frame)
                cv2.waitKey(1)
            self.pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            return
        x, y, w, h = max(faces, key=lambda box: box[2] * box[3])
        roi = gray[y:y + int(h * 0.62), x:x + w]
        eyes = self.eye.detectMultiScale(roi, 1.1, 4, minSize=(20, 12))
        eyes = sorted(eyes, key=lambda box: box[2] * box[3], reverse=True)[:2]
        if len(eyes) == 0:
            debug_text = 'FACE / NO EYES'
            if self.show_debug:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, debug_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                cv2.imshow('eye_tracking_debug', frame)
                cv2.waitKey(1)
            self.pub.publish(Float32MultiArray(data=[0.0, 1.0, 1.0]))
            return
        centers = []
        for ex, ey, ew, eh in eyes:
            patch = roi[ey:ey + eh, ex:ex + ew]
            threshold = np.percentile(patch, 20)
            dark = patch < threshold
            ys, xs = np.where(dark)
            if len(xs) > 8:
                centers.append((ex + float(xs.mean()), ex + ew / 2.0, ew))
        if not centers:
            debug_text = 'EYES / NO PUPIL'
            if self.show_debug:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, debug_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                cv2.imshow('eye_tracking_debug', frame)
                cv2.waitKey(1)
            self.pub.publish(Float32MultiArray(data=[0.0, 1.0, 1.0]))
            return
        offsets = [(pupil - eye_center) / max(width / 2.0, 1.0) for pupil, eye_center, width in centers]
        horizontal = float(np.clip(np.mean(offsets), -1.0, 1.0))
        if self.show_debug:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, 'TRACK h={:+.2f}'.format(horizontal), (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('eye_tracking_debug', frame)
            cv2.waitKey(1)
        self.pub.publish(Float32MultiArray(data=[horizontal, 0.0, 1.0]))

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraEyeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
