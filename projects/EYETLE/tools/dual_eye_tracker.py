#!/usr/bin/env python3
"""Two UVC close-up eye cameras. Explicit ROI and three-point calibration."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np


def find_pupil(frame, roi):
    """Conservative dark ellipse candidate; returns None for ambiguous images."""
    x, y, w, h = roi
    gray = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2GRAY)
    if gray.size == 0 or np.std(gray) < 8:
        return None
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    threshold = min(float(np.percentile(gray, 15)), float(np.mean(gray) - 12))
    mask = cv2.inRange(gray, 0, max(0, threshold))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if len(c) < 5 or not .003*w*h < area < .18*w*h:
            continue
        (cx, cy), (a, b), angle = cv2.fitEllipse(c)
        if min(a, b) < 4 or min(a, b)/max(a, b) < .45:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if bx <= 1 or by <= 1 or bx+bw >= w-1 or by+bh >= h-1:
            continue
        fill = area / max(np.pi*a*b/4, 1)
        if not .65 < fill < 1.2:
            continue
        candidates.append((area*min(a,b)/max(a,b), ((cx+x, cy+y), (a,b), angle)))
    candidates.sort(key=lambda item: item[0], reverse=True)
    if not candidates or (len(candidates)>1 and candidates[1][0] > candidates[0][0]*.8):
        return None
    return candidates[0][1]


def gaze(position, calibration):
    left, center, right = (calibration[k] for k in ('left','center','right'))
    delta = position-center
    endpoint = right if delta*(right-center) >= 0 else left
    sign = 1 if endpoint == right else -1
    return float(np.clip(sign*delta/(endpoint-center), -1, 1))



def eye_patch(frame, roi):
    """Small appearance template, retaining brightness to reject black occlusions."""
    x, y, w, h = roi
    gray = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, (64, 32), interpolation=cv2.INTER_AREA).astype(np.float32)/255.


class EyeAppearance:
    """User-labelled open/closed templates; unfamiliar appearances stay unknown."""
    def __init__(self):
        self.open = {}
        self.closed = None

    @property
    def ready(self):
        return len(self.open) == 3 and self.closed is not None

    def set_closed(self, patches):
        self.closed = None
        if len(self.open) != 3 or len(patches) < 20:
            return False
        template = np.median(patches, axis=0)
        # Reject unstable capture or an accidental second open-eye sample.
        if max(float(np.mean(np.abs(p-template))) for p in patches) > .06:
            return False
        if min(float(np.mean(np.abs(template-p))) for p in self.open.values()) < .045:
            return False
        self.closed = template
        return True

    def classify(self, patch, pupil):
        if not self.ready or patch is None:
            return 'unknown'
        opened = min(float(np.mean(np.abs(patch-p))) for p in self.open.values())
        closed = float(np.mean(np.abs(patch-self.closed)))
        if closed < .09 and opened-closed > .025:
            return 'closed'
        if pupil is not None and opened < .12 and closed-opened > .025:
            return 'open'
        return 'unknown'


class BinocularState:
    """Bridge bounded transitions (100 ms default); persistent loss is invalid."""
    def __init__(self,transition_grace=.1):
        if not np.isfinite(transition_grace) or not 0<transition_grace<=.2:
            raise ValueError('Transition grace must be in (0, 0.2] seconds')
        self.transition_grace=transition_grace
        self.last = None
        self.last_seen = None

    def update(self, states, now, ready):
        if not ready or (self.last_seen is not None and now-self.last_seen > .4):
            self.last = None
            self.last_seen = None
        if not ready:
            return False, False
        if states == ['open', 'open'] or states == ['closed', 'closed']:
            # Require a confirmed open pair before accepting a closure, including
            # immediately after closed-eye calibration or tracking loss.
            if self.last is None and states[0] == 'closed':
                return False, False
            self.last = states[0]
            self.last_seen = now
            return self.last == 'closed', True
        if self.last_seen is not None and now-self.last_seen <= self.transition_grace:
            return self.last == 'closed', True
        self.last = None
        self.last_seen = None
        return False, False


def orient_frame(frame, rotation=0, mirror=False):
    """Correct mounting orientation before preview, ROI selection and inference."""
    rotations = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    if rotation in rotations:
        frame = cv2.rotate(frame, rotations[rotation])
    return cv2.flip(frame, 1) if mirror else frame


def preview_left(args):
    """Preview mounted camera images without publishing control signals."""
    dual = getattr(args, 'preview_both', False)
    devices = [args.left_camera, args.right_camera] if dual else [args.left_camera]
    rotations = [args.left_rotation, args.right_rotation]
    mirrors = [args.left_mirror, args.right_mirror]
    if dual and devices[0] == devices[1]:
        raise ValueError('Two distinct camera devices are required')
    caps = []
    count = 0
    window = 'EYETLE dual camera preview' if dual else 'EYETLE left eye'
    try:
        for device in devices:
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            caps.append(cap)
            if not cap.isOpened():
                raise RuntimeError('Cannot open camera: '+device)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print('Camera preview: '+', '.join(devices), flush=True)
        if not args.no_display:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 1280 if dual else 640, 480)
        while True:
            # Grab both before decoding to reduce sequential read skew.
            if not all([cap.grab() for cap in caps]):
                raise RuntimeError('Camera frame grab failed')
            frames = []
            for i, cap in enumerate(caps):
                ok, frame = cap.retrieve()
                if not ok:
                    raise RuntimeError('Camera frame read failed: '+devices[i])
                frame = orient_frame(frame, rotations[i], mirrors[i])
                frame = cv2.resize(frame, (640,480))
                cv2.putText(frame, ('LEFT' if i == 0 else 'RIGHT')+' EYE | Esc/Q:exit',
                            (12,28), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,0), 1)
                frames.append(frame)
            count += 1
            if not args.no_display:
                cv2.imshow(window, np.hstack(frames))
                if cv2.waitKey(1)&255 in (27, ord('q')):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                    break
            if args.max_frames and count >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        for cap in caps:
            cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    paths = sorted(str(p) for p in Path('/dev/v4l/by-path').glob('*-video-index0'))
    parser.add_argument('--left-camera', default=paths[0] if paths else '/dev/video0')
    parser.add_argument('--right-camera', default=paths[1] if len(paths)>1 else '/dev/video2')
    parser.add_argument('--preview-both', action='store_true', help='Preview both cameras without inference or ROS control output')
    parser.add_argument('--preview-left', action='store_true', help='Preview only the left camera; no ROS control output')
    parser.add_argument('--left-rotation', type=int, choices=(0,90,180,270), default=180, help='Clockwise mounting correction; current left camera is inverted')
    parser.add_argument('--right-rotation', type=int, choices=(0,90,180,270), default=0)
    parser.add_argument('--left-mirror', action='store_true')
    parser.add_argument('--right-mirror', action='store_true')
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--publish-ros', action='store_true')
    parser.add_argument('--report')
    args = parser.parse_args()
    if args.preview_left or args.preview_both:
        preview_left(args)
        return
    devices = [args.left_camera, args.right_camera]
    if Path(devices[0]).resolve() == Path(devices[1]).resolve():
        parser.error('Two distinct camera devices are required')
    caps, node, pub = [], None, None
    stats = dict(devices=devices, frames=0, pupil_frames=[0,0], valid_frames=0)
    rois = [None, None]
    calibration = [{}, {}]
    history = [deque(maxlen=20), deque(maxlen=20)]
    patches = [deque(maxlen=20), deque(maxlen=20)]
    appearance = [EyeAppearance(), EyeAppearance()]
    binocular = BinocularState()
    capture_at = None
    closed_samples = [[], []]
    last_horizontal = 0.
    message = 'R: select eye ROIs | 1:left 2:center 3:right | B:closed calibration | Esc:exit'
    start = time.monotonic()
    try:
        if args.publish_ros:
            import rclpy
            from std_msgs.msg import Float32MultiArray
            rclpy.init()
            node = rclpy.create_node('dual_eye_tracker')
            pub = node.create_publisher(Float32MultiArray, '/eye_tracking/signals', 10)
        for device in devices:
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            caps.append(cap)
            if not cap.isOpened():
                raise RuntimeError('Cannot open camera: '+device)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        stats['formats'] = [dict(width=c.get(3), height=c.get(4), fps=c.get(5)) for c in caps]
        print(json.dumps(stats), flush=True)
        while True:
            # Grab both before decoding to reduce software-induced skew. No hardware sync.
            grabbed = [c.grab() for c in caps]
            samples = [c.retrieve() if ok else (False,None) for c,ok in zip(caps,grabbed)]
            if not all(ok for ok,_ in samples):
                raise RuntimeError('Camera disconnected or frame read failed')
            frames = [orient_frame(samples[0][1], args.left_rotation, args.left_mirror),
                      orient_frame(samples[1][1], args.right_rotation, args.right_mirror)]
            pupils = [find_pupil(f,roi) if roi else None for f,roi in zip(frames,rois)]
            current_patches = [eye_patch(f,roi) if roi else None for f,roi in zip(frames,rois)]
            now = time.monotonic()
            if capture_at is not None:
                if now >= capture_at:
                    for samples, patch in zip(closed_samples, current_patches):
                        samples.append(patch)
                    message = 'Keep BOTH eyes closed: collecting %d/20' % len(closed_samples[0])
                    if len(closed_samples[0]) >= 20:
                        accepted = [a.set_closed(samples) for a,samples in zip(appearance,closed_samples)]
                        if not all(accepted):
                            for a in appearance:a.closed = None
                        message = 'Blink calibration saved; reopen eyes' if all(accepted) else 'Closed calibration rejected; press B to retry'
                        capture_at = None
                else:
                    message = 'Close BOTH eyes and hold: capture in %.1fs' % (capture_at-now)
            for i,p in enumerate(pupils):
                if p is None:
                    history[i].clear()
                    patches[i].clear()
                else:
                    history[i].append(p[0][0]/frames[i].shape[1])
                    patches[i].append(current_patches[i])
                    stats['pupil_frames'][i] += 1
            ready = all(len(c)==3 for c in calibration) and all(a.ready for a in appearance) and capture_at is None
            states = [a.classify(patch,p) for a,patch,p in zip(appearance,current_patches,pupils)]
            closed, valid = binocular.update(states, now, ready)
            if valid and states == ['open','open']:
                last_horizontal = float(np.mean([gaze(np.median(h),c) for h,c in zip(history,calibration)]))
            horizontal = last_horizontal if valid else 0.
            if pub:
                pub.publish(Float32MultiArray(data=[horizontal,float(closed),float(valid)]))
                rclpy.spin_once(node, timeout_sec=0.)
            stats['frames'] += 1
            stats['valid_frames'] += int(valid)
            if not args.no_display:
                panels = []
                for i,(f,p,roi) in enumerate(zip(frames,pupils,rois)):
                    panel = f.copy()
                    if roi:
                        x,y,w,h = roi
                        cv2.rectangle(panel,(x,y),(x+w,y+h),(255,180,0),1)
                    if p is not None:
                        cv2.ellipse(panel,p,(0,255,0),2)
                    cv2.putText(panel, f'CAM {i+1} '+states[i].upper(), (12,30),0,.7,(0,255,255),2)
                    panels.append(panel)
                canvas = np.hstack(panels)
                cv2.putText(canvas, message,(10,canvas.shape[0]-40),0,.55,(0,255,255),1)
                cv2.putText(canvas,f'valid={int(valid)} closed={int(closed)} gaze={horizontal:+.2f} calibrated={ready}',(10,canvas.shape[0]-15),0,.6,(0,255,255),1)
                cv2.imshow('EYETLE dual eyes',canvas)
                key = cv2.waitKey(1)&255
                if key == 27:
                    break
                if key in (ord('r'),ord('R')):
                    if pub:
                        pub.publish(Float32MultiArray(data=[0.,0.,0.]))
                    capture_at = None
                    appearance = [EyeAppearance(), EyeAppearance()]
                    binocular = BinocularState()
                    for samples in patches:samples.clear()
                    rois = []
                    for i,f in enumerate(frames):
                        title = f'Select eye CAM {i+1} - ENTER confirms'
                        roi = tuple(map(int,cv2.selectROI(title,f,False)))
                        cv2.destroyWindow(title)
                        rois.append(roi if roi[2]>10 and roi[3]>10 else None)
                    calibration = [{},{}]
                    for h in history:h.clear()
                    for samples in patches:samples.clear()
                if key in (ord('b'),ord('B')):
                    if all(roi is not None for roi in rois) and all(len(a.open)==3 for a in appearance):
                        for a in appearance:a.closed = None
                        binocular = BinocularState()
                        closed_samples = [[], []]
                        capture_at = time.monotonic()+2.
                        message = 'Close BOTH eyes, hold for 4 seconds, then reopen'
                    else:
                        message = 'First select ROIs and complete open-eye calibration 1/2/3'
                if capture_at is None and key in (ord('1'),ord('2'),ord('3')):
                    label = {ord('1'):'left',ord('2'):'center',ord('3'):'right'}[key]
                    if all(len(h)==20 for h in history):
                        for c,h,a,samples in zip(calibration,history,appearance,patches):
                            c[label]=float(np.median(h))
                            a.open[label]=np.median(list(samples),axis=0)
                            a.closed=None
                        binocular = BinocularState()
                        message = 'Saved '+label+' | R:ROI 1:left 2:center 3:right B:closed Esc:exit'
                        if all(len(c)==3 for c in calibration):
                            if not all((c['left']-c['center'])*(c['right']-c['center'])<0 and min(abs(c['left']-c['center']),abs(c['right']-c['center']))>.015 for c in calibration):
                                calibration = [{},{}]
                                appearance = [EyeAppearance(), EyeAppearance()]
                                message = 'Calibration rejected: repeat 1/2/3 with distinct gaze positions'
                    else:
                        message = 'Need 20 consecutive pupil frames in BOTH eyes; adjust ROI/focus'
                    for h in history:h.clear()
                    for samples in patches:samples.clear()
            if args.max_frames and stats['frames'] >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        for c in caps:c.release()
        if node:
            if pub:pub.publish(Float32MultiArray(data=[0.,0.,0.]))
            node.destroy_node()
            rclpy.shutdown()
        if not args.no_display:cv2.destroyAllWindows()
        stats['elapsed_seconds'] = time.monotonic()-start
        if args.report:Path(args.report).write_text(json.dumps(stats,indent=2)+'\n')
        print(json.dumps(stats),flush=True)


if __name__ == '__main__':
    main()
