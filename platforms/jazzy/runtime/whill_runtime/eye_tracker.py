"""Dual UVC camera fusion frontend, with guided calibration and schema 1 output."""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
from .eye_core import Blinks, Calibration
from .vision import EyeDetector, Segmenter, ROOT


class LatestCamera:
    def __init__(self, device, rotation):
        self.device, self.rotation = device, rotation
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f'Cannot open camera {device}')
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.capture.set(cv2.CAP_PROP_FPS,30)
        self.lock = threading.Lock()
        self.latest = None
        self.running = True
        self.thread = threading.Thread(target=self.read,daemon=True)
        self.thread.start()

    def read(self):
        sequence = 0
        while self.running:
            ok, frame = self.capture.read()
            if not ok:
                time.sleep(.02)
                continue
            stamp = time.monotonic()
            if self.rotation == 180:
                frame = cv2.rotate(frame,cv2.ROTATE_180)
            sequence += 1
            with self.lock:
                self.latest = (sequence,stamp,frame)

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.capture.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left',default='/dev/video4')
    parser.add_argument('--right',default='/dev/video2')
    parser.add_argument('--left-rotation',type=int,choices=[0,180],default=180)
    parser.add_argument('--right-rotation',type=int,choices=[0,180],default=0)
    parser.add_argument('--publish-ros',action='store_true')
    parser.add_argument('--no-display',action='store_true')
    parser.add_argument('--max-frames',type=int,default=0)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--recalibrate',action='store_true')
    parser.add_argument('--calibration-file',type=Path,default=ROOT/'local/fusion-calibration.json')
    args=parser.parse_args()
    if args.publish_ros and args.no_display:
        parser.error('No-display diagnostics cannot publish control samples')
    node=publisher=feedback=None
    if args.publish_ros:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('whill_dual_eye_tracker')
        publisher=node.create_publisher(String,'/eye_tracking/fusion_samples',1)
        def receive(msg):
            nonlocal feedback
            try:
                feedback=json.loads(msg.data).get('status','')
            except (ValueError,AttributeError):
                feedback='WAITING FOR DRIVE STATUS'
        node.create_subscription(String,'/eye_control/feedback',receive,1)
    session=str(uuid.uuid4())
    model=ROOT/'projects/EYETLE/models/ritnet/ritnet_320x240.onnx'
    binding=dict(cameras=[str(Path(p).resolve()) for p in [args.left,args.right]],
        usb_paths=[str((Path('/sys/class/video4linux')/Path(p).resolve().name/'device').resolve()) for p in [args.left,args.right]],
        rotations=[args.left_rotation,args.right_rotation],
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        purest_sha256=hashlib.sha256((ROOT/'.local/lib/libwhill_purest.so').read_bytes()).hexdigest(),
        algorithm='full-frame-onnx-purest-reconstruction-v1')
    if binding['cameras'][0]==binding['cameras'][1]:
        parser.error('Left and right cameras must be distinct')
    calibration=Calibration(args.calibration_file,binding,args.recalibrate)
    cameras=[]
    detectors=[]
    records=[]
    last_sequences=[0,0]
    blinks=Blinks()
    sequence=0
    fresh_count=0
    title='WHILL Eyes - R recalibrate | Q quit'
    started=time.monotonic()
    last_pair=started
    try:
        segmenter=Segmenter()
        detectors=[EyeDetector(segmenter),EyeDetector(segmenter)]
        cameras=[LatestCamera(args.left,args.left_rotation)]
        cameras.append(LatestCamera(args.right,args.right_rotation))
        if not args.no_display:
            cv2.namedWindow(title,cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title,1280,800)
        while not args.max_frames or sequence<args.max_frames:
            pair=[c.snapshot() for c in cameras]
            if any(p is None or p[0]==previous for p,previous in zip(pair,last_sequences)):
                if time.monotonic()-last_pair>3:
                    raise RuntimeError('No new binocular frames for 3 seconds')
                time.sleep(.003)
                continue
            last_pair=time.monotonic()
            last_sequences=[p[0] for p in pair]
            source_t=min(p[1] for p in pair)
            stamp_ns=time.time_ns()-int((time.monotonic()-source_t)*1e9)
            detections=[d.run(p[2],p[1]) for d,p in zip(detectors,pair)]
            now=time.monotonic()
            fresh=now-source_t<.25 and abs(pair[0][1]-pair[1][1])<.08
            fresh_count+=int(fresh)
            if not args.no_display:
                calibration.feed(detections,now,fresh)
            horizontal,direction_valid=calibration.direction(detections)
            states=[d['state'] for d in detections]
            event,long_close=blinks.update(states,source_t,fresh)
            sequence+=1
            sample=dict(schema=1,session=session,sequence=sequence,stamp_ns=stamp_ns,
                source_t=source_t,horizontal=horizontal,fresh=bool(fresh),
                calibrated=calibration.ready,open=all(s=='open' for s in states),
                motion_valid=bool(fresh and calibration.ready and direction_valid and all(s=='open' for s in states)),
                gesture_continuity=bool(fresh and all(t is None or source_t-t<=.12 for t in blinks.unknown)),
                long_close=long_close,blink_event=event)
            if publisher is not None:
                publisher.publish(String(data=json.dumps(sample)))
                rclpy.spin_once(node,timeout_sec=0)
            if args.report:
                records.append(dict(sequence=sequence,states=states,fresh=bool(fresh),
                    age_ms=round((now-source_t)*1000,2),features=[d['features'] for d in detections],
                    horizontal=horizontal,motion_valid=sample['motion_valid'],blink_event=event))
            if not args.no_display:
                canvas=np.zeros((800,1280,3),np.uint8)
                target=calibration.ORDER[calibration.step] if not calibration.ready else 1
                cv2.circle(canvas,([130,640,1150][target],350),22,(0,255,255),-1)
                for i,(p,d) in enumerate(zip(pair,detections)):
                    small=cv2.resize(p[2],(480,360))
                    canvas[420:780,80+i*640:560+i*640]=small
                    cv2.putText(canvas,f'{"LEFT" if i==0 else "RIGHT"}: {d["state"]}',(80+i*640,410),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
                progress=min(1.,(calibration.samples[-1][0]-calibration.samples[0][0])/1.5) if calibration.samples else 0.
                lines=[calibration.label,calibration.message or (f'HOLD PROGRESS: {progress*100:.0f}%' if not calibration.ready else ''),
                       f'h={horizontal:+.2f}  fresh={fresh}  frame age={(now-source_t)*1000:.0f}ms',
                       'KEEP HEAD STILL; LOOK AT THE YELLOW DOT',
                       (feedback or 'Two short blinks start; gaze left/right turns; Space in driving window stops')[:135]]
                for i,line in enumerate(lines):
                    cv2.putText(canvas,line,(30,45+i*40),cv2.FONT_HERSHEY_SIMPLEX,.65,(200,255,200),2)
                cv2.imshow(title,canvas)
                key=cv2.waitKey(1)&0xff
                if key in (27,ord('q')) or cv2.getWindowProperty(title,cv2.WND_PROP_AUTOSIZE)<0:
                    break
                if key==ord('r'):
                    calibration=Calibration(args.calibration_file,binding,True)
                    blinks=Blinks()
                    session=str(uuid.uuid4())
                    sequence=0
    except KeyboardInterrupt:
        pass
    finally:
        if publisher is not None and rclpy.ok():
            sample=dict(schema=1,session=session,sequence=sequence+1,stamp_ns=time.time_ns(),
                source_t=time.monotonic(),horizontal=0.,fresh=False,calibrated=False,
                open=False,motion_valid=False,gesture_continuity=False,long_close=False,blink_event=None)
            publisher.publish(String(data=json.dumps(sample)))
        for c in cameras:
            c.close()
        for d in detectors:
            d.purest.close()
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
        cv2.destroyAllWindows()
        if args.report:
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(dict(binding=binding,seconds=time.monotonic()-started,
                frames=records),indent=2)+'\n')
    print(f'Eye tracker: {sequence} binocular frames, {fresh_count} fresh')


if __name__=='__main__':
    main()
