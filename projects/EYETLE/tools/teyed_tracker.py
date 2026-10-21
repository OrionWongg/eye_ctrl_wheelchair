#!/usr/bin/env python3
"""Binocular control: TEyeD (default) or Itoh 3D direction, plus RITnet closure."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
from ritnet_tracker import LatestCamera
from dual_eye_tracker import orient_frame
from teyed_backend import EyeNetwork, BinocularInput, eye_state
from ritnet_blink import RitnetBlink

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--eyes',choices=['left','both'],default='both')
    p.add_argument('--direction-method',choices=['teyed','itoh'],default='teyed')
    p.add_argument('--itoh-library',default=str(ROOT/'build/itoh/libitoh.so'))
    p.add_argument('--itoh-focal-px',type=float,default=476.70143703768406,
                   help='Itoh camera focal length at 640x480; default nominal, not measured')
    paths=list(Path('/dev/v4l/by-path').glob('*-video-index0'))
    for side,port,rotation in [('left','4.4.1',180),('right','4.4.2',0)]:
        device=next((str(x) for x in paths if f':{port}:1.0-' in str(x)),f'/dev/v4l/by-path/missing-{side}-camera')
        p.add_argument(f'--{side}-camera',default=device)
        p.add_argument(f'--{side}-rotation',type=int,choices=[0,90,180,270],default=rotation)
        p.add_argument(f'--{side}-gaze-sign',type=int,choices=[-1,1],default=-1)
    p.add_argument('--gaze-engine',default=str(ROOT/'models/teyed/teyed.engine'))
    p.add_argument('--blink-engine',default=str(ROOT/'models/teyed/meye.engine'))
    p.add_argument('--blink-method',choices=['ritnet','meye'],default='ritnet',
                   help='Closure detector: RITnet by default; MEYE for comparison')
    p.add_argument('--ritnet-engine',default=str(ROOT/'models/ritnet/ritnet_640x480.engine'))
    p.add_argument('--ritnet-crop-engine',default=str(ROOT/'models/ritnet/ritnet_320x240.engine'))
    p.add_argument('--ritnet-batch-engine',default=str(ROOT/'models/ritnet/ritnet_320x240_b2.engine'))
    p.add_argument('--gaze-span',type=float,default=.25)
    p.add_argument('--agreement',type=float,default=.15)
    p.add_argument('--blink-threshold',type=float,
                   help='MEYE only: closed probability threshold (default 0.35)')
    p.add_argument('--publish-ros',action='store_true')
    p.add_argument('--no-display',action='store_true')
    p.add_argument('--max-frames',type=int,default=0)
    p.add_argument('--report')
    p.add_argument('--left-video',help='Offline replay instead of cameras; never publishes ROS')
    p.add_argument('--right-video')
    args=p.parse_args()
    if not np.isfinite(args.itoh_focal_px) or args.itoh_focal_px<=0:p.error('Invalid Itoh focal length')
    if args.blink_threshold is not None and args.blink_method!='meye':
        p.error('--blink-threshold applies only to --blink-method meye; RITnet uses segmentation')
    if args.blink_threshold is None:args.blink_threshold=.35
    if not np.isfinite(args.blink_threshold) or not .2<args.blink_threshold<=1:
        p.error('--blink-threshold must be finite and in (0.2, 1]')
    if args.left_video and args.publish_ros:p.error('Offline replay cannot publish control signals')
    if args.right_video and not args.left_video:p.error('--right-video requires --left-video')
    if args.left_video and args.eyes=='both' and not args.right_video:p.error('Both eye videos required')
    devices=[args.left_camera] if args.eyes=='left' else [args.left_camera,args.right_camera]
    if len(devices)==2 and Path(devices[0]).resolve()==Path(devices[1]).resolve():p.error('Two different cameras required')
    count=len(devices);signs=[args.left_gaze_sign,args.right_gaze_sign][:count]
    fusion=BinocularInput(count,signs,args.gaze_span,args.agreement)
    rotations=[args.left_rotation,args.right_rotation][:count]
    cameras=[];videos=[];gaze=blink=node=pub=None
    feedback={'state':'NO_CONTROLLER','at':0.}
    def receive(msg):
        try:
            value=json.loads(msg.data)
            feedback.update(state=str(value['state']),at=time.monotonic())
        except (ValueError,KeyError,TypeError):pass
    def publish(h=0.,closed=False,valid=False):
        if pub:pub.publish(Float32MultiArray(data=[float(h),float(closed),float(valid)]))
    n=valid_frames=closed_frames=0;last_status=None;last_log=0.;started=time.monotonic();durations=[];gaze_details=[]
    try:
        cv2.setNumThreads(2)
        if args.direction_method=='itoh':
            from itoh_backend import ItohEyes
            gaze=ItohEyes(count,args.itoh_library,args.itoh_focal_px)
        else:gaze=EyeNetwork(args.gaze_engine,'teyed')
        if args.blink_method=='ritnet':
            blink=RitnetBlink(args.ritnet_engine,args.ritnet_crop_engine,args.ritnet_batch_engine,count)
            blink.warmup()
        else:blink=EyeNetwork(args.blink_engine,'meye')
        if args.publish_ros:
            import rclpy
            from std_msgs.msg import Float32MultiArray,String
            rclpy.init();node=rclpy.create_node(args.direction_method+'_eye_tracker')
            pub=node.create_publisher(Float32MultiArray,'/eye_tracking/signals',10)
            node.create_subscription(String,'/eye_control/feedback',receive,10)
        if args.left_video:
            for path in [args.left_video,args.right_video][:count]:
                cap=cv2.VideoCapture(path);videos.append(cap)
                if not cap.isOpened():raise RuntimeError('Cannot open '+path)
            rates=[v.get(cv2.CAP_PROP_FPS) for v in videos]
            if min(rates)<=0 or max(rates)-min(rates)>.01:raise ValueError('Replay videos need matching valid FPS')
        else:
            for d in devices:cameras.append(LatestCamera(d))
        if args.direction_method=='itoh':
            print('ITOH: Slowly look around to build both eye models (30 spatially spread observations). Then look straight ahead for auto center. R: rebuild models; C: recenter; Esc: exit.',flush=True)
        else:
            print('TEYED: Look straight ahead with both eyes open for 2 stable seconds. No S key or eye sweep. C: reset center; Esc: exit.',flush=True)
        print(args.direction_method.upper()+' direction + '+args.blink_method.upper()+' closure',flush=True)
        if args.blink_method=='meye':
            print(f'MEYE blink threshold={args.blink_threshold:.2f}; eye confidence >=0.80; open probability <=0.20',flush=True)
        seq=[0]*count;last_frame=time.monotonic()
        while True:
            if node:rclpy.spin_once(node,timeout_sec=0.)
            if videos:
                reads=[v.read() for v in videos]
                if not all(ok for ok,_ in reads):break
                stamp=n/rates[0];stamps=[stamp]*count;frames=[f for _,f in reads];fresh=True
            else:
                if any(c.error for c in cameras):raise RuntimeError('Camera disconnected')
                incoming=[c.latest for c in cameras];now=time.monotonic()
                if any(s is None or s[0]==q for s,q in zip(incoming,seq)):
                    if now-last_frame>.25:publish()
                    if now-last_frame>2.:raise RuntimeError('Camera frame timeout')
                    time.sleep(.002);continue
                seq=[s[0] for s in incoming];last_frame=now
                stamps=[s[1] for s in incoming];stamp=min(stamps);frames=[s[2] for s in incoming]
                fresh=max(s[1] for s in incoming)-stamp<.08
            frames=[orient_frame(f,r) for f,r in zip(frames,rotations)]
            t=time.perf_counter()
            gaze_details=[]
            if args.direction_method=='itoh':values,gaze_details=gaze.infer(frames)
            else:values=[gaze.infer(f) for f in frames]
            tags=[];blink_details=[]
            if args.blink_method=='ritnet':states,blink_details=blink.infer(frames,stamps)
            else:
                tags=[blink.infer(f) for f in frames]
                states=[eye_state(tag,args.blink_threshold) for tag in tags]
            if not videos:fresh=fresh and time.monotonic()-stamp<.25
            h,closed,valid,reason=fusion.update(values,states,stamp,fresh)
            publish(h,closed,valid)
            durations.append((time.perf_counter()-t)*1000);n+=1
            valid_frames+=int(valid);closed_frames+=int(valid and closed)
            state=feedback['state'] if time.monotonic()-feedback['at']<.5 else 'NO_CONTROLLER'
            status=(tuple(states),closed,valid,reason,state,tuple(d['reason'] for d in gaze_details))
            if status!=last_status or stamp-last_log>=2.:
                print(args.direction_method.upper()+'_INPUT '+json.dumps(dict(t=stamp,eyes=states,tags=[x.tolist() for x in tags],
                    gaze=[[float(v) if np.isfinite(v) else None for v in x] for x in values],
                    direction_method=args.direction_method,gaze_details=gaze_details,
                    center=None if not fusion.center.ready else fusion.center.center.tolist(),gaze_signs=signs,
                    blink_method=args.blink_method,blink_details=blink_details,
                    blink_threshold=args.blink_threshold if args.blink_method=='meye' else None,center_ready=fusion.center.ready,
                    center_progress=fusion.center.progress,closed=closed,valid=valid,reason=reason,
                    horizontal=h,controller=state)),flush=True)
                last_status=status;last_log=stamp
            if not args.no_display:
                panels=[]
                for i,frame in enumerate(frames):
                    panel=cv2.resize(frame,(640,480))
                    cv2.putText(panel,f'{"LEFT" if i==0 else "RIGHT"}: {states[i]} | x={values[i][0]:+.3f}',(10,28),0,.6,(0,255,255),1)
                    if gaze_details:
                        d=gaze_details[i]
                        if d['detected']:
                            cx,cy,w,h,angle=d['ellipse']
                            cv2.ellipse(panel,((cx,cy),(w,h),angle),(0,255,0),1)
                            if d['valid']:
                                sx,sy,sz=d['sphere_center']
                                origin=(round(320+args.itoh_focal_px*sx/sz),round(240+args.itoh_focal_px*sy/sz))
                                cv2.arrowedLine(panel,origin,(round(cx),round(cy)),(0,200,255),2)
                        cv2.putText(panel,f"{d['reason']} {d['observations']}/30",(10,55),0,.45,(0,255,255),1)
                    panels.append(panel)
                canvas=np.hstack(panels)
                hint=(f'Look straight ahead: {fusion.center.progress:.0%}' if not fusion.center.ready else
                      'Center ready; start turtle controller to drive' if state=='NO_CONTROLLER' else
                      'Blink twice to enable' if state=='DISARMED' else
                      'Open eyes, then blink once to resume' if state=='PAUSED' else 'Long close: stop')
                if gaze_details and not all(d['model_built'] for d in gaze_details):hint='Slowly look around to build both eye models'
                cv2.putText(canvas,hint,(10,410),0,.65,(0,255,255),2)
                cv2.putText(canvas,f'{state} | {reason} | valid={int(valid)} gaze={h:+.2f}',(10,445),0,.55,(0,255,255),1)
                cv2.imshow('EYETLE '+args.direction_method.upper()+' + '+args.blink_method.upper(),canvas)
                key=cv2.waitKey(1)&255
                if key==27:break
                if key in (ord('c'),ord('C')):fusion.reset();publish()
                if args.direction_method=='itoh' and key in (ord('r'),ord('R')):gaze.reset();fusion.reset();publish()
            if args.max_frames and n>=args.max_frames:break
    except KeyboardInterrupt:pass
    finally:
        publish()
        if node:node.destroy_node();rclpy.try_shutdown()
        for c in cameras:c.close()
        for cap in videos:cap.release()
        for net in (gaze,blink):
            if net:net.close()
        if not args.no_display:cv2.destroyAllWindows()
        report=dict(direction_method=args.direction_method,blink_method=args.blink_method,frames=n,valid_frames=valid_frames,valid_closed_frames=closed_frames,
                    gaze_details=gaze_details,
                    center_ready=fusion.center.ready,center=None if not fusion.center.ready else fusion.center.center.tolist(),
                    elapsed=time.monotonic()-started,inference_pair_ms_p50=float(np.median(durations)) if durations else None,
                    inference_pair_ms_p95=float(np.percentile(durations,95)) if durations else None)
        print(json.dumps(report),flush=True)
        if args.report:Path(args.report).write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
