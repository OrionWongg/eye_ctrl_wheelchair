#!/usr/bin/env python3
"""Pretrained RITnet near-eye segmentation and ROS gesture input."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np

from dual_eye_tracker import orient_frame, gaze, BinocularState
from ritnet_backend import TensorRTModel, AdaptiveRitnet, segmentation_metrics, draw_segmentation
from gaze_calibration import CalibrationStore, GazeCalibration, camera_identity, model_signature
from intent_feedback import parse_feedback, display_lines
from gaze_3d import EyeGeometry, CenterCalibration

ROOT=Path(__file__).resolve().parents[1]


class LatestCamera:
    """Continuously drain UVC; consumers take the newest completed frame only."""
    def __init__(self,device):
        self.cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        if not self.cap.isOpened():raise RuntimeError('Cannot open camera: '+device)
        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        # Multiple driver buffers allow USB capture/decode overlap. The reader
        # drains continuously, so application latency stays at the newest frame.
        self.cap.set(cv2.CAP_PROP_FPS,30);self.cap.set(cv2.CAP_PROP_BUFFERSIZE,4)
        self.latest=None;self.error=None;self.stopping=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()
    def run(self):
        seq=0
        while not self.stopping.is_set():
            ok,frame=self.cap.read()
            if not ok:
                self.error='Camera disconnected or read failed';return
            seq+=1
            self.latest=(seq,time.monotonic(),frame)
    def close(self):
        self.stopping.set();self.thread.join(timeout=2)
        self.cap.release()


class EyeState:
    """Segmentation-based closure with recent tracking and surrounding-image checks.

    RITnet does not have a closed-eye class. These are conservative temporal
    heuristics, not a separately trained blink network.
    """
    def __init__(self):
        self.open_time=None;self.reference=None;self.outer=None
        self.area=None;self.bbox=None;self.diagnostics={}
    def update(self,frame,labels,confidence,metrics,now,appearance_frame=None,roi=None):
        gray=cv2.cvtColor(cv2.resize(frame,(labels.shape[1],labels.shape[0])),cv2.COLOR_BGR2GRAY).astype(np.float32)/255.
        # Use the camera field of view for surrounding appearance, even when
        # segmentation runs on a tight crop. Keep local segmentation geometry.
        appearance=gray if appearance_frame is None else cv2.cvtColor(
            cv2.resize(appearance_frame,(320,240)),cv2.COLOR_BGR2GRAY).astype(np.float32)/255.
        state='unknown'
        self.diagnostics=dict(reason='NO_RECENT_OPEN',outer_fraction=None,outer_delta=None,area_ratio=None)
        if metrics['pupil'] is not None and metrics['confidence']>=.85 and metrics['eye_area']>labels.size*.01:
            state='open';self.open_time=now
            self.diagnostics['reason']='PUPIL_VISIBLE'
            self.bbox=metrics['eye_bbox']
            x,y,w,h=self.bbox
            area=float(np.count_nonzero(labels[y:y+h,x:x+w]))
            self.area=area if self.area is None else .9*self.area+.1*area
            # Map the eye exclusion (including its margin) to the appearance view.
            if appearance_frame is None:
                x0,y0,x1,y1=x-10,y-10,x+w+10,y+h+10
            else:
                rx,ry,rw,rh=roi
                sx=appearance.shape[1]/appearance_frame.shape[1]
                sy=appearance.shape[0]/appearance_frame.shape[0]
                x0=(rx+(x-10)*rw/labels.shape[1])*sx
                x1=(rx+(x+w+10)*rw/labels.shape[1])*sx
                y0=(ry+(y-10)*rh/labels.shape[0])*sy
                y1=(ry+(y+h+10)*rh/labels.shape[0])*sy
            self.outer=np.ones(appearance.shape,dtype=bool)
            self.outer[max(0,int(np.floor(y0))):max(0,int(np.ceil(y1))),
                       max(0,int(np.floor(x0))):max(0,int(np.ceil(x1)))]=False
            self.reference=appearance
        elif self.open_time is not None and now-self.open_time<1.5 and self.area:
            outer_fraction=float(self.outer.mean())
            outer_delta=float(np.mean(np.abs(appearance[self.outer]-self.reference[self.outer]))) if outer_fraction else None
            stable=outer_fraction>.2 and outer_delta<.09
            x,y,w,h=self.bbox
            area_ratio=float(np.count_nonzero(labels[y:y+h,x:x+w]))/self.area
            collapsed=area_ratio<.18
            self.diagnostics.update(outer_fraction=outer_fraction,outer_delta=outer_delta,area_ratio=area_ratio)
            confident=float(np.mean(confidence))>.85
            textured=float(np.std(gray))>.025
            self.diagnostics['reason']=('INSUFFICIENT_SURROUNDINGS' if outer_fraction<=.2 else
                'SURROUNDINGS_UNSTABLE' if not stable else
                'EYE_NOT_COLLAPSED' if not collapsed else 'LOW_CONFIDENCE' if not confident else
                'NO_TEXTURE' if not textured else 'EYE_COLLAPSED')
            if stable and collapsed and confident and textured:
                state='closed'
        # Unknown is an observation, not a repeat of the previous open/closed
        # label. BlinkEvents owns the bounded temporal bridge.
        return state


def main():
    p=argparse.ArgumentParser(description=__doc__)
    paths=sorted(str(x) for x in Path('/dev/v4l/by-path').glob('*-video-index0'))
    p.add_argument('--eyes',choices=('left','both'),default='both',
                   help='Use both eyes by default; left selects single-eye diagnostics')
    p.add_argument('--left-camera',default=paths[0] if paths else '/dev/video2')
    p.add_argument('--right-camera',default=paths[1] if len(paths)>1 else '/dev/video0')
    p.add_argument('--left-rotation',type=int,choices=(0,90,180,270),default=180)
    p.add_argument('--right-rotation',type=int,choices=(0,90,180,270),default=0)
    p.add_argument('--engine',default=str(ROOT/'models/ritnet/ritnet_640x480.engine'))
    p.add_argument('--crop-engine',default=str(ROOT/'models/ritnet/ritnet_320x240.engine'))
    p.add_argument('--batch-engine',default=str(ROOT/'models/ritnet/ritnet_320x240_b2.engine'))
    p.add_argument('--full-frame',action='store_true',help='Disable automatic native-scale crop')
    p.add_argument('--publish-ros',action='store_true')
    p.add_argument('--publish-stamped',action='store_true',
                   help='Also publish EyeSample (implies --publish-ros); source time is oldest camera read completion, not hardware exposure')
    p.add_argument('--no-display',action='store_true')
    p.add_argument('--max-frames',type=int,default=0)
    p.add_argument('--report')
    p.add_argument('--gaze-method',choices=('3d','legacy'),default='3d',help='3D eyeball orientation (default) or previous three-point mapping')
    p.add_argument('--center-file',default=str(ROOT/'state/ritnet-center-3d.json'))
    p.add_argument('--camera-focal-px',type=float,help='Focal length at full capture resolution; default from datasheet diagonal FOV 80 degrees')
    p.add_argument('--gaze-span-deg',type=float,default=25.,help='Angular offset mapped to full horizontal command')
    p.add_argument('--left-gaze-sign',type=int,choices=(-1,1),default=-1)
    p.add_argument('--right-gaze-sign',type=int,choices=(-1,1),default=-1)
    p.add_argument('--calibration-file',default=str(ROOT/'state/ritnet-calibration.json'))
    p.add_argument('--calibration-samples',type=int,default=60)
    p.add_argument('--calibration-settle',type=float,default=.4)
    p.add_argument('--recalibrate',action='store_true',help='Ignore saved calibration this run and start a new capture')
    args=p.parse_args()
    if args.calibration_samples<10 or args.calibration_settle<0:p.error('Need >=10 calibration samples and nonnegative settling time')
    if not np.isfinite(args.gaze_span_deg) or args.gaze_span_deg<=0:p.error('Need positive finite angular span')
    if args.camera_focal_px is not None and (not np.isfinite(args.camera_focal_px) or args.camera_focal_px<=0):p.error('Invalid focal length')
    if not Path(args.engine).exists():p.error('Missing engine. Run tools/build_ritnet.sh first.')
    devices=[args.left_camera] if args.eyes=='left' else [args.left_camera,args.right_camera]
    if len(devices)==2 and Path(devices[0]).resolve()==Path(devices[1]).resolve():p.error('Two different cameras required')
    cv2.setNumThreads(2)
    cameras=[];model=None;node=None;pub=None;stamped_pub=None
    source_session=str(uuid.uuid4());source_sequence=0;source_frame_at=None;source_frame_stamp_ns=None

    def publish_signal(horizontal,closed,valid,frame_at=None):
        nonlocal source_sequence,source_frame_at,source_frame_stamp_ns
        if frame_at is not None and frame_at!=source_frame_at:
            source_frame_at=frame_at
            source_frame_stamp_ns=int((time.time()-max(0.,time.monotonic()-frame_at))*1_000_000_000)
        if pub:
            pub.publish(Float32MultiArray(data=[float(horizontal),float(closed),float(valid)]))
        if stamped_pub:
            sample=EyeSample()
            source_sequence+=1
            sample.source_session_id=source_session;sample.sequence=source_sequence
            sample.horizontal=float(horizontal);sample.closed=bool(closed);sample.valid=bool(valid)
            # SYSTEM/WALL timestamp of the oldest completed camera read. This
            # preserves inference/queue age; it is NOT hardware exposure time.
            # An invalid notification before any read uses its notification time.
            # Reusing a frame for an invalid/exit notification keeps its exact
            # original stamp, rather than remapping it with clock-read jitter.
            stamp_ns=source_frame_stamp_ns if source_frame_stamp_ns is not None else time.time_ns()
            sample.header.stamp.sec=stamp_ns//1_000_000_000
            sample.header.stamp.nanosec=stamp_ns%1_000_000_000
            stamped_pub.publish(sample)
    control_feedback={'stamp':0.,'state':'NO_CONTROLLER','resume_phase':'','resume_progress':0.}
    def receive_control_feedback(msg):
        try:
            control_feedback.update(parse_feedback(msg.data,time.monotonic()))
        except (ValueError,TypeError,AttributeError):
            return
    samples=[];ages=[];pair_times=[];frames=0;found=[0]*len(devices)
    state_counts=[dict(open=0,closed=0,unknown=0) for _ in devices]
    calibration=None;geometry=[];geometry_times=[]
    signs=[args.left_gaze_sign] if len(devices)==1 else [args.left_gaze_sign,args.right_gaze_sign]
    hist=[deque(maxlen=3) for _ in devices]
    eye_states=[EyeState() for _ in devices]
    previous_rois=[None for _ in devices]
    binocular=BinocularState()
    last_gaze=0.;last_seq=[0]*len(devices);last_frame=time.monotonic()
    last_draw=0.;preview_frames=0;last_geometry_log=0.
    last_input_status=None;last_input_log=0.
    started=None
    try:
        if args.full_frame:
            model=TensorRTModel(args.engine)
            for _ in range(10):model.infer(np.zeros((480,640,3),np.uint8))
        else:
            model=AdaptiveRitnet(args.engine,args.crop_engine,args.batch_engine if len(devices)==2 else None)
            model.warmup()
        if args.publish_ros or args.publish_stamped:
            import rclpy
            from std_msgs.msg import Float32MultiArray, String
            if args.publish_stamped:
                try:
                    from wheelchair_interfaces.msg import EyeSample
                except ImportError as exc:
                    raise RuntimeError('--publish-stamped requires wheelchair_interfaces/EyeSample; build and source the wheelchair workspace, or set EYE_WHEELCHAIR_SETUP to its install/setup.bash') from exc
            rclpy.init();node=rclpy.create_node('ritnet_eye_tracker')
            pub=node.create_publisher(Float32MultiArray,'/eye_tracking/signals',10)
            if args.publish_stamped:
                stamped_pub=node.create_publisher(EyeSample,'/eye_tracking/samples',1)
            node.create_subscription(String,'/eye_control/feedback',receive_control_feedback,1)
        for d in devices:cameras.append(LatestCamera(d))
        signature=model_signature([args.engine] if args.full_frame else [args.engine,args.crop_engine,args.batch_engine])
        bindings=[dict(camera=camera_identity(d),rotation=args.left_rotation if i==0 else args.right_rotation,
                       width=int(c.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),height=int(c.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                       model=signature,feature='ritnet-normalized-pupil-x-v1') for i,(d,c) in enumerate(zip(devices,cameras))]
        if args.gaze_method=='3d':
            for binding in bindings:
                width,height=binding['width'],binding['height']
                if binding['rotation'] in (90,270):width,height=height,width
                geometry.append(EyeGeometry((width,height),args.camera_focal_px))
                binding.update(feature='pye3d-0.3.2-normal-yaw-v1',focal_px=geometry[-1].focal,
                               gaze_sign=signs[len(geometry)-1],gaze_span_deg=args.gaze_span_deg)
            calibration=CenterCalibration(args.center_file,bindings,args.calibration_samples,args.calibration_settle,args.gaze_span_deg)
            print('3D gaze: automatic eye model + one straight-ahead reference; nominal 80-degree diagonal camera intrinsics unless --camera-focal-px supplied.',flush=True)
            print('Slowly look around until 3D_READY, then look straight ahead and press S. A/D are not needed.',flush=True)
        else:
            store=CalibrationStore(args.calibration_file)
            calibration=GazeCalibration(store,bindings,args.calibration_samples,args.calibration_settle)
            if store.warning:print(store.warning,flush=True)
        if args.recalibrate:calibration.restart()
        print(f'Calibration: {calibration.status}; method={args.gaze_method}',flush=True)
        last_frame=time.monotonic()
        print('RITnet TensorRT ready:',devices,flush=True)
        while True:
            if any(c.error for c in cameras):raise RuntimeError(str([c.error for c in cameras]))
            incoming=[c.latest for c in cameras]
            now=time.monotonic()
            if any(s is None or s[0]==seq for s,seq in zip(incoming,last_seq)):
                if now-last_frame>2:raise RuntimeError('No fresh camera frames for 2 seconds')
                time.sleep(.002);continue
            last_seq=[s[0] for s in incoming];last_frame=now
            if started is None:started=now
            pair_start=time.perf_counter()
            show_frame=not args.no_display and now-last_draw>=1/15
            panels=[];states=[];positions=[];raw_positions=[];angles=[];geometry_ok=[]
            oriented=[orient_frame(raw,args.left_rotation if i==0 else args.right_rotation) for i,(_,_,raw) in enumerate(incoming)]
            if args.full_frame:
                views=[(f,*model.infer(f),(0,0,f.shape[1],f.shape[0])) for f in oriented]
            else:
                views=model.infer_views(oriented)
            for i,((_,stamp,_),frame,view) in enumerate(zip(incoming,oriented,views)):
                local,labels,confidence,ms,roi=view
                if previous_rois[i]!=roi:
                    eye_states[i]=EyeState();previous_rois[i]=roi
                metrics=segmentation_metrics(labels,confidence)
                state=eye_states[i].update(local,labels,confidence,metrics,time.monotonic())
                x0,y0,rw,rh=roi
                if not args.full_frame:
                    full_metrics=dict(metrics)
                    if metrics['pupil']:
                        (cx,cy),axes,angle=metrics['pupil']
                        full_metrics['pupil']=((x0+cx*rw/labels.shape[1],y0+cy*rh/labels.shape[0]),axes,angle)
                    model.feedback(i,state,full_metrics,time.monotonic())
                age=time.monotonic()-stamp
                if age>.25:state='unknown'
                ages.append(age*1000);samples.append(ms)
                states.append(state)
                state_counts[i][state]+=1
                if state=='open' and metrics['pupil'] is not None:
                    pos=(x0+metrics['pupil'][0][0]*rw/labels.shape[1])/frame.shape[1]
                    hist[i].append(pos);found[i]+=1
                    raw_positions.append(pos if metrics['confidence']>=.85 else None)
                    positions.append(float(np.median(list(hist[i])[-3:])))
                else:
                    hist[i].clear();positions.append(None);raw_positions.append(None)
                if geometry:
                    bbox=metrics['eye_bbox']
                    if bbox is not None:
                        bx,by,bw,bh=bbox
                        bbox=(x0+bx*rw/labels.shape[1],y0+by*rh/labels.shape[0],bw*rw/labels.shape[1],bh*rh/labels.shape[0])
                    geometry_start=time.perf_counter()
                    angle,ok=geometry[i].update(frame,metrics['pupil'],roi,labels.shape,metrics['confidence'],state,stamp,bbox)
                    geometry[i].ms=(time.perf_counter()-geometry_start)*1000
                    angles.append(angle);geometry_ok.append(ok);geometry_times.append(geometry[i].ms)
                if show_frame:
                    panel=frame.copy()
                    panel[y0:y0+rh,x0:x0+rw]=draw_segmentation(local,labels,metrics)
                    cv2.putText(panel,f'{"LEFT" if i==0 else "RIGHT"} {state.upper()}  conf={metrics["confidence"]:.2f}  {ms:.1f}ms',
                                (10,28),0,.6,(0,255,255),1)
                    if geometry:
                        cv2.putText(panel,geometry[i].reason,(10,52),0,.45,(0,220,255),1)
                        cv2.putText(panel,f'yaw={angles[-1] if angles[-1] is not None else 0.:+.1f} deg | 3D {geometry[i].ms:.1f} ms',(10,74),0,.45,(0,220,255),1)
                    if panel.shape[0]!=480:
                        panel=cv2.resize(panel,(round(panel.shape[1]*480/panel.shape[0]),480))
                    panels.append(panel)
            if geometry and now-last_geometry_log>=2.:
                print('GAZE3D '+json.dumps([dict(reason=g.reason,**g.diagnostics) for g in geometry]),flush=True)
                last_geometry_log=now
            synced=max(s[1] for s in incoming)-min(s[1] for s in incoming)<.08
            fresh=all(time.monotonic()-s[1]<.25 for s in incoming)
            if geometry:
                if any(g.recenter_required for g in geometry):
                    calibration.invalidate_rotation()
                    for g in geometry:g.recenter_required=False
                    print('Camera rotation detected: stopped. After model settles, look straight ahead and press S.',flush=True)
                geometry_valid=all(geometry_ok)
                calibration.observe(angles,states,min(s[1] for s in incoming),fresh and synced and geometry_valid)
                candidate,agreed=calibration.horizontal(angles,signs)
                ready=calibration.ready and geometry_valid and agreed
                closed,valid=binocular.update(states if len(states)==2 else states*2,time.monotonic(),ready and synced and fresh)
                if valid and all(state=='open' for state in states):last_gaze=candidate
            else:
                calibration.observe(raw_positions,states,min(s[1] for s in incoming),fresh and synced)
                ready=calibration.ready
                closed,valid=binocular.update(states if len(states)==2 else states*2,time.monotonic(),ready and synced and fresh)
                if valid and states[0]=='open' and all(x is not None for x in positions):
                    last_gaze=float(np.mean([gaze(x,c) for x,c in zip(positions,calibration.values)]))
            if pub:
                publish_signal(last_gaze if valid else 0.,closed,valid,min(s[1] for s in incoming))
                rclpy.spin_once(node,timeout_sec=0.)
            # Log transitions as well as a heartbeat: periodic geometry logs alone
            # miss short blinks and the invalid sample that clears blink counting.
            blockers=[]
            if not calibration.ready:blockers.append('CENTER_NOT_READY')
            if not synced:blockers.append('CAMERAS_OUT_OF_SYNC')
            if not fresh:blockers.append('STALE_FRAME')
            if geometry:
                for i,ok in enumerate(geometry_ok):
                    if not ok:blockers.append(('LEFT' if i==0 else 'RIGHT')+': '+geometry[i].reason)
                if calibration.ready and geometry_valid and not agreed:blockers.append('GAZE_DISAGREEMENT')
            if not valid and not blockers:blockers.append('EYE_STATE_NOT_CONFIRMED')
            feedback_state=(control_feedback['state'] if time.monotonic()-control_feedback['stamp']<.5 else 'NO_CONTROLLER')
            input_status=(tuple(states),bool(closed),bool(valid),tuple(blockers),feedback_state)
            if input_status!=last_input_status or now-last_input_log>=2.:
                print('EYE_INPUT '+json.dumps(dict(eyes=states,closed=bool(closed),valid=bool(valid),
                    blockers=blockers,controller=feedback_state,horizontal=last_gaze if valid else 0.)),flush=True)
                last_input_status=input_status;last_input_log=now
            frames+=1
            pair_times.append((time.perf_counter()-pair_start)*1000)
            if not args.no_display:
                if show_frame:
                    canvas=np.hstack(panels)
                    feedback_fresh=time.monotonic()-control_feedback['stamp']<.5
                    controller_state=control_feedback['state'] if feedback_fresh else 'NO_CONTROLLER'
                    phase=control_feedback['resume_phase']
                    hints={'OPEN_FIRST':'Open eyes, then blink once to resume',
                           'BLINK_TO_RESUME':'Blink once to resume',
                           'REOPEN_TO_RESUME':'Reopen eyes to resume'}
                    hint=(hints.get(phase,'') if controller_state=='PAUSED' else
                          'Double blink to enable' if controller_state=='DISARMED' else
                          'Long close: pause' if controller_state=='ACTIVE' else
                          'Waiting for safety / valid tracking' if controller_state in ('BLOCKED','INVALID') else
                          'Start the controller to drive')
                    cv2.rectangle(canvas,(0,canvas.shape[0]-151),(canvas.shape[1],canvas.shape[0]-55),(24,30,40),-1)
                    if control_feedback.get('interaction_mode') in ('intent_confirm','directional_intent'):
                        for row,line in enumerate(display_lines(control_feedback,feedback_fresh)):
                            width=cv2.getTextSize(line,0,.50,1)[0][0]
                            scale=min(.50,.50*(canvas.shape[1]-20)/max(width,1))
                            cv2.putText(canvas,line,(10,canvas.shape[0]-127+row*27),0,scale,(255,255,255),1)
                    else:
                        cv2.putText(canvas,f'{controller_state}: {hint}',(10,canvas.shape[0]-112),0,.55,(255,255,255),1)
                    cv2.putText(canvas,calibration.status,(10,canvas.shape[0]-40),0,.5,(0,255,255),1)
                    cv2.putText(canvas,f'{frames/max(time.monotonic()-started,.001):.1f} FPS | calibrated={ready} valid={int(valid)} gaze={last_gaze:+.2f}',
                                (10,canvas.shape[0]-15),0,.5,(0,255,255),1)
                    cv2.imshow('EYETLE RITnet',canvas)
                    last_draw=now;preview_frames+=1
                key=cv2.waitKey(1)&255
                if key==27:break
                reset_control=False
                if key in (ord('c'),ord('C')):
                    calibration.restart();reset_control=True
                keymap={ord('a'):'left',ord('A'):'left',ord('1'):'left',
                        ord('s'):'center',ord('S'):'center',ord('2'):'center',
                        ord('d'):'right',ord('D'):'right',ord('3'):'right'}
                if key in keymap:
                    reset_control=calibration.begin(keymap[key],time.monotonic()) or reset_control
                if key in (10,13) and calibration.pending_save:
                    calibration.save_pending()
                if reset_control:
                    for h in hist:h.clear()
                    binocular=BinocularState();last_gaze=0.
                    if pub:publish_signal(0.,False,False)
            if args.max_frames and frames>=args.max_frames:break
    except KeyboardInterrupt:
        pass
    finally:
        finished=time.monotonic()
        if node:
            if pub:publish_signal(0.,False,False)
            node.destroy_node();rclpy.try_shutdown()
        for g in geometry:g.close()
        for c in cameras:c.close()
        if model:model.close()
        if not args.no_display:cv2.destroyAllWindows()
        elapsed=finished-started if started else 0
        report=dict(frames=frames,preview_frames=preview_frames,devices=devices,
                    controller_feedback_state=control_feedback['state'] if time.monotonic()-control_feedback['stamp']<.5 else 'NO_CONTROLLER',
                    controller_resume_progress=control_feedback['resume_progress'],
                    gaze_method=args.gaze_method,shift_events=[g.shift_events for g in geometry],geometry_reasons=[g.reason for g in geometry],
                    geometry_ms_p50=float(np.median(geometry_times)) if geometry_times else None,
                    geometry_ms_p95=float(np.percentile(geometry_times,95)) if geometry_times else None,
                    calibration_file=args.center_file if geometry else args.calibration_file,calibration_loaded=calibration.loaded if calibration else 0,
                    calibrated=calibration.ready if calibration else False,open_frames=found,state_frames=state_counts,elapsed_seconds=elapsed,
                    camera_pipeline_fps=frames/elapsed if elapsed else 0,
                    inference_ms_p50=float(np.median(samples)) if samples else None,
                    inference_ms_p95=float(np.percentile(samples,95)) if samples else None,
                    processing_ms_p95=float(np.percentile(pair_times,95)) if pair_times else None,
                    frame_age_after_processing_ms_p95=float(np.percentile(ages,95)) if ages else None,
                    note='Frame age excludes exposure/USB latency. Batch inference time is amortized per eye. Open detections are not accuracy ground truth.')
        print(json.dumps(report),flush=True)
        if args.report:Path(args.report).write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
