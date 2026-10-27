#!/usr/bin/env python3
"""RITnet + PuReST with centered guided calibration and saved direction mapping."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
from ritnet_tracker import LatestCamera
from ritnet_blink import RitnetBlink
from dual_eye_tracker import orient_frame
from gaze_calibration import camera_identity,model_signature
from fusion_calibration import GuidedCalibration
from fusion_control import FusionControl,combine_eye_state
from purest_backend import PuReST
from camera_paths import fusion_camera_defaults

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--eyes',choices=['left','both'],default='both')
    defaults = fusion_camera_defaults()
    for side,rotation in [('left',180),('right',0)]:
        device=defaults.get(side, f'/dev/v4l/by-path/missing-{side}-camera')
        p.add_argument(f'--{side}-camera',default=device)
        p.add_argument(f'--{side}-rotation',type=int,choices=[0,90,180,270],default=rotation)
    p.add_argument('--calibration-file',default=str(ROOT/'state/fusion-calibration.json'))
    p.add_argument('--recalibrate',action='store_true')
    p.add_argument('--saved-calibration-only',action='store_true')
    p.add_argument('--three-point',action='store_true',help='Legacy center/left/right profile file instead of center-only start')
    p.add_argument('--gaze-span',type=float,default=.19,help='Center-only: full-frame pupil-x offset for a full turn command')
    p.add_argument('--allow-calibration-hub-move',action='store_true')
    p.add_argument('--publish-ros',action='store_true')
    p.add_argument('--ipad-controls',action='store_true')
    p.add_argument('--no-display',action='store_true',help='Diagnostics only; no unattended calibration or ROS')
    p.add_argument('--max-frames',type=int,default=0)
    p.add_argument('--report')
    p.add_argument('--teyed-reference',action='store_true',help='Log TEyeD as a comparison, not a motion vote')
    args=p.parse_args()
    if args.ipad_controls and not args.publish_ros:p.error('--ipad-controls requires --publish-ros')
    if not args.three_point and (args.saved_calibration_only or args.recalibrate):
        p.error('--saved-calibration-only/--recalibrate require --three-point')
    if args.saved_calibration_only and args.recalibrate:
        p.error('Saved calibration is read-only; --recalibrate is disabled')
    if args.no_display and args.publish_ros:p.error('The guided control flow requires a display')
    count=1 if args.eyes=='left' else 2
    devices=[args.left_camera,args.right_camera][:count];rotations=[args.left_rotation,args.right_rotation][:count]
    for side, device in zip(('left', 'right'), devices):
        if not Path(device).exists():
            p.error(f'{side} camera path does not exist: {device}; '
                    f'use --{side}-camera with a capture device from /dev/v4l/by-path/')
    print('Eye cameras: ' + ', '.join(devices), flush=True)
    if count==2 and Path(devices[0]).resolve()==Path(devices[1]).resolve():p.error('Two different cameras required')
    ui=None;blink=None;pure=None;reference=None;node=None;pub=None;cameras=[];cal=None;control=None
    feedback=dict(blockers=[],state='NO_CONTROLLER',reason='',blink_count=0,event_decision='NONE',target='turtle',at=0.)
    source_session=str(uuid.uuid4());source_sequence=0
    trace=None
    def write_trace(record):
        nonlocal trace
        if trace is None:return
        try:
            trace.write(json.dumps(record,allow_nan=False)+'\n')
            trace.flush()
        except OSError as exc:
            print('FUSION_TRACE disabled: '+str(exc),flush=True)
            try:trace.close()
            except OSError:pass
            trace=None
    def receive(msg):
        try:
            d=json.loads(msg.data)
            if ui and ui.ipad_panel:ui.ipad_panel.update(d)
            feedback.update(blockers=d.get('blockers',[]),state=str(d['state']),reason=str(d.get('reason','')),
                                                  blink_count=int(d.get('blink_count',0)), target=str(d.get('target','turtle')),
                                                  event_decision=str(d.get('event_decision','NONE')),at=time.monotonic())
        except (ValueError,KeyError,TypeError):pass
    def publish(h=0.,closed=False,valid=False,source_t=None,fresh=False,calibrated=False,
                open_eyes=False,gesture_continuity=False,long_close=False,event=None):
        nonlocal source_sequence
        if pub:
            source_sequence+=1
            source_t=time.monotonic() if source_t is None else source_t
            stamp_ns=node.get_clock().now().nanoseconds-int(max(0.,time.monotonic()-source_t)*1e9)
            data=dict(schema=1,session=source_session,sequence=source_sequence,stamp_ns=stamp_ns,
                source_t=source_t,horizontal=float(h),motion_valid=bool(valid),fresh=bool(fresh),
                calibrated=bool(calibrated),open=bool(open_eyes),gesture_continuity=bool(gesture_continuity),
                long_close=bool(long_close),blink_event=event)
            pub.publish(String(data=json.dumps(data,allow_nan=False)))
            write_trace(dict(type='sample',wall_time=time.time(),received_monotonic=time.monotonic(),data=data))
    n=valid_count=closed_count=0;times=[];last_status=None;last_log=last_draw=0.
    try:
        if args.publish_ros:
            try:
                trace_dir=Path(os.environ.get('EYE_FUSION_TRACE_DIR',str(ROOT/'model-results/fusion-traces')))
                trace_dir.mkdir(parents=True,exist_ok=True)
                trace_path=trace_dir/(time.strftime('%Y%m%d-%H%M%S')+'-'+source_session+'.jsonl')
                trace=trace_path.open('w')
                print('FUSION_TRACE '+str(trace_path),flush=True)
            except OSError as exc:print('FUSION_TRACE unavailable: '+str(exc),flush=True)
        cv2.setNumThreads(2)
        if not args.no_display:
            from fusion_guide import GuideWindow
            ui=GuideWindow(ipad_controls=args.ipad_controls,center_only=not args.three_point);ui.poll()
        engine_paths=[ROOT/'models/ritnet'/f'ritnet_{size}.engine' for size in ('640x480','320x240','320x240_b2')]
        blink=RitnetBlink(*map(str,engine_paths),count);blink.warmup()
        pure=PuReST(count)
        if args.teyed_reference:
            from teyed_backend import EyeNetwork
            reference=EyeNetwork(ROOT/'models/teyed/teyed.engine','teyed')
        if args.publish_ros:
            import rclpy
            from std_msgs.msg import String
            rclpy.init();node=rclpy.create_node('fusion_eye_tracker')
            pub=node.create_publisher(String,'/eye_tracking/fusion_samples',1)
            node.create_subscription(String,'/eye_control/feedback',receive,1)
            if ui and ui.ipad_panel:
                ui_commands=node.create_publisher(String,'/whill/controller/ui',1)
                def send_ui_command(action):
                    ui_commands.publish(String(data=json.dumps(dict(
                        action=action,stamp_ns=node.get_clock().now().nanoseconds))))
                ui.ipad_panel.send=send_ui_command
        for d in devices:cameras.append(LatestCamera(d))
        binding=dict(cameras=[camera_identity(d) for d in devices],rotations=rotations,
            sizes=[[int(c.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(c.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))] for c in cameras],
            models=model_signature(engine_paths+[pure.library]),feature='full-image-pupil-x-v1',quality_threshold=.6)
        cal=GuidedCalibration(args.calibration_file,binding,count,
            saved_only=args.saved_calibration_only,allow_hub_move=args.allow_calibration_hub_move,
            center_only=not args.three_point,span=args.gaze_span)
        calibration_info=dict(mode='three-point' if args.three_point else 'center-only',span=cal.span,
            path=str(Path(args.calibration_file).resolve()),loaded=cal.loaded,
            saved_only=cal.saved_only,binding_remapped=cal.binding_remapped,
            sha256=hashlib.sha256(Path(args.calibration_file).read_bytes()).hexdigest()
                if Path(args.calibration_file).is_file() else None)
        print('FUSION_CALIBRATION '+json.dumps(calibration_info),flush=True)
        write_trace(dict(type='calibration',wall_time=time.time(),data=calibration_info))
        if args.recalibrate:cal.restart(full=True)
        control=FusionControl(cal)
        print('FUSION: RITnet + PuReST; '+('guided center/left/right; saved='+str(cal.loaded)
              if args.three_point else f'center-only start; span={cal.span}'),flush=True)
        seq=[0]*count;last_frame=time.monotonic()
        while True:
            if node:rclpy.spin_once(node,timeout_sec=0.)
            if ui:
                action=ui.poll()
                if action=='exit':break
                if action in ('center','full'):
                    publish();control.reset_gesture();cal.restart(full=action=='full')
                if action=='save':cal.save_pending()
            if any(c.error for c in cameras):raise RuntimeError('Camera disconnected')
            incoming=[c.latest for c in cameras];now=time.monotonic()
            if any(s is None or s[0]==q for s,q in zip(incoming,seq)):
                if now-last_frame>.25:publish()
                if now-last_frame>2.:raise RuntimeError('Camera frame timeout')
                time.sleep(.002);continue
            seq=[s[0] for s in incoming];last_frame=now;stamps=[s[1] for s in incoming];stamp=min(stamps)
            frames=[orient_frame(s[2],r) for s,r in zip(incoming,rotations)]
            start=time.perf_counter()
            rstates,details=blink.infer(frames,stamps)
            search_rois=[]
            for frame,d,rs in zip(frames,details,rstates):
                fh,fw=frame.shape[:2];roi=d['roi']
                if roi==[0,0,fw,fh] or tuple(roi)==(0,0,fw,fh):
                    if rs=='open' and d['pupil_center'] is not None:
                        cx,cy=d['pupil_center'];w,h=min(320,fw),min(240,fh)
                        roi=(int(np.clip(cx-w/2,0,fw-w)),int(np.clip(cy-h/2,0,fh-h)),w,h)
                    else:roi=None
                search_rois.append(roi)
            pdetails=pure.infer(frames,search_rois,stamps)
            features=np.full((count,2),np.nan);quality=np.zeros((count,2));states=[]
            for i,(r,d,pd) in enumerate(zip(rstates,details,pdetails)):
                width=frames[i].shape[1]
                if d['pupil_center'] is not None and d['pupil_confidence']>=.85:
                    features[i,0]=d['pupil_center'][0]/width;quality[i,0]=d['pupil_confidence']
                accepted=pd['valid']
                if accepted and d['pupil_center'] is not None:
                    accepted=bool(np.linalg.norm(np.asarray(pd['center'])-d['pupil_center'])<=width*.06)
                pd['accepted']=accepted
                if accepted:
                    features[i,1]=pd['center'][0]/width;quality[i,1]=pd['confidence']
                states.append(combine_eye_state(r,accepted))
            ref=[reference.infer(f).tolist() for f in frames] if reference else None
            fresh=max(stamps)-stamp<.08 and time.monotonic()-stamp<.25
            if args.no_display:
                # Unsupervised hardware diagnostics must not learn a new center.
                h,closed,valid,reason=0.,False,False,'DIAGNOSTIC_ONLY'
            else:
                h,closed,valid,reason=control.update(features,quality,states,stamp,fresh,gesture_states=rstates)
                valid=control.motion_valid;closed=all(s=='closed' for s in rstates)
                if not valid and reason=='READY':reason='GESTURE_TRANSITION'
            publish(h,closed,valid,stamp,fresh,cal.ready,control.open_eyes,
                    control.gestures.continuity,control.long_close,control.outgoing_event)
            n+=1;valid_count+=int(valid);closed_count+=int(fresh and closed)
            times.append((time.perf_counter()-start)*1000)
            state=feedback['state'] if time.monotonic()-feedback['at']<.5 else 'NO_CONTROLLER'
            status=(tuple(states),tuple(rstates),valid,closed,reason,state,cal.phase,feedback['blink_count'],control.gestures.status,
                    control.event['id'] if control.event else None,feedback['event_decision'])
            record=dict(t=stamp,eyes=states,ritnet_states=rstates,
                    features=[[float(v) if np.isfinite(v) else None for v in row] for row in features],quality=quality.tolist(),
                    purest=pdetails,ritnet=details,teyed_reference=ref,phase=cal.phase,progress=cal.progress,
                    selected=None if cal.profile is None else cal.profile['selected'],
                    blink_event=control.event,blink_phase=control.gestures.status,gesture_continuity=control.gestures.continuity,
                    long_close=control.long_close,
                    valid=valid,closed=closed,horizontal=h,reason=reason,controller=state,
                    controller_reason=feedback['reason'],controller_blockers=feedback['blockers'],controller_blinks=feedback['blink_count'],
                    controller_event_decision=feedback['event_decision'],
                    eye_directions=control.eye_directions,
                    sample_sequence=source_sequence, session=source_session,
                    frame_age_ms=(time.monotonic()-stamp)*1000.,processing_ms=times[-1],
                    gesture_starts=list(control.gestures.starts),
                    gesture_unknown_since=list(control.gestures.unknown_since))
            write_trace(dict(type='frame',wall_time=time.time(),data=record))
            if status!=last_status or now-last_log>=1.:
                print('FUSION_INPUT '+json.dumps(record,allow_nan=False),flush=True)
                last_status=status;last_log=now
            if ui and now-last_draw>=.1:
                ui.render(cal.phase,cal.progress,cal.message,state,valid,h,states,cal.index,feedback['blink_count'],feedback['target'],feedback['blockers']);last_draw=now
            if args.max_frames and n>=args.max_frames:break
    except KeyboardInterrupt:pass
    finally:
        publish()
        if trace:trace.close()
        if node:node.destroy_node();rclpy.try_shutdown()
        for c in cameras:c.close()
        for net in (blink,pure,reference):
            if net:net.close()
        if ui:ui.destroy()
        report=dict(frames=n,valid_frames=valid_count,closed_frames=closed_count,
            ready=cal.ready if cal else False,phase=cal.phase if cal else None,
            pair_ms_p50=float(np.median(times)) if times else None,pair_ms_p95=float(np.percentile(times,95)) if times else None)
        print(json.dumps(report),flush=True)
        if args.report:Path(args.report).write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
