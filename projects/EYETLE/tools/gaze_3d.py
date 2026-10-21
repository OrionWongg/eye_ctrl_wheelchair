"""RITnet ellipses -> pye3d eye orientation, with conservative quality gating.

Camera coordinates are those of the FULL oriented frame, never the moving ROI.
Model fitting is automatic; a saved straight-ahead reference is personalization,
not a replacement for measured camera intrinsics or slippage validation.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
from gaze_calibration import CalibrationStore


def full_frame_ellipse(ellipse, roi, label_shape):
    """Transform ellipse as a conic, including anisotropic resizing."""
    (cx, cy), (a, b), angle = ellipse
    x, y, w, h = roi
    sx, sy = w/label_shape[1], h/label_shape[0]
    if not all(math.isfinite(v) for v in (cx,cy,a,b,angle,sx,sy)) or min(a,b,sx,sy)<=0:
        raise ValueError('Invalid ellipse')
    t = np.deg2rad(angle)
    rot = np.array([[np.cos(t),-np.sin(t)],[np.sin(t),np.cos(t)]])
    transform = np.diag([sx,sy]) @ rot
    covariance = transform @ np.diag([(a/2)**2,(b/2)**2]) @ transform.T
    vals, vecs = np.linalg.eigh(covariance)
    minor = vecs[:,0]
    return {'center':(x+cx*sx,y+cy*sy),
            'axes':tuple(2*np.sqrt(np.maximum(vals,0))),
            'angle':float(np.degrees(np.arctan2(minor[1],minor[0])) % 180)}


class EyeGeometry:
    def __init__(self, resolution, focal_length=None, detector=None):
        self.resolution = tuple(resolution)
        self.focal = float(focal_length or math.hypot(*resolution)/(2*math.tan(math.radians(40))))
        if not math.isfinite(self.focal) or self.focal<=0:raise ValueError('Invalid camera focal length')
        if detector is None:
            from pye3d.camera import CameraModel
            from pye3d.detector_3d import Detector3D
            detector = Detector3D(CameraModel(self.focal,self.resolution),
                                  threshold_long_term=.85, calculate_rms_residual=True)
        self.detector = detector
        self.has_diversity=False
        self.history = deque(maxlen=150)
        self.centers = deque(maxlen=30)
        self.last_at = self.last_good = self.last_center = None
        self.started = None
        self.stable = False
        self.reason = 'MODEL_WARMUP: slowly look around'
        self.angle = None
        self.ms = 0.
        self.last_result = None
        self.motion_guard=FrameMotionGuard()
        self.recenter_required=False
        self.shift_events=0
        self.blocked_until=0.
        self.refit_pending=False
        self.diagnostics={}

    def update(self, frame, ellipse, roi, label_shape, confidence, state, stamp, eye_bbox=None):
        self.ms=0.
        if self.last_at is not None and (stamp<=self.last_at or stamp-self.last_at>.4):
            self.stable=False;self.centers.clear();self.last_good=None
        self.last_at=stamp
        shifted,rotated=self.motion_guard.update(frame,eye_bbox,stamp)
        if shifted:
            self.shift_events+=1
            # Do not repeatedly reset an initializing model. A previously
            # stable model is refit once, after surrounding motion has settled.
            self.refit_pending=self.refit_pending or self.stable
            self.centers.clear();self.last_good=None
            self.blocked_until=stamp+.7
            self.stable=False
            self.recenter_required=self.recenter_required or rotated
        if self.refit_pending and stamp>=self.blocked_until:
            self.detector.reset();self.history.clear();self.centers.clear();self.has_diversity=False
            self.last_center=self.last_good=self.started=None
            self.refit_pending=False
        if state=='closed':
            ok=self.stable and self.last_good is not None and 0<=stamp-self.last_good<=1.2
            self.reason='BLINK' if ok else 'CLOSED_WITHOUT_STABLE_MODEL'
            return self.angle,ok
        if state!='open' or ellipse is None or confidence<.85:
            self.stable=False;self.centers.clear();self.reason='NO_RELIABLE_PUPIL'
            return None,False
        start=time.perf_counter()
        try:
            data={'ellipse':full_frame_ellipse(ellipse,roi,label_shape),
                  'confidence':float(confidence),'timestamp':float(stamp)}
            result=self.detector.update_and_detect(data,cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
            self.last_result=result
            normal=np.asarray(result['circle_3d']['normal'],float)
            sphere=np.asarray(self.detector.short_term_model.sphere_center,float)
            projected=np.asarray(result['ellipse']['center'],float)
            if normal.shape!=(3,) or sphere.shape!=(3,) or not np.isfinite(normal).all() or not np.isfinite(sphere).all():
                raise ValueError('nonfinite model')
            self.diagnostics=dict(model_confidence=float(result.get('model_confidence',0)),
                                  sphere_center=list(map(float,result['sphere']['center'])),
                                  diameter_mm=float(result['diameter_3d']),shift_events=self.shift_events)
            if not .8<np.linalg.norm(normal)<1.2 or normal[2]>=-.1:
                raise ValueError('invalid gaze normal')
            if result.get('model_confidence',0)<.9:
                z=result['sphere']['center'][2];diam=result['diameter_3d']
                raise ValueError(f'model range: z={z:.1f}mm pupil={diam:.1f}mm')
            # A high segmentation confidence is not proof of a good 3D fit.
            residual=np.linalg.norm(projected-np.array(data['ellipse']['center']))
            if not math.isfinite(residual) or residual>max(3.,.15*max(data['ellipse']['axes'])):
                raise ValueError('reprojection mismatch')
            angle=float(np.degrees(np.arctan2(normal[0],-normal[2])))
            if self.started is None:self.started=stamp
            self.history.append((stamp,*data['ellipse']['center']))
            if self.last_center is not None and np.linalg.norm(sphere-self.last_center)>1.5:
                self.centers.clear();self.stable=False
            self.last_center=sphere.copy()
            self.centers.append((stamp,sphere.copy()))
            # Variation is needed to fit an eyeball; a fixed eye cannot identify it.
            points=np.array([p[1:] for p in self.history])
            diverse=len(points)>=45 and np.linalg.norm(np.ptp(points,axis=0))>=10.
            centers=np.array([p[1] for p in self.centers])
            settled=(len(centers)>=12 and stamp-self.centers[0][0]>=.6 and
                     np.max(np.linalg.norm(centers-np.median(centers,axis=0),axis=1))<1.)
            diverse=diverse or self.has_diversity
            self.stable=bool(diverse and stamp-self.started>=2. and settled and stamp>=self.blocked_until)
            if self.stable:self.has_diversity=True
            spread=float(np.max(np.linalg.norm(centers-np.median(centers,axis=0),axis=1)))
            self.diagnostics.update(short_sphere_center=list(map(float,sphere)),
                                    center_spread_mm=spread,center_samples=len(centers),
                                    diversity_learned=self.has_diversity,
                                    input_span_px=float(np.linalg.norm(np.ptp(points,axis=0))),
                                    motion_hold_s=max(0.,self.blocked_until-stamp))
            self.reason=('3D_READY' if self.stable else
                         'WAIT_MOTION: keep glasses still' if stamp<self.blocked_until else
                         'WAIT_EYE_SWEEP: slowly look around once' if not diverse else
                         'WAIT_CENTER_STABILITY: hold gaze steady' if not settled else
                         'MODEL_WARMUP: collecting initial observations')
            self.angle=angle
            if self.stable:self.last_good=stamp
            return angle,self.stable
        except (ValueError,KeyError,TypeError,ArithmeticError,np.linalg.LinAlgError) as exc:
            self.stable=False;self.centers.clear();self.reason='3D_UNSTABLE: '+str(exc)
            return None,False
        finally:self.ms=(time.perf_counter()-start)*1000

    def close(self):
        self.detector._cleanup_models()


class CenterCalibration:
    """Atomic per-camera angular centers, separate schema from legacy calibration."""
    def __init__(self, path, bindings, samples=60, settle=.4, span=25.):
        self.path=Path(path).expanduser();self.bindings=bindings
        self.samples=samples;self.settle=settle;self.span=span
        self.centers=[None]*len(bindings);self.loaded=0;self.capture_at=None
        self.buffers=[deque(maxlen=samples) for _ in bindings]
        self.pending_save=False;self.error='';self.last_stamp=None
        try:
            data=json.loads(self.path.read_text())
            if data.get('schema')!='eye-3d-center-v1':raise ValueError('wrong center schema')
            for i,binding in enumerate(bindings):
                record=data.get('profiles',{}).get(CalibrationStore.key(binding),{})
                value=record.get('center_deg')
                if not record.get('requires_confirmation',False) and record.get('binding')==binding and isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and abs(value)<85:
                    self.centers[i]=float(value);self.loaded+=1
        except FileNotFoundError:pass
        except (OSError,ValueError,TypeError,AttributeError) as exc:self.error=f'Cannot load center: {exc}'

    @property
    def ready(self):return all(x is not None for x in self.centers) and self.capture_at is None and not self.pending_save

    @property
    def status(self):
        if self.pending_save:return 'Save failed: Enter retries; '+self.error
        if self.capture_at is not None:return f'CENTER capture {min(map(len,self.buffers))}/{self.samples}: look straight ahead'
        if self.ready:return '3D center loaded/confirmed | S: recenter, C: reset'
        return '3D: slowly look around, then look straight ahead and press S'

    def restart(self):
        self.centers=[None]*len(self.bindings);self.capture_at=None;self.pending_save=False
        for buf in self.buffers:buf.clear()

    def invalidate_rotation(self):
        self.restart()
        tmp=None
        try:
            if not self.path.exists():return
            data=json.loads(self.path.read_text())
            if data.get('schema')!='eye-3d-center-v1':return
            for binding in self.bindings:
                record=data.get('profiles',{}).get(CalibrationStore.key(binding))
                if record is not None:record['requires_confirmation']=True
            with tempfile.NamedTemporaryFile(mode='w',dir=self.path.parent,delete=False) as f:
                tmp=f.name;json.dump(data,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.path);tmp=None
        except (OSError,ValueError,TypeError) as exc:
            # Stop the publisher if invalidation cannot be persisted. Otherwise
            # the next launch could silently reload the known-invalid reference.
            raise RuntimeError('Cannot invalidate rotated camera reference: '+str(exc)) from exc
        finally:
            if tmp is not None:Path(tmp).unlink(missing_ok=True)

    def begin(self,direction,now):
        if direction!='center':return False
        self.capture_at=now+self.settle;self.pending_save=False
        for buf in self.buffers:buf.clear()
        return True

    def observe(self,angles,states,stamp,valid):
        if self.last_stamp is not None and stamp<=self.last_stamp:return
        self.last_stamp=stamp
        if self.capture_at is None or stamp<self.capture_at:return
        if not valid or any(s!='open' for s in states) or any(a is None or not math.isfinite(a) for a in angles):
            for buf in self.buffers:buf.clear()
            return
        for buf,angle in zip(self.buffers,angles):buf.append(float(angle))
        if not all(len(buf)==self.samples for buf in self.buffers):return
        if any(np.percentile(buf,90)-np.percentile(buf,10)>3. for buf in self.buffers):return
        self.centers=[float(np.median(buf)) for buf in self.buffers]
        self.pending_save=True;self.save_pending()

    def save_pending(self):
        if not self.pending_save:return
        tmp=None
        try:
            data={'schema':'eye-3d-center-v1','profiles':{}}
            if self.path.exists():
                old=json.loads(self.path.read_text())
                if old.get('schema')!='eye-3d-center-v1':raise ValueError('Refusing to replace a different calibration format')
                data=old
            for binding,center in zip(self.bindings,self.centers):
                data['profiles'][CalibrationStore.key(binding)]={'binding':binding,'center_deg':center}
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w',dir=self.path.parent,delete=False) as f:
                tmp=f.name;json.dump(data,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.path);tmp=None
            self.capture_at=None;self.pending_save=False;self.error=''
        except (OSError,ValueError,TypeError,KeyError) as exc:self.error=str(exc)
        finally:
            if tmp is not None:Path(tmp).unlink(missing_ok=True)

    def horizontal(self,angles,signs):
        if not self.ready or any(a is None for a in angles):return 0.,False
        offsets=[((a-c+180)%360-180)*s for a,c,s in zip(angles,self.centers,signs)]
        # Conflicting binocular directions cannot be averaged into a false center.
        if len(offsets)>1 and max(offsets)-min(offsets)>10.:return 0.,False
        return float(np.clip(np.mean(offsets)/self.span,-1,1)),True


class FrameMotionGuard:
    """Track surrounding skin/background, excluding the segmented eye.

    This is a conservative image-motion alarm, not a measured headset pose.
    Gaze motion must not be treated as camera motion merely because the pupil moved.
    """
    def __init__(self):
        self.gray=None;self.points=None;self.moves=deque();self.last_at=None;self.eye_bbox=None

    def update(self,frame,eye_bbox,stamp):
        h,w=frame.shape[:2];small=cv2.resize(frame,(160,120),interpolation=cv2.INTER_AREA)
        gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
        shifted=rotated=False
        if self.last_at is not None and not 0<stamp-self.last_at<.25:
            self.gray=None;self.points=None;self.moves.clear()
        if self.gray is not None and self.points is not None and len(self.points)>=12:
            tracked,status,_=cv2.calcOpticalFlowPyrLK(self.gray,gray,self.points,None,winSize=(15,15),maxLevel=2)
            if tracked is not None:
                back,back_status,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,tracked,None,winSize=(15,15),maxLevel=2)
                if back is None or back_status is None:
                    good=np.zeros(len(self.points),dtype=bool)
                else:
                    good=(status.ravel()==1)&(back_status.ravel()==1)&(np.linalg.norm((back-self.points).reshape(-1,2),axis=1)<.35)
                old=self.points[good];new=tracked[good]
                if len(old)>=12:
                    matrix,inliers=cv2.estimateAffinePartial2D(old,new,method=cv2.RANSAC,ransacReprojThreshold=1.)
                    spread=False
                    if matrix is not None and inliers is not None:
                        support=old.reshape(-1,2)[inliers.ravel().astype(bool)]
                        quadrants=set((int(x>=80),int(y>=60)) for x,y in support)
                        area=cv2.contourArea(cv2.convexHull(support)) if len(support)>=3 else 0.
                        spread=len(quadrants)>=3 and area>160*120*.15
                    if matrix is not None and inliers.sum()>=12 and inliers.mean()>=.6 and spread:
                        dx,dy=matrix[:,:2]@np.array([80.,60.])+matrix[:,2]-np.array([80.,60.])
                        rotation=math.degrees(math.atan2(matrix[1,0],matrix[0,0]))
                        scale=math.log(max(math.hypot(matrix[0,0],matrix[1,0]),1e-6))
                        self.moves.append((stamp,dx,dy,rotation,scale))
                        while self.moves and stamp-self.moves[0][0]>.5:self.moves.popleft()
                        total=np.sum([m[1:] for m in self.moves],axis=0)
                        rotated=abs(total[2])>1.
                        shifted=bool(np.linalg.norm(total[:2])>1. or rotated or abs(total[3])>.02)
                        if shifted:self.moves.clear()
        mask=np.full(gray.shape,255,np.uint8)
        if eye_bbox is not None:self.eye_bbox=eye_bbox
        eye_bbox=self.eye_bbox
        if eye_bbox is not None:
            x,y,bw,bh=eye_bbox
            x0=max(0,int(x*160/w)-10);y0=max(0,int(y*120/h)-10)
            x1=min(160,int((x+bw)*160/w)+10);y1=min(120,int((y+bh)*120/h)+10)
            mask[y0:y1,x0:x1]=0
        self.points=cv2.goodFeaturesToTrack(gray,maxCorners=60,qualityLevel=.02,minDistance=5,mask=mask)
        self.gray=gray;self.last_at=stamp
        return shifted,rotated
