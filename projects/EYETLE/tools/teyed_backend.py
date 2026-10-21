"""Near-eye direction and eye-state inference, without online eyeball fitting."""
from collections import deque
import ctypes
import time
import cv2
import numpy as np
from ritnet_backend import TensorRTModel
from dual_eye_tracker import BinocularState


class EyeNetwork(TensorRTModel):
    def __init__(self,path,kind):
        super().__init__(path)
        self.kind=kind

    def infer(self,frame):
        if self.kind=='meye':
            h,w=frame.shape[:2];side=min(h,w)
            frame=frame[(h-side)//2:(h+side)//2,(w-side)//2:(w+side)//2]
            frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            frame=cv2.resize(frame,(self.width,self.height),interpolation=cv2.INTER_CUBIC)
            value=frame.astype(np.float32)[None,None]/255.
        else:
            frame=cv2.resize(frame,(self.width,self.height),interpolation=cv2.INTER_AREA)
            value=(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB).astype(np.float32)/255.-.5)*2.
            value=value.transpose(2,0,1)[None]
        host,ptr=self.buffers['eye'];np.copyto(host,value)
        self.check(self.cuda.cudaMemcpy(ptr,ctypes.c_void_p(host.ctypes.data),host.nbytes,1))
        if not self.context.execute_async_v3(self.stream.value):raise RuntimeError('Inference failed')
        self.check(self.cuda.cudaStreamSynchronize(self.stream))
        out,ptr=self.buffers['segmentation']
        self.check(self.cuda.cudaMemcpy(ctypes.c_void_p(out.ctypes.data),ptr,out.nbytes,2))
        return out.reshape(-1).copy()


def eye_state(tags,closed_threshold=.35):
    if not np.isfinite(closed_threshold) or not .2<closed_threshold<=1:
        raise ValueError('Closed threshold must be finite and in (0.2, 1]')
    eye,blink=tags
    if not np.isfinite(tags).all() or not 0<=eye<=1 or not 0<=blink<=1:return 'unknown'
    if eye<.8:return 'unknown'
    if blink>=closed_threshold:return 'closed'
    if blink<=.2:return 'open'
    return 'unknown'


class AutoCenter:
    """One startup center, acquired only during a prompted stable open interval.

    Stability cannot prove the user is looking straight ahead. The UI instruction
    is part of this protocol. Never adapt the center during active control.
    """
    def __init__(self,count,duration=2.,spread=.06):
        self.count=count;self.duration=duration;self.spread=spread
        self.reset()

    def reset(self):
        self.center=None;self.samples=deque();self.progress=0.;self.last=None

    @property
    def ready(self):return self.center is not None

    def update(self,values,states,now,fresh):
        if self.ready:return
        a=np.asarray(values)
        ok=(fresh and a.shape==(self.count,2) and np.isfinite(a).all() and
            np.all(np.abs(a)<=1) and states==['open']*self.count)
        if self.last is not None and not 0<now-self.last<=.25:ok=False
        self.last=now
        if not ok:
            self.samples.clear();self.progress=0.;return
        self.samples.append((now,a.copy()))
        while len(self.samples)>1 and now-self.samples[1][0]>=self.duration:self.samples.popleft()
        stack=np.array([a for _,a in self.samples])
        if np.max(np.ptp(stack,axis=0))>self.spread:
            self.samples.clear();self.samples.append((now,a.copy()));self.progress=0.;return
        elapsed=now-self.samples[0][0]
        self.progress=min(1.,elapsed/self.duration)
        if elapsed>=self.duration and len(self.samples)>=20:self.center=np.median(stack,axis=0)


class BinocularInput:
    def __init__(self,count,signs,span=.25,agreement=.15):
        if count not in (1,2) or len(signs)!=count or any(s not in (-1,1) for s in signs):raise ValueError('Invalid eye configuration')
        if not np.isfinite([span,agreement]).all() or min(span,agreement)<=0:raise ValueError('Invalid gaze thresholds')
        self.center=AutoCenter(count);self.signs=np.array(signs)
        self.span=span;self.agreement=agreement
        self.blink=BinocularState();self.last_open=None;self.horizontal=0.

    def reset(self):
        self.center.reset();self.blink=BinocularState();self.last_open=None;self.horizontal=0.

    def update(self,values,states,now,fresh):
        self.center.update(values,states,now,fresh)
        reason='READY'
        permitted=fresh and self.center.ready
        if not fresh:reason='STALE_OR_UNSYNCED'
        elif not self.center.ready:reason='LOOK_STRAIGHT_AUTO_CENTER'
        candidate=None
        if permitted and states==['open']*len(self.signs):
            a=np.asarray(values)
            if a.shape!=(len(self.signs),2) or not np.isfinite(a).all() or np.any(np.abs(a)>1):
                permitted=False;reason='INVALID_GAZE'
            else:
                offsets=(a[:,0]-self.center.center[:,0])*self.signs
                if np.ptp(offsets)>self.agreement:
                    permitted=False;reason='GAZE_DISAGREEMENT'
                else:candidate=float(np.clip(np.mean(offsets)/self.span,-1,1))
        elif permitted:
            # Blink inference is independent of gaze estimates on closed eyes.
            permitted=self.last_open is not None and 0<=now-self.last_open<=1.2
            if not permitted:reason='NO_RECENT_OPEN_GAZE'
        closed,valid=self.blink.update(states if len(states)==2 else states*2,now,permitted)
        if valid and candidate is not None:self.horizontal=candidate;self.last_open=now
        if not valid:
            self.last_open=None
            if reason=='READY':reason='EYE_STATE_UNKNOWN_OR_DISAGREES'
        return self.horizontal if valid else 0.,closed,valid,reason
