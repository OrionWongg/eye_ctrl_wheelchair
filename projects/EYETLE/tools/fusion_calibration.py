"""Guided 2D calibration with per-eye/per-detector scale and atomic reuse.

center_only: look at the center once per start; left/right use a fixed span
in full-frame pupil-x units, so no profile file is read or written.
"""
from collections import deque
import json
import os
from pathlib import Path
import tempfile
import numpy as np


def same_binding(saved, current, allow_hub_move=False):
    """Allow only the known 4.4 -> 4.1 hub move, preserving eye order."""
    if saved == current:
        return True
    if not allow_hub_move or not isinstance(saved, dict):
        return False
    def normalized(binding):
        result = dict(binding)
        result['cameras'] = [
            str(p).replace(':4.1.1:1.0-video-index0', ':4.4.1:1.0-video-index0')
                  .replace(':4.1.2:1.0-video-index0', ':4.4.2:1.0-video-index0')
            for p in binding.get('cameras', [])]
        return result
    return normalized(saved) == normalized(current)


class GuidedCalibration:
    def __init__(self,path,binding,count,hold=1.5,settle=1.,min_samples=15,
                 saved_only=False,allow_hub_move=False,center_only=False,span=.19):
        self.path=Path(path);self.binding=binding;self.count=count
        self.saved_only=saved_only;self.center_only=center_only;self.span=float(span)
        if center_only and not .02<=self.span<=.5:raise ValueError('Gaze span must be in [0.02, 0.5]')
        self.binding_remapped=False
        self.hold=hold;self.settle=settle;self.min_samples=min_samples
        self.profile=None;self.loaded=False;self.error='';self.pending_save=False
        if center_only:
            self.restart(full=False);return
        try:
            data=json.loads(self.path.read_text())
            if data.get('schema')=='eye-fusion-v1' and same_binding(data.get('binding'),binding,allow_hub_move):
                self._validate(data['points'],data['selected'])
                self.profile=data;self.loaded=True
                self.binding_remapped=data['binding'] != binding
        except FileNotFoundError:pass
        except (OSError,ValueError,KeyError,TypeError) as exc:self.error=str(exc)
        if saved_only and not self.loaded:
            raise ValueError(f'Saved eye calibration unavailable or binding mismatch: {self.path}; {self.error}')
        self.restart(full=False)

    def _validate(self,points,selected):
        p=np.asarray(points,dtype=float);m=np.asarray(selected,dtype=bool)
        if p.shape!=(3,self.count,2) or m.shape!=(self.count,2) or not np.isfinite(p).all():raise ValueError('Invalid profile shape')
        if not m.any(axis=1).all():raise ValueError('Each eye needs a usable detector')
        left,center,right=p
        if np.any(m & (((left-center)*(right-center)>=0) | (np.minimum(abs(left-center),abs(right-center))<.008))):
            raise ValueError('Directions not distinguishable')
        if np.any((p<0)|(p>1)):raise ValueError('Invalid feature range')

    def restart(self,full=False):
        if self.center_only:
            # A new center replaces the whole synthetic profile; nothing is saved.
            self.profile=None;self.steps=['center']
            self.index=0;self.phase='center';self.ready=False;self.progress=0.
            self.entered=None;self.last=None;self.buffer=deque();self.captures={}
            self.center=None;self.pending_save=False;self.message='准备看正中间的圆点'
            return
        if full and self.saved_only:
            self.restart(full=False)
            self.message='使用原仿真标定，仅重新确认正中；不会重新标定或保存'
            return
        if full:self.profile=None;self.loaded=False
        self.steps=['confirm'] if self.profile is not None else ['center','left','right','confirm']
        self.index=0;self.phase=self.steps[0];self.ready=False;self.progress=0.
        self.entered=None;self.last=None;self.buffer=deque();self.captures={}
        self.center=None;self.pending_save=False;self.message='准备看目标点'

    def observe(self,features,quality,states,now,fresh):
        if self.ready or self.pending_save or self.phase=='failed':return
        if self.entered is None:self.entered=now
        if self.last is not None and not 0<now-self.last<=.25:self.buffer.clear();self.progress=0.
        self.last=now
        if now-self.entered<self.settle:
            self.message='请看目标点，保持头部不动';return
        a=np.asarray(features,dtype=float);q=np.asarray(quality,dtype=float)
        mask=np.isfinite(a)&np.isfinite(q)&(q>=.6)&(a>=0)&(a<=1)
        if self.profile is not None:mask &= np.asarray(self.profile['selected'],bool)
        if not fresh or states!=['open']*self.count or not mask.any(axis=1).all():
            self.buffer.clear();self.progress=0.;self.message='未看清眼睛，请睁眼并保持位置';return
        self.buffer.append((now,np.where(mask,a,np.nan)))
        while len(self.buffer)>1 and now-self.buffer[1][0]>=self.hold:self.buffer.popleft()
        elapsed=now-self.buffer[0][0];self.progress=min(1.,elapsed/self.hold)
        stack=np.array([v for _,v in self.buffer]);med=np.full((self.count,2),np.nan)
        stable=np.zeros((self.count,2),bool)
        for i in range(self.count):
            for j in range(2):
                v=stack[:,i,j];v=v[np.isfinite(v)]
                if len(v)>=self.min_samples and np.percentile(v,90)-np.percentile(v,10)<=.025:
                    med[i,j]=np.median(v);stable[i,j]=True
        self.message='保持注视，正在采集'
        if elapsed<self.hold:return
        if not stable.any(axis=1).all():self.message='注视仍在变化，请继续看目标点';return
        if self.phase=='confirm':
            selected=np.asarray(self.profile['selected'],bool)
            if not np.all(stable[selected]):self.message='请保持睁眼，等待画面稳定';return
            original=np.asarray(self.profile['points'])[1]
            if np.any(np.abs(med[selected]-original[selected])>.06):
                self.message='佩戴位置变化较大，请点“重新标定”';self.progress=0.;return
            self.center=np.where(selected,med,original)
            if not self.loaded:
                self.pending_save=True;self.save_pending()
            else:self.ready=True;self.phase='ready';self.message='已就绪'
            return
        if self.center_only:
            selected=stable&np.isfinite(med)
            self.center=np.where(selected,med,.5)
            # Looking left raises full-frame pupil x for both mounted cameras.
            points=[self.center+self.span,self.center,self.center-self.span]
            self.profile=dict(schema='eye-fusion-center-v1',points=np.asarray(points).tolist(),
                              selected=selected.tolist(),span=self.span)
            self.ready=True;self.phase='ready';self.message='已就绪'
            return
        self.captures[self.phase]=med
        if self.phase=='right':
            p=np.array([self.captures[k] for k in ('left','center','right')])
            m=np.isfinite(p).all(axis=0)
            m &= (p[0]-p[1])*(p[2]-p[1])<0
            m &= np.minimum(abs(p[0]-p[1]),abs(p[2]-p[1]))>=.008
            if not m.any(axis=1).all():
                self.message='左右区分不足，请点“重新标定”后跟随目标';self.phase='failed';return
            p=np.where(m[None],p,.5)
            self.profile=dict(schema='eye-fusion-v1',binding=self.binding,points=p.tolist(),selected=m.tolist())
        self.index+=1;self.phase=self.steps[self.index]
        self.entered=now;self.buffer.clear();self.progress=0.

    def save_pending(self):
        if self.saved_only or self.center_only or not self.pending_save:return
        tmp=None
        try:
            self._validate(self.profile['points'],self.profile['selected'])
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w',dir=self.path.parent,delete=False) as f:
                tmp=f.name;json.dump(self.profile,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.path);tmp=None
            self.pending_save=False;self.ready=True;self.phase='ready';self.message='已就绪';self.error=''
        except (OSError,ValueError) as exc:self.error=str(exc);self.message='保存失败，请点“重试保存”'
        finally:
            if tmp is not None:Path(tmp).unlink(missing_ok=True)

    def horizontal(self,features,quality):
        if not self.ready:return 0.,False,[],'CALIBRATING'
        a=np.asarray(features,float);q=np.asarray(quality,float)
        p=np.asarray(self.profile['points']);selected=np.asarray(self.profile['selected'],bool)
        delta=a-self.center
        right=p[2]-p[1];left=p[0]-p[1]
        toward_right=delta*right>=0
        denominator=np.where(toward_right,right,-left)
        norm=np.divide(delta,denominator,out=np.zeros_like(delta),where=denominator!=0)
        available=selected&np.isfinite(a)&np.isfinite(q)&(q>=.6)
        eyes=[];weights=[]
        for i in range(self.count):
            v=norm[i,available[i]];w=q[i,available[i]]
            # An eye without a direction feature is skipped; the other eye steers.
            if len(v)==0:continue
            if np.ptp(v)>.5:return 0.,False,eyes,'DETECTORS_DISAGREE'
            eyes.append(float(np.clip(np.average(v,weights=w),-1,1)));weights.append(float(w.mean()))
        if not eyes:return 0.,False,eyes,'NO_DIRECTION_FEATURE'
        # Only opposite, clearly turned eyes are a conflict; amplitude differences are normal.
        if min(eyes)<-.5 and max(eyes)>.5:return 0.,False,eyes,'EYES_DISAGREE'
        return float(np.average(eyes,weights=weights)),True,eyes,'READY'
