"""Original PuReST temporal pupil tracking inside RITnet's eye ROI."""
import ctypes
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

class PuReST:
    def __init__(self,count,library=None):
        self.library=Path(library or ROOT/'build/purest/libpurest.so')
        if not self.library.exists():raise RuntimeError('Build PuReST: python3 tools/purest/build.py')
        self.lib=ctypes.CDLL(str(self.library))
        self.lib.purest_create.argtypes=[];self.lib.purest_create.restype=ctypes.c_void_p
        for name in ['purest_destroy','purest_reset']:
            getattr(self.lib,name).argtypes=[ctypes.c_void_p];getattr(self.lib,name).restype=None
        self.lib.purest_process.argtypes=[ctypes.c_void_p,
            np.ctypeslib.ndpointer(dtype=np.uint8,ndim=2,flags='C_CONTIGUOUS'),ctypes.c_int,ctypes.c_int,
            np.ctypeslib.ndpointer(dtype=np.float64,ndim=1,flags='C_CONTIGUOUS')]
        self.lib.purest_process.restype=ctypes.c_int
        self.handles=[];self.last_roi=[None]*count;self.last_stamp=[None]*count
        for _ in range(count):
            h=self.lib.purest_create()
            if not h:self.close();raise RuntimeError('Cannot create PuReST')
            self.handles.append(h)
    def infer(self,frames,rois,stamps):
        result=[]
        for i,(frame,roi,stamp) in enumerate(zip(frames,rois,stamps)):
            if roi is None:
                self.lib.purest_reset(self.handles[i]);self.last_roi[i]=None;self.last_stamp[i]=stamp
                result.append(dict(valid=False,center=None,confidence=0.,error_code=0));continue
            x,y,w,h=map(int,roi)
            if min(w,h)<32 or x<0 or y<0 or x+w>frame.shape[1] or y+h>frame.shape[0]:raise ValueError('Invalid PuReST eye ROI')
            if self.last_roi[i]!=tuple(roi) or (self.last_stamp[i] is not None and not 0<stamp-self.last_stamp[i]<=.25):
                self.lib.purest_reset(self.handles[i])
            self.last_roi[i]=tuple(roi);self.last_stamp[i]=stamp
            gray=np.ascontiguousarray(cv2.cvtColor(frame[y:y+h,x:x+w],cv2.COLOR_BGR2GRAY))
            out=np.zeros(7,np.float64);code=self.lib.purest_process(self.handles[i],gray,w,h,out)
            valid=bool(code==0 and out[6] and np.isfinite(out).all())
            result.append(dict(valid=valid,center=[float(out[0]+x),float(out[1]+y)] if valid else None,
                               confidence=float(out[5]) if valid else 0.,error_code=code))
        return result
    def close(self):
        for h in self.handles:self.lib.purest_destroy(h)
        self.handles=[]
