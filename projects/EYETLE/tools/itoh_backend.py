"""Ctypes bridge to the pinned Itoh/Orlosky/Swirski C++ implementation."""
import ctypes
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

class ItohEye:
    def __init__(self,library=None,focal=476.70143703768406):
        if not np.isfinite(focal) or focal<=0:raise ValueError('Invalid focal length')
        self.library=Path(library or ROOT/'build/itoh/libitoh.so')
        if not self.library.exists():raise RuntimeError('Build Itoh first: ./tools/build_itoh.sh')
        self.lib=ctypes.CDLL(str(self.library))
        self.lib.itoh_create.argtypes=[ctypes.c_double];self.lib.itoh_create.restype=ctypes.c_void_p
        self.lib.itoh_destroy.argtypes=[ctypes.c_void_p];self.lib.itoh_destroy.restype=None
        self.lib.itoh_reset.argtypes=[ctypes.c_void_p];self.lib.itoh_reset.restype=None
        byte=np.ctypeslib.ndpointer(dtype=np.uint8,ndim=2,flags='C_CONTIGUOUS')
        doubles=np.ctypeslib.ndpointer(dtype=np.float64,ndim=1,flags='C_CONTIGUOUS')
        self.lib.itoh_process.argtypes=[ctypes.c_void_p,byte,doubles];self.lib.itoh_process.restype=ctypes.c_int
        self.lib.itoh_observe.argtypes=[ctypes.c_void_p,byte,doubles,doubles];self.lib.itoh_observe.restype=ctypes.c_int
        self.handle=self.lib.itoh_create(focal)
        if not self.handle:raise RuntimeError('Cannot create Itoh tracker')
        self.detail={}

    def infer(self,frame,ellipse=None):
        if frame.shape[:2]!=(480,640):raise ValueError('Original Itoh pupil fitter requires oriented 640x480 input')
        gray=np.ascontiguousarray(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame,dtype=np.uint8)
        out=np.zeros(17,np.float64)
        if ellipse is None:code=self.lib.itoh_process(self.handle,gray,out)
        else:
            e=np.asarray(ellipse,dtype=np.float64)
            if e.shape!=(5,):raise ValueError('Ellipse requires cx,cy,width,height,angle')
            code=self.lib.itoh_observe(self.handle,gray,np.ascontiguousarray(e),out)
        # np.all returns np.bool_; metadata is also serialized on shutdown.
        valid=bool(code==0 and bool(out[2]) and np.isfinite(out).all())
        reason=('ITOH_ERROR' if code else 'NO_PUPIL' if not out[0] else
                'SWEEP_TO_BUILD' if not out[1] else 'ITOH_READY' if valid else 'UNRELIABLE_3D_RESET_IF_PERSISTENT')
        def vector(a):return [float(x) if np.isfinite(x) else None for x in a]
        self.detail=dict(reason=reason,error_code=code,detected=bool(out[0]),model_built=bool(out[1]),
            valid=valid,observations=int(out[3]),target_observations=30,ellipse=vector(out[4:9]),
            normal=vector(out[9:12]),sphere_center=vector(out[12:15]),
            pupil_radius_mm=float(out[15]) if np.isfinite(out[15]) else None,
            reliability=float(out[16]) if np.isfinite(out[16]) else None)
        return out[9:11].copy() if valid else np.full(2,np.nan)

    def reset(self):self.lib.itoh_reset(self.handle)
    def close(self):
        if self.handle:self.lib.itoh_destroy(self.handle);self.handle=None

class ItohEyes:
    def __init__(self,count,library=None,focal=476.70143703768406):
        self.eyes=[]
        try:
            for _ in range(count):self.eyes.append(ItohEye(library,focal))
        except Exception:
            self.close();raise
    def infer(self,frames):
        values=[eye.infer(frame) for eye,frame in zip(self.eyes,frames)]
        return values,[eye.detail for eye in self.eyes]
    def reset(self):
        for eye in self.eyes:eye.reset()
    def close(self):
        for eye in self.eyes:eye.close()
