"""RITnet TensorRT 10 runtime; CUDA Runtime via ctypes, no PyTorch at runtime."""
import ctypes
import ctypes.util
from pathlib import Path
import time

import cv2
import numpy as np


class TensorRTModel:
    def __init__(self, path):
        import tensorrt as trt
        self.cuda=ctypes.CDLL(ctypes.util.find_library('cudart') or 'libcudart.so.12')
        for name, args in {
            'cudaMalloc':[ctypes.POINTER(ctypes.c_void_p),ctypes.c_size_t],
            'cudaFree':[ctypes.c_void_p],
            'cudaMemcpy':[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_int],
            'cudaStreamCreate':[ctypes.POINTER(ctypes.c_void_p)],
            'cudaStreamSynchronize':[ctypes.c_void_p],
            'cudaStreamDestroy':[ctypes.c_void_p],
        }.items():
            getattr(self.cuda,name).argtypes=args
            getattr(self.cuda,name).restype=ctypes.c_int
        self.logger=trt.Logger(trt.Logger.WARNING)
        self.runtime=trt.Runtime(self.logger)
        self.engine=self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:raise RuntimeError('Invalid TensorRT engine; rebuild on this Jetson')
        self.context=self.engine.create_execution_context()
        self.stream=ctypes.c_void_p()
        self.check(self.cuda.cudaStreamCreate(ctypes.byref(self.stream)))
        self.buffers={}
        for name in ('eye','segmentation'):
            shape=tuple(self.engine.get_tensor_shape(name))
            host=np.empty(shape,dtype=trt.nptype(self.engine.get_tensor_dtype(name)))
            ptr=ctypes.c_void_p()
            self.check(self.cuda.cudaMalloc(ctypes.byref(ptr),host.nbytes))
            self.buffers[name]=(host,ptr)
            self.context.set_tensor_address(name,ptr.value)
        self.height,self.width=self.buffers['eye'][0].shape[-2:]
        self.clahe=cv2.createCLAHE(clipLimit=1.5,tileGridSize=(8,8))
        self.gamma=(255.*np.linspace(0,1,256)**.8).astype(np.uint8)

    @staticmethod
    def check(code):
        if code:raise RuntimeError(f'CUDA runtime error {code}')

    def preprocess(self,frame):
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
        gray=cv2.resize(gray,(self.width,self.height),interpolation=cv2.INTER_AREA)
        gray=self.clahe.apply(cv2.LUT(gray,self.gamma))
        return np.ascontiguousarray((gray.astype(np.float32)/255.-.5)/.5)[None,None]

    def infer_batch(self,frames):
        start=time.perf_counter()
        host,ptr=self.buffers['eye']
        if len(frames)!=host.shape[0]:raise ValueError('Input count differs from engine batch')
        for i,frame in enumerate(frames):np.copyto(host[i:i+1],self.preprocess(frame))
        self.check(self.cuda.cudaMemcpy(ptr,ctypes.c_void_p(host.ctypes.data),host.nbytes,1))
        if not self.context.execute_async_v3(self.stream.value):raise RuntimeError('TensorRT inference failed')
        self.check(self.cuda.cudaStreamSynchronize(self.stream))
        out,ptr=self.buffers['segmentation']
        self.check(self.cuda.cudaMemcpy(ctypes.c_void_p(out.ctypes.data),ptr,out.nbytes,2))
        return [(row[0].astype(np.uint8),row[1].copy()) for row in out],(time.perf_counter()-start)*1000

    def infer(self,frame):
        results,ms=self.infer_batch([frame])
        return *results[0],ms

    def close(self):
        for _,ptr in self.buffers.values():self.check(self.cuda.cudaFree(ptr))
        self.buffers.clear()
        if self.stream:
            self.check(self.cuda.cudaStreamDestroy(self.stream));self.stream=None


def segmentation_metrics(labels,confidence):
    """0 background, 1 sclera, 2 iris, 3 pupil (OpenEDS)."""
    pupil=((labels==3)&(confidence>.65)).astype(np.uint8)
    count,cc,stats,centers=cv2.connectedComponentsWithStats(pupil,8)
    result={'pupil':None,'confidence':0.,'eye_area':int(np.count_nonzero(labels)),
            'pupil_area':0,'openness':0.,'eye_bbox':None}
    eye=((labels>0)&(confidence>.65)).astype(np.uint8)
    contours,_=cv2.findContours(eye,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contour=max(contours,key=cv2.contourArea)
        if len(contour)>=5:
            _,(a,b),_=cv2.fitEllipse(contour)
            result['openness']=float(min(a,b)/max(a,b,1))
    if count<=1:return result
    candidates=[]
    for i in range(1,count):
        x,y,w,h,area=stats[i]
        # Reject clipped/border components and require iris surrounding the pupil.
        margin=max(2,int(min(labels.shape)*.025))
        if x<margin or y<margin or x+w>labels.shape[1]-margin or y+h>labels.shape[0]-margin:
            continue
        if not 8<=area<=labels.size*.08:continue
        component=(cc==i).astype(np.uint8)
        ring=(cv2.dilate(component,np.ones((9,9),np.uint8))>0)&(component==0)
        if ring.sum()==0 or np.mean(labels[ring]==2)<.35:continue
        contours,_=cv2.findContours(component,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contour=max(contours,key=cv2.contourArea)
        if len(contour)<5:continue
        ellipse=cv2.fitEllipse(contour)
        (_, _),(a,b),_=ellipse
        if min(a,b)/max(a,b,1)<.25:continue
        candidates.append((int(area),i,ellipse))
    if not candidates:return result
    candidates.sort(key=lambda x:x[0],reverse=True)
    if len(candidates)>1 and candidates[1][0]>.6*candidates[0][0]:return result
    area,i,ellipse=candidates[0]
    _,eye_cc,eye_stats,_=cv2.connectedComponentsWithStats((labels>0).astype(np.uint8),8)
    cy,cx=np.argwhere(cc==i)[0]
    component_id=eye_cc[cy,cx]
    ex,ey,ew,eh,_=map(int,eye_stats[component_id])
    result.update(pupil=ellipse,confidence=float(confidence[cc==i].mean()),pupil_area=area,
                  eye_bbox=(ex,ey,ew,eh))
    return result


def draw_segmentation(frame,labels,metrics):
    colors=np.array([[0,0,0],[180,100,0],[0,180,0],[0,0,230]],dtype=np.uint8)
    color=cv2.resize(colors[labels],(frame.shape[1],frame.shape[0]),interpolation=cv2.INTER_NEAREST)
    mask=cv2.resize(labels,(frame.shape[1],frame.shape[0]),interpolation=cv2.INTER_NEAREST)>0
    result=frame.copy()
    result[mask]=cv2.addWeighted(frame,.65,color,.35,0)[mask]
    if metrics['pupil']:
        (x,y),(a,b),angle=metrics['pupil']
        sx,sy=frame.shape[1]/labels.shape[1],frame.shape[0]/labels.shape[0]
        cv2.circle(result,(round(x*sx),round(y*sy)),3,(0,255,255),-1)
    return result


class AdaptiveRitnet:
    """Find the eye at full resolution, then infer a native-scale automatic crop.

    Cropping preserves pupil pixel size; shrinking the entire frame does not.
    Returned masks always use original frame coordinates.
    """
    def __init__(self,full_engine,crop_engine,batch_engine=None):
        self.full=TensorRTModel(full_engine)
        try:self.crop=TensorRTModel(crop_engine)
        except Exception:
            self.full.close();raise
        self.batch=TensorRTModel(batch_engine) if batch_engine and Path(batch_engine).exists() else None
        self.rois={};self.unknown_since={};self.frame_sizes={};self.view_diagnostics={}
    def warmup(self):
        for model in (self.full,self.crop,self.batch):
            if model is not None:
                for _ in range(5):model.infer_batch([np.zeros((model.height,model.width,3),np.uint8)]*model.buffers['eye'][0].shape[0])
    def infer(self,frame,eye=0):
        start=time.perf_counter();h,w=frame.shape[:2]
        self.frame_sizes[eye]=(w,h)
        roi=self.rois.get(eye)
        if roi is None:
            labels,confidence,_=self.full.infer(frame)
            labels=cv2.resize(labels,(w,h),interpolation=cv2.INTER_NEAREST)
            confidence=cv2.resize(confidence,(w,h),interpolation=cv2.INTER_LINEAR)
            metrics=segmentation_metrics(labels,confidence)
            if metrics['pupil'] is not None and metrics['confidence']>=.85:
                x,y,ew,eh=metrics['eye_bbox']
                cw,ch=min(w,self.crop.width),min(h,self.crop.height)
                if ew<cw*.95 and eh<ch*.95:
                    (cx,cy),_,_=metrics['pupil']
                    x0=int(np.clip(cx-cw/2,0,w-cw));y0=int(np.clip(cy-ch/2,0,h-ch))
                    self.rois[eye]=(x0,y0,cw,ch)
        else:
            x,y,cw,ch=roi
            small,prob,_=self.crop.infer(frame[y:y+ch,x:x+cw])
            labels=np.zeros((h,w),np.uint8);confidence=np.ones((h,w),np.float32)
            labels[y:y+ch,x:x+cw]=cv2.resize(small,(cw,ch),interpolation=cv2.INTER_NEAREST)
            confidence[y:y+ch,x:x+cw]=cv2.resize(prob,(cw,ch),interpolation=cv2.INTER_LINEAR)
        return labels,confidence,(time.perf_counter()-start)*1000
    def infer_views(self,frames):
        """Return local masks plus crop offsets, avoiding full-frame CPU scans."""
        views=[]
        self.view_diagnostics={}
        for i,frame in enumerate(frames):self.frame_sizes[i]=(frame.shape[1],frame.shape[0])
        if len(frames)==2 and self.batch and all(i in self.rois for i in (0,1)):
            crops=[]
            for i,frame in enumerate(frames):
                x,y,w,h=self.rois[i];crops.append(frame[y:y+h,x:x+w])
            results,ms=self.batch.infer_batch(crops)
            views=[(crop,labels,conf,ms/2,self.rois[i]) for i,(crop,(labels,conf)) in enumerate(zip(crops,results))]
            return self.recover_views(frames,views)
        for i,frame in enumerate(frames):
            h,w=frame.shape[:2]
            roi=self.rois.get(i)
            if roi is None:
                labels,conf,ms=self.infer(frame,i)
                views.append((frame,labels,conf,ms,(0,0,w,h)))
            else:
                x,y,w,h=roi;crop=frame[y:y+h,x:x+w]
                labels,conf,ms=self.crop.infer(crop)
                views.append((crop,labels,conf,ms,roi))
        return self.recover_views(frames,views)

    def recover_views(self,frames,views):
        """Verify a lost crop against the same full camera frame before closure.

        A pupil outside the crop must not look like an eyelid closing. Keep the
        crop reference for actual blinks when the full view also has no pupil.
        """
        recovered=[]
        for i,(frame,view) in enumerate(zip(frames,views)):
            local,labels,conf,ms,roi=view
            fh,fw=frame.shape[:2]
            metrics=segmentation_metrics(labels,conf)
            diagnostic=dict(recovery='NONE',closure_blocked=False)
            if roi!=(0,0,fw,fh) and (metrics['pupil'] is None or metrics['confidence']<.85):
                full_labels,full_conf,extra_ms=self.full.infer(frame)
                full_labels=cv2.resize(full_labels,(fw,fh),interpolation=cv2.INTER_NEAREST)
                full_conf=cv2.resize(full_conf,(fw,fh),interpolation=cv2.INTER_LINEAR)
                full_metrics=segmentation_metrics(full_labels,full_conf)
                if full_metrics['pupil'] is not None and full_metrics['confidence']>=.85:
                    # Use recovered coordinates now, not half a second later.
                    view=(frame,full_labels,full_conf,ms+extra_ms,(0,0,fw,fh))
                    cx,cy=full_metrics['pupil'][0];cw,ch=min(fw,self.crop.width),min(fh,self.crop.height)
                    self.rois[i]=(int(np.clip(cx-cw/2,0,fw-cw)),
                                  int(np.clip(cy-ch/2,0,fh-ch)),cw,ch)
                    diagnostic['recovery']='FULL_FRAME_PUPIL'
                else:
                    x,y,w,h=roi
                    outside=(full_labels>0)&(full_conf>.65)
                    outside[y:y+h,x:x+w]=False
                    diagnostic['outside_eye_pixels']=int(outside.sum())
                    diagnostic['closure_blocked']=bool(outside.sum()>fw*fh*.01)
                    diagnostic['recovery']='EYE_OUTSIDE_CROP' if diagnostic['closure_blocked'] else 'FULL_FRAME_NO_PUPIL'
                    view=(local,labels,conf,ms+extra_ms,roi)
                    if diagnostic['closure_blocked']:self.rois.pop(i,None)
            self.view_diagnostics[i]=diagnostic
            recovered.append(view)
        return recovered

    def feedback(self,eye,state,metrics,now):
        if state=='unknown':
            self.unknown_since.setdefault(eye,now)
            if now-self.unknown_since[eye]>.5:self.rois.pop(eye,None)
        else:
            self.unknown_since.pop(eye,None)
            if state=='open' and metrics['pupil'] and eye in self.rois:
                x,y,w,h=self.rois[eye];(cx,cy),_,_=metrics['pupil']
                # Recenter before the pupil reaches the border rejection zone.
                if not (x+w*.3<cx<x+w*.7 and y+h*.3<cy<y+h*.7):
                    fw,fh=self.frame_sizes[eye]
                    self.rois[eye]=(int(np.clip(cx-w/2,0,fw-w)),
                                    int(np.clip(cy-h/2,0,fh-h)),w,h)
    def close(self):
        if self.batch:self.batch.close()
        self.crop.close();self.full.close()
