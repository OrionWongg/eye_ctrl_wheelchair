"""RITnet eye closure, independent of gaze estimation and 3D calibration."""
from ritnet_backend import AdaptiveRitnet, segmentation_metrics
from ritnet_tracker import EyeState


class RitnetBlink:
    def __init__(self,full_engine,crop_engine,batch_engine,count,model=None):
        self.model=model if model is not None else AdaptiveRitnet(
            full_engine,crop_engine,batch_engine if count==2 else None)
        self.states=[EyeState() for _ in range(count)]
        self.rois=[None]*count

    def warmup(self):self.model.warmup()

    def infer(self,frames,stamps):
        states=[];details=[]
        for i,(view,stamp) in enumerate(zip(self.model.infer_views(frames),stamps)):
            local,labels,confidence,ms,roi=view
            # Appearance references must not span different crops.
            if self.rois[i]!=roi:
                self.states[i]=EyeState();self.rois[i]=roi
            metrics=segmentation_metrics(labels,confidence)
            state=self.states[i].update(local,labels,confidence,metrics,stamp,
                                        appearance_frame=frames[i],roi=roi)
            recovery=getattr(self.model,'view_diagnostics',{}).get(i,{})
            if recovery.get('closure_blocked') and state=='closed':
                state='unknown'
                self.states[i].diagnostics['reason']='EYE_OUTSIDE_CROP'
            full_metrics=dict(metrics)
            if metrics['pupil'] is not None:
                (cx,cy),axes,angle=metrics['pupil'];x,y,w,h=roi
                full_metrics['pupil']=((x+cx*w/labels.shape[1],y+cy*h/labels.shape[0]),axes,angle)
            self.model.feedback(i,state,full_metrics,stamp)
            states.append(state)
            details.append(dict(state=state,crop_tracking=dict(recovery),closure_diagnostics=dict(self.states[i].diagnostics),pupil_confidence=float(metrics['confidence']),
                pupil_center=list(full_metrics['pupil'][0]) if full_metrics['pupil'] is not None else None,
                pupil_area=int(metrics['pupil_area']),eye_area=int(metrics['eye_area']),
                segmentation_confidence=float(confidence.mean()),roi=list(roi),inference_ms=float(ms)))
        return states,details

    def close(self):self.model.close()
