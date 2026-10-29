"""Calibrated detector fusion and independent binocular closure gating."""
from dual_eye_tracker import BinocularState
from blink_events import BlinkEvents
import math

TURN_THRESHOLD=.55


class FusionControl:
    def __init__(self,calibration,input_loss_window=.2):
        if not math.isfinite(input_loss_window) or input_loss_window < 0:
            raise ValueError('input_loss_window must be finite and nonnegative')
        self.input_loss_window=input_loss_window
        self.calibration=calibration;self.reset_gesture()
    def reset_gesture(self):
        self.blink=BinocularState(transition_grace=.2);self.last_open=None;self.horizontal=0.;self.eye_directions=[]
        if hasattr(self,'gestures'):self.gestures.reset()
        else:self.gestures=BlinkEvents(self.calibration.count)
        self.event=None;self.outgoing_event=None;self.long_close=False;self.motion_valid=False;self.open_eyes=False
        if hasattr(self,'left_gestures'):self.left_gestures.reset()
        else:self.left_gestures=BlinkEvents(1)
        self.left_event=None;self.input_grace=False;self.loss_since=None;self.last_motion=None
        self.eye_states=['unknown']*self.calibration.count
    def update(self,features,quality,states,now,fresh,gesture_states=None):
        self.calibration.observe(features,quality,states,now,fresh)
        raw_states=states if gesture_states is None else gesture_states
        self.eye_states=list(raw_states)
        left_event,_=self.left_gestures.update(raw_states[:1],now,fresh and self.calibration.ready)
        if not self.left_gestures.continuity:self.left_event=None
        elif left_event is not None:self.left_event=left_event
        elif self.left_event is not None and now-self.left_event['end']>.18:self.left_event=None
        self.event,self.long_close=self.gestures.update(raw_states,now,fresh and self.calibration.ready)
        if not self.gestures.continuity or self.long_close:self.outgoing_event=None
        elif self.event is not None:self.outgoing_event=self.event
        elif self.outgoing_event is not None and now-self.outgoing_event['end']>.18:self.outgoing_event=None
        self.open_eyes=all(s=='open' for s in raw_states)
        self.motion_valid=False
        permitted=fresh and self.calibration.ready;candidate=None
        reason='READY' if permitted else 'CALIBRATING' if fresh else 'STALE_OR_UNSYNCED'
        if permitted and all(s=='open' for s in states):
            candidate,permitted,self.eye_directions,reason=self.calibration.horizontal(features,quality)
            self.motion_valid=bool(permitted and self.open_eyes)
        elif permitted:
            permitted=self.last_open is not None and 0<=now-self.last_open<=1.2
            if not permitted:reason='NO_RECENT_OPEN_GAZE'
        closed,valid=self.blink.update(states if len(states)==2 else states*2,now,permitted)
        if valid and candidate is not None:self.horizontal=candidate;self.last_open=now
        if not valid:
            self.last_open=None
            if reason=='READY':reason='EYE_STATE_UNKNOWN_OR_DISAGREES'
        self.input_grace=False
        if self.motion_valid:
            self.loss_since=None;self.last_motion=self.horizontal
        elif not fresh or not self.calibration.ready or 'closed' in raw_states:
            # An observed closure always stops; it cannot borrow the loss window.
            self.loss_since=None;self.last_motion=None
        else:
            if self.loss_since is None:self.loss_since=now
            self.input_grace=(self.last_motion is not None and
                              0<=now-self.loss_since<self.input_loss_window)
            if self.input_grace:
                self.horizontal=self.last_motion
                self.motion_valid=True
                reason='INPUT_LOSS_GRACE'
        return self.horizontal if valid or self.input_grace else 0.,closed,valid,reason


def combine_eye_state(ritnet_state,pure_valid):
    # A temporal pupil tracker can follow a dark eyelid through closure.
    # It supplies direction features, not a veto on RITnet's closure evidence.
    if ritnet_state=='closed':return 'closed'
    if pure_valid:
        return 'open'
    return ritnet_state
