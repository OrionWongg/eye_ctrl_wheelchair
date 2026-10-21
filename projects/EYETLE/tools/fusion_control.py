"""Calibrated detector fusion and independent binocular closure gating."""
from dual_eye_tracker import BinocularState
from blink_events import BlinkEvents

TURN_THRESHOLD=.55


class FusionControl:
    def __init__(self,calibration):
        self.calibration=calibration;self.reset_gesture()
    def reset_gesture(self):
        self.blink=BinocularState(transition_grace=.2);self.last_open=None;self.horizontal=0.;self.eye_directions=[]
        if hasattr(self,'gestures'):self.gestures.reset()
        else:self.gestures=BlinkEvents(self.calibration.count)
        self.event=None;self.outgoing_event=None;self.long_close=False;self.motion_valid=False;self.open_eyes=False
    def update(self,features,quality,states,now,fresh,gesture_states=None):
        self.calibration.observe(features,quality,states,now,fresh)
        raw_states=states if gesture_states is None else gesture_states
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
        return self.horizontal if valid else 0.,closed,valid,reason


def combine_eye_state(ritnet_state,pure_valid):
    # A temporal pupil tracker can follow a dark eyelid through closure.
    # It supplies direction features, not a veto on RITnet's closure evidence.
    if ritnet_state=='closed':return 'closed'
    if pure_valid:
        return 'open'
    return ritnet_state
