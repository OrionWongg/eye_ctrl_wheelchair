import json
from eye_shared_control.confirm import ConfirmMachine, ConfirmConfig
from eye_shared_control.fusion import FusionInput


class Feed:
    def __init__(self):
        self.m = ConfirmMachine(ConfirmConfig(enter=.55, leave=.25))
        self.f = FusionInput(self.m)
        self.now = 10.
        self.seq = 0

    def sample(self, eyes=('open', 'open'), event=None, grace=False, valid=True, **changes):
        self.now += .05
        self.seq += 1
        self.m.status(self.now, 'navigation', 1, True)
        d = dict(schema=1, session='eyes', sequence=self.seq, stamp_ns=int(self.now*1e9),
                 source_t=self.now, horizontal=0., fresh=True, calibrated=True,
                 open=list(eyes)==['open', 'open'], motion_valid=valid,
                 gesture_continuity=True, long_close=False, blink_event=None,
                 eyes=list(eyes), input_grace=grace, left_blink_event=event)
        d.update(changes)
        self.f.receive(json.dumps(d), self.now, self.now, True)
        if self.m.pending:
            self.m.acknowledge(self.now, self.m.pending.event_id, True)
        self.m.drain_events()

    def start(self):
        self.sample()
        self.sample(('closed', 'open'), valid=False)
        self.sample(('closed', 'open'), valid=False)
        self.sample(event=dict(id=1, start=self.now-.05, end=self.now+.05, durations=[.1]))
        assert self.f.state == 'ACTIVE'


def test_left_blink_starts_and_both_open_resume_without_event():
    f = Feed()
    f.start()
    f.sample(('open', 'closed'), valid=False)
    assert f.f.state == 'PAUSED'
    f.sample(('open', 'unknown'), grace=True)
    assert f.f.state == 'PAUSED'
    f.sample()
    assert f.f.state == 'ACTIVE' and f.f.reason == 'BOTH_EYES_OPEN'


def test_unknown_grace_holds_active_then_pauses_and_recovers():
    f = Feed()
    f.start()
    f.sample(('unknown', 'open'), grace=True, gesture_continuity=False)
    assert f.f.state == 'ACTIVE'
    f.sample(('unknown', 'open'), valid=False, gesture_continuity=False)
    assert f.f.state == 'PAUSED'
    f.sample()
    assert f.f.state == 'ACTIVE'


def test_right_blink_cannot_arm_and_single_eye_cannot_resume():
    f = Feed()
    f.sample()
    f.sample(('open', 'closed'), valid=False)
    f.sample()
    assert f.f.state == 'DISARMED'
    f.start()
    f.sample(('closed', 'open'), valid=False)
    f.sample(('open',))
    assert f.f.state == 'PAUSED'


def test_closure_overrides_grace_and_stale_input_stops():
    f = Feed()
    f.start()
    f.sample(('closed', 'open'), grace=True)
    assert f.f.state == 'PAUSED'
    f.sample()
    assert f.f.active
    f.sample(stamp_ns=1)
    assert not f.f.active
