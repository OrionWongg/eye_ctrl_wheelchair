import pytest
from eye_shared_control.core import EyeMachine, Command, State, Config

class Harness:
    def __init__(self):
        self.m = EyeMachine()
        self.now = 0.
        self.sequence = 0
        self.required = False
        self.ready = True
        self.session = 'safety-a'

    def advance(self, seconds=.1, h=0., closed=False, valid=True):
        steps = max(1, int(round(seconds / .01)))
        for _ in range(steps):
            self.now += seconds / steps
            self.sequence += 1
            self.m.status(self.now, self.session, self.sequence, self.required, self.ready)
            out = self.m.signal(self.now, h, closed, valid)
        return out

    def blink(self):
        self.advance(.1, closed=True)
        self.advance(.1)

    def resume(self):
        self.advance(.1)
        self.advance(.1, closed=True)
        return self.advance(.1)

    def arm(self):
        self.advance(.1)
        self.blink()
        self.blink()
        return self.advance(.3)


def test_single_blink_and_gaze_do_not_start():
    h = Harness()
    h.advance(1., -.8)
    h.blink()
    assert not h.advance(.3).enabled
    assert h.m.command == Command.STOP


def test_double_blink_and_sustained_direction_and_return():
    h = Harness()
    assert h.arm().command == Command.FORWARD
    assert h.advance(.3, -.6).command == Command.LEFT
    assert h.advance(.5, -.6).command == Command.LEFT
    assert h.advance(.3, .6).command == Command.RIGHT
    assert h.advance(.3, 0.).command == Command.FORWARD


def test_dwell_and_hysteresis_reject_jitter():
    h = Harness()
    h.arm()
    for _ in range(8):
        h.advance(.1, -.26)
        assert h.advance(.1, -.24).command == Command.FORWARD
    assert h.advance(.3, -.6).command == Command.LEFT
    assert h.advance(.3, -.16).command == Command.LEFT
    assert h.advance(.1, -.14).command == Command.LEFT
    assert h.advance(.3, -.14).command == Command.FORWARD


def test_short_blink_preserves_direction_but_long_close_stops():
    h = Harness()
    h.arm()
    h.advance(.3, -.6)
    assert h.advance(.15, .6, closed=True).command == Command.LEFT
    assert h.advance(.1, -.6).command == Command.LEFT
    result = h.advance(.66, closed=True)
    assert not result.enabled and result.command == Command.STOP
    assert not h.advance(.5).enabled


def test_invalid_stops_immediately_and_recovery_needs_fresh_blink():
    h = Harness()
    h.arm()
    assert not h.advance(.01, valid=False).enabled
    assert not h.advance(.5).enabled
    assert h.resume().enabled


def test_source_timeout_cannot_be_hidden_by_next_arrival():
    h = Harness()
    h.arm()
    h.now += .3
    h.sequence += 1
    h.m.status(h.now, h.session, h.sequence, False, True)
    result = h.m.signal(h.now, 0., False, True)
    assert not result.enabled
    assert not h.advance(.5).enabled


def test_healthy_motion_continues_beyond_five_seconds():
    h = Harness()
    h.arm()
    assert h.advance(10.).enabled
    assert not h.advance(.01, valid=False).enabled


def test_block_handshake_does_not_erase_new_authorization():
    h = Harness()
    h.arm()
    h.required, h.ready = True, False
    assert not h.advance(.1).enabled
    h.blink()
    h.blink()
    assert not h.advance(.3).enabled
    h.ready = True
    assert not h.advance(.3).enabled
    assert h.resume().enabled
    assert h.advance(.5).enabled  # repeated requires=true + ready=true acknowledgements
    h.required = False
    assert h.advance(.2).enabled
    h.required, h.ready = True, False
    assert not h.advance(.01).enabled


def test_safety_session_change_and_status_timeout():
    h = Harness()
    h.arm()
    h.session = 'safety-b'
    assert not h.advance(.01).enabled
    h.arm()
    h.m.status_at = h.now - .4
    assert not h.m.tick(h.now).enabled


def test_replayed_status_cannot_make_status_fresh():
    h = Harness()
    h.arm()
    stamp = h.m.status_at
    h.m.status(h.now + .3, h.session, h.sequence, False, True)
    assert h.m.status_at == stamp


def test_nonfinite_signal_invalid_even_when_marked_valid():
    h = Harness()
    h.arm()
    assert not h.advance(.01, float('nan')).enabled


def test_long_closure_without_new_samples_stops_by_source_watchdog():
    h = Harness()
    h.arm()
    h.advance(.1, closed=True)
    assert not h.m.tick(h.now + .26).enabled


def test_bad_thresholds_rejected():
    with pytest.raises(ValueError):
        EyeMachine(Config(leave=.3))


def test_retired_safety_session_cannot_restore_old_state():
    h = Harness()
    h.arm()
    h.session = 'safety-b'
    h.advance(.01)
    h.m.status(h.now + .01, 'safety-a', 99999, False, True)
    assert h.m.session == 'safety-b'
    assert not h.m.tick(h.now + .01).enabled


def test_healthy_status_can_arm_without_ready_flag():
    h = Harness()
    h.ready = False
    assert h.arm().enabled


def test_repeated_double_blink_does_not_change_active_session():
    h = Harness()
    h.arm()
    start = h.m.active_at
    h.advance(3.)
    h.blink()
    h.blink()
    assert h.m.active_at == start
    assert h.advance(2.).enabled


def test_nonfinite_timeout_configuration_rejected():
    import pytest
    from eye_shared_control.core import Config, EyeMachine
    for key in ('source_timeout','status_timeout','blink_min','blink_max'):
        for value in (float('nan'),float('inf')):
            cfg=Config();setattr(cfg,key,value)
            with pytest.raises(ValueError):EyeMachine(cfg)


def test_pause_reopen_stays_stopped_and_single_blink_resumes():
    h=Harness();h.arm();h.advance(.7,closed=True)
    assert h.m.state==State.PAUSED and h.m.session_enabled
    assert not h.advance(2.).enabled
    assert h.resume().enabled


def test_resume_requires_complete_short_blink_and_rejects_noise():
    h=Harness();h.arm();h.advance(.7,closed=True);h.advance(.1)
    h.advance(.02,closed=True)
    assert not h.advance(.1).enabled
    h.advance(.5,closed=True)
    assert not h.advance(.1).enabled
    assert not h.advance(.1,closed=True).enabled
    assert h.advance(.01).enabled


def test_invalid_signal_cancels_partially_completed_resume():
    h=Harness();h.arm();h.advance(.7,closed=True);h.advance(.1)
    h.advance(.1,closed=True)
    h.advance(.01,valid=False)
    assert not h.advance(1.).enabled
    assert h.m.resume.progress==0
    assert h.resume().enabled


def test_safety_block_cancels_blink_resume_and_cannot_be_bypassed():
    h=Harness();h.arm();h.advance(.7,closed=True);h.advance(.1)
    h.advance(.1,closed=True)
    h.required,h.ready=True,False
    assert not h.resume().enabled
    h.ready=True
    assert not h.advance(1.).enabled
    assert h.resume().enabled


def test_startup_cannot_arm_with_resume_gesture():
    h=Harness()
    assert not h.resume().enabled
    assert h.arm().enabled


def test_eye_timeout_cancels_progress_even_if_next_sample_is_center():
    h=Harness();h.arm();h.advance(.7,closed=True);h.advance(.1)
    h.advance(.1,closed=True);h.now+=.3
    assert not h.advance(.01).enabled
    assert not h.advance(1.).enabled
    assert h.resume().enabled


def test_blink_started_while_blocked_cannot_resume_on_clear():
    h=Harness();h.arm();h.required,h.ready=True,False
    h.advance(.05,closed=True)
    h.ready=True
    h.advance(.05,closed=True)
    assert not h.advance(.1).enabled
    assert h.resume().enabled
