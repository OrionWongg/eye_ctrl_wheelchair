import math
import pytest
from eye_shared_control.confirm import ConfirmMachine, ConfirmConfig


class Harness:
    def __init__(self, cfg=None):
        self.m = ConfirmMachine(cfg)
        self.now = 0.
        self.context = 1
        self.events = []

    def advance(self, duration=.1, h=0., closed=False, valid=True):
        for _ in range(max(1, round(duration / .01))):
            self.now += .01
            self.m.status(self.now, 'manager', self.context, True,
                          task_id='active', state=2, executed_action=1)
            self.m.signal(self.now, h, closed, valid)
            self.events.extend(self.m.drain_events())

    def blink(self, h=0.):
        self.advance(.1, h, True)
        self.advance(.1, h)

    def confirm(self, h=0.):
        self.advance(.3, h)
        self.blink(h)
        self.blink(h)
        return self.m.pending

    def motions(self):
        return {e.event_id: e for e in self.events if e.kind != 3}


@pytest.mark.parametrize('direction', [-.8, 0., .8])
def test_gaze_never_starts_motion(direction):
    h = Harness()
    h.advance(4, direction)
    assert h.m.candidate
    assert not h.motions()
    assert h.m.task['task_id'] == 'active'

@pytest.mark.parametrize('horizontal',[-1.01,1.01])
def test_out_of_range_valid_flag_does_not_authorize(horizontal):
    h=Harness();h.confirm()
    h.advance(.01,horizontal)
    assert not h.m.source_valid and h.m.pending is None
    assert any(e.kind==3 for e in h.events)


def test_single_blink_no_start_or_resume():
    h = Harness()
    h.advance(.4)
    h.blink()
    h.advance(.6)
    assert not h.motions()


@pytest.mark.parametrize('direction,kind,yaw', [(0., 1, 0.), (-.8, 2, math.pi / 12), (.8, 2, -math.pi / 12)])
def test_double_blink_submits_once_then_consumes(direction, kind, yaw):
    h = Harness()
    e = h.confirm(direction)
    assert e and e.kind == kind
    assert e.candidate.relative_yaw == pytest.approx(yaw)
    assert h.m.candidate is None
    assert h.m.advertised() == e.candidate
    h.blink(direction)  # Third blink cannot form another transaction.
    assert len(h.motions()) == 1
    assert all(item is e for item in h.events if item.kind != 3)
    h.m.acknowledge(h.now, e.event_id, True, 'accepted')
    assert h.m.advertised() is None
    h.blink(direction)
    assert len(h.motions()) == 1


def test_blink_pupil_noise_preserves_frozen_candidate():
    h = Harness()
    h.advance(.4, -.8)
    c = h.m.candidate
    h.advance(.1, .99, True)
    h.advance(.1, -.8)
    h.advance(.1, .99, True)
    h.advance(.1, -.8)
    assert h.m.pending.candidate == c


def test_stable_reselection_cancels_transaction():
    h = Harness()
    h.advance(.4, -.8)
    h.blink(-.8)
    h.advance(.3, .8)
    h.blink(.8)
    assert not h.motions()
    assert h.m.candidate.relative_yaw < 0


def test_no_candidate_no_default_forward():
    h = Harness()
    h.blink()
    h.blink()
    assert not h.motions()


def test_context_change_during_blink_rejects_old_confirmation():
    h = Harness()
    h.advance(.4)
    h.blink()
    h.context += 1
    h.blink()
    assert not h.motions()


def test_expired_candidate_cannot_submit():
    h = Harness(ConfirmConfig(candidate_ttl=.2))
    h.advance(.4)
    h.blink()
    h.blink()
    assert not h.motions()


def test_expired_double_gap_does_not_submit():
    h = Harness()
    h.advance(.4)
    h.blink()
    h.advance(.5)
    h.blink()
    assert not h.motions()


@pytest.mark.parametrize('fault', ['invalid', 'timeout', 'long_close'])
def test_fault_cancels_and_requires_new_confirmation(fault):
    h = Harness()
    e = h.confirm()
    h.m.acknowledge(h.now, e.event_id, True)
    if fault == 'invalid':
        h.advance(.1, valid=False)
    elif fault == 'timeout':
        h.now += .3
        h.m.tick(h.now)
        h.events.extend(h.m.drain_events())
    else:
        h.advance(.8, closed=True)
    assert any(e.kind == 3 for e in h.events)
    count = len(h.motions())
    h.advance(.5)
    h.blink()
    h.advance(.5)
    assert len(h.motions()) == count


def test_cancel_discards_unsent_event_and_retries():
    h = Harness()
    h.confirm()
    h.m.invalidate('USER_CANCEL', h.now)
    assert not h.m.pending
    assert all(e.kind == 3 for e in h.m.drain_events())
    h.advance(.8)
    assert len(h.motions()) == 1


def test_ack_timeout_withdraws_original_candidate():
    h = Harness()
    e = h.confirm()
    original_expiry = e.candidate.expires
    h.advance(.3)
    assert h.m.advertised().expires == original_expiry
    h.advance(1.3)
    assert h.m.pending is None
    assert h.m.ack == 'TIMEOUT'
    assert len(h.motions()) == 1


def test_wrong_ack_cannot_consume_pending_event():
    h = Harness()
    e = h.confirm()
    assert not h.m.acknowledge(h.now, e.event_id + 1, True)
    assert h.m.pending is e
    assert h.m.acknowledge(h.now, e.event_id, False, 'unsafe')
    assert h.m.ack == 'REJECTED'


def test_status_timeout_cancels_and_timestamp_age_is_not_refreshed():
    h = Harness()
    h.confirm()
    h.now += .36
    h.m.signal(h.now, 0., False, True)
    assert h.m.pending is None
    h.m.status(h.now, 'manager', 1, True)
    h.m.signal(h.now, 0., False, True, source_age=.24)
    assert h.m.healthy(h.now)
    assert not h.m.healthy(h.now + .02)


def test_candidate_version_unchanged_for_heartbeat():
    h = Harness()
    h.advance(.4)
    v = h.m.candidate.candidate_version
    h.advance(2)
    assert h.m.candidate.candidate_version == v


def test_unavailable_context_withdraws_selection():
    h = Harness()
    h.advance(.4)
    h.m.status(h.now, 'manager', 1, False)
    assert h.m.advertised() is None


def test_long_close_only_one_cancel_per_closure():
    h = Harness()
    h.advance(.4)
    h.advance(2, closed=True)
    assert len([e for e in h.events if e.kind == 3]) == 1


def test_latency_records_are_associated_and_reception_named():
    h = Harness()
    e = h.confirm()
    h.m.acknowledge(h.now, e.event_id, True)
    measurements = h.m.drain_measurements()
    assert {'preview', 'stable', 'confirmation', 'ack'} <= {m['stage'] for m in measurements}
    for record in measurements:
        assert {'stage', 'monotonic_s', 'event_id', 'candidate_id', 'task_id'} <= record.keys()


def test_can_accept_false_does_not_withdraw_inflight_submission():
    h = Harness()
    event = h.confirm()
    h.m.status(h.now, 'manager', h.context, False, state=3)
    assert h.m.pending is event
    assert h.m.advertised() == event.candidate
    h.now += .05
    h.m.signal(h.now, -.8, False, True)
    assert h.m.pending is event
    assert h.m.candidate is None
    assert h.m.acknowledge(h.now, event.event_id, True)


def test_long_close_cancels_even_when_context_disallows_new_confirmation():
    h = Harness()
    for _ in range(90):
        h.now += .01
        h.m.status(h.now, 'manager', h.context, False)
        h.m.signal(h.now, 0., True, True)
        h.events.extend(h.m.drain_events())
    assert len([e for e in h.events if e.kind == 3]) == 1
    assert not h.motions()


def test_invalid_signal_then_publish_tick_preserves_cancel_and_removes_motion():
    h = Harness()
    h.confirm()
    pending = h.m.pending
    h.m.outbox.append(pending)  # Simulate a retry queued before the invalid frame.
    h.now += .02
    h.m.signal(h.now, 0., False, False)
    # ConfirmTransport.receive calls publish(), which ticks before draining events.
    h.m.tick(h.now + .001)
    h.m.tick(h.now + .002)
    outgoing = h.m.drain_events()
    assert len(outgoing) == 1
    assert outgoing[0].kind == 3
    assert not h.m.pending
    h.m.tick(h.now + .003)
    assert not h.m.drain_events()  # No fresh cancellation event each timer tick.


def test_stop_latency_measures_onset_and_pairs_cancel_event_id():
    h = Harness()
    h.advance(.4)
    expected_onset = h.now + .01
    h.advance(.8, closed=True)
    cancellations = [e for e in h.events if e.kind == 3]
    assert len(cancellations) == 1
    event = cancellations[0]
    measurements = h.m.drain_measurements()
    gesture = next(m for m in measurements if m['stage'] == 'stop_gesture')
    recognized = next(m for m in measurements if m['stage'] == 'stop_recognized')
    assert gesture['event_id'] == recognized['event_id'] == event.event_id
    assert event.event_id > 0
    assert gesture['monotonic_s'] == pytest.approx(expected_onset)
    delay = recognized['monotonic_s'] - gesture['monotonic_s']
    assert h.m.cfg.long_close <= delay < h.m.cfg.long_close + .011
    assert recognized['monotonic_s'] == event.created


def test_stable_and_confirmation_share_candidate_id():
    h = Harness()
    event = h.confirm(-.8)
    measurements = h.m.drain_measurements()
    stable = next(m for m in measurements if m['stage'] == 'stable')
    confirmation = next(m for m in measurements if m['stage'] == 'confirmation')
    assert stable['candidate_id'] == confirmation['candidate_id'] == event.candidate.candidate_id
    assert confirmation['event_id'] == event.event_id
    assert stable['monotonic_s'] < confirmation['monotonic_s']
