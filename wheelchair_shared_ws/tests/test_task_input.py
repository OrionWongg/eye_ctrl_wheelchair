import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/wheelchair_control'))
from wheelchair_control.task_input import KeyboardState, InputBoundary


def ready():
    state = KeyboardState('s')
    state.status(('manager', 1), True, 0)
    return state


def test_requires_fresh_context_and_explicit_confirmation():
    state = KeyboardState('s')
    assert state.select('continue', 0) is None
    state = ready()
    assert state.select('left', .1)['relative_yaw'] > 0
    assert state.pending is None
    assert state.confirm(.2)['kind'] == 2
    assert state.confirm(.3) is None


def test_retry_original_identity_bounded_and_consumed():
    state = ready()
    state.select('continue', .1)
    event = state.confirm(.2)
    state.status(('manager', 1), False, .4)
    assert state.retry(.4) is event
    assert not state.ack('old-session', event['event_id'])
    assert state.ack('s', event['event_id'])
    assert state.confirm(.4) is None
    assert state.retry(.5) is None


def test_context_change_cancel_and_expiry_invalidate():
    state = ready()
    state.select('right', .1)
    state.confirm(.2)
    state.status(('manager', 2), True, .3)
    assert state.retry(.4) is None
    state.select('right', .4)
    event = state.confirm(.4)
    assert state.cancel()['event_id'] > event['event_id']
    assert state.retry(.5) is None
    assert state.select('continue', 2) is None


def test_pending_does_not_extend_original_deadline():
    state = ready()
    state.select('continue', .1)
    state.confirm(.2)
    for now in [.4, .6, .8, 1., 1.2, 1.4]:
        state.status(('manager', 1), True, now)
        assert state.retry(now)
    assert state.retry(1.6) is None
    assert state.candidate is None


def test_candidate_cannot_be_confirmed_after_expiry():
    state = ready()
    state.select('left', 0)
    state.status(('manager', 1), True, 1.6)
    assert state.confirm(1.6) is None


def test_latency_report_deduplicates_missing_and_session_keys():
    spec = importlib.util.spec_from_file_location('report', ROOT / 'scripts/intent_latency_report.py')
    report = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(report)
    rows = [dict(stage=stage, monotonic_s=stamp, source_session_id=session, event_id=1)
            for stage, stamp, session in [('confirmation', 1., 'a'), ('confirmation', 1.1, 'a'),
                                          ('ack', 1.2, 'a'), ('ack', 5., 'b')]]
    result = report.summarize(rows)['metrics']
    assert result['confirmation_to_ack']['count'] == 1
    assert abs(result['confirmation_to_ack']['p95_ms'] - 200) < 1e-8
    assert result['ack_to_motion']['p50_ms'] is None


def clock_ready():
    boundary = InputBoundary()
    assert not boundary.clock(10., 0.)
    assert boundary.clock(10.1, .1)
    return boundary


def test_retired_manager_and_backward_context_cannot_return():
    boundary = clock_ready()
    assert boundary.status('a', 'a', 2, 5, 10.1, 10.1)
    assert not boundary.status('a', 'a', 1, 6, 10.1, 10.1)
    assert boundary.status('b', 'b', 0, 1, 10.1, 10.1)
    assert not boundary.status('a', 'a', 3, 500, 10.1, 10.1)
    assert not boundary.status('b', 'wrong', 0, 2, 10.1, 10.1)
    assert not boundary.status('b', 'b', 0, 1, 10.1, 10.1)


def test_ack_requires_current_manager_valid_status_fresh_stamp_and_live_transaction():
    boundary = clock_ready()
    boundary.status('a', 'a', 1, 1, 10.1, 10.1)
    assert boundary.ack('a', 1, 10.1, 10.2, .1, .2)
    assert boundary.ack('a', 2, 10.1, 10.2, .1, .2)
    assert not boundary.ack('old', 1, 10.1, 10.2, .1, .2)
    assert not boundary.ack('a', 0, 10.1, 10.2, .1, .2)
    assert not boundary.ack('a', 3, 10.1, 10.2, .1, .2)
    assert not boundary.ack('a', 1, 9., 10.2, .1, .2)
    assert not boundary.ack('a', 1, 12., 10.2, .1, .2)
    assert not boundary.ack('a', 1, 10.1, 10.2, .1, 1.7)


def test_clock_pause_jump_blocks_status_until_new_progress():
    boundary = clock_ready()
    assert not boundary.clock(10.1, .4)
    assert not boundary.status('a', 'a', 1, 1, 10.1, 10.1)
    assert boundary.clock(10.2, .5)
    assert not boundary.clock(9., .6)
    assert boundary.clock(9.1, .7)
    assert not boundary.clock(12., .8)
    assert boundary.clock(12.1, .9)


def test_not_available_preserves_pending_but_prohibits_new_confirmation():
    state = ready()
    state.select('left', .1)
    state.status(('manager', 1), False, .2)
    assert state.confirm(.2) is None
    state.status(('manager', 1), True, .3)
    event = state.confirm(.3)
    state.status(('manager', 1), False, .4)
    assert state.pending is event
    assert state.select('right', .4) is None
