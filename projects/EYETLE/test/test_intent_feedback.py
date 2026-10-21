"""Feedback-only tests: no model, detector, camera or ROS imports."""
from pathlib import Path
import sys
import math
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from intent_feedback import parse_feedback, display_lines


def test_new_feedback_displays_candidate_task_action_and_ack():
    sample = dict(interaction_mode='intent_confirm', preview=-1,
                  preview_relative_yaw=math.radians(15), stable=True, healthy=True,
                  ack='ACCEPTED', reason='ACCEPTED',
                  task=dict(state=2, executed_action=2, task_id='task-1', reason='HEADING_TRACKING'))
    parsed = parse_feedback(sample, 12.)
    text = '\n'.join(display_lines(parsed))
    for expected in ('LEFT 15.0 deg', 'stable: YES', 'Task: RUNNING', 'action: LEFT',
                     'ACK: ACCEPTED', 'HEADING_TRACKING'):
        assert expected in text
    assert parsed['state'] == 'RUNNING' and parsed['stamp'] == 12.
    assert parsed['task_id'] == 'task-1'
    assert text.isascii()


def test_continue_rejection_and_source_invalid_feedback():
    parsed = parse_feedback(dict(interaction_mode='intent_confirm', preview=0,
                                 ack='REJECTED', reason='UNSAFE', stable=False,
                                 task=dict(state=5, executed_action=0)), 1.)
    text = '\n'.join(display_lines(parsed))
    for expected in ('CONTINUE (limited task)', 'stable: NO', 'ACK: REJECTED',
                     'Task: BLOCKED', 'action: STOP', 'UNSAFE', 'INPUT INVALID'):
        assert expected in text


def test_stale_feedback_never_displays_old_motion_as_current():
    parsed = parse_feedback(dict(interaction_mode='intent_confirm', preview=1,
                                 consumed=True, task=dict(state=2, executed_action=3)), 1.)
    assert 'CONSUMED' in display_lines(parsed)[0]
    text = '\n'.join(display_lines(parsed, fresh=False))
    assert 'UNKNOWN' in text and 'RUNNING' not in text and 'RIGHT' not in text


def test_legacy_feedback_preserves_resume_fields():
    parsed = parse_feedback('{"state":"PAUSED","resume_phase":"BLINK_TO_RESUME","resume_progress":0.4}', 3.)
    assert parsed['interaction_mode'] == 'gaze_joystick'
    assert parsed['state'] == 'PAUSED'
    assert parsed['resume_phase'] == 'BLINK_TO_RESUME'
    assert parsed['resume_progress'] == .4


@pytest.mark.parametrize('payload', ['[]', '{}garbage', dict(interaction_mode='intent_confirm', preview=5),
                                    dict(interaction_mode='intent_confirm', preview_relative_yaw=float('nan'))])
def test_malformed_feedback_rejected(payload):
    with pytest.raises(ValueError):
        parse_feedback(payload, 1.)


def test_raw_preview_does_not_mislabel_older_stable_candidate():
    parsed = parse_feedback(dict(interaction_mode='intent_confirm', preview=1,
                                 preview_relative_yaw=-math.pi / 12,
                                 candidate_kind=2, candidate_relative_yaw=math.pi / 12,
                                 stable=True), 1.)
    line = display_lines(parsed)[0]
    assert 'Preview: RIGHT' in line
    assert 'candidate: LEFT 15.0 deg' in line
    assert 'stable: YES' in line


def test_directional_feedback_displays_navigation_phase_and_keyboard_semantics():
    parsed=parse_feedback(dict(interaction_mode='directional_intent',preview=-1,candidate_kind=2,
        candidate_relative_yaw=.26,stable=True,healthy=True,ack='ACCEPTED',
        task=dict(state='SEEK_ENTRY',executed_action=1)),1.)
    text='\n'.join(display_lines(parsed))
    assert 'LEFT INTENT (A)' in text and 'SEEK_ENTRY' in text and 'FORWARD' in text
    assert '15.0 deg' not in text
