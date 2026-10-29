from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from fusion_control import FusionControl


class Calibration:
    count = 2
    ready = True
    def observe(self, *args): pass
    def horizontal(self, features, quality):
        return features, quality, [], 'READY' if quality else 'NO_DIRECTION_FEATURE'


def sample(control, t, eyes=('open', 'open'), direction=.8, quality=True, fresh=True):
    return control.update(direction, quality, list(eyes), t, fresh)


def test_loss_window_holds_direction_expires_and_resets():
    c = FusionControl(Calibration())
    sample(c, 1.)
    sample(c, 1.05, ('unknown', 'open'), direction=-.8)
    assert c.input_grace and c.motion_valid and c.horizontal == .8
    sample(c, 1.24, ('unknown', 'open'))
    assert c.input_grace
    sample(c, 1.26, ('unknown', 'open'))
    assert not c.input_grace and not c.motion_valid
    sample(c, 1.3)
    sample(c, 1.35, quality=False)
    assert c.input_grace
    sample(c, 1.4)
    assert not c.input_grace and c.motion_valid


@pytest.mark.parametrize('eyes', [('closed', 'open'), ('open', 'closed'), ('closed', 'closed')])
def test_closure_never_uses_loss_window(eyes):
    c = FusionControl(Calibration())
    sample(c, 1.)
    sample(c, 1.05, eyes)
    assert not c.motion_valid and not c.input_grace
    sample(c, 1.1, ('unknown', 'open'))
    assert not c.input_grace


def test_no_initial_grace_and_stale_input_does_not_borrow_window():
    c = FusionControl(Calibration())
    sample(c, 1., ('unknown', 'unknown'))
    assert not c.input_grace
    sample(c, 1.05)
    sample(c, 1.1, fresh=False)
    assert not c.motion_valid and not c.input_grace


def test_left_blink_independent_of_right_eye():
    c = FusionControl(Calibration())
    sample(c, 1.)
    sample(c, 1.05, ('open', 'closed'))
    sample(c, 1.15)
    assert c.left_event is None
    sample(c, 1.2, ('closed', 'open'))
    sample(c, 1.3)
    assert c.left_event is not None and len(c.left_event['durations']) == 1
    assert c.event is None
