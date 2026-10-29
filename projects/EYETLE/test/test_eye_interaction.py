from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from eye_interaction import EyeInteraction


def feed(m, t, eyes=('open', 'open'), valid=True, grace=False, event=None, h=.7):
    return m.update(list(eyes), valid, grace, event, h, t)


def test_start_short_blink_long_close_and_reopen():
    m = EyeInteraction()
    assert feed(m, 0.) == 'DISARMED'
    assert feed(m, .1, ('open', 'closed'), valid=False) == 'DISARMED'
    assert feed(m, .2) == 'DISARMED'
    assert feed(m, .3, event={'id': 1}) == 'ACTIVE'
    assert feed(m, .4, ('closed', 'open'), valid=False) == 'ACTIVE'
    assert feed(m, .55) == 'ACTIVE'
    assert m.action == 'RIGHT'
    assert feed(m, 1., ('closed', 'open'), valid=False) == 'ACTIVE'
    assert feed(m, 1.19, ('closed', 'open'), valid=False) == 'ACTIVE'
    assert feed(m, 1.21, ('closed', 'open'), valid=False) == 'PAUSED'
    assert m.action == 'STOP'
    assert feed(m, 1.3, ('open', 'unknown'), valid=False) == 'PAUSED'
    assert feed(m, 1.4) == 'ACTIVE'


def test_direction_must_stay_in_same_region_for_window():
    m = EyeInteraction()
    feed(m, 0., event={'id': 1}, h=0.)
    feed(m, .1, h=-.8)
    feed(m, .29, h=-.7)
    assert m.action == 'FORWARD'
    feed(m, .3, h=0.)  # A brief excursion must not accumulate.
    feed(m, .4, h=-.8)
    feed(m, .61, h=-.7)
    assert m.action == 'LEFT'
    feed(m, .7, h=.8)
    feed(m, .8, h=0.)  # A different candidate restarts the window.
    feed(m, .91, h=0.)
    assert m.action == 'LEFT'
    feed(m, 1.01, h=0.)
    assert m.action == 'FORWARD'


def test_blink_cancels_pending_direction_and_unknown_does_not_resume():
    m = EyeInteraction()
    feed(m, 0., event={'id': 1}, h=0.)
    feed(m, .1, h=-.8)
    feed(m, .2, ('closed', 'open'), valid=False)
    feed(m, .3, h=-.8)
    feed(m, .41, h=-.8)
    assert m.action == 'FORWARD'
    feed(m, .51, h=-.8)
    assert m.action == 'LEFT'
    feed(m, .6, ('unknown', 'open'), grace=True)
    feed(m, .81, ('unknown', 'open'), valid=False)
    assert m.state == 'PAUSED'
    assert feed(m, .9, grace=True) == 'PAUSED'
    assert feed(m, 1.) == 'ACTIVE'


def test_window_end_to_end_with_detection():
    from fusion_control import FusionControl
    class Calibration:
        count = 2
        ready = True
        def observe(self, *args): pass
        def horizontal(self, features, quality): return features, quality, [], 'READY'
    detector = FusionControl(Calibration())
    interaction = EyeInteraction()
    def sample(t, eyes):
        h, _, _, _ = detector.update(.7, True, eyes, t, True)
        return interaction.update(eyes, detector.motion_valid, detector.input_grace,
                                  detector.left_event, h, t)
    assert sample(1., ['open', 'open']) == 'DISARMED'
    sample(1.05, ['closed', 'open'])
    assert sample(1.15, ['open', 'open']) == 'ACTIVE'
    assert sample(1.2, ['unknown', 'open']) == 'ACTIVE'
    assert sample(1.35, ['unknown', 'open']) == 'ACTIVE'
    assert sample(1.41, ['unknown', 'open']) == 'PAUSED'
    assert sample(1.45, ['open', 'open']) == 'ACTIVE'
    # Processing/arrival time is not an input to the independent reducer.
    assert sample(100., ['open', 'open']) == 'ACTIVE'
