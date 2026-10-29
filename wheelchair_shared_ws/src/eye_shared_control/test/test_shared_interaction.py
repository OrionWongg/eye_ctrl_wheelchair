"""Replay the same detector result through both real output adapters."""
import json
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'projects/EYETLE/tools'))
sys.path.insert(0, str(ROOT / 'projects/Ble2WChair/src/whill_controller'))
from eye_interaction import EyeInteraction
from eye_shared_control.confirm import ConfirmMachine, ConfirmConfig
from eye_shared_control.fusion import FusionInput, parse_sample
from whill_controller.eye_drive import EyeDrive


@pytest.mark.parametrize('window', [.2, .35])
def test_offline_navigation_and_ipad_share_filtered_actions(window):
    local = EyeInteraction(window)
    machine = ConfirmMachine(ConfirmConfig(enter=.55, leave=.25))
    navigation = FusionInput(machine)
    ipad = EyeDrive()
    now = 10.
    sequence = 0
    def sample(eyes=('open', 'open'), h=0., event=None, grace=False):
        nonlocal now, sequence
        now += .05
        sequence += 1
        valid = list(eyes) == ['open', 'open'] or grace
        local.update(list(eyes), valid, grace, event, h, now)
        d = dict(schema=1, session='detector', sequence=sequence, stamp_ns=int(now*1e9),
                 source_t=now, horizontal=h, fresh=True, calibrated=True,
                 open=list(eyes)==['open', 'open'], motion_valid=valid,
                 gesture_continuity=True, long_close=False, blink_event=None,
                 eyes=list(eyes), input_grace=grace, left_blink_event=None,
                 interaction=local.snapshot())
        payload = json.dumps(d)
        machine.status(now, 'navigation', 1, True)
        navigation.receive(payload, now, now, True)
        if machine.pending:
            machine.acknowledge(now, machine.pending.event_id, True)
        machine.drain_events()
        target = ipad.update(now, True, payload, now)
        assert navigation.state == ipad.fusion.state == local.state
        expected = {'STOP': None, 'LEFT': 'a', 'FORWARD': 'w', 'RIGHT': 'd'}[local.action]
        assert target == expected
        if local.state == 'ACTIVE':
            assert navigation.turn == ipad.fusion.turn == local.direction(local.horizontal)
        return local.action
    sample()
    assert sample(event={'id': 1}) == 'FORWARD'
    for _ in range(2):
        assert sample(('closed', 'open')) == 'FORWARD'
    assert sample() == 'FORWARD'
    for _ in range(2):
        assert sample(h=-.8) == 'FORWARD'
    sample()
    for _ in range(10): sample(h=-.8)
    assert local.action == 'LEFT'
    for _ in range(10): sample(h=.8)
    assert local.action == 'RIGHT'
    for _ in range(10): sample(('closed', 'closed'))
    assert local.state == 'PAUSED'
    assert sample() == 'FORWARD'
    for _ in range(10): sample(('unknown', 'open'), grace=True)
    assert local.state == 'PAUSED'
    assert sample() == 'FORWARD'
    # Explicit transport disarm must not be undone by a held ACTIVE result.
    ipad.update(now, False)
    now += .05
    d = dict(schema=1, session='detector', sequence=sequence+1, stamp_ns=int(now*1e9),
             source_t=now, horizontal=0., fresh=True, calibrated=True, open=True,
             motion_valid=True, gesture_continuity=True, long_close=False,
             blink_event=None, interaction=local.snapshot())
    assert ipad.update(now, True, json.dumps(d), now) is None


def test_snapshot_is_independent_and_has_consistent_action():
    local = EyeInteraction()
    result = local.snapshot()
    local.start()
    local.update(['open', 'open'], True, False, None, -.8, 1.)
    assert result['state'] == 'DISARMED'
    assert local.snapshot()['action'] == 'LEFT'
