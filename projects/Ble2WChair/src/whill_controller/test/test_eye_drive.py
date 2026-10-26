"""Exercise real gesture semantics and calibrated serial transactions offline."""
import json
from whill_controller.eye_drive import EyeDrive
from eye_shared_control.confirm import ConfirmConfig, ConfirmMachine
from eye_shared_control.fusion import FusionInput


class Feed:
    def __init__(self):
        self.drive = EyeDrive()
        self.t = 10.
        self.seq = 0
        self.target = None

    def sample(self, **changes):
        self.t += .05
        self.seq += 1
        data = dict(schema=1, session='test', sequence=self.seq,
                    stamp_ns=int(self.t * 1e9), source_t=self.t, horizontal=0.,
                    fresh=True, calibrated=True, open=True, motion_valid=True,
                    gesture_continuity=True, long_close=False, blink_event=None)
        data.update(changes)
        self.target = self.drive.update(self.t, True, json.dumps(data), self.t)
        return self.target

    def blink(self, ident):
        start = self.t + .05
        for _ in range(3):
            assert self.sample(open=False, motion_valid=False) is None
        return self.sample(blink_event=dict(
            id=ident, start=start, end=self.t+.05, durations=[.15]))

    def start(self):
        for _ in range(8):
            assert self.sample() is None
        assert self.blink(1) is None
        assert self.blink(2) == 'w'


def test_start_turn_timeout_and_resume():
    f = Feed()
    f.start()
    for _ in range(11):
        f.sample()
    assert f.sample(horizontal=-.8) == 'a'
    for _ in range(22):
        f.sample(horizontal=-.8)
    assert f.target == 'w'
    assert f.drive.update(f.t + .3, True) is None
    for _ in range(8):
        assert f.sample() is None
    assert f.blink(3) == 'w'


def test_disarm_source_restart_and_malformed_input():
    f = Feed()
    f.start()
    assert f.drive.update(f.t, False) is None
    assert f.sample() is None
    assert f.blink(3) is None
    assert f.blink(4) == 'w'
    assert f.sample(session='new') is None
    assert f.drive.update(f.t, True, '{}', f.t) is None


def test_stale_replay_and_closure_stop():
    f = Feed()
    f.start()
    assert f.sample(stamp_ns=1) is None
    assert f.sample() is None
    f.blink(3)
    seq = f.seq
    for _ in range(7):
        f.sample(sequence=seq)
    assert f.target is None
    f.blink(4)
    assert f.sample(open=False, motion_valid=False) is None


def test_ipad_gestures_match_simulation_with_navigation_accepting():
    """Only the output transaction differs; compare the actual shared reducer."""
    actual = EyeDrive()
    machine = ConfirmMachine(ConfirmConfig(enter=.55, leave=.25))
    simulation = FusionInput(machine)

    class ComparedDrive:
        def update(self, now, ready, payload=None, wall=None):
            machine.status(now, 'simulation', 1, ready)
            if not ready:
                simulation.pause('IPAD_NOT_READY', now, disarm=True)
            elif payload is not None:
                simulation.receive(payload, now, wall, True)
            simulation.tick(now)
            # Model an available navigation backend, not its physical trajectory.
            if machine.pending:
                machine.acknowledge(now, machine.pending.event_id, True)
            machine.drain_events()
            machine.measurements.clear()
            target = actual.update(now, ready, payload, wall)
            for field in ('state', 'reason', 'turn', 'turn_armed', 'event_id',
                          'event_decision', 'session_enabled', 'active'):
                assert getattr(actual.fusion, field) == getattr(simulation, field), field
            expected = ({-1: 'a', 0: 'w', 1: 'd'}[simulation.turn]
                        if ready and simulation.active and machine.healthy(now) else None)
            assert target == expected
            return target

    f = Feed()
    f.drive = ComparedDrive()
    f.start()
    for _ in range(12):
        f.sample()
    for direction in (-.8, 0., .8, 0.):
        for _ in range(25):
            f.sample(horizontal=direction)
    assert f.sample(open=False, motion_valid=False) is None
    f.blink(3)
    assert f.sample(motion_valid=False) is None
    f.blink(4)
    f.t += .3
    assert f.drive.update(f.t, True) is None
    f.sample()
    f.blink(5)
    assert f.sample(long_close=True) is None
    assert f.sample(session='restarted') is None
    assert f.drive.update(f.t, False) is None
