"""Translate the shared atomic eye protocol into calibrated iPad targets."""
from eye_shared_control.confirm import ConfirmConfig, ConfirmMachine
from eye_shared_control.fusion import FusionInput


class DirectFusion(FusionInput):
    """Use gesture state directly, without fabricating navigation acceptance."""

    def emit(self, now, direction):
        """Expose active/turn state without creating a navigation transaction."""


class EyeDrive:
    """Keep existing blink, freshness, replay and one-second turn semantics."""

    def __init__(self):
        self.machine = ConfirmMachine(ConfirmConfig(enter=.55, leave=.25))
        self.fusion = DirectFusion(self.machine)
        self.received_at = None
        self.received_age = None
        self.receive_gap = None

    def update(self, now, ready, payload=None, wall=None, blocked_reason='IPAD_NOT_READY'):
        """Return a keyboard target or None (release); time is monotonic."""
        self.machine.status(now, 'ipad', 1, ready)
        if not ready:
            self.fusion.pause(blocked_reason, now, disarm=True)
        elif payload is not None:
            accepted, age = self.fusion.receive(payload, now, wall, True)
            if accepted:
                self.receive_gap = None if self.received_at is None else now-self.received_at
                self.received_at, self.received_age = now, age
        self.fusion.tick(now)
        self.machine.drain_events()
        self.machine.measurements.clear()
        if not ready or not self.fusion.active or not self.machine.healthy(now):
            return None
        return {-1: 'a', 0: 'w', 1: 'd'}[self.fusion.turn]

    def timing(self, now):
        """Transport diagnostics only; never alter shared gesture decisions."""
        return dict(
            sample_sequence=self.fusion.sequence,
            receive_age_ms=None if self.received_age is None else self.received_age*1000.,
            receive_gap_ms=None if self.receive_gap is None else self.receive_gap*1000.,
            receive_silence_ms=None if self.received_at is None else (now-self.received_at)*1000.,
            source_age_ms=None if self.machine.source_at is None else
                (now-self.machine.source_at+self.machine.source_age)*1000.)
