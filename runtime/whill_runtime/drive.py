"""Simulation gesture gate. No physical wheelchair interfaces."""
import math
from eye_shared_control.fusion import parse_sample


class EyeDrive:
    def __init__(self):
        self.session = None
        self.retired = set()
        self.sequence = self.event = 0
        self.blinks = []
        self.enabled = self.active = False
        self.received = -math.inf
        self.turn_until = self.gaze_after = -math.inf
        self.turn = 0.
        self.turn_ready = True
        self.reason = 'DOUBLE BLINK TO START'

    def stop(self, reason, disarm=False):
        self.active = False
        self.turn = 0.
        self.reason = reason
        if disarm:
            self.enabled = False
            self.blinks.clear()

    def receive(self, payload, now, wall_ns):
        try:
            sample = parse_sample(payload)
        except (ValueError, KeyError, TypeError):
            self.stop('INVALID SAMPLE', True)
            return
        session = sample['session']
        if session in self.retired:
            return
        if session != self.session:
            if self.session:
                self.retired.add(self.session)
            self.stop('NEW CAMERA SESSION', True)
            self.session = session
            self.sequence = self.event = 0
        if sample['sequence'] <= self.sequence:
            return
        self.sequence = sample['sequence']
        age = (wall_ns - sample['stamp_ns']) / 1e9
        if not 0 <= age < .25 or not sample['fresh'] or not sample['gesture_continuity']:
            self.stop('STALE OR LOST INPUT', True)
            return
        self.received = now - age
        if sample['long_close']:
            self.stop('LONG CLOSURE', True)
            return
        valid = sample['calibrated'] and sample['open'] and sample['motion_valid']
        if not valid:
            self.stop('PAUSED: WAIT FOR VALID EYES')
            return
        event = sample['blink_event']
        if event and event['id'] > self.event:
            self.event = event['id']
            event_time = now - age - (sample['source_t'] - event['end'])
            self.blinks = [t for t in self.blinks if .12 <= event_time - t <= .9]
            self.blinks.append(event_time)
            if self.enabled or len(self.blinks) >= 2:
                self.enabled = self.active = True
                self.blinks.clear()
                self.gaze_after = now + .5
                self.turn_ready = True
                self.reason = 'ACTIVE'
            else:
                self.reason = 'ONE BLINK: BLINK AGAIN'
        h = sample['horizontal']
        if abs(h) < .25:
            self.turn_ready = True
        if self.active and now >= self.gaze_after and now >= self.turn_until and self.turn_ready and abs(h) > .55:
            self.turn = -.5 if h > 0 else .5
            self.turn_until = now + 1.
            self.turn_ready = False

    def velocity(self, now):
        if now - self.received >= .25:
            self.stop('INPUT TIMEOUT', True)
        if not self.active:
            return 0., 0.
        return .25, self.turn if now < self.turn_until else 0.
