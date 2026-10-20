"""Arrival-time eye joystick state machine; no ROS or detector dependency."""
from dataclasses import dataclass
from enum import IntEnum, Enum
import math
from .resume import ResumeGesture

class Command(IntEnum):
    STOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3

class State(str, Enum):
    DISARMED = 'DISARMED'
    ACTIVE = 'ACTIVE'
    PAUSED = 'PAUSED'
    BLOCKED = 'BLOCKED'
    INVALID = 'INVALID'

@dataclass
class Config:
    dwell: float = .250
    enter: float = .25
    leave: float = .15
    long_close: float = .650
    blink_min: float = .060
    blink_max: float = .400
    double_gap: float = .450
    source_timeout: float = .250
    status_timeout: float = .350

@dataclass(frozen=True)
class Intent:
    command: Command
    enabled: bool
    valid: bool
    reason: str
    state: State

class EyeMachine:
    def __init__(self, config=None):
        self.cfg = config or Config()
        c = self.cfg
        if not all(math.isfinite(v) for v in vars(c).values()):
            raise ValueError('eye parameters must be finite')
        if not (0 <= c.leave < c.enter <= 1 and 0 < c.blink_min < c.blink_max < c.long_close
                and min(c.dwell, c.double_gap, c.source_timeout, c.status_timeout) > 0):
            raise ValueError('invalid eye state machine configuration')
        self.session_enabled = False
        self.resume = ResumeGesture(c.blink_min, c.blink_max, c.source_timeout)
        self.state = State.DISARMED
        self.reason = 'STARTUP'
        self.command = Command.STOP
        self.source_at = self.status_at = None
        self.source_valid = False
        self.session = None
        self.retired_sessions = set()
        self.status_sequence = -1
        self.requires_rearm = True
        self.ready = False
        self.closed_at = self.first_blink = None
        self.was_closed = False
        self.active_at = None
        self.candidate = None
        self.candidate_at = None
        self.direction = Command.FORWARD

    def stop(self, state, reason):
        self.state, self.reason, self.command = state, reason, Command.STOP
        self.resume.reset()
        self.active_at = self.first_blink = None
        self.candidate = self.candidate_at = None

    def invalidate(self, reason):
        self.source_valid = False
        self.stop(State.INVALID, reason)
        self.closed_at = None
        self.was_closed = False

    def status(self, now, session, sequence, requires_rearm, ready, reason=''):
        if not session or session in self.retired_sessions:
            return
        if session == self.session and sequence <= self.status_sequence:
            return
        self.tick(now)
        changed = session != self.session
        if changed and self.session is not None:
            self.retired_sessions.add(self.session)
        self.session, self.status_sequence, self.status_at = session, sequence, now
        was_required = self.requires_rearm
        self.requires_rearm, self.ready = requires_rearm, (not requires_rearm or ready)
        if changed:
            self.stop(State.BLOCKED, 'SAFETY_SESSION_CHANGED')
        elif requires_rearm and (not was_required or not ready):
            self.stop(State.BLOCKED, reason or 'SAFETY_REARM_REQUIRED')
        # requires=true, ready=true is the acknowledgement of disabled STOP.
        # A new double blink may now authorize motion; repeated acknowledgements
        # must not erase it before the safety node observes the new intent.

    def tick(self, now):
        if self.source_at is None or now - self.source_at > self.cfg.source_timeout:
            self.invalidate('EYE_SOURCE_TIMEOUT')
        elif self.status_at is None or now - self.status_at > self.cfg.status_timeout:
            self.ready = False
            self.stop(State.BLOCKED, 'SAFETY_STATUS_TIMEOUT')
        elif self.was_closed and self.closed_at is not None and now - self.closed_at >= self.cfg.long_close:
            self.stop(State.PAUSED if self.session_enabled else State.DISARMED, 'LONG_EYE_CLOSE')
        enabled = self.state == State.ACTIVE and self.source_valid
        return Intent(self.command if enabled else Command.STOP, enabled,
                      self.source_valid, self.reason, self.state)

    def signal(self, now, horizontal, closed, valid):
        # Expire old source/status before a new arrival can make it fresh again.
        self.tick(now)
        self.source_at = now
        if not valid or not math.isfinite(horizontal) or abs(horizontal) > 1:
            self.invalidate('EYE_INVALID')
            return self.tick(now)
        self.source_valid = True
        if self.state in (State.INVALID, State.BLOCKED) and self.ready:
            self.stop(State.PAUSED if self.session_enabled else State.DISARMED, 'CONFIRM_REQUIRED')
        resumed = False
        if self.state == State.PAUSED:
            permitted = self.ready and self.status_at is not None and now-self.status_at <= self.cfg.status_timeout
            resumed = self.resume.update(now, horizontal, closed, permitted)
        if closed:
            if not self.was_closed:
                self.closed_at = now
                self.candidate = self.candidate_at = None
            self.was_closed = True
            return self.tick(now)
        if self.was_closed:
            duration = now - self.closed_at
            self.was_closed, self.closed_at = False, None
            if self.cfg.blink_min <= duration <= self.cfg.blink_max:
                if self.first_blink is not None and now - self.first_blink <= self.cfg.double_gap:
                    if not self.session_enabled and self.ready and self.status_at is not None and now - self.status_at <= self.cfg.status_timeout:
                        self.session_enabled = True
                        self.resume.reset()
                        self.state, self.reason = State.ACTIVE, 'DOUBLE_BLINK_CONFIRMED'
                        self.active_at = now
                        # Even on authorization, a direction must satisfy dwell.
                        self.command = Command.STOP
                        self.direction = Command.FORWARD
                    self.first_blink = None
                else:
                    self.first_blink = now
            else:
                self.first_blink = None
        if resumed:
            self.state, self.reason = State.ACTIVE, 'SINGLE_BLINK_RESUME_CONFIRMED'
            self.active_at = now
            self.direction = self.command = Command.FORWARD
            self.candidate = Command.FORWARD; self.candidate_at = now
            self.first_blink = None
            self.resume.reset()
        if self.state == State.ACTIVE:
            target = self._direction(horizontal)
            if target != self.candidate:
                self.candidate, self.candidate_at = target, now
            elif now - self.candidate_at >= self.cfg.dwell:
                self.command = self.direction = target
                self.reason = 'USER_GAZE'
        return self.tick(now)

    def _direction(self, h):
        if h <= -self.cfg.enter:
            return Command.LEFT
        if h >= self.cfg.enter:
            return Command.RIGHT
        if self.direction == Command.LEFT and h < -self.cfg.leave:
            return Command.LEFT
        if self.direction == Command.RIGHT and h > self.cfg.leave:
            return Command.RIGHT
        return Command.FORWARD
