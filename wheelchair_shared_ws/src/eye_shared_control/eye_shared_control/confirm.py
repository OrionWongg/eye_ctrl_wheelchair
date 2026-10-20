"""Explicit candidate confirmation; pure monotonic-time state machine.

Healthy observations never issue motion. Only a completed double blink emits
an event, with a consumed frozen candidate and bounded same-event retries.
"""
from dataclasses import dataclass, replace
import math
from .core import Config


@dataclass
class ConfirmConfig(Config):
    relative_yaw: float = math.radians(15)
    candidate_ttl: float = 1.5
    event_ttl: float = 1.5
    ack_timeout: float = 1.5
    retry_interval: float = .1
    post_blink_gaze_delay: float = .5


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    candidate_version: int
    kind: int
    relative_yaw: float
    context_id: str
    context_version: int
    expires: float
    stable_at: float


@dataclass(frozen=True)
class Event:
    event_id: int
    kind: int
    candidate: object
    created: float
    ttl: float
    reason: str = ''


class ConfirmMachine:
    def __init__(self, config=None):
        self.cfg = config or ConfirmConfig()
        c = self.cfg
        if not all(math.isfinite(v) for v in vars(c).values()) or not (
                0 <= c.leave < c.enter <= 1 and 0 < c.blink_min < c.blink_max < c.long_close
                and min(c.dwell, c.double_gap, c.source_timeout, c.status_timeout,
                        c.candidate_ttl, c.event_ttl, c.ack_timeout) > 0
                and 0 <= c.post_blink_gaze_delay <= 2
                and .1 <= c.retry_interval <= c.ack_timeout
                and c.ack_timeout <= c.event_ttl <= 2 and 0 < c.relative_yaw <= math.pi / 2):
            raise ValueError('invalid confirmation configuration')
        self.context = ('', 0)
        self.status_at = self.source_at = None
        self.source_valid = self.can_accept = False
        self.source_age = 0.
        self.preview = 0
        self.preview_at = None
        self.candidate = self.frozen = None
        self.version = self.event_id = 0
        self.closed_at = self.first_blink = None
        self.was_closed = self.long_sent = False
        self.pending = None
        self.retry_at = None
        self.outbox = []
        self.measurements = []
        self.reason = 'STARTUP'
        self.fault_latched = False
        self.ack = 'NONE'
        self.task = {}
        self.last_now = None

    def record(self, name, now, **fields):
        self.measurements.append(dict(stage=name, monotonic_s=now, event_id=fields.pop('event_id', self.pending.event_id if self.pending else 0), candidate_id=fields.pop('candidate_id', self.advertised().candidate_id if self.advertised() else ''), task_id=self.task.get('task_id', ''), **fields))

    def clear_selection(self, reset_closure=True):
        self.candidate = self.frozen = None
        self.preview_at = self.first_blink = None
        if reset_closure:
            self.closed_at = None
            self.was_closed = False

    def clear_transaction(self):
        self.frozen = None
        self.first_blink = None

    def invalidate(self, reason, now=None):
        now = now if now is not None else (self.last_now or 0.)
        self.source_valid = False
        self.clear_selection()
        self.pending = None
        # Transport calls tick again before draining. Preserve its queued CANCEL
        # on repeated invalidation, while deleting every pending confirmation.
        self.outbox = [event for event in self.outbox if event.kind == 3]
        self.reason = reason
        if not self.fault_latched:
            self.event_id += 1
            self.outbox.append(Event(self.event_id, 3, None, now, self.cfg.event_ttl, reason))
            self.record('stop_recognized', now, event_id=self.event_id, reason=reason)
        self.fault_latched = True

    def status(self, now, context_id, context_version, can_accept, **task):
        if self.context != (context_id, context_version):
            self.clear_selection(reset_closure=False)
            self.pending = None
            self.outbox = [e for e in self.outbox if e.kind == 3]
        self.context = (context_id, context_version)
        self.status_at, self.can_accept = now, can_accept
        self.task = task
        if not can_accept:
            self.clear_selection(reset_closure=False)

    def healthy(self, now):
        return (self.source_valid and self.source_at is not None
                and now - self.source_at + self.source_age < self.cfg.source_timeout)

    def context_fresh(self, now):
        return (bool(self.context[0]) and self.status_at is not None
                and 0 <= now - self.status_at < self.cfg.status_timeout)

    def tick(self, now):
        if self.last_now is not None and now < self.last_now:
            self.invalidate('MONOTONIC_CLOCK_JUMP', now)
        self.last_now = now
        if self.source_at is not None and not self.healthy(now):
            self.invalidate('SOURCE_TIMEOUT', now)
        elif self.status_at is not None and not self.context_fresh(now):
            self.invalidate('TASK_STATUS_TIMEOUT', now)
        if self.was_closed and self.closed_at is not None and now - self.closed_at >= self.cfg.long_close:
            started = self.closed_at
            if not self.long_sent:
                self.invalidate('LONG_CLOSE', now)
                self.record('stop_gesture', started, event_id=self.event_id,
                            recognition_delay=now - started)
                self.long_sent = True
                self.was_closed, self.closed_at = True, started
        if self.first_blink is not None and now - self.first_blink > self.cfg.double_gap:
            self.clear_transaction()
        if self.frozen and self.frozen.expires <= now:
            self.clear_transaction()
        if self.candidate and self.candidate.expires <= now:
            self.candidate = None
        if self.pending:
            deadline = self.pending.created + min(self.cfg.ack_timeout, self.pending.ttl)
            if now >= deadline:
                self.reason, self.ack = 'ACK_TIMEOUT', 'TIMEOUT'
                self.pending = None
                self.clear_selection()
            elif now >= self.retry_at:
                self.outbox.append(self.pending)
                self.retry_at = now + self.cfg.retry_interval

    def signal(self, now, horizontal, closed, valid, source_age=0.):
        self.tick(now)
        if not all(math.isfinite(v) for v in (horizontal, source_age)) or abs(horizontal)>1 or not valid or not (
                0 <= source_age < self.cfg.source_timeout):
            self.invalidate('INVALID_SOURCE', now)
            return
        self.source_at, self.source_age, self.source_valid = now, source_age, True
        # A held long closure cannot repeatedly cancel or establish candidates.
        if self.long_sent and closed:
            return
        if not closed:
            self.long_sent = False
        self.fault_latched = False
        selectable = self.context_fresh(now) and self.can_accept
        if not selectable:
            self.clear_selection(reset_closure=False)
        if closed:
            if not self.was_closed:
                self.closed_at = now
                self.was_closed = True
                if self.first_blink is None:
                    self.frozen = self.candidate
                    self.record('blink_onset', now)
            return
        if self.was_closed:
            duration = now - self.closed_at
            self.was_closed, self.closed_at = False, None
            if not self.cfg.blink_min <= duration <= self.cfg.blink_max:
                self.clear_transaction()
            elif self.frozen is not None:
                if self.first_blink is None:
                    self.first_blink = now
                elif now - self.first_blink <= self.cfg.double_gap and self.frozen.expires > now:
                    self.submit(now)
                    return
                else:
                    self.clear_transaction()
        if self.pending or not selectable:
            return
        # Direction hysteresis applies only to preview, never to task authority.
        direction = (1 if horizontal >= self.cfg.enter else -1 if horizontal <= -self.cfg.enter
                     else self.preview if abs(horizontal) > self.cfg.leave else 0)
        if direction != self.preview or self.preview_at is None:
            self.preview, self.preview_at = direction, now
            self.record('preview', now, direction=direction)
        if now - self.preview_at >= self.cfg.dwell:
            yaw = -self.preview * self.cfg.relative_yaw
            kind = 1 if not self.preview else 2
            if self.candidate is None or (self.candidate.kind, self.candidate.relative_yaw) != (kind, yaw):
                self.clear_transaction()  # Stable reselection cancels an old blink transaction.
                self.version += 1
                self.candidate = Candidate(str(self.version), self.version, kind, yaw,
                                           *self.context, now + self.cfg.candidate_ttl, now)
                self.record('stable', now, candidate_id=self.candidate.candidate_id)
            elif self.frozen is None:
                self.candidate = replace(self.candidate, expires=now + self.cfg.candidate_ttl)

    def submit(self, now):
        c = self.frozen
        if c is None or c != self.candidate or c.expires <= now:
            self.clear_transaction()
            return
        self.event_id += 1
        event = Event(self.event_id, c.kind, c, now,
                      min(self.cfg.event_ttl, c.expires - now))
        self.pending = event
        self.retry_at = now + self.cfg.retry_interval
        self.outbox.append(event)
        self.ack, self.reason = 'WAITING', 'CONFIRMED'
        self.record('confirmation', now, event_id=event.event_id,
                    stable_to_confirmation=now - c.stable_at)
        self.clear_selection()  # Consume immediately; advertised candidate comes from pending.

    def acknowledge(self, now, event_id, accepted, reason=''):
        if not self.pending or self.pending.event_id != event_id:
            return False
        self.record('ack', now, event_id=event_id,
                    confirmation_to_ack=now - self.pending.created, accepted=accepted)
        self.pending = None
        self.outbox = [e for e in self.outbox if e.event_id != event_id]
        self.ack = 'ACCEPTED' if accepted else 'REJECTED'
        self.reason = reason
        self.clear_selection()
        return True

    def advertised(self):
        return self.pending.candidate if self.pending else self.candidate

    def drain_events(self):
        events, self.outbox = self.outbox, []
        return events

    def drain_measurements(self):
        events, self.measurements = self.measurements, []
        return events
