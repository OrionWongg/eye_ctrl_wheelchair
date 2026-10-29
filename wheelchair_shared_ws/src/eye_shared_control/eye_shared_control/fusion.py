"""Atomic EYETLE schema 1 input, with completed blink events (no blink inference)."""
import json
import math


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def parse_sample(payload):
    d = json.loads(payload)
    if not isinstance(d, dict) or type(d.get('schema')) is not int or d['schema'] != 1:
        raise ValueError('schema')
    if not isinstance(d['session'], str) or not 0 < len(d['session']) <= 128:
        raise ValueError('session')
    if type(d['sequence']) is not int or d['sequence'] < 1 or type(d['stamp_ns']) is not int or d['stamp_ns'] <= 0:
        raise ValueError('sequence/stamp')
    if not number(d['source_t']) or not number(d['horizontal']) or abs(d['horizontal']) > 1:
        raise ValueError('coordinates')
    for key in ('fresh', 'calibrated', 'open', 'motion_valid', 'gesture_continuity', 'long_close'):
        if type(d[key]) is not bool:
            raise ValueError(key)
    if 'eyes' in d:
        if (not isinstance(d['eyes'], list) or len(d['eyes']) not in (1, 2)
                or any(s not in ('open', 'closed', 'unknown') for s in d['eyes'])
                or type(d.get('input_grace')) is not bool):
            raise ValueError('eye states')
    for e in (d['blink_event'], d.get('left_blink_event')):
        if e is None:
            continue
        if not isinstance(e, dict) or type(e['id']) is not int or e['id'] < 1:
            raise ValueError('event id')
        durations = e['durations']
        if (not number(e['start']) or not number(e['end']) or not 0 < e['end'] - e['start'] <= .58
                or not 0 <= d['source_t'] - e['end'] <= .25
                or not isinstance(durations, list) or len(durations) not in (1, 2)
                or not all(number(v) and .06 <= v <= .4 for v in durations)):
            raise ValueError('event timing')
    if 'interaction' in d:
        result = d['interaction']
        if (not isinstance(result, dict) or result.get('state') not in ('DISARMED', 'ACTIVE', 'PAUSED')
                or not isinstance(result.get('reason'), str)
                or not number(result.get('horizontal')) or abs(result['horizontal']) > 1
                or type(result.get('event_id')) is not int or result['event_id'] < 0):
            raise ValueError('interaction result')
        actions = ('LEFT', 'RIGHT', 'FORWARD') if result['state'] == 'ACTIVE' else ('STOP',)
        if result.get('action') not in actions:
            raise ValueError('interaction action')
    return d


class FusionInput:
    """Per-eye packets arm on a left blink and resume when both eyes reopen.

    Legacy packets retain double-blink arming. Gaze edges select direction while
    active. Direction persists while gaze remains past the enter threshold;
    the wheelchair planner owns the physical turn and its duration.
    """
    def __init__(self, machine):
        self.machine = machine
        self.session = None
        self.retired = set()
        self.sequence = self.event_id = 0
        self.session_enabled = self.active = False
        self.blinks = []
        self.authorization = None
        self.barrier = -float('inf')
        self.horizontal = 0.
        self.horizontal_at = -math.inf
        self.gaze_after = -math.inf
        self.turn = 0
        self.turn_armed = True
        self.event_decision = 'NONE'
        self.reason = 'STARTUP'
        self.barrier_reason = 'STARTUP'
        self.decision_details = {}
        self.signal_timeout = .4
        self.health_valid = False
        self.filtered_result = None
        self.filtered_disarmed = False
        self.filtered_event_floor = 0

    def tracking_healthy(self, now):
        m = self.machine
        return (self.health_valid and m.source_at is not None
                and 0 <= now-m.source_at+m.source_age < m.cfg.source_timeout)

    @property
    def state(self):
        return 'ACTIVE' if self.active else 'PAUSED' if self.session_enabled else 'DISARMED'

    def pause(self, reason, now, keep_events=False, barrier=None, disarm=False):
        self.active = False
        self.health_valid = False
        self.turn = 0
        self.turn_armed = True
        if disarm:
            self.session_enabled = False
            self.filtered_disarmed = reason != 'SOURCE_CHANGED'
            self.filtered_event_floor = self.event_id
            self.filtered_result = None
        if not keep_events:
            self.blinks.clear()
            self.authorization = None
            new_barrier = now if barrier is None else barrier
            if new_barrier >= self.barrier:
                self.barrier, self.barrier_reason = new_barrier, reason
        self.reason = reason
        self.machine.invalidate(reason, now)

    def emit(self, now, direction):
        from .confirm import Candidate
        m = self.machine
        m.version += 1
        m.candidate = m.frozen = Candidate(str(m.version), m.version,
            1 if direction == 0 else 2, -direction*m.cfg.relative_yaw,
            *m.context, now+m.cfg.candidate_ttl, now)
        m.submit(now)
        m.record('gaze_action', now, direction=direction)

    def tick(self, now):
        m = self.machine
        m.tick(now)
        self.blinks = [t for t in self.blinks if 0 <= now-t <= .9]
        if self.authorization and now-self.authorization[0] > .25:
            self.authorization = None
            self.blinks.clear()
            self.event_decision = 'AUTHORIZATION_EXPIRED'
        if not m.healthy(now):
            # Motion still stops at 250 ms frame age. Processing latency must
            # not move the gesture barrier past a newly acquired blink. As in
            # the turtle controller, actual arrival silence has a separate
            # 400 ms watchdog; source continuity remains independently gated.
            if m.source_at is not None and now-m.source_at+m.source_age >= m.cfg.source_timeout:
                if now-m.source_at > self.signal_timeout:
                    self.pause('SIGNAL_TIMEOUT', now)
                else:
                    self.pause('FRAME_AGE_TIMEOUT', now, keep_events=True)
            return
        if not m.context_fresh(now) or not m.can_accept:
            if self.active:
                self.pause('NAVIGATION_NOT_READY', now)
            return
        if self.active and m.ack in ('REJECTED', 'TIMEOUT'):
            self.pause('NAVIGATION_'+m.ack, now)
            return
        if self.filtered_result is not None:
            # EYETLE has already run the shared interaction reducer. Do not
            # infer blinks or apply a second dwell/hysteresis/post-blink delay.
            result = self.filtered_result
            if self.authorization and self.authorization[1] == 'START_BUTTON':
                self.filtered_disarmed = False
                self.authorization = None
            if result['state'] != 'ACTIVE' or self.filtered_disarmed:
                return
            direction = {'LEFT': -1, 'FORWARD': 0, 'RIGHT': 1}[result['action']]
            if not self.active or (direction != self.turn and m.pending is None):
                self.session_enabled = self.active = True
                self.turn = direction
                self.emit(now, direction)
            self.reason = result['reason']
            return
        if self.authorization and not self.active:
            self.session_enabled = self.active = True
            self.reason = self.authorization[1]
            self.authorization = None
            self.blinks.clear()
            self.turn = 0
            self.turn_armed = True
            self.gaze_after = now + m.cfg.post_blink_gaze_delay
            self.emit(now, 0)
        if not self.active:
            return
        # Use acquisition time: a delayed pre-deadline sample must not turn
        # into a steering command when a timer crosses the settling deadline.
        # This only gates gaze steering; STOP and health checks above still run.
        if self.horizontal_at < self.gaze_after:
            return
        if abs(self.horizontal) < m.cfg.leave:
            self.turn_armed = True
            if self.turn:
                self.turn = 0
                self.emit(now, 0)
        elif self.turn_armed and m.pending is None:
            direction = -1 if self.horizontal < -m.cfg.enter else 1 if self.horizontal > m.cfg.enter else 0
            if direction:
                if self.turn != direction:
                    self.turn, self.turn_armed = direction, False
                    self.emit(now, direction)

    def receive(self, payload, now, wall, clock_valid):
        m = self.machine
        try:
            d = parse_sample(payload)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.pause('INVALID_FUSION_SAMPLE', now)
            return False, 0.
        if d['session'] in self.retired:
            return False, 0.
        if d['session'] != self.session:
            if self.session is not None:
                self.retired.add(self.session)
            self.pause('SOURCE_CHANGED', now, disarm=True)
            self.session = d['session']
            self.sequence = self.event_id = 0
        if d['sequence'] <= self.sequence:
            return False, 0.
        self.sequence = d['sequence']
        if 'interaction' not in d:
            self.filtered_result = None
        self.tick(now)
        age = wall-d['stamp_ns']/1e9
        if (not clock_valid or not -.05 <= age < m.cfg.source_timeout
                or not d['fresh'] or not d['calibrated']):
            self.pause('STALE_OR_INVALID_FUSION', now,
                       barrier=now-max(0., age) if clock_valid else now)
            return True, age
        stamp = now-max(0., age)
        if 'interaction' in d:
            self.receive_interaction(d['interaction'], now, stamp, max(0., age))
            return True, age
        binocular = 'eyes' in d
        if not binocular and not d['gesture_continuity']:
            self.pause('GESTURE_TRACKING_LOST', now, barrier=stamp)
        if d['long_close']:
            self.pause('LONG_EYE_CLOSE', now)
            return True, age
        both_open = binocular and d['eyes'] == ['open', 'open']
        grace = (binocular and d['input_grace'] and self.active
                 and 'closed' not in d['eyes'])
        ready = (d['motion_valid'] and (both_open or grace) if binocular else
                 d['motion_valid'] and d['open'] and d['gesture_continuity'])
        if not grace:
            self.horizontal = d['horizontal']
        self.horizontal_at = stamp
        if not ready:
            self.pause('INVALID_INPUT', now, keep_events=True)
        # Fresh closed samples refresh reception, not permission to move.
        m.source_at, m.source_age, m.source_valid = now, max(0., age), ready
        # A positively tracked short closure is a STOP gesture, not a broken
        # input device. STOP still revokes motion, without a redundant health
        # fault racing the resume handshake.
        self.health_valid = ready or (not d['open'] and d['gesture_continuity'])
        if ready:
            m.fault_latched = False
        # New detector packets carry per-eye observations. Legacy publishers
        # retain their original paired-blink protocol.
        e = d.get('left_blink_event') if binocular else d['blink_event']
        if e is not None and e['id'] > self.event_id:
            self.event_id = e['id']
            start = stamp-(d['source_t']-e['start'])
            end = stamp-(d['source_t']-e['end'])
            if not binocular and not d['gesture_continuity']:
                self.event_decision = 'REJECT_CONTINUITY'
            elif start < self.barrier:
                self.event_decision = 'REJECT_BEFORE_STOP'
            elif not 0 <= now-end <= .25:
                self.event_decision = 'REJECT_EVENT_AGE'
            elif self.blinks and end-self.blinks[-1] < .12:
                self.event_decision = 'REJECT_COOLDOWN'
            else:
                self.event_decision = 'ACCEPTED'
                self.gaze_after = now + m.cfg.post_blink_gaze_delay
                self.blinks.append(end)
                if not self.session_enabled and (binocular or len(self.blinks) >= 2):
                    self.authorization = (end, 'LEFT_BLINK_CONFIRMED' if binocular else 'DOUBLE_BLINK_CONFIRMED')
                elif self.session_enabled and not self.active:
                    self.authorization = (end, 'SINGLE_BLINK_RESUME_CONFIRMED')
            self.decision_details = dict(event_start=start, event_end=end,
                event_age_ms=(now-end)*1000., barrier=self.barrier,
                barrier_reason=self.barrier_reason, sample_age_ms=age*1000.,
                motion_ready=ready, gesture_continuity=d['gesture_continuity'])
        if binocular and self.session_enabled and not self.active and ready and both_open:
            self.authorization = (now, 'BOTH_EYES_OPEN')
        self.tick(now)
        return True, age

    def receive_interaction(self, result, now, stamp, age):
        """Translate the detector's shared result; only transport gates live here."""
        m = self.machine
        self.filtered_result = result
        self.event_id = result['event_id']
        if self.filtered_disarmed and self.event_id > self.filtered_event_floor:
            self.filtered_disarmed = False
        ready = result['state'] == 'ACTIVE' and not self.filtered_disarmed
        if not ready:
            self.pause(result['reason'], now, keep_events=True)
            self.session_enabled = result['state'] != 'DISARMED' and not self.filtered_disarmed
        self.horizontal = result['horizontal']
        self.horizontal_at = stamp
        m.source_at, m.source_age, m.source_valid = now, age, ready
        self.health_valid = ready
        if ready:
            m.fault_latched = False
        self.event_decision = 'SHARED_INTERACTION'
        self.tick(now)
