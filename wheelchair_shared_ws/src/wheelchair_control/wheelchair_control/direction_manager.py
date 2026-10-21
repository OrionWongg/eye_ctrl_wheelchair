"""Event-only navigation authorization; legacy candidate rules are not involved."""
from dataclasses import replace
from .task_manager import (TaskManager, Task, Event, Ack, Output, IDLE, RUNNING,
                           STOPPING, BLOCKED, FAULT)

START, PREFER_LEFT, PREFER_RIGHT, CANCEL_PREFERENCE, STOP_EVENT = range(1, 6)
TURN_AROUND = 6
NAV_CRUISE, NAV_LEFT, NAV_RIGHT = 3, 4, 5
NAV_UTURN = 6
NAV_TURN_LEFT, NAV_TURN_RIGHT, NAV_TURN_AROUND = 7, 8, 9


class DirectionManager(TaskManager):
    def __init__(self, cfg=None, session=None):
        super().__init__(cfg, session)
        self.consumed_preference = False
        self.event_origin = 0.
        self.restart_after = float('-inf')

    def stop(self, reason, now, terminal=BLOCKED, invalidate_context=True, event=None):
        super().stop(reason, now, terminal, invalidate_context, event)
        self.consumed_preference = False
        self.restart_after = now

    def on_lower(self, data, now):
        super().on_lower(data, now)
        if self.active and data.get('task_id') == self.active.task_id:
            if self.active.turn_only and data.get('reason')=='NAV_TURN_COMPLETED':
                self.stop('NAV_TURN_COMPLETED',now,IDLE)
                return
            if data.get('reason') in ('NAV_CRUISE', 'NAV_SEARCH_EXPIRED'):
                self.consumed_preference = True

    def on_event(self, event, now, ros_now):
        key = (event.session, event.event_id)
        prior = self.events.get(key)
        if prior is not None:
            if prior[0] != event:
                ack = Ack(event.session, event.event_id, False, '', 'EVENT_ID_REUSED')
                self.acks.append(ack)
                return ack
            if prior[1] is not None:
                self.acks.append(prior[1])
            return prior[1]
        if (event.protocol_version != 1 or not event.session or len(event.session) > 128
                or event.session in self.retired or event.event_id < 0
                or not self.valid_time(event.stamp, event.ttl, ros_now, self.cfg.max_event_ttl)):
            return self._ack(event, False, 'INVALID_OR_EXPIRED_EVENT')
        if event.event_id <= self.high_event.get(event.session, -1):
            return self._ack(event, False, 'OUT_OF_ORDER_EVENT')
        self.high_event[event.session] = event.event_id
        if event.kind == STOP_EVENT:
            self.stop('USER_STOP', now, IDLE, event=event)
            return self._ack(event, True, 'STOPPED')
        if event.session != self.source_session or not self.healthy(now) or not self.links_healthy(now):
            return self._ack(event, False, 'NOT_HEALTHY')
        if event.kind not in (START, PREFER_LEFT, PREFER_RIGHT, CANCEL_PREFERENCE, TURN_AROUND):
            return self._ack(event, False, 'INVALID_DIRECTION_KIND')
        # A delayed pre-stop motion event cannot restore a cancelled authorization.
        origin = now - max(0., ros_now-event.stamp)
        current_turn_only=(self.active.turn_only if self.active else
                           self.pending.get('turn_only',False) if self.pending else True)
        turn_only=current_turn_only and event.kind in (PREFER_LEFT,PREFER_RIGHT,TURN_AROUND)
        if self.active is None and self.pending is None:
            if event.kind not in (START,PREFER_LEFT,PREFER_RIGHT,TURN_AROUND):
                return self._ack(event, False, 'START_REQUIRED')
            if origin <= self.restart_after:
                return self._ack(event, False, 'NEW_MOTION_EVENT_REQUIRED')
        current = self.pending['event'] if self.pending else self.active.event if self.active else None
        if current and ((event.kind == START and not current_turn_only) or
                        (event.kind == current.kind and not self.consumed_preference)):
            return self._ack(event, True, 'UNCHANGED',
                             self.pending['task_id'] if self.pending else self.active.task_id)
        task_id = self.session+':'+event.session+':'+str(event.event_id)
        # First preference during cruise can be interpreted while approaching.
        # Replacements/cancellation must first finish the old physical motion.
        immediate = (self.active is not None and event.kind in (PREFER_LEFT, PREFER_RIGHT)
                     and (self.active.event.kind == START or self.consumed_preference))
        if immediate:
            self.active = Task(task_id, event, origin, self.pose, self.pose[2])
            self.event_origin = origin
            self.consumed_preference = False
            return self._ack(event, True, 'ACCEPTED', task_id)
        self.stop('REPLACING' if current else 'STARTING', now, IDLE)
        self.pending = dict(event=event, task_id=task_id, reference=self.pose,
                            deadline=origin+event.ttl, origin=origin, stop_issued_at=now,turn_only=turn_only)
        self.events[key] = (event, None)
        self.state = STOPPING
        return None

    def tick(self, now, ros_now):
        if not self.healthy(now):
            if self.active or self.pending:
                self.invalidate('INPUT_HEALTH_TIMEOUT', now)
        elif (self.active or self.pending) and not self.links_healthy(now):
            self.invalidate('LOWER_HEALTH_TIMEOUT_OR_FAULT', now)
        if self.active and self.lower.get('task_id') == self.active.task_id and self.lower.get('requires_rearm'):
            self.stop(self.lower.get('reason', 'NAV_BLOCKED'), now)
        if self.pending:
            p = self.pending
            if now >= p['deadline']:
                self.stop('EVENT_EXPIRED_WAITING_STOP', now)
            elif (self.stopped() and now-p['stop_issued_at'] >= self.cfg.stop_delay+self.cfg.stop_dwell
                  and min(self.lower_at, self.execution_at, self.pose_at) > p['stop_issued_at']+self.cfg.stop_delay
                  and (self.lower.get('ready_to_rearm') or not self.lower.get('requires_rearm', True))):
                e = p['event']
                self.active = Task(p['task_id'], e, p['origin'], self.pose, self.pose[2],turn_only=p.get('turn_only',False))
                self.event_origin = p['origin']
                self.pending = None
                self.consumed_preference = False
                self._ack(e, True, 'ACCEPTED', self.active.task_id)
        if self.active is None:
            if self.pending is None and self.stopped():
                self.state = self.terminal
            return Output(reason=self.reason)
        self.state = RUNNING
        t = self.active
        kind = {PREFER_LEFT: NAV_LEFT, PREFER_RIGHT: NAV_RIGHT, TURN_AROUND: NAV_UTURN}.get(t.event.kind, NAV_CRUISE)
        if t.turn_only:kind={PREFER_LEFT:NAV_TURN_LEFT,PREFER_RIGHT:NAV_TURN_RIGHT,TURN_AROUND:NAV_TURN_AROUND}[t.event.kind]
        return Output(1, True, t.task_id, kind, t.reference, t.target_yaw, 'DIRECTIONAL_INTENT')
