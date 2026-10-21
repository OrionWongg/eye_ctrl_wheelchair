"""Finite, immutable authorization layered over the existing safety predictor.

The manager owns elapsed time and accumulated distance. This layer owns the
fixed geometric reference and the heading endpoint; input renewals cannot reset
any sub-action or expand its authority.
"""
import math
from .geometry import wrap
from .protocol import Packet, STOP, FORWARD, LEFT, RIGHT

CONTINUE, ADJUST_HEADING = 1, 2


class TaskScope:
    def __init__(self, owner, packet):
        self.owner = owner
        self.task_id = packet.task_id
        self.kind = packet.task_kind
        self.reference = (packet.reference_x, packet.reference_y, packet.reference_yaw)
        self.target = packet.target_yaw
        self.command = packet.command
        self.phase = 'WAIT_STOPPED'
        self.stopped_since = None
        self.brake_at = None
        self.motion_at = None
        self.started = None
        self.pulse = False
        self.pulse_duration = owner.cfg.micro_turn_min
        self.pulse_start_speed = 0.
        self.force_micro = False
        self.pulse_origin = None
        self.stagnant_pulses = 0
        self.signature = self.fields(packet)
        self.open_correction = None
        self.open_correction_packet = None

    @staticmethod
    def fields(packet):
        return (packet.task_id, packet.task_kind, packet.command, packet.reference_x,
                packet.reference_y, packet.reference_yaw, packet.target_yaw)

    def tick(self, packet, state, obstacles, boundary, now, stopped):
        from .planner import Decision
        p = self.owner
        c = p.cfg
        if (self.signature != self.fields(packet) or not self.task_id or
                self.kind not in (CONTINUE, ADJUST_HEADING) or
                not all(math.isfinite(x) for x in (*self.reference, self.target))):
            return p.trip('INVALID_TASK_SCOPE')
        expected = (FORWARD,) if self.kind == CONTINUE else (LEFT, RIGHT)
        if self.command not in expected:
            return p.trip('INVALID_TASK_COMMAND')
        p.last_diagnostics['task_id'] = self.task_id
        p.last_diagnostics['task_reference'] = self.reference
        if self.kind == CONTINUE:
            return self.continue_tick(state, obstacles, boundary, now, stopped)
        signed = wrap(self.target-self.reference[2])
        direction = 1 if self.command == LEFT else -1
        if signed*direction <= 0 or abs(signed) > c.task_heading_limit+1e-9:
            return p.trip('INVALID_TASK_HEADING')
        if not self.within_heading_scope(state.yaw):
            return p.trip('TASK_HEADING_SCOPE_LIMIT')
        if self.phase == 'COMPLETED':
            return Decision(STOP, 'PASS', 'TASK_HEADING_COMPLETED')
        return self.heading_tick(state, obstacles, boundary, now, stopped, direction)

    def within_heading_scope(self, yaw):
        c = self.owner.cfg
        direction = 1 if self.command == LEFT else -1
        target_delta = abs(wrap(self.target-self.reference[2]))
        progress = direction*wrap(yaw-self.reference[2])
        upper = min(c.task_heading_limit, target_delta+c.task_heading_tolerance)
        return -c.task_heading_tolerance-1e-9 <= progress <= upper+1e-9

    def heading_path_authorized(self, result):
        return bool(result.poses) and all(self.within_heading_scope(pose[2]) for pose in result.poses)

    def pulse_remaining(self, state, direction):
        # Pulses start at measured stop. With angular_accel as an upper bound,
        # gaining this much same-direction speed proves at least this much
        # execution exposure has already elapsed at the odometry sample itself.
        # Speed falling during braking only makes this remaining bound larger.
        consumed = max(0., (direction*state.w-self.pulse_start_speed)/self.owner.cfg.angular_accel)
        return max(0., self.pulse_duration-consumed)

    def within_continue_scope(self, pose):
        c = self.owner.cfg
        x, y, heading = self.reference
        lateral = -(pose[0]-x)*math.sin(heading)+(pose[1]-y)*math.cos(heading)
        return (abs(wrap(pose[2]-heading)) <= c.task_heading_limit+1e-9 and
                abs(lateral) <= c.assist_lateral+1e-9)

    def continue_tick(self, state, obstacles, boundary, now, stopped):
        from .passage import PassageController
        p = self.owner
        if not self.within_continue_scope(state.pose):
            return p.trip('TASK_DIRECTION_OR_LATERAL_LIMIT')
        if p.local is None:
            p.local = PassageController(p, task_scoped=True)
            p.local.request_heading = self.reference[2]
        # Perform initial discovery exactly once before choosing open-room
        # heading hold. Corrections preserve this controller and its empty scope.
        if p.local.discovery_done and p.local.passage is None:
            error = wrap(self.reference[2]-state.yaw)
            threshold = max(p.cfg.corridor_heading_enter, p.cfg.task_heading_tolerance)*1.5
            if self.open_correction is None and abs(error) > threshold:
                self.open_correction_packet = Packet(
                    'internal', 0, 0., .15, LEFT if error>0 else RIGHT, True,
                    task_id=self.task_id+'/heading-hold', task_kind=ADJUST_HEADING,
                    reference_x=state.x, reference_y=state.y, reference_yaw=state.yaw,
                    target_yaw=self.reference[2])
                self.open_correction = TaskScope(p, self.open_correction_packet)
                self.open_correction.force_micro = True
            if self.open_correction is not None:
                decision = self.open_correction.tick(self.open_correction_packet, state,
                                                     obstacles, boundary, now, stopped)
                # An internal correction never completes the parent CONTINUE.
                p.last_diagnostics['task_id'] = self.task_id
                p.last_diagnostics['task_reference'] = self.reference
                if decision.reason == 'TASK_HEADING_COMPLETED':
                    from .planner import Decision
                    self.open_correction = None
                    self.open_correction_packet = None
                    decision = Decision(STOP, 'ASSISTING', 'TASK_CONTINUE_HEADING_ALIGNED')
            else:
                decision = p.local.tick(state, obstacles, boundary, now, stopped)
        else:
            decision = p.local.tick(state, obstacles, boundary, now, stopped)
        if decision.action != STOP:
            # Validate the complete lease-loss and braking path against the
            # original task band, even when passage correction found a safe wall
            # clearance. Safe geometry never grants a different direction.
            candidate = p.last_diagnostics.get('candidate', {})
            poses = candidate.get('poses', ())
            if not poses or not all(self.within_continue_scope(pose) for pose in poses):
                return p.trip('TASK_PREDICTED_SCOPE_LIMIT')
        return decision

    def stopped(self, state, now, stopped):
        c = self.owner.cfg
        stationary = (stopped and abs(state.v) < .015 and abs(state.w) < .025 and
                      state.execution_phase in (None, 0))
        if not stationary:
            self.stopped_since = None
            return False
        if self.brake_at is not None and now-self.brake_at < c.transport_delay+c.stop_dwell:
            return False
        if self.stopped_since is None:
            self.stopped_since = now
        return now-self.stopped_since >= c.stop_dwell-1e-9

    def brake(self, now):
        from .planner import Decision
        self.phase = 'SETTLE'
        self.brake_at = now
        self.stopped_since = None
        return Decision(STOP, 'ASSISTING', 'TASK_HEADING_BRAKE')

    def brake_threshold(self, remaining):
        return remaining

    def heading_remaining(self, state, direction):
        return direction*wrap(self.target-state.yaw)

    def pulse_durations(self,remaining):
        return (.30,.20,.10)

    def heading_tick(self, state, obstacles, boundary, now, stopped, direction):
        from .planner import Decision
        p = self.owner
        c = p.cfg
        remaining = self.heading_remaining(state,direction)
        p.last_diagnostics.update(task_heading_target=self.target,
                                  task_heading_error=remaining, task_heading_phase=self.phase)
        stopping = p.predictor.check(state, STOP, obstacles, boundary)
        if not stopping.safe:
            return p.trip('TASK_HEADING_STOPPING_RISK')
        if self.phase in ('WAIT_STOPPED', 'SETTLE'):
            if not self.stopped(state, now, stopped):
                return Decision(STOP, 'ASSISTING', 'TASK_HEADING_WAIT_STOPPED')
            if abs(remaining) <= c.task_heading_tolerance:
                self.phase = 'COMPLETED'
                return Decision(STOP, 'PASS', 'TASK_HEADING_COMPLETED')
            if remaining < 0:
                # Never reverse direction to repair overshoot.
                return p.trip('TASK_HEADING_OVERSHOOT')
            if self.pulse_origin is not None and self.pulse:
                improvement = direction*wrap(state.yaw-self.pulse_origin)
                self.stagnant_pulses = self.stagnant_pulses+1 if improvement<c.corridor_min_improvement else 0
                p.last_diagnostics['task_heading_stagnant_pulses'] = self.stagnant_pulses
                hard_limited = abs(wrap(self.target-self.reference[2]))+c.task_heading_tolerance>c.task_heading_limit
                if improvement<1e-4:
                    return p.trip('TASK_HEADING_ACTION_UNEXECUTABLE')
                if hard_limited and self.stagnant_pulses>=2:
                    # Close to the hard angle ceiling, repeatedly truncated
                    # episodes cannot become an endless retry loop. Away from
                    # that ceiling small resolved pulses can still converge to
                    # the target; the manager's total time budget remains fixed.
                    return p.trip('TASK_HEADING_AUTHORITY_UNEXECUTABLE')
            self.phase = 'TURN'
            self.motion_at = None
            self.started = now
            self.pulse_origin = state.yaw
            self.pulse_start_speed = max(0., direction*state.w)
            full = p.predictor.check(state, self.command, obstacles, boundary)
            full_angle = direction*wrap(full.endpoint.yaw-state.yaw)
            # Small endpoint errors and constrained geometry use an independently
            # time-bounded executor episode. Large turns retain normal leases.
            self.pulse = (self.force_micro or not full.safe or not self.heading_path_authorized(full)
                          or full_angle > remaining+c.task_heading_tolerance)
            if self.pulse:
                short = None
                geometric_candidate = False
                for duration in self.pulse_durations(remaining):
                    if duration<c.micro_turn_min-1e-9 or duration>c.minimum_action+1e-9:
                        continue
                    # Only the previously validated 100/200/300 ms episodes.
                    # The full lease-loss and braking path must fit both the
                    # collision map and the original angular authorization.
                    candidate = p.predictor.check(state, self.command, obstacles, boundary,
                                                  duration=duration, max_run=duration)
                    geometric_candidate = geometric_candidate or candidate.safe
                    if candidate.safe and self.heading_path_authorized(candidate):
                        short = candidate
                        self.pulse_duration = duration
                        break
                if short is None:
                    return p.trip('TASK_HEADING_NO_AUTHORIZED_MICRO_TURN' if geometric_candidate
                                  else 'TASK_HEADING_BLOCKED')
                p.last_diagnostics['candidate'] = short.summary()
            else:
                p.last_diagnostics['candidate'] = full.summary()
        if now-self.started > c.assist_duration:
            return p.trip('TASK_HEADING_TURN_TIMEOUT')
        # Odom closes the target loop. Account for a command already in transit,
        # a planner period, stale feedback and acceleration before STOP arrives.
        feedback_age=state.feedback_age if state.feedback_age is not None else state.observation_age
        delay = c.transport_delay+c.period+(feedback_age or 0.)
        speed = max(0., direction*state.w)
        peak = min(abs(c.velocity(self.command)[1]), speed+c.angular_accel*delay)
        brake_angle = (speed+peak)*delay/2+peak*peak/(2*c.angular_brake)
        if remaining <= c.task_heading_tolerance or brake_angle >= self.brake_threshold(remaining):
            return self.brake(now)
        result = p.predictor.check(state, self.command, obstacles, boundary, duration=0.,
                                   max_run=self.pulse_remaining(state, direction) if self.pulse else None)
        p.last_diagnostics['candidate'] = result.summary()
        if not result.safe or not self.heading_path_authorized(result):
            return self.brake(now)
        running = state.execution_phase == 1 and state.current_action == self.command
        if state.execution_phase is None:
            running = abs(state.w) > .02
        if running and self.motion_at is None:
            self.motion_at = now
        if self.pulse and self.motion_at is not None and now-self.motion_at >= self.pulse_duration-1e-9:
            return self.brake(now)
        if self.pulse:
            p.last_diagnostics['task_pulse_duration'] = self.pulse_duration
            p.last_diagnostics['task_pulse_remaining'] = self.pulse_remaining(state, direction)
        return Decision(self.command, 'ASSISTING',
                        'TASK_HEADING_MICRO_TURN' if self.pulse else 'TASK_HEADING_TURN')
