"""Protocol v1 transport, kept separate from the legacy joystick publisher."""
import json
import math
import time
import uuid
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, String, Empty
from wheelchair_interfaces.msg import (CandidateStatus, IntentEvent, InputHealth,
                                      IntentAck, TaskStatus, EyeSample, DirectionEvent, NavigationIntentStatus)
from .confirm import ConfirmConfig, ConfirmMachine


class ConfirmTransport:
    def __init__(self, node, directional=False):
        self.node = node
        self.directional = directional
        cfg = ConfirmConfig()
        for key, value in vars(cfg).items():
            setattr(cfg, key, float(node.declare_parameter(key, value).value))
        node.machine = self.machine = ConfirmMachine(cfg)
        self.session = str(uuid.uuid4())
        self.sequence = self.sample_sequence = 0
        self.manager_session = None
        self.retired_managers = set()
        self.status_sequence = -1
        self.sensor_session = None
        self.retired_sensors = set()
        self.sensor_sequence = -1
        self.last_ros = None
        self.ros_changed = time.monotonic()
        self.clock_valid = False
        self.event_messages = {}
        self.last_candidate = None
        self.received_at = None
        self.received_age = 0.
        self.last_fusion_log = None
        self.sample_mode = node.declare_parameter('eye_sample_mode', 'legacy').value
        if self.sample_mode not in ('legacy', 'stamped', 'fusion'):
            raise ValueError('eye_sample_mode must be legacy, stamped or fusion')
        if self.sample_mode == 'fusion':
            from .fusion import FusionInput
            cfg.enter = float(node.declare_parameter('fusion_enter', .55).value)
            cfg.leave = float(node.declare_parameter('fusion_leave', .25).value)
            node.machine = self.machine = ConfirmMachine(cfg)
            self.fusion_input = FusionInput(self.machine)
            self.last_start_stamp = 0
            node.create_subscription(String, '/eye_control/start', self.start_button, 1)
            node.create_subscription(Empty, '/wheelchair/emergency_stop',
                lambda msg: self.fusion_input.pause('EMERGENCY_STOP', time.monotonic(), disarm=True), 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        events = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.candidate_pub = node.create_publisher(CandidateStatus, '/wheelchair/eye/candidate', qos)
        self.event_pub = node.create_publisher(DirectionEvent if directional else IntentEvent,
            '/wheelchair/eye/direction_event' if directional else '/wheelchair/eye/event', events)
        self.health_pub = node.create_publisher(InputHealth, '/wheelchair/eye/health', qos)
        self.feedback = node.create_publisher(String, '/wheelchair/eye_feedback', qos)
        self.common_feedback = node.create_publisher(String, '/eye_control/feedback', qos)
        self.latency_pub = node.create_publisher(String, '/wheelchair/eye_latency_events', events)
        if self.sample_mode == 'legacy':
            node.create_subscription(Float32MultiArray, '/eye_tracking/signals', self.legacy, sensor)
        elif self.sample_mode == 'fusion':
            node.create_subscription(String, '/eye_tracking/fusion_samples', self.fusion, qos)
        else:
            node.create_subscription(EyeSample, '/eye_tracking/samples', self.stamped, sensor)
        node.create_subscription(NavigationIntentStatus if directional else TaskStatus,
            '/wheelchair/navigation_status' if directional else '/wheelchair/task_status', self.status, qos)
        node.create_subscription(IntentAck, '/wheelchair/intent_ack', self.ack, events)
        node.timer = node.create_timer(.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def start_button(self, msg):
        now = time.monotonic()
        try:
            command = json.loads(msg.data)
            stamp = command['stamp_ns']
            if (type(stamp) is not int or stamp <= self.last_start_stamp
                    or not 0 <= time.time()-stamp/1e9 < .25
                    or command['session'] != self.fusion_input.session):
                return
        except (ValueError, KeyError, TypeError, OverflowError):
            return
        self.last_start_stamp = stamp
        f, m = self.fusion_input, self.machine
        blockers = self.start_blockers(now)
        if blockers or f.active:
            self.node.get_logger().warning('启动未执行：'+json.dumps(dict(
                blockers=blockers, active=f.active, navigation=m.task,
                source_age_ms=None if m.source_at is None else
                (now-m.source_at+m.source_age)*1000.), ensure_ascii=False))
            return
        f.authorization = (now, 'START_BUTTON')
        f.tick(now)
        self.publish()

    def start_blockers(self, now):
        m = self.machine
        return [reason for failed, reason in (
            (not self.clock_valid, 'ROS_CLOCK_UNAVAILABLE'),
            (not m.healthy(now), 'EYE_INPUT_INVALID_OR_STALE'),
            (not self.fusion_input.tracking_healthy(now), 'EYE_TRACKING_NOT_READY'),
            (not m.context_fresh(now), 'NAVIGATION_STATUS_STALE'),
            (not m.can_accept, 'NAVIGATION_NOT_READY:'+str(m.task.get('reason', '')))) if failed]

    def fusion(self, msg):
        now = time.monotonic()
        accepted, age = self.fusion_input.receive(msg.data, now, time.time(), self.clock_valid)
        if accepted:
            self.sample_sequence += 1
            self.received_at, self.received_age = now, age
        diagnostic = (self.fusion_input.state, self.fusion_input.reason,
                      self.fusion_input.event_id, self.machine.event_id, self.fusion_input.event_decision)
        if diagnostic != self.last_fusion_log:
            self.node.get_logger().info('EYE_DRIVE '+json.dumps(dict(
                state=diagnostic[0], reason=diagnostic[1], blink_id=diagnostic[2],
                source_session=self.fusion_input.session, sample_sequence=self.fusion_input.sequence,
                sample_age_ms=age*1000.,
                direction_event_id=diagnostic[3], event_decision=diagnostic[4],
                horizontal=self.fusion_input.horizontal,
                decision=self.fusion_input.decision_details)))
            self.last_fusion_log = diagnostic
        self.publish()

    def fresh(self, msg, timeout):
        ns = self.node.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        return self.clock_valid and 0 <= ns - stamp < timeout * 1e9

    def legacy(self, msg):
        now = time.monotonic()
        if len(msg.data) != 3 or not all(math.isfinite(v) for v in msg.data):
            self.machine.invalidate('MALFORMED_EYE_SIGNAL', now)
            return
        h, closed, valid = msg.data
        if closed not in (0., 1.) or valid not in (0., 1.):
            self.machine.invalidate('MALFORMED_EYE_FLAGS', now)
            return
        self.sample_sequence += 1
        self.receive(now, h, bool(closed), bool(valid), 0.)

    def stamped(self, msg):
        now = time.monotonic()
        if not msg.source_session_id or msg.source_session_id in self.retired_sensors:
            return
        if msg.source_session_id != self.sensor_session:
            if self.sensor_session is not None:
                self.retired_sensors.add(self.sensor_session)
                self.machine.invalidate('SENSOR_SESSION_CHANGED', now)
            self.sensor_session, self.sensor_sequence = msg.source_session_id, -1
        if msg.sequence <= self.sensor_sequence:
            return  # Replays do not refresh source health.
        self.sensor_sequence = msg.sequence
        # Health belongs to this transport session. A detector restart has its
        # own retired-session guard, but must not roll our health sample counter
        # backwards and make every new sample look replayed to the manager.
        self.sample_sequence += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = time.time() - stamp  # Source SYSTEM time, deliberately not simulated /clock.
        self.receive(now, msg.horizontal, msg.closed, msg.valid, age)

    def receive(self, now, h, closed, valid, age):
        self.received_at, self.received_age = now, age
        self.machine.record('received', now, sample_sequence=self.sample_sequence)
        if self.sample_mode == 'stamped' and math.isfinite(age) and age >= 0.:
            # EyeSample wall timestamps must share this host's system clock.
            # Convert the measured wall age onto this host's monotonic axis.
            self.machine.record('source_sample', now - age,
                                sample_sequence=self.sample_sequence,
                                source_time_mapping='same_host_wall_age_to_monotonic')
        if not self.clock_valid:
            self.machine.invalidate('ROS_CLOCK_UNAVAILABLE', now)
            return
        self.machine.signal(now, h, closed, valid, age)
        self.publish()  # Preview/event feedback need not wait for the 20 Hz heartbeat.

    def status(self, msg):
        if (not self.directional and msg.protocol_version != 1) or not self.fresh(msg, self.machine.cfg.status_timeout):
            return
        if not msg.manager_session_id or msg.manager_session_id in self.retired_managers:
            return
        if msg.manager_session_id != self.manager_session:
            if self.manager_session is not None:
                self.retired_managers.add(self.manager_session)
                if self.sample_mode == 'fusion':
                    self.fusion_input.pause('NAVIGATION_SESSION_CHANGED', time.monotonic())
            self.manager_session, self.status_sequence = msg.manager_session_id, -1
        if msg.sequence <= self.status_sequence:
            return
        self.status_sequence = msg.sequence
        if self.directional:
            if self.sample_mode == 'fusion' and self.machine.pending is not None:
                # Navigation publishes old STOPPED/BLOCKED context until the
                # new event is received and the stop/start handshake completes.
                # Do not cancel our own transaction from that old context.
                # ACK rejection/timeout, local input validity and clock guards
                # still revoke motion; the manager enforces lower-level faults.
                self.machine.status_at = time.monotonic()
                return
            if (self.sample_mode == 'fusion' and self.fusion_input.active
                    and msg.phase == 'BLOCKED'):
                self.fusion_input.pause('NAVIGATION_BLOCKED:'+msg.reason, time.monotonic())
            self.machine.status(time.monotonic(), msg.manager_session_id+':'+(msg.task_id or msg.phase), 1, msg.can_start,
                task_id=msg.task_id, state=msg.phase, executed_action=msg.executed_action, reason=msg.reason,
                remaining_time=msg.remaining_time, remaining_distance=msg.remaining_distance)
            return
        self.machine.status(time.monotonic(), msg.context_id, msg.context_version, msg.can_accept,
                            task_id=msg.task_id, state=msg.state, executed_action=msg.executed_action,
                            reason=msg.reason, remaining_time=msg.remaining_time,
                            remaining_distance=msg.remaining_distance)

    def ack(self, msg):
        if (msg.protocol_version == 1 and msg.source_session_id == self.session
                and msg.manager_session_id == self.manager_session
                and msg.status in (1, 2) and self.fresh(msg, self.machine.cfg.event_ttl)):
            self.machine.acknowledge(time.monotonic(), msg.event_id, msg.status == 1, msg.reason)

    def base(self, msg, stamp):
        msg.header.stamp = stamp
        msg.protocol_version = 1
        msg.source_session_id = self.session
        return msg

    def publish(self):
        now = time.monotonic()
        ros = self.node.get_clock().now()
        ns = ros.nanoseconds
        jumped = self.last_ros is not None and (ns < self.last_ros or ns - self.last_ros > 1000000000)
        if ns != self.last_ros:
            self.ros_changed = now
        self.clock_valid = ns > 0 and now - self.ros_changed <= .30 and not jumped
        self.last_ros = ns
        if not self.clock_valid:
            if self.sample_mode == 'fusion':
                self.fusion_input.pause('ROS_CLOCK_JUMP_OR_PAUSE', now)
            else:
                self.machine.invalidate('ROS_CLOCK_JUMP_OR_PAUSE', now)
        if self.sample_mode == 'fusion':
            self.fusion_input.tick(now)
        else:
            self.machine.tick(now)
        stamp = ros.to_msg()
        self.sequence += 1
        c = self.machine.advertised()
        msg = self.base(CandidateStatus(), stamp)
        msg.sequence = self.sequence
        advertised = c or self.last_candidate
        if advertised:
            for name in ('candidate_id', 'candidate_version', 'kind', 'relative_yaw',
                         'context_id', 'context_version'):
                setattr(msg, name, getattr(advertised, name))
        else:
            msg.context_id, msg.context_version = self.machine.context
        msg.stable = msg.confirmable = bool(c and c.expires > now)
        msg.ttl_ms = max(0, int((c.expires - now) * 1000)) if c else 0
        msg.hint = ('继续当前方向（有限任务）' if c and c.kind == 1 else
                    ('左转' if c.relative_yaw > 0 else '右转') +
                    f' {abs(math.degrees(c.relative_yaw)):.1f}°' if c else '等待稳定候选')
        if self.directional:
            msg.hint=('启动/继续直行' if c and c.kind==1 else
                      ('左侧方向意图' if c.relative_yaw>0 else '右侧方向意图') if c else ('左眼眨一次启动；看左右转向，双眼睁开恢复' if self.sample_mode == 'fusion' else '看方向，双眨眼确认'))
        self.candidate_pub.publish(msg)
        if c:
            self.last_candidate = c
        health = self.base(InputHealth(), stamp)
        health.sequence = self.sequence
        health.sample_sequence = self.sample_sequence
        health.reception_timed = self.sample_mode == 'legacy'
        age = now - self.received_at + self.received_age if self.received_at is not None else 1e6
        health.source_age_ms = max(0., age * 1000.) if math.isfinite(age) else 1e9
        health.source_valid = self.clock_valid and (self.fusion_input.tracking_healthy(now)
            if self.sample_mode == 'fusion' else self.machine.healthy(now))
        health.ttl_ms = int(self.machine.cfg.source_timeout * 1000)
        self.health_pub.publish(health)
        for event in self.machine.drain_events():
            e = self.event_messages.get(event.event_id)
            if e is None:
                e = self.base(DirectionEvent() if self.directional else IntentEvent(), stamp)
                e.event_id = event.event_id
                e.kind = direction_kind(event) if self.directional else event.kind
                e.ttl_ms = max(1, int(event.ttl * 1000))
                if event.candidate and not self.directional:
                    for name in ('candidate_id', 'candidate_version', 'context_id',
                                 'context_version', 'relative_yaw'):
                        setattr(e, name, getattr(event.candidate, name))
                elif not self.directional:
                    e.context_id, e.context_version = self.machine.context
                self.event_messages[event.event_id] = e
                self.machine.record('event_publish', now, event_id=event.event_id, kind=event.kind)
            self.event_pub.publish(e)
        pending = self.machine.pending
        self.event_messages = {k: v for k, v in self.event_messages.items()
                               if pending and k == pending.event_id}
        feedback = String()
        feedback.data = json.dumps(dict(state=self.fusion_input.state if self.sample_mode == 'fusion' else 'WHEELCHAIR',
            target='wheelchair',
            blockers=self.start_blockers(now) if self.sample_mode == 'fusion' else [],
            event_decision=self.fusion_input.event_decision if self.sample_mode == 'fusion' else 'NONE',
            blink_count=len(self.fusion_input.blinks) if self.sample_mode == 'fusion' and not self.fusion_input.session_enabled else 0,
            interaction_mode='directional_intent' if self.directional else 'intent_confirm',
            preview=self.machine.preview,
            preview_relative_yaw=-self.machine.preview*self.machine.cfg.relative_yaw,
            candidate=msg.hint, candidate_kind=c.kind if c else 0,
            candidate_relative_yaw=c.relative_yaw if c else 0., stable=msg.stable,
            consumed=bool(pending), ack=self.machine.ack, reason=self.fusion_input.reason if self.sample_mode == 'fusion' else self.machine.reason,
            healthy=health.source_valid, task=self.machine.task), ensure_ascii=False)
        self.feedback.publish(feedback)
        self.common_feedback.publish(feedback)
        preview_published_at = time.monotonic()
        for measurement in self.machine.drain_measurements():
            measurement.setdefault('sample_sequence', self.sample_sequence)
            if measurement['stage'] == 'preview':
                elapsed = max(0., preview_published_at - measurement['monotonic_s'])
                measurement['monotonic_s'] = preview_published_at
                measurement['reception_to_preview_publish_s'] = elapsed
                if self.sample_mode == 'stamped':
                    measurement['source_to_preview_publish_s'] = self.received_age + elapsed
            record = String()
            record.data = json.dumps(dict(source_session_id=self.session,
                reception_timed=health.reception_timed, clock_domain='local',
                timing_basis='reception' if health.reception_timed else 'source_wall_clock', **measurement))
            self.latency_pub.publish(record)


def direction_kind(event):
    """Map confirmed horizontal intent and cancellation onto keyboard semantics."""
    if event.kind==3:return 5  # STOP always wins; no candidate required.
    if event.candidate is None:raise ValueError('Direction event requires a confirmed candidate')
    if event.kind==1 and event.candidate.kind==1:return 1  # W
    if event.kind==2 and event.candidate.kind==2:
        return 2 if event.candidate.relative_yaw>0 else 3  # A / D
    raise ValueError('Unsupported confirmed direction')
