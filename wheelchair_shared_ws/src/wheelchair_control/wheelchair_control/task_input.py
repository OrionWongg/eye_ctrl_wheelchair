"""Simulation keyboard/event source. Never publishes legacy or chassis commands."""
import argparse
from dataclasses import dataclass, field
import json
import math
import os
import select
import sys
import termios
import time
import tty
import uuid


@dataclass
class KeyboardState:
    session: str
    context: tuple = ()
    context_at: float = -1e9
    available: bool = False
    candidate: dict = None
    pending: dict = None
    counter: int = 0
    version: int = 0
    pending_at: float = 0.0
    retry_at: float = 0.0
    candidate_at: float = 0.0

    def status(self, context, available, now):
        changed = context != self.context
        if changed:
            self.candidate = self.pending = None
        self.context, self.context_at, self.available = context, now, available
        return changed

    def select(self, direction, now):
        if not self.available or now - self.context_at > 0.5 or self.pending:
            return None
        self.version += 1
        self.candidate = dict(candidate_id=f'{self.session}:{self.version}',
                              candidate_version=self.version,
                              kind=1 if direction == 'continue' else 2,
                              relative_yaw={'continue': 0., 'left': math.pi / 12,
                                            'right': -math.pi / 12}[direction],
                              context_id=self.context[0], context_version=self.context[1])
        self.candidate_at = now
        return self.candidate

    def confirm(self, now):
        if (self.pending or not self.candidate or not self.available
                or now - self.context_at > 0.5 or now - self.candidate_at > 1.5):
            return None
        self.counter += 1
        self.pending = dict(self.candidate, event_id=self.counter)
        self.pending_at = self.retry_at = now
        return self.pending

    def retry(self, now):
        if not self.pending:
            return None
        if (now - self.pending_at >= 1.5 or now - self.candidate_at >= 1.5
                or now - self.context_at > 0.5):
            self.pending = self.candidate = None
            return None
        if now - self.retry_at >= 0.1:
            self.retry_at = now
            return self.pending
        return None

    def ack(self, session, event_id):
        if session != self.session or not self.pending or self.pending['event_id'] != event_id:
            return False
        self.pending = self.candidate = None
        return True

    def cancel(self):
        self.candidate = self.pending = None
        self.counter += 1
        return dict(event_id=self.counter, kind=3)


@dataclass
class InputBoundary:
    """ROS-independent freshness, retired-session and clock checks."""
    manager: str = ''
    retired: set = field(default_factory=set)
    sequences: dict = field(default_factory=dict)
    context_version: int = -1
    ros_previous: float = None
    mono_previous: float = None
    progressed_at: float = None
    clock_ok: bool = False

    def clock(self, ros_now, now):
        if self.ros_previous is None:
            self.ros_previous, self.mono_previous, self.progressed_at = ros_now, now, now
            return False
        delta, elapsed = ros_now - self.ros_previous, now - self.mono_previous
        jumped = delta < -1e-6 or delta - elapsed > 0.5
        if jumped:
            self.clock_ok = False
            self.progressed_at = now
        elif delta > 1e-9:
            self.clock_ok = True
            self.progressed_at = now
        elif now - self.progressed_at > 0.25:
            self.clock_ok = False
        self.ros_previous, self.mono_previous = ros_now, now
        return self.clock_ok

    @staticmethod
    def fresh(stamp, ros_now):
        return math.isfinite(stamp) and -0.1 <= ros_now - stamp <= 0.5

    def status(self, manager, context_id, version, sequence, stamp, ros_now):
        if (not self.clock_ok or not manager or manager != context_id or manager in self.retired
                or not self.fresh(stamp, ros_now)
                or sequence <= self.sequences.get(manager, -1)
                or (manager == self.manager and version < self.context_version)):
            return False
        if self.manager and manager != self.manager:
            self.retired.add(self.manager)
        self.manager, self.context_version = manager, version
        self.sequences[manager] = sequence
        return True

    def ack(self, manager, status, stamp, ros_now, pending_at, now):
        return (self.clock_ok and manager == self.manager and status in (1, 2)
                and self.fresh(stamp, ros_now) and 0 <= now - pending_at < 1.5)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--submit', choices=['continue', 'left', 'right'])
    parser.add_argument('--seconds', type=float, default=10.0,
                        help='scripted source lifetime; expiry always sends CANCEL')
    parser.add_argument('--idle-timeout', type=float, default=15.0)
    parser.add_argument('--log', help='append structured JSONL timing records')
    options, ros_args = parser.parse_known_args(args)
    if options.seconds <= 0 or options.idle_timeout <= 0:
        parser.error('durations must be positive')
    if not options.submit and not sys.stdin.isatty():
        parser.error('interactive input requires a terminal; use --submit for scripts')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from wheelchair_interfaces.msg import CandidateStatus, IntentEvent, InputHealth, IntentAck, TaskStatus
    rclpy.init(args=ros_args)
    node = Node('intent_keyboard')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
    event_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
    pubs = {name: node.create_publisher(cls, '/wheelchair/keyboard/' + name,
                                      event_qos if name == 'event' else qos)
            for name, cls in [('candidate', CandidateStatus), ('event', IntentEvent), ('health', InputHealth)]}
    model = KeyboardState(uuid.uuid4().hex)
    sequence = 0
    candidate_sequence = 0
    boundary = InputBoundary()
    cached_event = None
    last_display = None
    logfile = open(options.log, 'a', buffering=1) if options.log else None

    def record(stage, **fields):
        if logfile:
            logfile.write(json.dumps(dict(stage=stage, monotonic_s=time.monotonic(),
                                          source_session_id=model.session, **fields)) + '\n')

    def on_status(msg):
        nonlocal last_display
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now = node.get_clock().now().nanoseconds * 1e-9
        if (msg.protocol_version != 1 or not boundary.status(msg.manager_session_id,
                msg.context_id, msg.context_version, msg.sequence, stamp, ros_now)):
            return
        changed = model.status((msg.context_id, msg.context_version), msg.can_accept, time.monotonic())
        display = (msg.task_id, msg.state, msg.executed_action, msg.reason, msg.can_accept)
        if display != last_display:
            state_name = {0: '空闲', 1: '已接受', 2: '执行中', 3: '正在停车', 4: '已完成', 5: '受阻', 6: '故障'}.get(msg.state, str(msg.state))
            action_name = {0: '停止', 1: '前进', 2: '左转', 3: '右转'}.get(msg.executed_action, str(msg.executed_action))
            print(f'\r任务={msg.task_id or "无"} 状态={state_name} 动作={action_name} '
                  f'可确认={msg.can_accept} 原因={msg.reason}', flush=True)
            record('task_status', task_id=msg.task_id, event_id=msg.event_id,
                   state=msg.state, reason=msg.reason)
            last_display = display
        if changed:
            publish_candidate(False)

    def on_ack(msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now = node.get_clock().now().nanoseconds * 1e-9
        if (msg.protocol_version == 1 and model.context and model.pending
                and boundary.ack(msg.manager_session_id, msg.status, stamp, ros_now, model.pending_at, now)
                and now - model.candidate_at < 1.5
                and cached_event is not None
                and stamp >= cached_event.header.stamp.sec + cached_event.header.stamp.nanosec * 1e-9
                and model.ack(msg.source_session_id, msg.event_id)):
            print(f'\r确认 {"接受" if msg.status == 1 else "拒绝"}: {msg.reason}', flush=True)
            record('ack', event_id=msg.event_id, task_id=msg.task_id, status=msg.status)
            publish_candidate(False)

    def fill(msg, values):
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.protocol_version = 1
        msg.source_session_id = model.session
        for key, value in values.items():
            setattr(msg, key, value)
        return msg

    def publish_candidate(valid=True):
        nonlocal candidate_sequence
        candidate_sequence += 1
        msg = fill(CandidateStatus(), model.candidate or {})
        msg.sequence = candidate_sequence
        remaining_ms = max(0, int((model.candidate_at + 1.5 - time.monotonic()) * 1000))
        msg.stable = msg.confirmable = bool(valid and model.candidate and remaining_ms > 0)
        msg.ttl_ms = min(1500, remaining_ms)
        msg.hint = '继续有限任务' if msg.kind == 1 else '有限左转 15°' if msg.relative_yaw > 0 else '有限右转 15°'
        if not msg.confirmable:
            msg.hint = '无可确认候选'
        pubs['candidate'].publish(msg)

    def submit_event(event):
        nonlocal cached_event
        cached_event = fill(IntentEvent(), event)
        cached_event.ttl_ms = 1500
        pubs['event'].publish(cached_event)
        record('cancel' if event['kind'] == 3 else 'confirmation', event_id=event['event_id'],
               candidate_id=event.get('candidate_id', ''))

    def cancel():
        submit_event(model.cancel())
        publish_candidate(False)

    subscriptions = [node.create_subscription(TaskStatus, '/wheelchair/task_status', on_status, qos),
                     node.create_subscription(IntentAck, '/wheelchair/intent_ack', on_ack, event_qos)]
    del subscriptions  # Node retains subscription ownership.
    original = None
    start = last_operation = time.monotonic()
    next_tick = start
    scripted_selected = None
    scripted_sent = False
    previous_valid = True
    print('W/A/D 选择继续/左转/右转；Enter 确认；Space/S 停止；Q 退出。仅仿真。', flush=True)
    try:
        if not options.submit:
            original = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            clock_ok = boundary.clock(node.get_clock().now().nanoseconds * 1e-9, time.monotonic())
            if not clock_ok:
                if model.candidate or model.pending or model.available:
                    cancel()
                model.available = False
                model.context_at = -1e9
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if options.submit and now - start >= options.seconds:
                break
            foreground = bool(options.submit or (sys.stdin.isatty()
                              and os.tcgetpgrp(sys.stdin.fileno()) == os.getpgrp()))
            valid = clock_ok and foreground and (bool(options.submit) or now - last_operation < options.idle_timeout)
            if previous_valid and not valid:
                cancel()
            previous_valid = valid
            if not options.submit and foreground and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                last_operation = now
                if key in ('q', '\x03', '\x04') or key == '':
                    break
                if key in (' ', 's'):
                    cancel()
                elif key in ('w', 'a', 'd') and clock_ok:
                    candidate = model.select({'w': 'continue', 'a': 'left', 'd': 'right'}[key], now)
                    if candidate:
                        publish_candidate()
                        print('\r候选：' + {'w': '继续', 'a': '左转 15°', 'd': '右转 15°'}[key]
                              + '；Enter 确认（1.5 秒内，最终仍需安全检查）', flush=True)
                        record('stable', candidate_id=candidate['candidate_id'])
                elif key in ('\r', '\n') and valid:
                    event = model.confirm(now)
                    if event:
                        publish_candidate()
                        submit_event(event)
                    else:
                        print('\r没有新鲜、稳定、可确认的候选，请重新选择。', flush=True)
            if options.submit and valid and not scripted_sent:
                if scripted_selected is None or not model.candidate:
                    if model.select(options.submit, now):
                        publish_candidate()
                        scripted_selected = now
                        record('stable', candidate_id=model.candidate['candidate_id'])
                elif now - scripted_selected >= 0.1:
                    event = model.confirm(now)
                    if event:
                        publish_candidate()
                        submit_event(event)
                        scripted_sent = True
            if now - model.context_at > 0.5 and (model.candidate or model.pending):
                cancel()
            retry = model.retry(now)
            if retry and cached_event is not None and cached_event.event_id == retry['event_id']:
                pubs['event'].publish(cached_event)  # original ID AND original ROS stamp
            if model.candidate and now - model.candidate_at > 1.5 and not model.pending:
                model.candidate = None
                publish_candidate(False)
            if now >= next_tick:
                sequence += 1
                health = fill(InputHealth(), {})
                health.sequence = health.sample_sequence = sequence
                health.source_valid = valid
                # Sample is a current foreground-terminal/lifetime observation, not an old key.
                health.source_age_ms = 0.0
                health.reception_timed = True
                health.ttl_ms = 250
                pubs['health'].publish(health)
                publish_candidate(valid)
                next_tick = now + 0.05
    except KeyboardInterrupt:
        pass
    finally:
        if original is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)
        if rclpy.ok():
            cancel()
            # Briefly allow volatile reliable messages to reach existing subscribers.
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.02)
        if logfile:
            logfile.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
