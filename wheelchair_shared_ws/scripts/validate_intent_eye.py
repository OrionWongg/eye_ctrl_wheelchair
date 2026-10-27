#!/usr/bin/env python3
"""Isolated Gazebo acceptance for synthetic eye confirmation (no real hardware).

Run after sourcing ROS and the built workspace:
  ROS_DOMAIN_ID=98 IGN_PARTITION=wheelchair_intent_eye_validation \
    ROS_LOCALHOST_ONLY=1 python3 scripts/validate_intent_eye.py
Only this script's launched process group is stopped on exit. No other test
or simulator should share this domain/partition. Results are under artifacts/intent-eye.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from process_cleanup import stop_group


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default='straight_wall', choices=['straight_wall', 'passage_110', 'slight_offset'])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '98' or os.environ.get('IGN_PARTITION') != 'wheelchair_intent_eye_validation':
        parser.error('Requires ROS_DOMAIN_ID=98 IGN_PARTITION=wheelchair_intent_eye_validation')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    # Imports after isolation check also allow --help without sourcing ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32MultiArray, String
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Contacts
    from wheelchair_interfaces.msg import CandidateStatus, IntentEvent, IntentAck, TaskStatus, ExecutionStatus

    root = Path(__file__).resolve().parents[1]
    output = args.output or root / 'artifacts' / 'intent-eye'
    output.mkdir(parents=True, exist_ok=True)
    log = (output / 'gazebo.log').open('w')
    timeline = (output / 'events.jsonl').open('w')
    result = dict(status='FAIL', scene=args.scene, synthetic_only=True,
                  ros_domain_id=98, partition='wheelchair_intent_eye_validation', cases=[])
    command = ['ros2', 'launch', 'wheelchair_control', 'shared.launch.py',
               'gui:=false', 'input_source:=eye', 'interaction_mode:=intent_confirm',
               'mode:=stop_only', 'scene:=' + args.scene]
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('intent_eye_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    eye_pub = node.create_publisher(Float32MultiArray, '/eye_tracking/signals', 1)
    data = dict(task=None, candidate=None, odom=None, execution=None, contacts=0)
    state = dict(eye=[0., 0., 1.], published_at=0., odom_at=0., task_at=0., path=0., prior_xy=None)
    events, statuses, acks = [], [], []
    start = time.monotonic()

    def record(stage, **fields):
        entry = dict(stage=stage, monotonic_s=time.monotonic(), **fields)
        timeline.write(json.dumps(entry) + '\n')
        timeline.flush()

    def on_task(msg):
        data['task'] = msg
        state['task_at'] = time.monotonic()
        statuses.append((time.monotonic(), msg))
        record('task_status', task_id=msg.task_id, event_id=msg.event_id, state=msg.state,
               reason=msg.reason, action=msg.executed_action, can_accept=msg.can_accept,
               remaining_time=msg.remaining_time, remaining_distance=msg.remaining_distance)

    def on_event(msg):
        events.append((time.monotonic(), msg))
        record('intent_event', source_session_id=msg.source_session_id, event_id=msg.event_id,
               kind=msg.kind, candidate_id=msg.candidate_id, relative_yaw=msg.relative_yaw)

    def on_ack(msg):
        acks.append(msg)
        record('intent_ack', event_id=msg.event_id, status=msg.status, reason=msg.reason)

    def on_odom(msg):
        data['odom'] = msg
        state['odom_at'] = time.monotonic()
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if state['prior_xy'] is not None:
            state['path'] += math.dist(xy, state['prior_xy'])
        state['prior_xy'] = xy

    def on_contacts(msg):
        count = sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in msg.contacts)
        data['contacts'] += count
        if count:
            record('physical_contact', count=count)

    def on_latency(msg):
        try:
            measurement = json.loads(msg.data)
            timeline.write(json.dumps(measurement) + '\n')
            timeline.flush()
        except ValueError:
            record('eye_latency_unparsed', payload=msg.data)

    node.create_subscription(TaskStatus, '/wheelchair/task_status', on_task, 10)
    node.create_subscription(CandidateStatus, '/wheelchair/eye/candidate', lambda m: data.update(candidate=m), 1)
    node.create_subscription(IntentEvent, '/wheelchair/eye/event', on_event, 10)
    node.create_subscription(IntentAck, '/wheelchair/intent_ack', on_ack, 10)
    node.create_subscription(Odometry, '/wheelchair/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(ExecutionStatus, '/wheelchair/executor_status', lambda m: data.update(execution=m), 10)
    node.create_subscription(Contacts, '/wheelchair/sim/contacts', on_contacts, 10)
    node.create_subscription(String, '/wheelchair/eye_latency_events', on_latency, 10)
    node.create_subscription(String, '/wheelchair/task_latency_events', on_latency, 10)

    def spin(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError('Owned launch exited; see gazebo.log')
            now = time.monotonic()
            if now - state['published_at'] >= 1 / 30:
                eye_pub.publish(Float32MultiArray(data=state['eye']))
                state['published_at'] = now
            rclpy.spin_once(node, timeout_sec=.003)

    def gaze(h=0., closed=False, valid=True, duration=.1):
        state['eye'] = [float(h), float(closed), float(valid)]
        spin(duration)

    def wait(predicate, timeout=12., description='condition'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            spin(.025)
            if predicate():
                return
        task = data['task']
        raise AssertionError(f'Timeout waiting for {description}; task={task}')

    def motion_events():
        return {(m.source_session_id, m.event_id): m for _, m in events if m.kind in (1, 2)}

    def speed():
        assert data['odom'] is not None, 'Missing odom'
        twist = data['odom'].twist.twist
        return abs(twist.linear.x), abs(twist.angular.z)

    def stopped():
        return (data['odom'] is not None and time.monotonic() - state['odom_at'] < .3
                and max(speed()) < .025 and data['execution'] is not None and data['execution'].stopped)

    def yaw():
        q = data['odom'].pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def ready():
        return (data['task'] is not None and data['task'].can_accept
                and time.monotonic() - state['task_at'] < .3 and stopped())

    def stable(h):
        gaze(h, duration=.38)
        wait(lambda: data['candidate'] is not None and data['candidate'].confirmable
             and data['candidate'].stable and (data['candidate'].kind == 1 if h == 0 else
                 data['candidate'].kind == 2 and data['candidate'].relative_yaw * h < 0),
             description='stable desired candidate')

    def blink(h):
        gaze(h, closed=True, duration=.11)
        gaze(h, duration=.11)

    def confirm(h):
        wait(ready, description='healthy stationary task ready')
        stable(h)
        old = set(motion_events())
        blink(h)
        blink(h)
        wait(lambda: bool(set(motion_events()) - old), 2., 'one confirmed eye event')
        new = set(motion_events()) - old
        assert len(new) == 1, 'More than one event from one double blink'
        event = motion_events()[new.pop()]
        wait(lambda: any(a.event_id == event.event_id and a.source_session_id == event.source_session_id for a in acks),
             2., 'confirmation ACK')
        ack = next(a for a in reversed(acks) if a.event_id == event.event_id and a.source_session_id == event.source_session_id)
        assert ack.status == 1, f'Confirmation rejected: {ack.reason}'
        wait(lambda: data['task'] is not None and data['task'].event_id == event.event_id
             and data['task'].state == 2, 4., 'task RUNNING')
        return event, data['task'].task_id

    def check_stationary(label, count, duration=.35):
        before = state['path']
        spin(duration)
        assert len(motion_events()) == count, label + ': unexpected motion event'
        assert stopped(), label + ': wheels moving'
        assert state['path'] - before < .025, label + ': actual displacement'

    def case(name, **facts):
        result['cases'].append(dict(name=name, status='PASS', **facts))
        record('case_pass', name=name, **facts)

    try:
        # Changing previews prevents a dwell candidate while source health and
        # the STOP/ready handshake are established. No fake manager/status input.
        deadline = time.monotonic() + 25.
        index = 0
        while time.monotonic() < deadline and not ready():
            gaze((-.8, .8, 0.)[index % 3], duration=.075)
            index += 1
        assert ready(), 'Initial source/STOP readiness handshake failed'
        # Explicit invalidation withdraws any candidate established during startup.
        gaze(valid=False, duration=.12)
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            gaze((-.8, .8, 0.)[index % 3], duration=.075)
            index += 1
            if ready() and data['candidate'] is not None and not data['candidate'].confirmable:
                break
        assert ready(), 'Source recovery not ready'
        assert data['candidate'] is not None and not data['candidate'].confirmable
        before_count = len(motion_events())
        # No open dwell before the first closure; no default CONTINUE is allowed.
        blink(state['eye'][0])
        blink(state['eye'][0])
        check_stationary('double blink without candidate', before_count)
        case('no_candidate_double_blink_no_motion', motion_events=before_count)

        for direction in (-.8, .8, 0.):
            gaze(direction, duration=.8)
            check_stationary('unconfirmed sustained gaze', before_count, .1)
        case('unconfirmed_left_right_center_no_motion')

        stable(-.8)
        blink(-.8)
        gaze(-.8, duration=.65)
        check_stationary('single blink', before_count)
        case('single_blink_no_start')

        angle_before = yaw()
        path_before = state['path']
        event, task_id = confirm(-.8)
        count = len(motion_events())
        # Healthy scans must leave the accepted task identity untouched.
        for direction in (.8, 0., -.8):
            gaze(direction, duration=.12)
            assert data['task'].task_id == task_id, 'Healthy gaze changed task identity'
        wait(lambda: any(m.task_id == task_id and m.state == 4 for _, m in statuses),
             12., 'finite heading completion')
        wait(stopped, 3., 'heading stopped')
        actual_angle = wrapped(yaw() - angle_before)
        assert math.radians(13) <= actual_angle <= math.radians(17), f'Wrong finite turn angle {math.degrees(actual_angle)}'
        check_stationary('heading completion does not auto-forward', count, .65)
        assert len({m.task_id for _, m in statuses if m.event_id == event.event_id and m.state in (1, 2, 4)}) == 1
        case('finite_heading_once_completed_no_auto_forward', task_id=task_id,
             actual_yaw_degrees=math.degrees(actual_angle), path_m=state['path'] - path_before,
             event_id=event.event_id)

        path_before = state['path']
        _, task_id = confirm(0.)
        for direction in (-.8, .8, 0., .8):
            gaze(direction, duration=.18)
            assert data['task'].task_id == task_id and data['task'].state == 2, 'Healthy scan interrupted task'
        progress = state['path'] - path_before
        assert progress > .04, f'CONTINUE failed to make actual progress: {progress}'
        count = len(motion_events())
        gesture_at = time.monotonic()
        gaze(closed=True, duration=.78)
        wait(stopped, 3., 'long closure actual stop')
        stopped_at = time.monotonic()
        assert any(t >= gesture_at and m.kind == 3 for t, m in events), 'Long closure did not publish CANCEL'
        gaze(duration=.5)
        blink(0.)
        gaze(duration=.5)
        check_stationary('long close recovery and single blink', count)
        case('continue_healthy_scans_same_task_long_close_cancel', task_id=task_id,
             actual_progress_m=progress, gesture_to_stopped_s=stopped_at - gesture_at)

        path_before = state['path']
        _, task_id = confirm(0.)
        gaze(duration=.45)
        assert state['path'] - path_before > .025, 'Second task never moved before invalid test'
        invalid_at = time.monotonic()
        gaze(valid=False, duration=.25)
        wait(stopped, 3., 'invalid-source stop')
        count = len(motion_events())
        gaze(duration=.7)
        check_stationary('valid source does not restore task', count)
        assert any(t >= invalid_at and m.kind == 3 for t, m in events)
        case('invalid_stops_and_healthy_recovery_no_restart', task_id=task_id,
             actual_progress_m=state['path'] - path_before)
        assert data['contacts'] == 0, 'Physical robot collision'
        result.update(status='PASS', robot_contacts=0, actual_total_path_m=state['path'],
                      unique_motion_events=len(motion_events()), elapsed_s=time.monotonic() - start)
    except Exception as exc:
        result.update(error=str(exc), robot_contacts=data['contacts'],
                      actual_total_path_m=state['path'], elapsed_s=time.monotonic() - start)
        raise
    finally:
        # Send invalid to the owned simulated stack before stopping its process group.
        try:
            gaze(valid=False, duration=.15)
        except Exception:
            pass
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
        stop_group(process)
        log.close()
        timeline.close()
        print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
