#!/usr/bin/env python3
"""Gazebo finite-task acceptance, isolated domain 96 / wheelchair_intent_task_validation.

Source the built ROS workspace, then run:
 ROS_DOMAIN_ID=96 IGN_PARTITION=wheelchair_intent_task_validation ROS_LOCALHOST_ONLY=1 \
 python3 scripts/validate_intent_tasks.py --case heading
Use --case all for separate launches of every case. Only owned processes are killed.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
from process_cleanup import stop_group

CASES = ('continue_budget', 'heading', 'replace', 'cancel', 'health_loss', 'source_exit',
         'manager_exit', 'planner_exit', 'executor_exit', 'branch', 'blocked_door', 'replay', 'clock_pause',
         'time_budget','corridor_align','door_pass','manager_restart')


def wrapped(value):
    return math.atan2(math.sin(value), math.cos(value))


def owned_pid(launch_pid, executable):
    """Identify exact executable only among descendants of our launch."""
    parents, commands = {}, {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            stat = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            parents[int(path.name)] = int(stat[1])
            commands[int(path.name)] = (path / 'cmdline').read_bytes().split(b'\0')
        except (OSError, ValueError):
            continue
    descendants = {launch_pid}
    while True:
        added = {pid for pid, parent in parents.items() if parent in descendants} - descendants
        if not added:
            break
        descendants.update(added)
    matches = [pid for pid in descendants if pid != launch_pid
               and any(Path(arg.decode(errors='replace')).name == executable
                       for arg in commands.get(pid, [])[:2])]
    assert len(matches) == 1, f'Expected one owned {executable}, found {matches}'
    return matches[0]


def run_case(name, output):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ros_gz_interfaces.msg import Contacts
    from wheelchair_interfaces.msg import CandidateStatus, IntentEvent, InputHealth, IntentAck, TaskStatus, ExecutionStatus, DiscreteAction
    scene = ('straight_wall' if name in ('heading', 'replace', 'replay') else
             'passage_branch' if name == 'branch' else
             'door_090' if name=='door_pass' else
             'door_blocked_exit' if name == 'blocked_door' else 'passage_110')
    folder = output / name
    folder.mkdir(parents=True, exist_ok=True)
    log = (folder / 'gazebo.log').open('w')
    timeline = (folder / 'events.jsonl').open('w')
    case_env=dict(os.environ,IGN_PARTITION='wheelchair_intent_task_validation_'+uuid.uuid4().hex[:12])
    result = dict(case=name, scene=scene, status='FAIL', simulated_only=True,
                  ros_domain_id=96, partition=case_env['IGN_PARTITION'], cases=[])
    process = subprocess.Popen(['ros2', 'launch', 'wheelchair_control', 'shared.launch.py',
                                'gui:=false', 'interaction_mode:=intent_confirm',
                                'input_source:=keyboard', 'mode:=corridor_assist', 'scene:=' + scene,
                                'task_max_time:='+('3.0' if name=='time_budget' else '10.0')],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True,env=case_env)
    source_process = None
    extra=[]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('intent_tasks_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    pubs = {key: node.create_publisher(cls, '/wheelchair/keyboard/' + key, 10 if key == 'event' else 1)
            for key, cls in [('candidate', CandidateStatus), ('event', IntentEvent), ('health', InputHealth)]}
    data = dict(task=None, odom=None, execution=None, action=None, contacts=0, task_at=0., odom_at=0.)
    state = dict(session=uuid.uuid4().hex, sequence=0, event_id=0, candidate=None,
                 candidate_deadline=0., tick_at=0., health=True, publish_source=name != 'source_exit',
                 path=0., xy=None, samples=[], shutdown=False)
    statuses, acks = [], []
    start = time.monotonic()

    def record(stage, **fields):
        timeline.write(json.dumps(dict(stage=stage, monotonic_s=time.monotonic(), **fields)) + '\n')
        timeline.flush()

    def status(msg):
        data.update(task=msg, task_at=time.monotonic())
        statuses.append((time.monotonic(), msg))
        record('task_status', source_session_id=msg.source_session_id, task_id=msg.task_id,
               event_id=msg.event_id, state=msg.state, reason=msg.reason, can_accept=msg.can_accept,
               remaining_time=msg.remaining_time, remaining_distance=msg.remaining_distance,
               executed_action=msg.executed_action)

    def ack(msg):
        acks.append(msg)
        record('ack', source_session_id=msg.source_session_id, event_id=msg.event_id,
               task_id=msg.task_id, status=msg.status, reason=msg.reason)

    def odom(msg):
        data.update(odom=msg, odom_at=time.monotonic())
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if state['xy'] is not None:
            state['path'] += math.dist(state['xy'], (p.x, p.y))
        state['xy'] = (p.x, p.y)
        state['samples'].append((p.x, p.y, yaw))
        record('odom', x=p.x, y=p.y, yaw=yaw, linear=msg.twist.twist.linear.x,
               angular=msg.twist.twist.angular.z)

    def contacts(msg):
        count = sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in msg.contacts)
        data['contacts'] += count
        if count:
            record('physical_contact', count=count)

    def latency(msg):
        try:
            row = json.loads(msg.data)
            assert isinstance(row, dict) and 'stage' in row
            # Preserve producer's stage and monotonic timestamp: no nested wrapper.
            timeline.write(json.dumps(row) + '\n')
            timeline.flush()
        except (ValueError, AssertionError):
            record('invalid_latency_record', raw=msg.data)

    def action(msg):
        data['action'] = msg
        record('safe_action', action=msg.action, reason=msg.reason, sequence=msg.sequence,max_run_ms=msg.max_run_ms)

    node.create_subscription(TaskStatus, '/wheelchair/task_status', status, 10)
    node.create_subscription(IntentAck, '/wheelchair/intent_ack', ack, 10)
    node.create_subscription(Odometry, '/wheelchair/odom', odom, qos_profile_sensor_data)
    node.create_subscription(ExecutionStatus, '/wheelchair/executor_status', lambda m: data.update(execution=m), 10)
    node.create_subscription(DiscreteAction, '/wheelchair/action_safe', action, 10)
    node.create_subscription(Contacts, '/wheelchair/sim/contacts', contacts, 10)
    for topic in ('/wheelchair/task_latency_events', '/wheelchair/eye_latency_events'):
        node.create_subscription(String, topic, latency, 10)

    def fill(msg):
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.protocol_version = 1
        msg.source_session_id = state['session']
        return msg

    def tick():
        now = time.monotonic()
        if not state['publish_source'] or now - state['tick_at'] < .04:
            return
        state['sequence'] += 1
        # Each tick is a fresh, finite-lifetime synthetic source sample.
        health = fill(InputHealth())
        health.sequence = health.sample_sequence = state['sequence']
        health.source_valid = state['health']
        health.source_age_ms = 0.
        health.reception_timed = True
        health.ttl_ms = 250
        pubs['health'].publish(health)
        candidate = state['candidate']
        if candidate is not None:
            candidate.header.stamp = node.get_clock().now().to_msg()
            candidate.sequence = state['sequence']
            candidate.ttl_ms = max(0, int((state['candidate_deadline'] - now) * 1000))
            candidate.stable = candidate.confirmable = candidate.ttl_ms > 0
            pubs['candidate'].publish(candidate)
        state['tick_at'] = now

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Owned launch exited; inspect gazebo.log'
            tick()
            rclpy.spin_once(node, timeout_sec=.004)

    def wait(predicate, timeout=12., label='condition'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            spin(.03)
            if predicate():
                return
        raise AssertionError(f'Timeout: {label}; latest task={data["task"]}')

    def speed():
        if data['odom'] is None:
            return math.inf
        twist = data['odom'].twist.twist
        return max(abs(twist.linear.x), abs(twist.angular.z))

    def stopped():
        return time.monotonic() - data['odom_at'] < .3 and speed() < .025

    def ready():
        return (data['task'] is not None and data['task'].can_accept
                and time.monotonic() - data['task_at'] < .3 and stopped())

    def cancel():
        state['candidate'] = None
        state['event_id'] += 1
        msg = fill(IntentEvent())
        msg.event_id, msg.kind, msg.ttl_ms = state['event_id'], 3, 1500
        pubs['event'].publish(msg)
        record('cancel', source_session_id=state['session'], event_id=msg.event_id)

    def submit(kind=1, angle=0., expect_accept=True):
        assert data['task'] is not None and data['task'].can_accept, 'No available task context'
        state['event_id'] += 1
        candidate = fill(CandidateStatus())
        candidate.candidate_id = state['session'] + ':' + str(state['event_id'])
        candidate.candidate_version = state['event_id']
        candidate.context_id, candidate.context_version = data['task'].context_id, data['task'].context_version
        candidate.kind, candidate.relative_yaw = kind, angle
        candidate.stable = candidate.confirmable = True
        candidate.ttl_ms = 1500
        state['candidate'], state['candidate_deadline'] = candidate, time.monotonic() + 1.5
        record('stable', source_session_id=state['session'], candidate_id=candidate.candidate_id)
        spin(.08)
        event = fill(IntentEvent())
        for key in ('candidate_id', 'candidate_version', 'context_id', 'context_version', 'kind', 'relative_yaw'):
            setattr(event, key, getattr(candidate, key))
        event.event_id, event.ttl_ms = state['event_id'], 1500
        pubs['event'].publish(event)
        record('confirmation', source_session_id=state['session'], event_id=event.event_id,
               candidate_id=event.candidate_id)
        deadline, resend_at = time.monotonic() + 2., time.monotonic()
        matched = lambda: [a for a in acks if a.source_session_id == state['session'] and a.event_id == event.event_id]
        while time.monotonic() < deadline and not matched():
            spin(.03)
            if time.monotonic() - resend_at >= .12 and time.monotonic() < state['candidate_deadline']:
                pubs['event'].publish(event)  # unchanged original header and event ID
                resend_at = time.monotonic()
        assert matched(), 'No bounded confirmation ACK'
        received = matched()[-1]
        state['candidate'] = None
        if expect_accept:
            assert received.status == 1, 'Confirmation rejected: ' + received.reason
        return event, received

    def running():
        return data['task'] is not None and data['task'].state == 2 and speed() > .04

    def terminal(task_id):
        return any(m.task_id == task_id and m.state in (4, 5, 6) for _, m in statuses)

    def hold_stopped(seconds=.7):
        before = state['path']
        spin(seconds)
        assert stopped() and state['path'] - before < .02, 'Motion restarted without new confirmation'

    def pause(value):
        completed = subprocess.run(['ign', 'service', '-s', '/world/wheelchair_arena/control',
                                    '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
                                    '--timeout', '2000', '--req', 'pause: ' + str(value).lower()],
                                   capture_output=True, text=True, timeout=4,env=case_env)
        assert completed.returncode == 0 and 'true' in completed.stdout.lower(), completed.stdout + completed.stderr
        record('gazebo_pause', paused=value)

    try:
        if name == 'source_exit':
            source_process = subprocess.Popen(['ros2', 'run', 'wheelchair_control', 'task_input',
                                               '--submit', 'continue', '--seconds', '30',
                                               '--ros-args', '-p', 'use_sim_time:=true'],
                                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True,env=case_env)
            wait(running, 35., 'external scripted source actual motion')
            before = state['path']
            spin(.4)
            assert state['path'] - before > .02
            os.killpg(source_process.pid, signal.SIGKILL)
            record('source_process_killed', pid=source_process.pid)
            wait(stopped, 3., 'source death watchdog stop')
            hold_stopped()
        else:
            wait(ready, 35., 'initial STOP/health readiness')
            if name=='corridor_align':
                angle=math.radians(5)
                req=f'name: "generic_wheelchair", position: {{x: 0, y: 0.01, z: 0.005}}, orientation: {{z: {math.sin(angle/2)}, w: {math.cos(angle/2)}}}'
                placement_deadline=time.monotonic()+10
                while time.monotonic()<placement_deadline:
                    placed=subprocess.run(['ign','service','-s','/world/wheelchair_arena/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','2000','--req',req],capture_output=True,text=True,timeout=4,env=case_env)
                    if placed.returncode==0 and 'true' in placed.stdout:break
                    spin(.2)
                else:raise AssertionError('Gazebo placement unavailable: '+placed.stdout+placed.stderr)
                spin(1.);wait(ready,10.,'placed chair settled');state['path']=0.;state['samples'].clear()
                spin(.1)
                assert abs(state['samples'][-1][2]-angle)<.01,'Placement did not reach observed odom'
            angle_before = state['samples'][-1][2]
            path_before = state['path']
            event, accepted = submit(2 if name in ('heading', 'replay') else 1,
                                     math.radians(15) if name in ('heading', 'replay') else 0.,
                                     expect_accept=True)
            task_id = accepted.task_id
            if name in ('time_budget','door_pass','corridor_align','manager_restart'):
                task_id=accepted.task_id
                if name in ('time_budget','door_pass'):
                    wait(lambda:terminal(task_id),15.,'bounded task completion')
                    wait(stopped,3.,'task physically stopped')
                    assert data['task'].state==4,data['task'].reason
                    if name=='time_budget':
                        assert data['task'].reason=='TASK_TIME_LIMIT'
                        assert .15<state['path']-path_before<1.5
                    else:
                        assert data['task'].reason.startswith('PASSAGE_EXIT_CONFIRM_DIRECTION')
                        assert state['path']-path_before>1.5
                    result.update(completion_reason=data['task'].reason,actual_task_path_m=state['path']-path_before)
                    hold_stopped()
                elif name=='corridor_align':
                    wait(lambda:state['path']-path_before>1.2 or terminal(task_id),15.,'corrected corridor actual progress')
                    assert state['path']-path_before>.8,'No useful progress within the fixed task time budget'
                    assert abs(state['samples'][-1][2])<math.radians(3)
                    assert data['task'].task_id==task_id and data['task'].state in (2,4),data['task'].reason
                    result.update(actual_progress_m=state['path']-path_before,corrected_yaw_degrees=math.degrees(state['samples'][-1][2]))
                    cancel();wait(stopped,3.,'correction cancellation');hold_stopped()
                else:
                    wait(running,5.,'pre-restart actual motion');spin(.4)
                    os.kill(owned_pid(process.pid,'task_manager'),signal.SIGKILL)
                    wait(stopped,3.,'manager kill actual stop')
                    old_manager=data['task'].manager_session_id
                    extra.append(subprocess.Popen(['ros2','run','wheelchair_control','task_manager','--ros-args','-p','use_sim_time:=true','-p','input_source:=keyboard'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
                    wait(lambda:data['task'].manager_session_id!=old_manager and ready(),12.,'new manager STOP ready')
                    pubs['event'].publish(event)
                    hold_stopped(1.)
                    assert data['task'].state not in (1,2)
                    result.update(old_manager=old_manager,new_manager=data['task'].manager_session_id,replay_restarted=False)
            elif name in ('branch', 'blocked_door') and accepted.status == 2:
                assert accepted.reason, 'Rejection must identify why'
                result['rejection_reason'] = accepted.reason
                geometric = any(word in accepted.reason.upper() for word in ('BRANCH', 'PASSAGE', 'BLOCK', 'DOOR', 'BOUNDARY', 'EXIT'))
                result['acceptance_class'] = ('geometric_rejection_not_throughput' if geometric
                                              else 'protocol_rejection_not_geometry_or_throughput')
                result['geometric_case_validated'] = geometric
                hold_stopped()
            elif name in ('heading', 'replay'):
                wait(lambda: terminal(task_id), 13., 'finite heading terminal')
                wait(stopped, 3., 'heading physically stopped')
                assert any(m.task_id == task_id and m.state == 4 for _, m in statuses), 'Heading failed rather than completed'
                delta = wrapped(state['samples'][-1][2] - angle_before)
                assert math.radians(13) <= delta <= math.radians(17), f'Incorrect finite turn: {math.degrees(delta)} deg'
                result['actual_yaw_degrees'] = math.degrees(delta)
                hold_stopped()
                if name == 'replay':
                    before_ids = {m.task_id for _, m in statuses if m.task_id}
                    replay_yaw = state['samples'][-1][2]
                    for _ in range(5):
                        pubs['event'].publish(event)
                        spin(.12)
                    hold_stopped()
                    assert {m.task_id for _, m in statuses if m.task_id} == before_ids, 'Replay created a new task'
                    assert abs(wrapped(state['samples'][-1][2] - replay_yaw)) < math.radians(1)
                    result['identical_event_replay_count'] = 5
            elif name in ('branch', 'blocked_door'):
                wait(lambda: terminal(task_id), 13., 'safe boundary/blocked terminal')
                wait(stopped, 3., 'boundary actual stop')
                reason = next(m.reason for _, m in reversed(statuses) if m.task_id == task_id and m.state in (4, 5, 6))
                assert any(word in reason.upper() for word in ('BRANCH', 'PASSAGE', 'BLOCK', 'DOOR', 'BOUNDARY', 'EXIT')), reason
                result.update(rejection_reason=reason, acceptance_class='correct_rejection_not_throughput')
                assert abs(wrapped(state['samples'][-1][2] - angle_before)) < math.radians(25), 'Unauthorized branch turn'
                hold_stopped()
            else:
                wait(running, 5., 'actual task motion')
                spin(.45)
                assert state['path'] - path_before > .03, 'No actual movement before interruption'
                if name == 'continue_budget':
                    wait(lambda: terminal(task_id), 13., 'task budget terminal')
                    wait(stopped, 3., 'budget stop')
                    distance = state['path'] - path_before
                    assert .15 < distance <= 3.01, f'Invalid finite path {distance}'
                    task_statuses = [(t, m) for t, m in statuses if m.task_id == task_id]
                    elapsed = task_statuses[-1][0] - task_statuses[0][0]
                    assert elapsed <= 11., f'Task exceeded budget plus stop feedback allowance: {elapsed}'
                    assert any(m.reason in ('TASK_TIME_LIMIT', 'TASK_DISTANCE_LIMIT') for _, m in task_statuses), 'Task did not terminate for its finite budget'
                    result.update(actual_task_path_m=distance, accepted_to_terminal_s=elapsed)
                    hold_stopped()
                elif name == 'replace':
                    second, second_ack = submit(2, -math.radians(15))
                    assert second_ack.task_id != task_id
                    wait(lambda: terminal(second_ack.task_id), 13., 'replacement finite turn completion')
                    wait(stopped, 3., 'replacement stopped')
                    assert any(m.task_id == second_ack.task_id and m.state == 4 for _, m in statuses)
                    assert any(m.state == 3 for _, m in statuses), 'No STOPPING transition during replacement'
                    delta = wrapped(state['samples'][-1][2] - angle_before)
                    assert math.radians(-17) <= delta <= math.radians(-13), 'Wrong replacement angle'
                    hold_stopped()
                else:
                    interrupted_at = time.monotonic()
                    if name == 'cancel':
                        cancel()
                    elif name == 'health_loss':
                        state['health'] = False
                    elif name.endswith('_exit'):
                        executable = {'manager_exit': 'task_manager', 'planner_exit': 'safety_gate', 'executor_exit': 'executor'}[name]
                        pid = owned_pid(process.pid, executable)
                        os.kill(pid, signal.SIGKILL)
                        record('owned_node_killed', executable=executable, pid=pid)
                    elif name == 'clock_pause':
                        pause(True)
                        spin(.7)
                        assert data['action'] is not None and data['action'].action == 0, 'Pause did not revoke safe action'
                        pause(False)
                    wait(stopped, 3., 'interruption actual stop')
                    result['interruption_to_stopped_s'] = time.monotonic() - interrupted_at
                    hold_stopped()
                    if name == 'health_loss':
                        state['health'] = True
                        hold_stopped()
        assert data['contacts'] == 0, 'Physical wheelchair contact'
        assert state['samples'], 'No measured trajectory'
        from wheelchair_control.geometry import clearance, transform, IndexedObstacles
        from wheelchair_sim.scenes import map_obstacles, boundary
        from wheelchair_control.planner import Config
        footprint = ((-.45, -.30), (.55, -.30), (.55, .30), (-.45, .30))
        config = Config(footprint=footprint, margin=.06)
        obstacles = IndexedObstacles(box.polygon() for box in map_obstacles(scene))
        minimum = min(clearance(transform(footprint, pose), obstacles, boundary(scene), config.padding) for pose in state['samples'])
        assert minimum > config.padding, f'Hard clearance violation: {minimum}'
        result.update(status='PASS', robot_contacts=0, actual_total_path_m=state['path'],
                      minimum_measured_clearance_m=minimum, hard_padding_m=config.padding, elapsed_s=time.monotonic() - start)
        result['cases'].append(dict(name=name, status='PASS'))
    except Exception as error:
        result.update(error=str(error), robot_contacts=data['contacts'], actual_total_path_m=state['path'])
    finally:
        try:
            cancel()
            state['health'] = False
            spin(.15)
        except Exception:
            pass
        stop_group(source_process)
        for child in extra:stop_group(child)
        node.destroy_node()
        rclpy.shutdown()
        stop_group(process)
        log.close()
        timeline.close()
        (folder / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('all',) + CASES, default='heading')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'artifacts/intent-tasks')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('IGN_PARTITION') != 'wheelchair_intent_task_validation':
        parser.error('Requires ROS_DOMAIN_ID=96 IGN_PARTITION=wheelchair_intent_task_validation')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output.mkdir(parents=True, exist_ok=True)
    # Prevent accidental concurrent validators from sharing this isolated domain.
    import fcntl
    with open('/tmp/wheelchair_intent_task_validation.lock', 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('Another intent-task validator owns this domain/partition')
        results = [run_case(case, args.output) for case in (CASES if args.case == 'all' else (args.case,))]
    summary = dict(status='PASS' if all(r['status'] == 'PASS' for r in results) else 'FAIL', cases=results)
    (args.output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
