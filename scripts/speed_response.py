#!/usr/bin/env python3
"""Passive discovery and offline five-direction wheelchair response analysis."""
import argparse
import bisect
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics


DIRECTIONS = ('up', 'up-left', 'left', 'up-right', 'right')


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    lo = int(position)
    return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (position - lo)


def segments(commands):
    trials, active, held = [], None, False
    for command in sorted(commands, key=lambda row: row['time_ns']):
        stamp = command['time_ns'] / 1e9
        action = command.get('action')
        if action in ('keyboard_drag_begin', 'eye_drag_begin'):
            held = True
        elif action == 'app_target_move':
            direction = command.get('direction')
            if direction not in DIRECTIONS:
                if active:
                    active.update(end=stamp, release=None, end_reason='target_changed')
                    trials.append(active)
                active = None
                continue
            if active and active['direction'] == direction:
                continue
            if active:
                active.update(end=stamp, release=None, end_reason='direction_changed_without_release')
                trials.append(active)
            active = dict(start=stamp, direction=direction,
                          held_at_start=held, target=command.get('target'))
        elif action in ('keyboard_drag_end', 'keyboard_disarm_button_release',
                        'eye_drag_end', 'eye_move_failed_release'):
            held = False
            if active:
                active.update(end=stamp, release=stamp, end_reason=action)
                trials.append(active)
                active = None
    if active:
        active.update(end=None, release=None, end_reason='recording_ended_while_held')
        trials.append(active)
    return trials


def timing(rows):
    gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    positive = [gap for gap in gaps if gap > 0]
    return dict(samples=len(rows), duplicate_or_backward=sum(gap <= 0 for gap in gaps),
                median_hz=1 / statistics.median(positive) if positive else None,
                p95_gap_s=percentile(positive, .95), max_gap_s=max(positive, default=None),
                receive_age_p95_s=percentile([row[7] for row in rows], .95))


def first_hold(rows, predicate, duration):
    start, previous, count = None, None, 0
    for row in rows:
        if not predicate(row) or (previous is not None and row[0] - previous > .15):
            start, count = None, 0
        if predicate(row):
            if start is None:
                start = row[0]
            count += 1
            if count >= 3 and row[0] - start >= duration:
                return start
        previous = row[0]
    return None


def stationary(row):
    return abs(row[4]) < .015 and abs(row[5]) < .015 and abs(row[6]) < .025


def evaluate(trial, rows, next_start=None):
    start, release = trial['start'], trial['release']
    stamps = [row[0] for row in rows]

    def between(lo, hi):
        return rows[bisect.bisect_left(stamps, lo):bisect.bisect_right(stamps, hi)]

    end = trial['end'] if trial['end'] is not None else (rows[-1][0] if rows else start)
    during = between(start, end)
    baseline = between(start - .4, start - .05)
    clean = (len(baseline) >= 3 and all(stationary(row) for row in baseline)
             and baseline[0][0] <= start - .3 and baseline[-1][0] >= start - .1
             and all(b[0] - a[0] <= .15 for a, b in zip(baseline, baseline[1:])))
    direction = trial['direction']
    sign = 1 if direction in ('left', 'up-left') else -1
    moving = ((lambda row: row[4] > .02) if direction == 'up' else
              (lambda row: row[6] * sign > .05))
    onset = first_hold(during, moving, .1)
    stop_end = min(next_start if next_start is not None else math.inf,
                   rows[-1][0] if rows else end)
    stop = first_hold(between(release, stop_end), stationary, .3) if release else None
    result = dict(direction=direction, target=trial.get('target'), start_s=start,
                  hold_s=end - start, released=release is not None,
                  held_at_start=trial['held_at_start'], start_stopped=clean,
                  end_reason=trial['end_reason'], onset_delay_s=onset - start if onset else None,
                  release_to_stop_s=stop - release if stop is not None else None,
                  samples=len(during), plateau_candidate=False, steady_vx_mps=None,
                  steady_vy_mps=None, steady_w_radps=None, steady_speed_mps=None,
                  plateau_start_s=None, plateau_duration_s=None)
    for column, name in ((4, 'vx_mps'), (5, 'vy_mps'), (6, 'w_radps')):
        values = [row[column] for row in during]
        result['peak_abs_' + name] = max(map(abs, values), default=None)
        result['p95_abs_' + name] = percentile(list(map(abs, values)), .95)
    result['peak_speed_mps'] = max((math.hypot(row[4], row[5]) for row in during), default=None)
    result['max_gap_s'] = max((b[0] - a[0] for a, b in zip(during, during[1:])), default=None)
    # Exploratory 1 s plateau: stable distribution and first/second-half means.
    # These tolerances describe identification quality, not collision limits.
    cursor = start + .5
    while cursor + 1. <= end:
        window = between(cursor, cursor + 1.)
        if (len(window) >= 10 and window[-1][0] - window[0][0] >= .9
                and max((b[0] - a[0] for a, b in zip(window, window[1:])), default=1.) <= .15
                and moving(window[len(window) // 2])):
            stable = True
            means = []
            for column, floor in ((4, .02), (5, .02), (6, .05)):
                values = [row[column] for row in window]
                center = statistics.median(values)
                halves = [statistics.mean(values[:len(values)//2]), statistics.mean(values[len(values)//2:])]
                stable &= (percentile(values, .9) - percentile(values, .1) <= max(floor, .2 * abs(center))
                           and abs(halves[1] - halves[0]) <= max(floor / 2, .1 * abs(center)))
                means.append(center)
            if stable:
                result.update(plateau_candidate=True, steady_vx_mps=means[0], steady_vy_mps=means[1],
                              steady_w_radps=means[2], steady_speed_mps=math.hypot(*means[:2]),
                              plateau_start_s=cursor, plateau_duration_s=1.)
        cursor += .25
    return result


def probe():
    import time
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    from rclpy.type_support import check_for_type_support
    from rcl_interfaces.srv import GetParameters
    for name in ('nav_msgs/msg/Odometry', 'sensor_msgs/msg/Imu', 'std_msgs/msg/String', 'tf2_msgs/msg/TFMessage'):
        check_for_type_support(get_message(name))
    rclpy.init()
    node = rclpy.create_node('speed_response_recorder_probe', enable_rosout=False, start_parameter_services=False)
    until = time.monotonic() + 2.
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    graph = {name: types for name, types in node.get_topic_names_and_types()}
    required = {'/wheelchair/odom': 'nav_msgs/msg/Odometry', '/whill/controller/command': 'std_msgs/msg/String'}
    publishers = {name: node.count_publishers(name) for name in required}
    ready = all(publishers[name] > 0 and graph.get(name) == [kind] for name, kind in required.items())
    active_calibration, calibration_sha256 = None, None
    client = node.create_client(GetParameters, '/whill_controller/get_parameters')
    if publishers['/whill/controller/command'] and client.wait_for_service(timeout_sec=.5):
        request = GetParameters.Request()
        request.names = ['calibration_file']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
        if future.done() and future.result() and future.result().values:
            active_calibration = future.result().values[0].string_value
            if not active_calibration:
                from ament_index_python.packages import get_package_share_directory
                active_calibration = str(Path(get_package_share_directory('whill_controller')) / 'config/ipad_calibration_20261009.json')
            path = Path(active_calibration).expanduser()
            if path.is_file():
                active_calibration = str(path.resolve())
                calibration_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
    print(json.dumps(dict(ready=ready, publishers=publishers, graph=graph,
                         active_calibration=active_calibration,
                         calibration_sha256=calibration_sha256), indent=2))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ready else 3


def analyze(bag, output):
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=info['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    commands, odom, invalid_odom = [], [], 0
    counts = collections.Counter()
    while reader.has_next():
        topic, raw, receipt = reader.read_next()
        counts[topic] += 1
        if topic not in ('/wheelchair/odom', '/whill/controller/command'):
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic.endswith('/command'):
            try:
                command = json.loads(msg.data)
                if isinstance(command.get('time_ns'), (int, float)):
                    commands.append(command)
            except (ValueError, TypeError):
                pass
        else:
            p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            values = (stamp, p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                      twist.linear.x, twist.linear.y, twist.angular.z, receipt / 1e9 - stamp)
            if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                    and all(math.isfinite(value) for value in values)):
                odom.append(values)
            else:
                invalid_odom += 1
    if not odom or not commands:
        raise ValueError('Need actual /wheelchair/odom and /whill/controller/command messages in the bag')
    health = timing(odom)
    unique = {row[0]: row for row in odom}
    rows = [unique[stamp] for stamp in sorted(unique)]
    trials = segments(commands)
    results = [dict(trial=index+1, **evaluate(trial, rows, trials[index+1]['start'] if index+1 < len(trials) else None))
               for index, trial in enumerate(trials)]
    summary = {}
    for direction in DIRECTIONS:
        group = [row for row in results if row['direction'] == direction]
        accepted = [row for row in group if row['plateau_candidate'] and row['start_stopped']
                    and row['released'] and row['release_to_stop_s'] is not None and row['held_at_start']]
        summary[direction] = dict(trials=len(group), clean_plateau_trials=len(accepted),
                                  observed_peak_vx_abs_mps=max((r['peak_abs_vx_mps'] for r in group if r['peak_abs_vx_mps'] is not None), default=None),
                                  observed_peak_w_abs_radps=max((r['peak_abs_w_radps'] for r in group if r['peak_abs_w_radps'] is not None), default=None))
        for key in ('steady_vx_mps', 'steady_vy_mps', 'steady_w_radps'):
            summary[direction][key + '_median'] = statistics.median([r[key] for r in accepted]) if accepted else None
    output.mkdir(parents=True, exist_ok=False)
    with (output / 'trials.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]) if results else ['trial'])
        writer.writeheader()
        writer.writerows(results)
    report = dict(bag=str(bag), odom_health=health, invalid_odom=invalid_odom,
                  counts=dict(counts), directions=summary,
                  note='Observed maxima are not validated actuator limits. Plateau tests are exploratory; no target-speed tracking accuracy is claimed.')
    (output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    sub.add_parser('probe')
    command = sub.add_parser('analyze')
    command.add_argument('bag', type=Path)
    command.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.mode == 'probe':
        return probe()
    analyze(args.bag.resolve(), (args.output or Path(str(args.bag) + '_analysis')).resolve())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
