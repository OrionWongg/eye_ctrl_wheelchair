#!/usr/bin/env python3
"""Manually gated, passive five-direction recorder. No motion publishers."""
import argparse
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty
from datetime import datetime

DIRECTIONS = {'1': 'left', '2': 'forward_left', '3': 'forward', '4': 'forward_right', '5': 'right'}
TOPICS = {'/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
          '/odin1/odometry_highfreq': 'nav_msgs/msg/Odometry',
          '/wheelchair/odom': 'nav_msgs/msg/Odometry',
          '/odin1/imu': 'sensor_msgs/msg/Imu', '/wheelchair/imu': 'sensor_msgs/msg/Imu',
          **{f'/whill/controller/{n}': 'std_msgs/msg/String' for n in ['command', 'event', 'input', 'status']}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'bag' /
                        ('five_marked_' + datetime.now().strftime('%Y%m%d_%H%M%S')))
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Please run in an interactive terminal.')
    import rclpy
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('five_direction_data_recorder')
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root / 'notes.txt').write_text('Model=\nAPP speed setting=\nInput amplitude=\nLoad=\nSurface=\n')
    (root / 'session.json').write_text(json.dumps({'directions': DIRECTIONS, 'topics': TOPICS,
        'domain': os.getenv('ROS_DOMAIN_ID', '0'), 'tail_seconds': 3,
        'markers': 'Manual operator timestamps, not measured iPad touch timestamps'}, indent=2))
    writer = None
    active = None
    trial = 0
    tail_end = None
    static = {}
    seen = {}
    counts = {}
    events = (root / 'markers.jsonl').open('a', buffering=1)
    def mark(event):
        stamp = node.get_clock().now().nanoseconds
        data = json.dumps(dict(event=event, time_ns=stamp, direction=active, trial=trial))
        events.write(data + '\n'); events.flush(); os.fsync(events.fileno())
        if writer is not None:
            writer.write('/experiment/marker', serialize_message(String(data=data)), stamp)
    def receive(topic, msg):
        now = node.get_clock().now().nanoseconds
        seen[topic] = time.monotonic()
        if topic == '/tf_static':
            for tf in msg.transforms:
                static[(tf.header.frame_id, tf.child_frame_id)] = tf
        if writer is not None:
            writer.write(topic, serialize_message(msg), now)
            counts[topic] = counts.get(topic, 0) + 1
    for topic, typ in TOPICS.items():
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        if topic == '/tf_static':
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(get_message(typ), topic, lambda msg, t=topic: receive(t, msg), qos)
    old = termios.tcgetattr(sys.stdin)
    print(f'输出: {root}\n调整状态：不录数据，可以挪车。\n'
          '1左 2左前 3前进 4右前 5右：开始一段；空格/0：松开并录停车3秒；q：退出。\n'
          '按数字后看到 START 再操作iPad；工具不控制轮椅。', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.002)
            if tail_end is not None and time.monotonic() >= tail_end:
                mark('trial_complete')
                writer = None  # rosbag2 SequentialWriter destructor closes and writes metadata
                (root / f'{trial:03d}_{active}_counts.json').write_text(json.dumps(counts, indent=2))
                active = None; tail_end = None
                print('已保存。调整状态：不录数据，可以挪车。', flush=True)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = sys.stdin.read(1)
            if key == 'q':
                break
            if key in DIRECTIONS:
                if writer is not None:
                    print('当前段未结束；请先松开iPad并按空格，等待停车记录完成。', flush=True)
                    continue
                if any(time.monotonic() - seen.get(t, -1e9) > 1 for t in
                       ['/odin1/odometry_highfreq', '/wheelchair/odom']) or not static:
                    print('尚未收到新鲜里程计或静态TF，请等待传感器初始化。', flush=True)
                    continue
                trial += 1; active = DIRECTIONS[key]; counts = {}
                writer = rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=str(root / f'{trial:03d}_{active}'), storage_id='sqlite3'),
                            rosbag2_py.ConverterOptions('', ''))
                for t, typ in {**TOPICS, '/experiment/marker': 'std_msgs/msg/String'}.items():
                    writer.create_topic(rosbag2_py.TopicMetadata(name=t, type=typ, serialization_format='cdr'))
                tfmsg = get_message('tf2_msgs/msg/TFMessage')()
                tfmsg.transforms = list(static.values())
                writer.write('/tf_static', serialize_message(tfmsg), node.get_clock().now().nanoseconds)
                mark('start')
                print(f'START #{trial} {active}：正在录制，请操作iPad。', flush=True)
            elif key in (' ', '0') and writer is not None and tail_end is None:
                mark('release'); tail_end = time.monotonic() + 3
                print('松开标记；继续记录3秒停车过程，暂勿挪车。', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if writer is not None:
            mark('incomplete_trial'); writer = None
        events.close(); node.destroy_node(); rclpy.shutdown()
        print(f'已关闭录制：{root}', flush=True)

if __name__ == '__main__':
    main()
