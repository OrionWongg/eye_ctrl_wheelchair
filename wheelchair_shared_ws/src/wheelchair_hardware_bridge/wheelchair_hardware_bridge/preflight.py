"""Read-only checks before starting a hardware keyboard perception stack."""
import time
import argparse
import math
import os

# Existing sensor drivers own raw clouds and mount TF. Derived topics must have
# exactly one owner, which the launcher is about to start.
DERIVED_TOPICS = (
    '/wheelchair/scan', '/wheelchair/planning_map', '/wheelchair/obstacle_map',
    '/wheelchair/odom', '/wheelchair/sim/drive', '/whill/controller/navigation',
)


def publisher_conflicts(node):
    conflicts = []
    for topic in DERIVED_TOPICS:
        publishers = node.get_publishers_info_by_topic(topic)
        if publishers:
            owners = ', '.join(f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in publishers)
            conflicts.append(f'{topic}: {len(publishers)} publisher(s): {owners}')
    return conflicts


def selected_mode(arguments):
    mode = 'both'
    for arg in arguments:
        if arg.startswith('lidar_mode:='):
            mode = arg.split(':=', 1)[1]
    if mode not in ('both', '201', '202'):
        raise ValueError('lidar_mode 必须为 both、201 或 202')
    return mode


def missing_inputs(messages, topics, now, transform_available):
    missing = []
    for topic in topics:
        msg = messages.get(topic)
        if msg is None:
            missing.append(f'{topic}: 尚未收到数据')
            continue
        stamp = msg.header.stamp
        age = now - (stamp.sec + stamp.nanosec / 1e9)
        if not math.isfinite(age) or not -.05 <= age < .3:
            missing.append(f'{topic}: 时间戳过期或非系统时钟，age={age:.3f}s')
            continue
        if topic == '/odin1/odometry_highfreq':
            if msg.header.frame_id != 'odom':
                missing.append(f'{topic}: 坐标系应为 odom，实际为 {msg.header.frame_id}')
            # Odin publishes constant mount extrinsics on /tf. Match OdomRelay:
            # query latest extrinsics, not a future high-rate odometry stamp.
            if not transform_available(msg.child_frame_id, 'base_link', None):
                missing.append(f'TF {msg.child_frame_id} -> base_link: 安装变换缺失')
        else:
            if not transform_available('base_link', msg.header.frame_id, stamp):
                missing.append(f'TF base_link -> {msg.header.frame_id}: 安装变换缺失')
            if not transform_available('odom', 'base_link', stamp):
                missing.append('TF odom -> base_link: 扫描时刻的变换尚未就绪')
    return list(dict.fromkeys(missing))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener, TransformException
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20.)
    parser.add_argument('launch_args', nargs='*')
    args = parser.parse_args()
    try:
        mode = selected_mode(args.launch_args)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be positive and finite')
    rclpy.init(args=[])
    node = Node('hardware_keyboard_preflight')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    sensors = ('201','202') if mode == 'both' else (mode,)
    topics = [f'/mid360_{number}/points_filtered' for number in sensors]
    topics.append('/odin1/odometry_highfreq')
    messages = {}
    subscriptions = [node.create_subscription(
        Odometry if topic == '/odin1/odometry_highfreq' else PointCloud2,
        topic, lambda msg, topic=topic: messages.__setitem__(topic, msg),
        qos_profile_sensor_data if topic == '/odin1/odometry_highfreq' else
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for topic in topics]
    def transform_available(target, source, stamp):
        if not target or not source:
            return False
        try:
            buffer.lookup_transform(target, source, Time() if stamp is None else Time.from_msg(stamp))
            return True
        except TransformException:
            return False
    try:
        # Discover existing derived publishers before allowing another stack.
        discovery = time.monotonic() + 2.
        while time.monotonic() < discovery:
            rclpy.spin_once(node, timeout_sec=.1)
        deadline = time.monotonic() + args.timeout
        next_notice = 0.
        while True:
            conflicts = publisher_conflicts(node)
            if conflicts:
                print('检测到已有控制/建图发布者，未启动新的硬件控制栈：', flush=True)
                for conflict in conflicts:
                    print(f'  {conflict}', flush=True)
                print('请先退出已有控制栈和独立 cloud_to_scan / scan_map；保留传感器驱动。', flush=True)
                return 1
            missing = missing_inputs(messages, topics,
                                     node.get_clock().now().nanoseconds / 1e9,
                                     transform_available)
            if not missing:
                print(f'感知就绪：MID360={mode}、Odin 里程计与扫描时刻 TF 均有效。', flush=True)
                return 0
            now = time.monotonic()
            if now >= next_notice:
                print('等待传感器就绪：' + '；'.join(missing), flush=True)
                next_notice = now + 5.
            if now >= deadline:
                print('启动已取消：未启动 costmap 或硬件输出。请先在另一终端运行并保持', flush=True)
                print(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')} ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '0')} ./scripts/start_sensors.sh --mid360 {mode}", flush=True)
                print('传感器与控制端的 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 需要一致。', flush=True)
                return 1
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
