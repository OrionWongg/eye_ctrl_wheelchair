#!/usr/bin/env python3
"""Merge two MID360 PointCloud2 topics and slice a horizontal band into LaserScan.

Subscriptions:
  /mid360_201/points_filtered  (sensor_msgs/PointCloud2, frame mid360_201_local)
  /mid360_202/points_filtered  (sensor_msgs/PointCloud2, frame mid360_202_local)

Publication:
  /wheelchair/scan    (sensor_msgs/LaserScan, frame base_link)
  /wheelchair/ground_points (sensor_msgs/PointCloud2, diagnostic ground returns)

The node requires all selected clouds within a 0.3 s window (both by default), transforms each to
base_link via tf2, separates ground using a measured fixed ground reference
(or the legacy height slice when disabled), bins obstacle (x, y) angles and takes the
minimum range in each bin.  Points farther than range_max or closer than
range_min are discarded before binning.
"""
import math
from .ground_filter import classify_ground, xyz_cloud

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Duration
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _iter_xyz(cloud: PointCloud2):
    """Read XYZ as a vectorized array, respecting row stride and byte order."""
    fields = {f.name: f for f in cloud.fields}
    if not all(n in fields and fields[n].datatype == 7 for n in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                     'formats': [endian + 'f4'] * 3,
                     'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                     'itemsize': cloud.point_step})
    rows = np.ndarray((cloud.height, cloud.width), dtype=dtype,
                      buffer=cloud.data, strides=(cloud.row_step, cloud.point_step))
    xyz = np.stack([rows[n].ravel() for n in ('x', 'y', 'z')], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def _cloud_to_base(cloud: PointCloud2, tf_buffer: tf2_ros.Buffer,
                   target_frame: str, timeout_sec: float = 0.05):
    """Return (N,3) numpy array of points transformed into target_frame, or None."""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame, cloud.header.frame_id,
            cloud.header.stamp,
            timeout=Duration(seconds=timeout_sec))
    except (LookupException, ConnectivityException, ExtrapolationException):
        return None

    t = transform.transform.translation
    r = transform.transform.rotation
    # Rotation: quaternion -> 3x3
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float32)
    tv = np.array([t.x, t.y, t.z], dtype=np.float32)

    pts = np.asarray(_iter_xyz(cloud), dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return (pts @ R.T) + tv


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        # Parameters
        self.declare_parameter('cluster_filter_enabled', False)
        self._cluster_enabled = self.get_parameter('cluster_filter_enabled').value
        if self._cluster_enabled:
            self.get_logger().info('Single-frame cluster filter: radius 1 m, link 5 cm, min 4 points per sensor')
        self.declare_parameter('ground_filter_enabled', False)
        self.declare_parameter('ground_z', -0.456)
        self.declare_parameter('ground_tolerance', 0.03)
        self.declare_parameter('obstacle_height', 0.90)
        self.declare_parameter('height_min', -0.15)  # metres in base_link
        self.declare_parameter('height_max',  0.15)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max',  math.pi)
        self.declare_parameter('angular_resolution', 0.5)  # degrees
        self.declare_parameter('cloud_sync_window', 0.3)   # seconds
        self.declare_parameter('target_frame', 'base_link')

        self.declare_parameter('right_cloud_topic', '/mid360_201/points_filtered')
        self.declare_parameter('left_cloud_topic', '/mid360_202/points_filtered')
        right = self.get_parameter('right_cloud_topic').value
        left = self.get_parameter('left_cloud_topic').value
        if not right or not left or right == left:
            raise ValueError('two distinct non-empty cloud topics are required')
        self.declare_parameter('lidar_mode', 'both')
        mode = self.get_parameter('lidar_mode').value
        if mode not in ('both', '201', '202'):
            raise ValueError('lidar_mode must be both, 201 or 202')
        topics = [right, left] if mode == 'both' else [right if mode == '201' else left]
        self._clouds: dict[str, PointCloud2 | None] = dict.fromkeys(topics)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cloud_subs = [self.create_subscription(
            PointCloud2, topic, lambda m, topic=topic: self._on_cloud(topic, m), SENSOR_QOS)
            for topic in topics]
        self.get_logger().info(f'2D lidar_mode={mode}, inputs={topics}; no automatic fallback')

        if self.get_parameter('ground_filter_enabled').value:
            classify_ground(np.empty((0, 3)), self.get_parameter('ground_z').value,
                            self.get_parameter('ground_tolerance').value,
                            self.get_parameter('obstacle_height').value)
        self._ground_pub = self.create_publisher(PointCloud2, '/wheelchair/ground_points', SENSOR_QOS)
        self._pub = self.create_publisher(LaserScan, '/wheelchair/scan', SENSOR_QOS)

    def _on_cloud(self, topic: str, msg: PointCloud2):
        self._clouds[topic] = msg
        # Require current coverage from every explicitly selected sensor.
        self._try_publish()

    def _try_publish(self):
        p = self.get_parameter
        window = p('cloud_sync_window').get_parameter_value().double_value
        target = p('target_frame').get_parameter_value().string_value
        h_min = p('height_min').get_parameter_value().double_value
        h_max = p('height_max').get_parameter_value().double_value
        a_min = p('angle_min').get_parameter_value().double_value
        a_max = p('angle_max').get_parameter_value().double_value
        res_deg = p('angular_resolution').get_parameter_value().double_value
        res = math.radians(res_deg)

        # A missing/stale sensor invalidates the merged observation.
        now = self.get_clock().now()
        all_pts = []
        latest_stamp = None
        for cloud in self._clouds.values():
            if cloud is None:
                return
            stamp = rclpy.time.Time.from_msg(cloud.header.stamp)
            age = (now - stamp).nanoseconds * 1e-9
            if age < -0.05 or age > window:
                return
            pts = _cloud_to_base(cloud, self._tf_buffer, target)
            if pts is not None and len(pts) > 0:
                all_pts.append(pts)
            else:
                return
            # Use the oldest contributing observation; never hide stale data.
            if latest_stamp is None or stamp < latest_stamp:
                latest_stamp = stamp

        if not all_pts or latest_stamp is None:
            return

        obstacle_parts, ground_parts, suppressed_parts = [], [], []
        for pts in all_pts:
            body = ((pts[:, 0] >= -0.42) & (pts[:, 0] <= 0.53) &
                    (pts[:, 1] >= -0.275) & (pts[:, 1] <= 0.275))
            pts = pts[~body]
            if p('ground_filter_enabled').value:
                ground, pts = classify_ground(
                    pts, p('ground_z').value, p('ground_tolerance').value,
                    p('obstacle_height').value)
                ground_parts.append(ground)
            else:
                pts = pts[(pts[:, 2] >= h_min) & (pts[:, 2] <= h_max)]
            suppressed = np.zeros(len(pts), dtype=bool)
            if getattr(self, '_cluster_enabled', False):
                from .cluster_filter import small_cluster_mask
                suppressed = small_cluster_mask(pts)
            obstacle_parts.append(pts)
            suppressed_parts.append(suppressed)
        pts = np.concatenate(obstacle_parts, axis=0)
        suppressed = np.concatenate(suppressed_parts)
        if ground_parts:
            self._ground_pub.publish(xyz_cloud(
                np.concatenate(ground_parts), latest_stamp.to_msg(), target))

        # Compute ranges and angles
        ranges_2d = np.hypot(pts[:, 0], pts[:, 1])
        angles = np.arctan2(pts[:, 1], pts[:, 0])

        pts2 = ranges_2d
        ang2 = angles

        # Build LaserScan
        n_bins = max(1, round((a_max - a_min) / res))
        scan_ranges = np.full(n_bins, float('inf'), dtype=np.float32)

        bin_idx = np.floor((ang2 - a_min) / res).astype(int)
        valid_bins = (bin_idx >= 0) & (bin_idx < n_bins)
        np.minimum.at(scan_ranges, bin_idx[valid_bins],
                      pts2[valid_bins].astype(np.float32))

        # Do not expose a farther free-space ray behind a rejected small cluster.
        blocked_bins = np.floor((angles[suppressed] - a_min) / res).astype(int)
        blocked_bins = blocked_bins[(blocked_bins >= 0) & (blocked_bins < n_bins)]
        scan_ranges[blocked_bins] = float('inf')

        msg = LaserScan()
        msg.header.stamp = latest_stamp.to_msg()
        msg.header.frame_id = target
        msg.angle_min = float(a_min)
        msg.angle_max = float(a_min + (n_bins - 1) * res)
        msg.angle_increment = res
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        # No radial cutoff; retain finite LaserScan metadata for consumers.
        msg.range_min = 0.0
        # Stay below the generated ROS Python float32 bound after wire rounding.
        msg.range_max = 1.0e6
        msg.ranges = scan_ranges.tolist()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
