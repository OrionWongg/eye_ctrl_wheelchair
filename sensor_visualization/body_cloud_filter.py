#!/usr/bin/env python3
"""Remove a base_link-aligned XY body footprint, preserving remaining point records."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


POINTCLOUD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)

def filter_cloud(msg, transform, bounds):
    fields = {f.name: f for f in msg.fields}
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                     'formats': [endian + formats[fields[k].datatype] for k in ('x', 'y', 'z')],
                     'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                     'itemsize': msg.point_step})
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    xyz = np.stack([points[k].reshape(-1) for k in ('x', 'y', 'z')], axis=1)
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    s = 2.0 / norm
    rotation = np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]])
    t = transform.translation
    base = xyz @ rotation.T + np.array([t.x, t.y, t.z])
    xmin, xmax, ymin, ymax = bounds
    inside = ((base[:, 0] >= xmin) & (base[:, 0] <= xmax)
              & (base[:, 1] >= ymin) & (base[:, 1] <= ymax))
    # Drop invalid points too. Copy full records so intensity/time/tag remain intact.
    keep = np.isfinite(xyz).all(axis=1) & ~inside
    records = np.ndarray((msg.height, msg.width), dtype=np.dtype(f'V{msg.point_step}'),
                         buffer=msg.data, strides=(msg.row_step, msg.point_step))
    result = PointCloud2()
    result.header = copy.deepcopy(msg.header)
    result.height = 1
    result.width = int(keep.sum())
    result.fields = copy.deepcopy(msg.fields)
    result.is_bigendian = msg.is_bigendian
    result.point_step = msg.point_step
    result.row_step = result.width * result.point_step
    result.data = records.reshape(-1)[keep].tobytes()
    result.is_dense = True
    return result


class BodyCloudFilter(Node):
    def __init__(self):
        super().__init__('body_cloud_filter')
        self.declare_parameter('body_frame', 'base_link')
        self.frame = self.get_parameter('body_frame').value
        defaults = {'x_min': -0.42, 'x_max': 0.63, 'y_min': -0.40, 'y_max': 0.40}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.bounds = tuple(self.get_parameter(k).value for k in defaults)
        if self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]:
            raise ValueError('Body footprint min must be less than max')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_ = []
        self.publishers_ = []
        for number in ('201', '202'):
            topic = f'/mid360_{number}/points'
            pub = self.create_publisher(PointCloud2, topic + '_filtered', POINTCLOUD_QOS)
            self.publishers_.append(pub)
            self.subscriptions_.append(self.create_subscription(
                PointCloud2, topic, lambda msg, pub=pub: self.process(msg, pub), POINTCLOUD_QOS))
        self.get_logger().info(f'Body mask in {self.frame}: XY={self.bounds}, all heights')

    def process(self, msg, publisher):
        try:
            transform = self.buffer.lookup_transform(
                self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            publisher.publish(filter_cloud(msg, transform.transform, self.bounds))
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for body TF; dropping cloud: {exc}', throttle_duration_sec=5.0)
        except (KeyError, ValueError, TypeError) as exc:
            self.get_logger().error(f'Unsupported point cloud: {exc}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = BodyCloudFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
