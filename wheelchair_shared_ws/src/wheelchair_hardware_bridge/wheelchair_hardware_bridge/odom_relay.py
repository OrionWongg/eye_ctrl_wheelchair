"""Odin system-time odometry/IMU adapter; converts IMU-origin odometry to base_link.

The Odin driver must use use_host_ros_time=1, including its TF publisher.
Original timestamps are retained so transport delays remain observable.
"""
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener, TransformException


def rotation(q):
    x, y, z, w = q
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid quaternion')
    x, y, z, w = np.asarray(q) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def multiply(a, b):
    x,y,z,w=a; X,Y,Z,W=b
    q=np.array([w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,
                w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z])
    return q/np.linalg.norm(q)


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def xyz(v):return np.array([v.x,v.y,v.z])
def quat(q):return np.array([q.x,q.y,q.z,q.w])
def assign(v, values):v.x,v.y,v.z=map(float,values)


def convert_odom(msg, transform):
    """transform is T_child_base; use the lever arm for pose, twist, covariance."""
    out=deepcopy(msg)
    offset=xyz(transform.translation); q_cb=quat(transform.rotation)
    r_cb=rotation(q_cb); q_oc=quat(msg.pose.pose.orientation); r_oc=rotation(q_oc)
    assign(out.pose.pose.position,xyz(msg.pose.pose.position)+r_oc@offset)
    q=multiply(q_oc,q_cb)
    out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
    assign(out.twist.twist.linear,r_cb.T@(xyz(msg.twist.twist.linear)+np.cross(xyz(msg.twist.twist.angular),offset)))
    assign(out.twist.twist.angular,r_cb.T@xyz(msg.twist.twist.angular))
    j=np.eye(6); j[:3,3:]=-skew(r_oc@offset)
    out.pose.covariance=(j@np.array(msg.pose.covariance).reshape(6,6)@j.T).ravel().tolist()
    j=np.zeros((6,6));j[:3,:3]=j[3:,3:]=r_cb.T;j[:3,3:]=-r_cb.T@skew(offset)
    out.twist.covariance=(j@np.array(msg.twist.covariance).reshape(6,6)@j.T).ravel().tolist()
    return out


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        for name, value in [('source_topic','/odin1/odometry_highfreq'),
                            ('target_topic','/wheelchair/odom'),('child_frame_id','base_link'),
                            ('imu_source_topic','/odin1/imu'),('imu_target_topic','/wheelchair/imu')]:
            self.declare_parameter(name,value)
        p=lambda name:self.get_parameter(name).value
        self.child=p('child_frame_id');self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(Odometry,p('target_topic'),10)
        self.imu_pub=self.create_publisher(Imu,p('imu_target_topic'),10)
        self.create_subscription(Odometry,p('source_topic'),self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Imu,p('imu_source_topic'),self.on_imu,qos_profile_sensor_data)

    def fresh(self,msg):
        age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if not -.05<=age<=.3:
            self.get_logger().warning('Odin timestamp is not current system time. Set use_host_ros_time: 1 and restart start_sensors.sh.',throttle_duration_sec=5.)
            return False
        return True

    def on_odom(self,msg):
        if not self.fresh(msg) or msg.header.frame_id!='odom':return
        try:
            # Mounting extrinsics are constant even if Odin publishes them on /tf.
            t=self.tf.lookup_transform(msg.child_frame_id,self.child,Time())
            out=convert_odom(msg,t.transform);out.child_frame_id=self.child
            self.pub.publish(out)
        except (TransformException,ValueError) as exc:
            self.get_logger().warning(f'Waiting for valid Odin-to-base extrinsics: {exc}',throttle_duration_sec=5.)

    def on_imu(self,msg):
        if not self.fresh(msg):return
        out=deepcopy(msg)
        # Odin publishes acceleration/gyro; the identity quaternion is not an attitude estimate.
        out.orientation_covariance[0]=-1.
        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args);node=OdomRelay()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
