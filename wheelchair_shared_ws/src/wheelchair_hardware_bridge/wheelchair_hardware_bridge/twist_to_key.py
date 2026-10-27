"""Forward each executor command to the opt-in calibrated iPad drag interface.

The controller owns manual arming, APP homing, press/drag/release and the 200 ms
receive watchdog. Do not deduplicate repeated commands: they are the lease.
The historical executable name is retained for existing launch files.

The executor's LEFT/RIGHT twist carries the ICC arc (v = icc_x * w) for the
simulator, but the iPad controller only accepts a pure key and disarms when
|v| and |w| both exceed 0.05. A turn is therefore sent as (0, w); pressing
A/D on the real WHILL produces the arc physically.
"""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped


def to_ipad_twist(msg, angular_threshold):
    """Return the single-key twist the iPad controller accepts."""
    out = deepcopy(msg)
    if abs(out.angular.z) > angular_threshold:
        out.linear.x = 0.
    return out


class TwistToKey(Node):
    def __init__(self):
        super().__init__('twist_to_key')
        src=self.declare_parameter('source_topic','/wheelchair/sim/drive').value
        tgt=self.declare_parameter('target_topic','/whill/controller/navigation').value
        self.declare_parameter('linear_threshold',.05)
        self.angular_threshold=float(self.declare_parameter('angular_threshold',.05).value)
        self.pub=self.create_publisher(TwistStamped,tgt,1)
        self.create_subscription(Twist,src,self.on_twist,qos_profile_sensor_data)
        self.get_logger().info(f'Executor {src} → calibrated iPad drag {tgt}; local Arm required')

    def on_twist(self,msg):
        out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg()
        out.header.frame_id='base_link';out.twist=to_ipad_twist(msg,self.angular_threshold)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args);node=TwistToKey()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
