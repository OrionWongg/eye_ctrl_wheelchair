"""Forward each executor command to the opt-in calibrated iPad drag interface.

The controller owns manual arming, APP homing, press/drag/release and the 200 ms
receive watchdog. Do not deduplicate repeated commands: they are the lease.
The historical executable name is retained for existing launch files.
"""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped


class TwistToKey(Node):
    def __init__(self):
        super().__init__('twist_to_key')
        src=self.declare_parameter('source_topic','/wheelchair/sim/drive').value
        tgt=self.declare_parameter('target_topic','/whill/controller/navigation').value
        self.pub=self.create_publisher(TwistStamped,tgt,1)
        self.create_subscription(Twist,src,self.on_twist,qos_profile_sensor_data)
        self.get_logger().info(f'Executor {src} → calibrated iPad drag {tgt}; local Arm required')

    def on_twist(self,msg):
        out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg()
        out.header.frame_id='base_link';out.twist=deepcopy(msg)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args);node=TwistToKey()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
