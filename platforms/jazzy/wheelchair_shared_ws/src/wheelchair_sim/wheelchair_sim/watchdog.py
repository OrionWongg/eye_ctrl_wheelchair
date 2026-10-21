"""Independent simulation base liveness watchdog, on monotonic wall time."""
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
class Watchdog(Node):
    def __init__(self):
        super().__init__('simulation_drive_watchdog')
        self.last=None
        self.target=Twist()
        self.pub=self.create_publisher(Twist,'/wheelchair/sim/cmd_vel',1)
        self.create_subscription(Twist,'/wheelchair/sim/drive',self.receive,1)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def receive(self,msg):
        self.last=time.monotonic(); self.target=msg
    def tick(self):
        self.pub.publish(self.target if self.last is not None and time.monotonic()-self.last<.20 else Twist())
def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node=Watchdog()
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt: pass
    finally: node.pub.publish(Twist()); node.destroy_node(); rclpy.try_shutdown()
