"""Keep ROS simulation time current while a control callback is computing.

Humble's TimeSource normally subscribes in the node's default callback group.
Only the existing clock source moves to this executor; safety state and sensor
callbacks remain serialized on the control node. No clock extrapolation occurs.
"""
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException


class ClockWorker:
    def __init__(self,node):
        self.node=None
        if not node.get_parameter('use_sim_time').value:return
        self.source=node._time_source  # Humble exposes TimeSource only here.
        self.node=Node(node.get_name()+'_clock',context=node.context,use_global_arguments=False,
            parameter_overrides=[Parameter('use_sim_time',value=False)],
            start_parameter_services=False,enable_rosout=False)
        self.source.attach_node(self.node)
        # The helper's own clock stays on wall time, avoiding a second /clock
        # subscription. Activate the transferred source before control starts.
        self.source.ros_time_is_active=True
        self.executor=SingleThreadedExecutor(context=node.context)
        self.executor.add_node(self.node)
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,name='ros-clock',daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set() and rclpy.ok(context=self.node.context):
                self.executor.spin_once(timeout_sec=.01)
                # Gazebo publishes at physics rate. Consume the latest sample
                # at up to 200 Hz instead of competing for the GIL at 1 kHz.
                self.stop.wait(.005)
        except ExternalShutdownException:pass

    def close(self):
        if self.node is None:return
        self.stop.set();self.executor.wake();self.thread.join()
        self.executor.shutdown()
        self.source.detach_node()
        self.node.destroy_node();self.node=None
