import time
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from wheelchair_control.clock_worker import ClockWorker


def test_clock_updates_without_spinning_control_node_and_does_not_extrapolate():
    context=Context();rclpy.init(context=context)
    node=Node('clock_worker_test',context=context,use_global_arguments=False,
              parameter_overrides=[Parameter('use_sim_time',value=True)])
    producer=Node('clock_worker_source',context=context,use_global_arguments=False)
    pub=producer.create_publisher(Clock,'/clock',1);worker=ClockWorker(node)
    try:
        # The control node is deliberately never spun, as with a busy callback.
        for seconds in (1,2):
            msg=Clock();msg.clock.sec=seconds;deadline=time.monotonic()+3
            while node.get_clock().now().nanoseconds!=seconds*10**9 and time.monotonic()<deadline:
                pub.publish(msg);time.sleep(.01)
            assert node.get_clock().now().nanoseconds==seconds*10**9
        time.sleep(.05)
        assert node.get_clock().now().nanoseconds==2*10**9
    finally:
        worker.close();producer.destroy_node();node.destroy_node();rclpy.shutdown(context=context)
