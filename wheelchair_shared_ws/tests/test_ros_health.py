import math,time
import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from wheelchair_interfaces.msg import ControlIntent,ExecutionStatus
from wheelchair_control.nodes import PlannerNode

def test_malformed_inputs_cannot_preserve_authorization():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=PlannerNode()
    try:
        node.core.latch=False
        msg=ControlIntent();msg.header.frame_id='camera_frame';msg.command=1;msg.enabled=True
        node.on_intent(msg)
        assert node.core.latch and node.core.reason=='INVALID_INTENT_FRAME'
        scan=LaserScan();scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360
        scan.range_min=.01;scan.range_max=10.;scan.ranges=[5.]*360
        node.on_scan(scan);assert node.scan_valid
        scan.ranges[30]=math.nan;node.on_scan(scan);assert not node.scan_valid
        scan.ranges=[5.]*90;node.on_scan(scan);assert not node.scan_valid
        node.odom_received=time.monotonic()
        odom=Odometry();odom.header.frame_id='wrong';node.on_odom(odom)
        assert not math.isfinite(node.odom_received)
        node.exec_guard.deadline=time.monotonic()+10
        status=ExecutionStatus();status.executed_action=99
        node.on_executor(status)
        assert not node.exec_guard.fresh(time.monotonic())
        assert node.core.latch and node.core.reason=='EXECUTOR_INVALID'
    finally:
        node.destroy_node();rclpy.shutdown()


def test_static_map_cache_still_rejects_invalid_and_accepts_changed_geometry():
    from geometry_msgs.msg import Point32,Polygon
    from wheelchair_interfaces.msg import ObstacleMap
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=PlannerNode()
    def polygon(points):return Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in points])
    msg=ObstacleMap();msg.header.frame_id='odom';msg.session_id='map-test';msg.complete=True
    msg.known_free_boundary=polygon([(-10,-10),(10,-10),(10,10),(-10,10)])
    msg.obstacles=[polygon([(3,0),(4,0),(4,1),(3,1)])]
    def deliver():
        msg.sequence+=1;msg.header.stamp=node.get_clock().now().to_msg();node.on_map(msg)
    try:
        deliver();assert node.map_valid
        cached=node.obstacles;deliver();assert node.obstacles is cached
        msg.obstacles=[Polygon()];deliver();assert not node.map_valid
        msg.obstacles=[polygon([(1,0),(2,0),(2,1),(1,1)])]
        deliver();assert node.map_valid and node.obstacles[0][0]==(1.,0.)
    finally:
        node.destroy_node();rclpy.shutdown()

def test_planning_deadline_never_publishes_late_motion():
    from wheelchair_control.planner import Decision
    from wheelchair_control.protocol import FORWARD,STOP
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=PlannerNode()
    sent=[]
    class Output:
        def publish(self,msg):sent.append(msg)
    try:
        node.actions=Output();node.fresh=lambda *args:True
        node.scan_valid=node.map_valid=True
        now=time.monotonic()
        node.odom_received=node.scan_received=node.map_received=now
        for guard in (node.map_guard,node.intent_guard,node.exec_guard):guard.deadline=now+10
        node.exec_status=ExecutionStatus();node.exec_status.stopped=True
        def slow(*args,**kwargs):
            time.sleep(.065)
            return Decision(FORWARD,'PASS','USER_REQUEST')
        node.core.tick=slow
        node.tick()
        assert sent[-1].action==STOP and sent[-1].reason=='PLANNING_TIMEOUT_FAULT'
        assert node.core.latch
    finally:
        node.destroy_node();rclpy.shutdown()
