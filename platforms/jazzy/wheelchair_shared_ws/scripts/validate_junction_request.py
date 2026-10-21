#!/usr/bin/env python3
"""Exercise the real C++ service on an empty map using the recorded office poses.

This isolates the corridor contract; it does not prove office collision clearance.
Run in an isolated ROS domain with no existing path planner.
"""
import json
import subprocess
import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from wheelchair_interfaces.srv import ComputeDiscretePath


def main():
    rclpy.init();node=rclpy.create_node('junction_request_regression')
    pub=node.create_publisher(OccupancyGrid,'/wheelchair/local_costmap/costmap',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client=node.create_client(ComputeDiscretePath,'/wheelchair/compute_discrete_path')
    process=subprocess.Popen(['ros2','run','wheelchair_nav2','discrete_path_planner'],
        stdout=subprocess.DEVNULL,start_new_session=True)
    grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.width=160;grid.info.height=100
    grid.info.resolution=.05;grid.info.origin.position.x=-1.;grid.info.origin.position.y=-2.
    grid.info.origin.orientation.w=1.;grid.data=[0]*16000
    def publish():grid.header.stamp=node.get_clock().now().to_msg();pub.publish(grid)
    node.create_timer(.05,publish)
    def call(origin,goal,reason):
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
        req=ComputeDiscretePath.Request();req.header.frame_id='odom';req.header.stamp=node.get_clock().now().to_msg()
        req.task_id='office-regression';req.request_id=1
        req.start=Pose2D(x=2.5849175,y=.8221296,theta=.3567718)
        req.goals=[Pose2D(x=goal[0],y=goal[1],theta=0.)]
        req.corridor_origin=Pose2D(x=origin[0],y=origin[1],theta=origin[2])
        req.corridor_half_width=.25;req.max_path_length=4.;req.allow_detour=True
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
        assert future.done(),'service timeout'
        result=future.result()
        assert result.reason==reason,(reason,result.reason)
        assert result.success==(reason=='PLANNED')
        if result.success:
            assert result.path.poses
            assert all(abs(p.pose.position.y-1.05)<=.25+1e-9 for p in result.path.poses)
        print(json.dumps(dict(reason=result.reason,expansions=result.expansions,compute_ms=result.compute_ms)),flush=True)
    try:
        assert client.wait_for_service(timeout_sec=10.)
        call((2.5849175,.8221296,.3567718),(4.0656133,1.05),'GOAL_OUTSIDE_CORRIDOR')
        call((0.,1.05,0.),(4.0656133,1.05),'PLANNED')
        call((0.,1.05,0.),(4.0656133,1.4),'GOAL_OUTSIDE_CORRIDOR')
        call((0.,1.2,0.),(4.0656133,1.2),'START_OUTSIDE_CORRIDOR')
        # Fill the map to verify collision is not bypassed by the corrected axis.
        grid.data=[100]*16000
        call((0.,1.05,0.),(4.0656133,1.05),'START_COLLISION')
    finally:
        from process_cleanup import stop_group
        stop_group(process);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
