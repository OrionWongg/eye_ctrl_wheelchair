#!/usr/bin/env python3
"""Run against an isolated launch; never publishes motion. Exit nonzero on missing IO."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from wheelchair_interfaces.msg import ObstacleMap
from tf2_msgs.msg import TFMessage

def main():
    rclpy.init(); node=Node('simulation_smoke_reader')
    latest={}
    node.create_subscription(Odometry,'/wheelchair/odom',lambda m:latest.update(odom=m),10)
    node.create_subscription(LaserScan,'/wheelchair/scan',lambda m:latest.update(scan=m),qos_profile_sensor_data)
    node.create_subscription(ObstacleMap,'/wheelchair/obstacle_map',lambda m:latest.update(obstacles=m),10)
    limit=time.monotonic()+30
    while len(latest)<3 and time.monotonic()<limit:rclpy.spin_once(node,timeout_sec=.1)
    assert len(latest)==3,list(latest)
    assert latest['odom'].header.frame_id=='odom'
    assert latest['odom'].child_frame_id=='base_link'
    assert latest['scan'].header.frame_id=='base_scan'
    assert len(latest['scan'].ranges)==720
    assert latest['obstacles'].header.frame_id=='odom'
    assert latest['obstacles'].complete
    assert len(latest['obstacles'].obstacles)>=5
    print(json.dumps({'odom_frame':latest['odom'].header.frame_id,'odom_child':latest['odom'].child_frame_id,'scan_frame':latest['scan'].header.frame_id,'scan_count':len(latest['scan'].ranges),'map_obstacles':len(latest['obstacles'].obstacles),'map_sequence':latest['obstacles'].sequence,'success':True}))
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
