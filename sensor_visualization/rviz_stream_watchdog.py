#!/usr/bin/env python3
"""Read-only live source ages for RViz, independent of the navigation stack."""
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker,MarkerArray

SOURCES=[
    ('MID360 .201','/mid360_201/points',PointCloud2),
    ('MID360 .202','/mid360_202/points',PointCloud2),
    ('Filtered .201','/mid360_201/points_filtered',PointCloud2),
    ('Filtered .202','/mid360_202/points_filtered',PointCloud2),
    ('Odin odom','/odin1/odometry_highfreq',Odometry),
    ('Scan','/wheelchair/scan',LaserScan),
    ('Occupancy map','/wheelchair/planning_map',OccupancyGrid),
    ('Nav2 costmap','/wheelchair/local_costmap/costmap',OccupancyGrid),
]


def describe(last,now):
    if last is None:return 'WAITING',False
    gap=now-last
    return f'{"STALE" if gap>.8 else "LIVE"} {gap:.1f}s',gap>.8


class StreamWatchdog(Node):
    def __init__(self):
        super().__init__('rviz_stream_watchdog')
        self.last={};self.last_state={}
        self.pub=self.create_publisher(MarkerArray,'/wheelchair/display/stream_status',1)
        for label,topic,typ in SOURCES:
            self.create_subscription(typ,topic,lambda m,k=label:self.receive(k),
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(TFMessage,'/tf',self.transforms,qos_profile_sensor_data)
        self.create_timer(.2,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self,label):self.last[label]=time.monotonic()

    def transforms(self,msg):
        if any(t.header.frame_id=='odom' and t.child_frame_id in ('imu','base_link') for t in msg.transforms):
            self.receive('Odin TF')

    def publish(self):
        now=time.monotonic();lines=['DATA RECEIPT AGE (seconds)'];fault=False;waiting=False
        for label in [s[0] for s in SOURCES]+['Odin TF']:
            status,stale=describe(self.last.get(label),now)
            lines.append(f'{label}: {status}');fault|=stale;waiting|=(status=='WAITING')
            state=status.split()[0]
            previous=self.last_state.get(label)
            if previous!=state:
                if stale:self.get_logger().warning(f'{label} stopped updating: {status}')
                elif previous=='STALE':self.get_logger().info(f'{label} resumed')
                self.last_state[label]=state
        lines.append('If these numbers freeze too, check the RViz window.')
        marker=Marker();marker.header.frame_id='base_footprint'
        marker.ns='stream_status';marker.id=1;marker.type=Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w=1.;marker.pose.position.x=.7
        marker.pose.position.y=-1.0;marker.pose.position.z=1.4;marker.scale.z=.12
        marker.color.r=1. if fault or waiting else .3;marker.color.g=.2 if fault else .7 if waiting else 1.
        marker.color.b=.2 if fault else .7;marker.color.a=1.
        marker.text='\n'.join(lines);marker.lifetime.sec=1
        self.pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init();node=StreamWatchdog()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
