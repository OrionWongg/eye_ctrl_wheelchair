"""Exercise actual ROS subscriptions without sensors or actuator nodes."""
import time
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from wheelchair_hardware_bridge.cloud_to_scan import CloudToScan, _iter_xyz
from wheelchair_hardware_bridge.ground_filter import xyz_cloud


def test_only_filtered_clouds_reach_projection(monkeypatch):
    from wheelchair_hardware_bridge import cloud_to_scan as module
    rclpy.init()
    node=None
    try:
        node=CloudToScan()
        monkeypatch.setattr(module,'_cloud_to_base',lambda msg,*args:_iter_xyz(msg))
        output=[]
        node._pub=type('Recorder',(),{'publish':lambda self,msg:output.append(msg)})()
        pubs={topic:node.create_publisher(PointCloud2,topic,qos_profile_sensor_data)
              for topic in ('/mid360_201/points','/mid360_202/points',
                            '/mid360_201/points_filtered','/mid360_202/points_filtered')}
        def pump(duration):
            end=time.monotonic()+duration
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        pump(.3)
        def send(topic,points):
            pubs[topic].publish(xyz_cloud(np.array(points),node.get_clock().now().to_msg(),'base_link'))
            pump(.03)
        # Raw self return is outside the converter's old 0.55 m front mask.
        for number in ('201','202'):
            send(f'/mid360_{number}/points',[[.60,0.,-.10],[1.,0.,-.10]])
        assert all(value is None for value in node._clouds.values())
        assert output==[]
        send('/mid360_201/points_filtered',[[1.,0.,-.10]])
        assert output==[]  # Both filtered sensors are required; no raw fallback.
        send('/mid360_202/points_filtered',[[1.,0.,-.10]])
        assert output and min(output[-1].ranges)==1.
        assert output[-1].header.stamp==node._clouds['/mid360_201/points_filtered'].header.stamp
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


import pytest


@pytest.mark.parametrize('mode',['201','202'])
def test_single_lidar_produces_scan_without_other_sensor(monkeypatch,mode):
    from wheelchair_hardware_bridge import cloud_to_scan as module
    rclpy.init(args=['--ros-args','-p',f"lidar_mode:='{mode}'"])
    node=None
    try:
        node=CloudToScan()
        monkeypatch.setattr(module,'_cloud_to_base',lambda msg,*args:_iter_xyz(msg))
        output=[]
        node._pub=type('Recorder',(),{'publish':lambda self,msg:output.append(msg)})()
        topic=f'/mid360_{mode}/points_filtered'
        assert list(node._clouds)==[topic]
        msg=xyz_cloud(np.array([[1.,0.,-.1]]),node.get_clock().now().to_msg(),'base_link')
        node._on_cloud(topic,msg)
        assert len(output)==1 and min(output[0].ranges)==1.
        msg.header.stamp.sec-=1
        node._on_cloud(topic,msg)
        assert len(output)==1  # Selected sensor freshness remains mandatory.
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
