import math
from types import SimpleNamespace
import numpy as np
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from wheelchair_hardware_bridge.cluster_filter import small_cluster_mask


def test_empty_small_chain_and_boundary():
    assert small_cluster_mask(np.empty((0,3))).size == 0
    p=np.array([[.7,0,0],[.73,0,0],[.76,0,0]])
    assert small_cluster_mask(p).all()
    assert not small_cluster_mask(np.r_[p,[[.79,0,0]]]).any()  # Chained 4-point cluster.
    assert not small_cluster_mask(np.array([[1.1,0,0]])).any()
    p=np.array([[.99,0,0],[1.01,0,0],[1.02,0,0],[1.03,0,0]])
    assert not small_cluster_mask(p).any()  # Margin supports boundary cluster.


def test_three_dimensional_separation():
    p=np.array([[.7,0,0],[.7,0,.06],[.7,0,.12],[.7,0,.18]])
    assert small_cluster_mask(p).all()


def test_adapter_per_sensor_and_no_free_ray_behind_rejected_cluster(monkeypatch):
    from wheelchair_hardware_bridge import cloud_to_scan as module
    values=dict(ground_filter_enabled=False,height_min=-.2,height_max=.2,
                cloud_sync_window=.3,target_frame='base_link',range_min=.15,
                range_max=20.,angle_min=-math.pi,angle_max=math.pi,angular_resolution=.5)
    a=PointCloud2();a.header.stamp=Time(seconds=9.9).to_msg()
    b=PointCloud2();b.header.stamp=Time(seconds=9.95).to_msg()
    scans=[]
    node=SimpleNamespace(get_parameter=lambda name:Parameter(name,value=values[name]),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10,clock_type=ClockType.ROS_TIME)),
        _clouds={'a':a,'b':b},_tf_buffer=None,_cluster_enabled=True,
        _pub=SimpleNamespace(publish=scans.append))
    # Each sensor has only two points: merging first would incorrectly retain four.
    # Far return in same bin must not clear the rejected near cluster's shadow.
    points=np.array([[.7,0,0],[.71,0,0],[2.,0,0],
                     [0,.7,0],[0,.71,0],[0,.72,0],[0,.73,0]])
    monkeypatch.setattr(module,'_cloud_to_base',lambda *args:points)
    module.CloudToScan._try_publish(node)
    assert len(scans)==1
    assert scans[0].header.stamp==a.header.stamp
    assert math.isinf(scans[0].ranges[360])
    assert scans[0].ranges[540] < .8
    # No warm-up or frame history: repeated callbacks give the same result.
    module.CloudToScan._try_publish(node)
    assert scans[1].ranges == scans[0].ranges
