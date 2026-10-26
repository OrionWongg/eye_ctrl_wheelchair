import math
from types import SimpleNamespace
import numpy as np
import pytest
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from wheelchair_hardware_bridge.ground_filter import classify_ground, xyz_cloud
from wheelchair_hardware_bridge.cloud_to_scan import _iter_xyz


def test_floor_noise_low_obstacle_and_below_floor():
    z=-.456
    points=np.array([[1,0,z],[1,1,z+.02],[1,-1,z-.02],
                     [2,0,z+.05],[2,1,z+.15],[2,-1,z+.24],
                     [3,0,z-.10],[3,1,z+1.2],[math.nan,0,z]])
    ground, obstacles=classify_ground(points,z,.03,.9)
    assert np.array_equal(ground,points[:3])
    assert np.array_equal(obstacles,points[3:7])


@pytest.mark.parametrize('tolerance,height',[(0,.9),(-.01,.9),(.9,.9),(.03,math.nan)])
def test_invalid_ground_configuration_rejected(tolerance,height):
    with pytest.raises(ValueError):classify_ground(np.empty((0,3)),-.456,tolerance,height)


def test_ground_cloud_roundtrip_and_empty():
    stamp=Time(seconds=10).to_msg()
    for points in (np.empty((0,3)),np.array([[1,2,-.456],[3,4,-.45]])):
        msg=xyz_cloud(points,stamp,'base_link')
        assert msg.header.stamp==stamp and msg.header.frame_id=='base_link'
        assert np.allclose(_iter_xyz(msg),points)


@pytest.mark.parametrize('only_ground',[False,True])
def test_real_scan_adapter_ground_separation(monkeypatch,only_ground):
    from wheelchair_hardware_bridge import cloud_to_scan as module
    values=dict(ground_filter_enabled=True,ground_z=-.456,ground_tolerance=.03,
                obstacle_height=.9,height_min=-.203,height_max=.397,
                cloud_sync_window=.3,target_frame='base_link',range_min=.15,
                range_max=20.,angle_min=-math.pi,angle_max=math.pi,angular_resolution=.5)
    a=PointCloud2();a.header.stamp=Time(seconds=9.9).to_msg()
    b=PointCloud2();b.header.stamp=Time(seconds=9.95).to_msg()
    scans=[];ground=[]
    node=SimpleNamespace(get_parameter=lambda name:Parameter(name,value=values[name]),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10,clock_type=ClockType.ROS_TIME)),
        _clouds={'a':a,'b':b},_tf_buffer=None,
        _pub=SimpleNamespace(publish=scans.append),_ground_pub=SimpleNamespace(publish=ground.append))
    points=[[1,0,-.456],[1,.5,-.446]]
    if not only_ground:
        points += [[2,0,-.406],[3,0,-.1]]  # 5 cm obstacle wins over taller distant one.
    monkeypatch.setattr(module,'_cloud_to_base',lambda *args:np.array(points))
    module.CloudToScan._try_publish(node)
    assert len(scans)==len(ground)==1
    assert scans[0].header.stamp==ground[0].header.stamp==a.header.stamp
    assert len(_iter_xyz(ground[0]))==4
    if only_ground:
        assert all(math.isinf(r) for r in scans[0].ranges)
    else:
        assert scans[0].ranges[360]==pytest.approx(2.)
        assert sum(math.isfinite(r) for r in scans[0].ranges)==1
        from wheelchair_hardware_bridge.scan_map import rasterize
        scan=scans[0]
        cells,origin=rasterize(scan.ranges,scan.angle_min,scan.angle_increment,
                               scan.range_min,scan.range_max,(0,0,0),.05,10.)
        occupied=np.argwhere(cells==100)
        assert len(occupied)==1
        row,col=occupied[0]
        assert origin[0]+col*.05==pytest.approx(2.,abs=.051)
        assert origin[1]+row*.05==pytest.approx(0.,abs=.051)
