from types import SimpleNamespace as NS
import pytest
from wheelchair_hardware_bridge.preflight import missing_inputs, selected_mode


def message(frame, age=0.):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=100, nanosec=int(-age*1e9))),child_frame_id='imu')


def test_single_and_dual_modes_are_explicit():
    assert selected_mode([])=='both'
    assert selected_mode(['hardware_output:=false','lidar_mode:=201'])=='201'
    with pytest.raises(ValueError):selected_mode(['lidar_mode:=20'])


def test_missing_odom_and_tf_cannot_pass_with_cloud_only():
    topics=['/mid360_201/points_filtered','/odin1/odometry_highfreq']
    cloud=message('mid360_raw')
    errors=missing_inputs({topics[0]:cloud},topics,100.,lambda *args:False)
    assert any('odom -> base_link' in e for e in errors)
    assert any('/odin1/odometry_highfreq' in e for e in errors)


def test_selected_sensor_with_fresh_odom_and_tf_passes():
    topics=['/mid360_202/points_filtered','/odin1/odometry_highfreq']
    messages={topics[0]:message('mid360_raw'),topics[1]:message('odom')}
    assert missing_inputs(messages,topics,100.,lambda *args:True)==[]
    messages[topics[1]]=message('odom',.4)
    assert any('时间戳' in e for e in missing_inputs(messages,topics,100.,lambda *args:True))


def test_wrong_odom_frame_is_not_ready():
    topic='/odin1/odometry_highfreq'
    assert missing_inputs({topic:message('wrong')},[topic],100.,lambda *args:True)


def test_mount_uses_latest_but_motion_tf_uses_scan_timestamp():
    topics=['/mid360_201/points_filtered','/odin1/odometry_highfreq']
    messages={topics[0]:message('mid360_raw'),topics[1]:message('odom')}
    calls=[]
    def available(target,source,stamp):
        calls.append((target,source,stamp))
        return (stamp is None) if target=='imu' else stamp is not None
    assert missing_inputs(messages,topics,100.,available)==[]
    assert ('imu','base_link',None) in calls
    assert ('odom','base_link',messages[topics[0]].header.stamp) in calls
