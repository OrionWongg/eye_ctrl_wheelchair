"""Static mounting TFs in metres; +X forward, +Y left, +Z up.

REP105 / Nav2-friendly tree.  Odin's internal odom->imu->lidar TF is disabled
(send_odom_baselink_tf: 0); odom_relay broadcasts odom->base_link from odometry
here, so one start_sensors.sh run owns the whole sensor-side tree:

  odom -> base_link -> {imu, lidar, camera_0, base_footprint,
                        mid360_201_local, mid360_202_local}

MID360 both roll=180 deg (inverted); right .201 pitch=-1 deg, yaw=0 deg;
left .202 pitch=0 deg, yaw=-1 deg.
Odin lidar/imu axes are aligned with base_link (identity rotation).
camera_0 RPY/xyz derived from odin_ws calib.yaml Tcl_0 (camera<-lidar, optical
z fwd / x right / y down) composed with base_link<-lidar=(0.18,0.28,0.23).

Run with existing sensor drivers:
  ros2 launch /home/whill/eye_ctrl_wheelchair/sensor_visualization/sensors_tf.launch.py
"""
from pathlib import Path
import yaml
from math import pi, radians
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                           '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                           '--frame-id', parent, '--child-frame-id', child])


def generate_launch_description():
    # Fixed-axis RPY relative to base_link: Rz(yaw) * Ry(pitch) * Rx(roll).
    right_mid360_rpy = (pi, radians(-1.0), radians(0.0))
    left_mid360_rpy = (pi, radians(0.0), radians(-1.0))
    # camera_0 optical frame (z fwd, x right, y down) vs base_link (x fwd, y left, z up).
    # T_base_cam = T_base_lidar @ inv(Tcl_0) from odin_ws calib.yaml.
    camera_0_xyz = (0.187415, 0.309663, 0.219460)
    camera_0_rpy = (-1.564317, 0.012941, -1.577013)
    bridge_cfg = (Path(get_package_share_directory('wheelchair_hardware_bridge'))
                  / 'config' / 'hardware_bridge.yaml')
    body_mask=yaml.safe_load(Path(__file__).with_name('body_filter.yaml').read_text())['body_cloud_filter']['ros__parameters']
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('odom', default_value='true'),
        # Odin lidar and imu share the device housing mount: +0.18,+0.28,+0.23.
        static_tf('base_to_odin_lidar', 'base_link', 'lidar', (0.18, 0.28, 0.23)),
        static_tf('base_to_odin_imu', 'base_link', 'imu', (0.18, 0.28, 0.23)),
        static_tf('base_to_odin_camera', 'base_link', 'camera_0', camera_0_xyz, camera_0_rpy),
        # Ground projection for a level wheelchair: 0.403 m lidar height + 0.10 m.
        static_tf('base_ground_reference', 'base_link', 'base_footprint', (0.0, 0.0, -0.503)),
        # Existing dual_mid360 launch owns local -> pointcloud frame (identity).
        static_tf('right_mid360_mount', 'base_link', 'mid360_201_local', (0.28, -0.21, -0.10), right_mid360_rpy),
        static_tf('left_mid360_mount', 'base_link', 'mid360_202_local', (0.28, 0.21, -0.10), left_mid360_rpy),
        # Sole owner of odom -> base_link. Also republishes /wheelchair/odom and
        # /wheelchair/imu so the control stack never runs a second broadcaster.
        Node(package='wheelchair_hardware_bridge', executable='odom_relay',
             name='odom_relay', parameters=[str(bridge_cfg)],
             condition=IfCondition(LaunchConfiguration('odom')), output='screen'),
        ExecuteProcess(
            cmd=['/usr/bin/python3', str(Path(__file__).with_name('body_cloud_filter.py')),
                 '--ros-args', '--params-file', str(Path(__file__).with_name('body_filter.yaml'))],
            output='screen'),
        ExecuteProcess(
            cmd=['/usr/bin/python3', str(Path(__file__).with_name('rviz_stream_watchdog.py'))],
            output='screen'),
        Node(package='wheelchair_control', executable='navigation_debug',
             name='wheelchair_navigation_debug',
             parameters=[{'use_sim_time': False,
                          'body_mask':[body_mask[k] for k in ('x_min','x_max','y_min','y_max')]}], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2', name='wheelchair_sensors_rviz',
             arguments=['-d', str(Path(__file__).with_name('sensors.rviz'))],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
