"""Mount positions in metres; +X forward, +Y left, +Z up.
MID360 roll=180 deg, pitch=-1 deg; right .201 yaw=+10 deg, left .202 yaw=0 deg.
Odin lidar axes are assumed aligned with base_link.
Odin driver owns odom -> imu -> lidar. Keep it and attach base_link below lidar.
Run with existing sensor drivers: ros2 launch /home/whill/eye_ctrl_wheelchair/sensor_visualization/sensors_tf.launch.py
"""
from pathlib import Path
from math import pi, radians
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
    left_mid360_rpy = (pi, radians(-1.0), radians(-1.0))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # Inverse of base_link -> lidar: (+0.18, +0.28, +0.23), identity rotation.
        static_tf('odin_mount_to_base', 'lidar', 'base_link', (-0.18, -0.28, -0.23)),
        # Ground projection for a level wheelchair: 0.403 m lidar height + 0.10 m.
        static_tf('base_ground_reference', 'base_link', 'base_footprint', (0.0, 0.0, -0.503)),
        # Existing dual_mid360 launch owns local -> pointcloud frame (identity).
        static_tf('right_mid360_mount', 'base_link', 'mid360_201_local', (0.28, -0.21, -0.10), right_mid360_rpy),
        static_tf('left_mid360_mount', 'base_link', 'mid360_202_local', (0.28, 0.21, -0.10), left_mid360_rpy),
        ExecuteProcess(
            cmd=['/usr/bin/python3', str(Path(__file__).with_name('body_cloud_filter.py')),
                 '--ros-args', '--params-file', str(Path(__file__).with_name('body_filter.yaml'))],
            output='screen'),
        ExecuteProcess(
            cmd=['/usr/bin/python3', str(Path(__file__).with_name('rviz_stream_watchdog.py'))],
            output='screen'),
        Node(package='rviz2', executable='rviz2', name='wheelchair_sensors_rviz',
             arguments=['-d', str(Path(__file__).with_name('sensors.rviz'))],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
