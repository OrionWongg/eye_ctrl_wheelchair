#!/usr/bin/env python3
"""Launch the dedicated WHILL pointer calibration logger."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the calibration logger launch description."""
    serial_port = LaunchConfiguration('serial_port')
    log_directory = LaunchConfiguration('log_directory')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='ESP32 USB serial device',
        ),
        DeclareLaunchArgument(
            'log_directory',
            default_value=os.path.expanduser(
                '~/.ros/whill_controller/calibration_logs'),
            description='Directory for calibration JSON and JSONL files',
        ),
        Node(
            package='whill_controller',
            executable='calibration_logger',
            name='whill_calibration_logger',
            parameters=[{
                'serial_port': serial_port,
                'baud': 115200,
                'homing_wait_ms': 1200,
                'log_directory': log_directory,
            }],
            output='screen',
        ),
    ])
