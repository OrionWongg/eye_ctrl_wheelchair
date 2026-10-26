#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('whill_controller')
    parameters = os.path.join(package_share, 'config', 'controller.yaml')

    hardware_enabled = LaunchConfiguration('hardware_enabled')
    whill_mode = LaunchConfiguration('whill_mode')
    calibration_file = LaunchConfiguration('calibration_file')
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument('eye_control_enabled', default_value='false'),
        DeclareLaunchArgument(
            'hardware_enabled',
            default_value='true',
            description='Open the ESP32 serial port and send commands',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='ESP32 USB serial device',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='ESP32 serial baud rate',
        ),
        DeclareLaunchArgument(
            'whill_mode',
            default_value='false',
            description='Enable WHILL app mapping and auto-positioning',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=os.environ.get('IPAD_CALIBRATION_FILE', os.path.join(
                package_share, 'config', 'ipad_calibration_20261009.json')),
            description='Named-point calibration JSON file',
        ),
        Node(
            package='whill_controller',
            executable='controller_node',
            name='whill_controller',
            parameters=[
                parameters,
                {
                    'eye_control_enabled': ParameterValue(
                        LaunchConfiguration('eye_control_enabled'), value_type=bool),
                    'hardware_enabled': ParameterValue(
                        hardware_enabled, value_type=bool),
                    'serial_port': serial_port,
                    'baud': ParameterValue(baud, value_type=int),
                    'whill_mode': ParameterValue(
                        whill_mode, value_type=bool),
                    'calibration_file': calibration_file,
                },
            ],
            output='screen',
        ),
    ])
