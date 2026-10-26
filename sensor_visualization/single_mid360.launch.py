"""One configured MID360, for stationary interference comparison."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    number = LaunchConfiguration('sensor').perform(context)
    if number not in ('201', '202'):
        raise ValueError('sensor must be 201 or 202')
    return [
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='single_mid360_driver', output='screen',
             parameters=[{'xfer_format': 0, 'multi_topic': 1, 'data_src': 0,
                          'publish_freq': 10.0, 'output_data_type': 0,
                          'frame_id': 'mid360', 'cmdline_input_bd_code': '', 'lvx_file_path': '',
                          'user_config_path': str(Path(__file__).with_name(f'mid360_{number}.json'))}],
             remappings=[(f'livox/lidar_192_168_86_{number}', f'/mid360_{number}/points'),
                         (f'livox/imu_192_168_86_{number}', f'/mid360_{number}/imu')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name=f'mid360_{number}_local_frame',
             arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
                        '--frame-id',f'mid360_{number}_local',
                        '--child-frame-id',f'mid360_192_168_86_{number}']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor', default_value='201', choices=['201','202']),
        OpaqueFunction(function=nodes),
    ])
