from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('interaction_mode',default_value='gaze_joystick',choices=['gaze_joystick','intent_confirm','directional_intent']),
        DeclareLaunchArgument('eye_sample_mode',default_value='legacy',choices=['legacy','stamped','fusion']),
        Node(package='eye_shared_control', executable='eye_intent', output='screen',
             parameters=[os.path.join(get_package_share_directory('eye_shared_control'), 'config', 'eye.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'interaction_mode':LaunchConfiguration('interaction_mode'),
                          'eye_sample_mode':LaunchConfiguration('eye_sample_mode')}])])
