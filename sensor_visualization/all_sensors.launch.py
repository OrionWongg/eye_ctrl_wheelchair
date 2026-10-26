"""Start both sensor drivers, mounting transforms, and one RViz window."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    livox = Path(get_package_share_directory('livox_ros_driver2'))
    odin = Path(get_package_share_directory('odin_ros_driver'))
    local = Path(__file__).resolve().parent
    selection = LaunchConfiguration('mid360')
    return LaunchDescription([
        DeclareLaunchArgument('mid360', default_value='both', choices=['both','201','202']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(local / 'single_mid360.launch.py')),
            launch_arguments={'sensor': selection}.items(),
            condition=IfCondition(PythonExpression(["'", selection, "' != 'both'"]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(livox / 'launch_ROS2' / 'dual_mid360_launch.py')),
            condition=IfCondition(PythonExpression(["'", selection, "' == 'both'"]))),
        # Only the main Odin driver is needed for raw clouds, RGB, IMU and odometry.
        # Its original demo launch also opens a separate RViz window.
        Node(package='odin_ros_driver', executable='host_sdk_sample',
             name='host_sdk_sample', output='screen',
             parameters=[{'config_file': str(odin / 'config' / 'control_command.yaml')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(local / 'sensors_tf.launch.py'))),
    ])
