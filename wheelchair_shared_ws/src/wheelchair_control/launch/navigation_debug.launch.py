"""Attach read-only RViz diagnostics to an existing wheelchair simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config=get_package_share_directory('wheelchair_control')+'/config/navigation_debug.rviz'
    params=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]
    monitor=Node(package='wheelchair_control',executable='navigation_debug',parameters=params,output='screen')
    rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',config],parameters=params,output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),monitor,rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz,on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=monitor,on_exit=[EmitEvent(event=Shutdown(reason='Diagnostic monitor exited'))])),
    ])
