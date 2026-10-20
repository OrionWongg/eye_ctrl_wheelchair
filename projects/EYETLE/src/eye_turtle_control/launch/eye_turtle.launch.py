from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim', executable='turtlesim_node', name='turtlesim',
            remappings=[('/turtle1/color_sensor', '/_eye_turtle_unused/color_sensor')]),
        Node(package='eye_turtle_control', executable='eye_turtle_controller', name='eye_turtle_controller'),
    ])
