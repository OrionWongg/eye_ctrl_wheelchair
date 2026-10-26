"""Own sensors and a stationary costmap test; never start actuator nodes."""
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('wheelchair_hardware_bridge')
    bridge = share + '/config/hardware_bridge.yaml'
    nav2 = share + '/config/nav2_hardware.yaml'
    mode = LaunchConfiguration('lidar_mode')
    sensors = ExecuteProcess(cmd=[LaunchConfiguration('sensor_script'), '--mid360', mode],
                             name='test_sensors', output='screen')
    ready = ExecuteProcess(cmd=[sys.executable, '-m', 'wheelchair_hardware_bridge.preflight',
                                '--timeout', '30', ['lidar_mode:=', mode]],
                           name='test_sensor_readiness', output='screen')
    map_nodes = [
        Node(package='wheelchair_hardware_bridge', executable='cloud_to_scan',
             name='cloud_to_scan', parameters=[bridge, {'lidar_mode': ParameterValue(mode, value_type=str),
                 'ground_tolerance': 0.08,
                 'cluster_filter_enabled': ParameterValue(LaunchConfiguration('cluster_filter_enabled'), value_type=bool)}  # Temporary stationary noise comparison only.
             ], output='screen'),
        Node(package='wheelchair_hardware_bridge', executable='odom_relay',
             name='odom_relay', parameters=[bridge], output='screen'),
        Node(package='wheelchair_hardware_bridge', executable='scan_map',
             name='scan_map', parameters=[bridge], output='screen'),
        Node(package='wheelchair_nav2', executable='wheelchair_costmap',
             parameters=[nav2, {'use_sim_time': False}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             namespace='wheelchair/local_costmap', name='lifecycle_manager',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['local_costmap']}], output='screen'),
    ]
    def after_readiness(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Sensor readiness failed; costmap test cancelled'))]
        return map_nodes
    def child_exit(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason=f'Test child exited: {event.process_name}'))]
    actions = [DeclareLaunchArgument('cluster_filter_enabled', default_value='true'),
               DeclareLaunchArgument('lidar_mode', default_value='201', choices=['201','202']),
               DeclareLaunchArgument('sensor_script'),
               RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_readiness)),
               RegisterEventHandler(OnProcessExit(target_action=sensors, on_exit=child_exit))]
    actions += [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=child_exit)) for node in map_nodes]
    return LaunchDescription(actions + [sensors, ready])
