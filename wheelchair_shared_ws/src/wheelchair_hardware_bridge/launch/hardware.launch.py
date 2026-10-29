"""Hardware launch: real MID360 + Odin sensors → avoidance stack → iPad wheelchair.

Replaces simulation.launch.py.  All nodes run with use_sim_time:=false.
start_sensors.sh owns the drivers, mounting TF, body filter and the single
odom_relay (odom→base_link); this launch starts cloud_to_scan, twist_to_key,
then the full wheelchair_control stack with eye input and real-hardware output.

Usage:
  ros2 launch wheelchair_hardware_bridge hardware.launch.py [args]

Key args (same as shared.launch.py where compatible):
  interaction_mode  directional_intent (default) | intent_confirm | gaze_joystick
  path_planner      discrete (default) | smac_lattice
  eye_sample_mode   fusion (default) | stamped | legacy
  input_source      eye (default) | keyboard
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo,
    RegisterEventHandler, EmitEvent,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def stop_on_child_exit(event, context):
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(reason=f'Hardware node exited: {event.process_name}'))]
    return []


def generate_launch_description():
    # ------------------------------------------------------------------ paths
    ctrl_share = get_package_share_directory('wheelchair_control')
    nav_share   = get_package_share_directory('wheelchair_nav2')
    bridge_share = get_package_share_directory('wheelchair_hardware_bridge')
    eye_share   = get_package_share_directory('eye_shared_control')

    # ---------------------------------------------------------------- configs
    bridge_cfg = bridge_share + '/config/hardware_bridge.yaml'
    timing_cfg = bridge_share + '/config/hardware_timing.yaml'
    source_root=Path(__file__).resolve().parents[2]
    dwb_cfg=str(source_root/'wheelchair_nav2/config/dwb_preview.yaml')

    # ---------------------------------------------------------- substitutions
    interaction = LaunchConfiguration('interaction_mode')
    lidar_mode = LaunchConfiguration('lidar_mode')
    output_enabled = PythonExpression(["'", LaunchConfiguration('hardware_output'),
        "' == 'true' and '", lidar_mode, "' == 'both'"])
    path_planner = LaunchConfiguration('path_planner')
    mode = LaunchConfiguration('mode')
    nav2_config = LaunchConfiguration('nav2_config')
    config = LaunchConfiguration('config')
    # Shared by both planner executables: switching backend must not change the
    # geometry or the budget the search is given, only the search itself.
    search_params = {
        'state_cell': ParameterValue(LaunchConfiguration('state_cell'), value_type=float),
        'state_heading_bins': ParameterValue(LaunchConfiguration('state_heading_bins'), value_type=int),
        'search_sampling_margin': ParameterValue(
            LaunchConfiguration('search_sampling_margin'), value_type=float),
        'search_time_limit': ParameterValue(
            LaunchConfiguration('search_time_limit'), value_type=float)}

    smac = PythonExpression([
        "'", interaction, "' == 'directional_intent' and '",
        path_planner, "' == 'smac_lattice'"])
    discrete = PythonExpression([
        "'", interaction, "' == 'directional_intent' and '",
        path_planner, "' == 'discrete'"])
    confirmed = PythonExpression(["'", interaction, "' == 'intent_confirm'"])
    directional = PythonExpression(["'", interaction, "' == 'directional_intent'"])
    legacy = PythonExpression(["'", interaction, "' == 'gaze_joystick'"])
    use_eye = PythonExpression(["'", LaunchConfiguration('input_source'), "' == 'eye'"])

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=stop_on_child_exit)),
        # ------------------------------------------------- declare args
        DeclareLaunchArgument('interaction_mode', default_value='directional_intent',
                              choices=['gaze_joystick', 'intent_confirm', 'directional_intent']),
        DeclareLaunchArgument('path_planner', default_value='discrete',
                              choices=['discrete', 'smac_lattice', 'smac_2d']),
        # A/B knobs, applied identically to whichever backend is selected.
        DeclareLaunchArgument('state_cell', default_value='0.10',
                              description='Discrete-search position deduplication cell (m)'),
        DeclareLaunchArgument('state_heading_bins', default_value='72',
                              description='Discrete-search heading deduplication bins'),
        DeclareLaunchArgument('search_sampling_margin', default_value=PythonExpression(["'0.05' if '", LaunchConfiguration('path_planner'), "' == 'smac_lattice' else '0.0'"]),
                              description='Search-only footprint reserve (m); connectors keep the hard margin'),
        DeclareLaunchArgument('search_time_limit', default_value=PythonExpression(["'0.35' if '", LaunchConfiguration('path_planner'), "' == 'smac_lattice' else '0.25'"]),
                              description='Planner service budget (s)'),
        DeclareLaunchArgument('input_source', default_value='eye'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('rviz_config', default_value=''),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': False}], output='screen'),
        DeclareLaunchArgument('enable_rviz_goal', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('lidar_mode', default_value='both', choices=['both','201','202']),
        LogInfo(msg='Single lidar costmap inspection: physical wheelchair output is disabled.',
                condition=IfCondition(PythonExpression(["'", lidar_mode, "' != 'both'"]))),
        DeclareLaunchArgument('continuous_navigation', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('velocity_topic', default_value='/wheelchair/sim/drive'),
        DeclareLaunchArgument('continuous_velocity_input', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('hardware_output', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('eye_sample_mode', default_value='fusion',
                              choices=['legacy', 'stamped', 'fusion']),
        DeclareLaunchArgument('task_max_time', default_value='10.0'),
        DeclareLaunchArgument('task_max_distance', default_value='3.0'),
        DeclareLaunchArgument('nav2_config',
                              default_value=bridge_share + '/config/nav2_hardware.yaml'),
        DeclareLaunchArgument('mode', default_value='stop_only',
                              choices=['stop_only', 'bounded_assist']),
        DeclareLaunchArgument('config', default_value=PythonExpression([
            "'", ctrl_share, "/config/narrow_control.yaml' if '", interaction,
            "' != 'gaze_joystick' else '", ctrl_share, "/config/control.yaml'"])),

        # start_sensors.sh already owns the sensor mounting TF and body filter.
        # Do not start duplicate TF publishers here.

        # ------------------------------------------- hardware bridge nodes
        Node(package='wheelchair_hardware_bridge', executable='cloud_to_scan',
             name='cloud_to_scan', parameters=[bridge_cfg,
                 {'lidar_mode': ParameterValue(lidar_mode, value_type=str)}], output='screen'),
        Node(package='wheelchair_hardware_bridge', executable='twist_to_key',
             name='twist_to_key', condition=IfCondition(output_enabled), parameters=[bridge_cfg,
                 {'source_topic': LaunchConfiguration('velocity_topic'),
                  'continuous_input': ParameterValue(PythonExpression(["'", LaunchConfiguration('continuous_navigation'), "' == 'true' or '", LaunchConfiguration('continuous_velocity_input'), "' == 'true'"]), value_type=bool)}], output='screen'),

        # ------------------------------------------- wheelchair_control stack
        # intent_mux (legacy gaze_joystick mode)
        Node(package='wheelchair_control', executable='intent_mux',
             condition=IfCondition(legacy),
             parameters=[{'allow_dynamic_map': True, 'input_source': LaunchConfiguration('input_source'),
                          'use_sim_time': False}],
             output='screen'),

        # task_manager (intent_confirm mode)
        Node(package='wheelchair_control', executable='task_manager',
             condition=IfCondition(confirmed),
             parameters=[ctrl_share + '/config/tasks.yaml', config, timing_cfg,
                         {'allow_dynamic_map': True, 'input_source': LaunchConfiguration('input_source'),
                          'use_sim_time': False,
                          'max_time': ParameterValue(
                              LaunchConfiguration('task_max_time'), value_type=float),
                          'max_distance': ParameterValue(
                              LaunchConfiguration('task_max_distance'), value_type=float)}],
             output='screen'),

        # direction_manager (directional_intent mode)
        Node(package='wheelchair_control', executable='direction_manager',
             condition=IfCondition(directional),
             parameters=[ctrl_share + '/config/tasks.yaml', config, timing_cfg,
                         {'allow_dynamic_map': True, 'input_source': LaunchConfiguration('input_source'),
                          'enable_rviz_goal': ParameterValue(LaunchConfiguration('enable_rviz_goal'), value_type=bool),
                          'use_sim_time': False}],
             output='screen'),

        # Perception is required by the safety gate in every interaction mode.
        Node(package='wheelchair_hardware_bridge', executable='scan_map',
             parameters=[bridge_cfg, {'use_sim_time': False}], output='screen'),
        Node(package='wheelchair_nav2', executable='wheelchair_costmap',
             condition=IfCondition(directional),
             parameters=[nav2_config, {'use_sim_time': False}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             namespace='wheelchair/local_costmap', name='lifecycle_manager',
             condition=IfCondition(directional),
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['local_costmap']}],
             output='screen'),

        # path planners
        Node(package='wheelchair_nav2', executable='discrete_path_planner',
             condition=IfCondition(discrete),
             parameters=[config, timing_cfg, nav2_config, search_params,
                         {'use_sim_time': False}], output='screen'),
        Node(package='wheelchair_nav2', executable='smac_lattice_interface',
             condition=IfCondition(smac),
             # search_params must come last: a later parameters entry overrides
             # an earlier one, and smac_lattice.yaml also defines
             # search_sampling_margin / search_time_limit.
             parameters=[config, timing_cfg, nav2_config,
                         nav_share + '/config/smac_lattice.yaml',
                         search_params,
                         {'use_sim_time': False,
                          'Lattice.lattice_filepath':
                          nav_share + '/config/wheelchair_lattice.json'}]),

        Node(package='wheelchair_nav2', executable='smac_2d_interface',
             condition=IfCondition(PythonExpression(["'", interaction,
                 "' == 'directional_intent' and '", path_planner, "' == 'smac_2d'"])),
             parameters=[config, timing_cfg, nav2_config, search_params,
                         {'use_sim_time': False, 'Lattice.allow_unknown': False,
                          'Lattice.tolerance': 0.0}]),

        # Native Nav2 controller; the lower safety gate owns FollowPath actions.
        Node(package='nav2_controller', executable='controller_server',
             namespace='wheelchair/dwb_preview', name='controller_server',
             condition=IfCondition(LaunchConfiguration('continuous_navigation')),
             parameters=[dwb_cfg],
             remappings=[('cmd_vel','/wheelchair/dwb_preview/cmd_vel')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             namespace='wheelchair/dwb_preview', name='lifecycle_manager',
             condition=IfCondition(LaunchConfiguration('continuous_navigation')),
             parameters=[{'use_sim_time':False,'autostart':True,'node_names':['controller_server']}],output='screen'),

        # executor
        Node(package='wheelchair_control', executable='executor',
             parameters=[config, timing_cfg, {'use_sim_time': False,
                 'continuous_navigation': ParameterValue(LaunchConfiguration('continuous_navigation'),value_type=bool)}], output='screen'),

        # safety_gate
        Node(package='wheelchair_control', executable='safety_gate',
             remappings=[
                 ('/wheelchair/compute_discrete_path',
                  PythonExpression(["'/wheelchair/smac/compute_discrete_path' if '",
                                    path_planner, "' in ('smac_lattice', 'smac_2d') else "
                                    "'/wheelchair/compute_discrete_path'"])),
                 ('/wheelchair/path_planner_health',
                  PythonExpression(["'/wheelchair/smac/path_planner_health' if '",
                                    path_planner, "' in ('smac_lattice', 'smac_2d') else "
                                    "'/wheelchair/path_planner_health'"])),
             ],
             parameters=[config, ctrl_share + '/config/directional.yaml', timing_cfg,
                         {'use_sim_time': False,
                          'allow_dynamic_map': True,
                          'continuous_navigation': ParameterValue(LaunchConfiguration('continuous_navigation'),value_type=bool),
                          'mode': mode,
                          # Keyboard runs hold-to-run: a stopped A/D turns while
                          # held. Eye input keeps one bounded heading per gesture.
                          'turn_mode': PythonExpression([
                              "'continuous' if '", LaunchConfiguration('input_source'),
                              "' == 'keyboard' else 'fixed'"]),
                          'path_planner_backend': PythonExpression([
                              "'", path_planner, "' if '", path_planner,
                              "' in ('smac_lattice', 'smac_2d') else 'nav2_dwb'"]),
                          'navigation_backend': PythonExpression([
                              "'nav2' if '", interaction,
                              "' == 'directional_intent' else 'none'"])}],
             output='screen'),

        # eye_intent (only when input_source:=eye)
        Node(package='eye_shared_control', executable='eye_intent',
             condition=IfCondition(use_eye),
             parameters=[eye_share + '/config/eye.yaml',
                         {'use_sim_time': False,
                          'interaction_mode': interaction,
                          'eye_sample_mode': LaunchConfiguration('eye_sample_mode')}],
             output='screen'),

        # ------------------------------------------- real wheelchair output
        # whill_controller bridges /whill/controller/input → iPad → hardware
        # eye_control_enabled:=false so the nav stack drives via twist_to_key
        Node(package='whill_controller', executable='controller_node',
             name='whill_controller',
             condition=IfCondition(output_enabled),
             parameters=[{
                 'eye_control_enabled': False,
                 'hardware_enabled': True,
                 'serial_port': '/dev/ble2wchair',
                 'whill_mode': True,
                 'keyboard_auto_drag': True,
                 'navigation_control_enabled': True,
                 'navigation_continuous_mapping': True,
                 'rviz_goal_authority_enabled': ParameterValue(LaunchConfiguration('enable_rviz_goal'), value_type=bool),
                 'navigation_hold_to_run': ParameterValue(PythonExpression([
                     "'", LaunchConfiguration('input_source'), "' == 'keyboard'"]), value_type=bool),
             }],
             output='screen'),
    ])
