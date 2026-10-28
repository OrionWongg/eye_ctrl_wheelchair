from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=get_package_share_directory('wheelchair_control')
    config=LaunchConfiguration('config')
    nav2_config=LaunchConfiguration('nav2_config')
    nav_share=get_package_share_directory('wheelchair_nav2')
    # One dict feeds both planner executables, so a backend switch changes only
    # the search algorithm, never the geometry or the budget it is given.
    search_params={'state_cell':ParameterValue(LaunchConfiguration('state_cell'),value_type=float),
                   'state_heading_bins':ParameterValue(LaunchConfiguration('state_heading_bins'),value_type=int),
                   'search_sampling_margin':ParameterValue(LaunchConfiguration('search_sampling_margin'),value_type=float),
                   'search_time_limit':ParameterValue(LaunchConfiguration('search_time_limit'),value_type=float)}
    smac=PythonExpression(["'",LaunchConfiguration('interaction_mode'),"' == 'directional_intent' and '",LaunchConfiguration('path_planner'),"' == 'smac_lattice'"])
    discrete=PythonExpression(["'",LaunchConfiguration('interaction_mode'),"' == 'directional_intent' and '",LaunchConfiguration('path_planner'),"' == 'discrete'"])
    confirmed=PythonExpression(["'",LaunchConfiguration('interaction_mode'),"' == 'intent_confirm'"])
    directional=PythonExpression(["'",LaunchConfiguration('interaction_mode'),"' == 'directional_intent'"])
    legacy=PythonExpression(["'",LaunchConfiguration('interaction_mode'),"' == 'gaze_joystick'"])
    return LaunchDescription([
        DeclareLaunchArgument('nav2_config',default_value=get_package_share_directory('wheelchair_nav2')+'/config/nav2.yaml'),
        DeclareLaunchArgument('path_planner',default_value='discrete',choices=['discrete','smac_lattice']),
        # A/B knobs, applied to whichever backend is selected. Keep these equal
        # across runs or the comparison measures the knobs, not the search.
        DeclareLaunchArgument('state_cell',default_value='0.10',description='Discrete-search position deduplication cell (m)'),
        DeclareLaunchArgument('state_heading_bins',default_value='72',description='Discrete-search heading deduplication bins'),
        DeclareLaunchArgument('search_sampling_margin',default_value=PythonExpression(["'0.05' if '", LaunchConfiguration('path_planner'), "' == 'smac_lattice' else '0.0'"]),description='Search-only footprint reserve (m); connectors keep the hard margin'),
        DeclareLaunchArgument('search_time_limit',default_value=PythonExpression(["'0.35' if '", LaunchConfiguration('path_planner'), "' == 'smac_lattice' else '0.25'"]),description='Planner service budget (s)'),
        DeclareLaunchArgument('input_source',default_value='keyboard'),
        DeclareLaunchArgument('interaction_mode',default_value='gaze_joystick',choices=['gaze_joystick','intent_confirm','directional_intent']),
        DeclareLaunchArgument('eye_sample_mode',default_value='legacy',choices=['legacy','stamped','fusion']),
        DeclareLaunchArgument('task_max_time',default_value='10.0'),
        DeclareLaunchArgument('task_max_distance',default_value='3.0'),
        DeclareLaunchArgument('mode',default_value='stop_only',choices=['stop_only','bounded_assist']),
        DeclareLaunchArgument('config',default_value=PythonExpression(["'",share,"/config/narrow_control.yaml' if '",LaunchConfiguration('interaction_mode'),"' != 'gaze_joystick' else '",share,"/config/control.yaml'"])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('wheelchair_sim')+'/launch/simulation.launch.py')),
        Node(package='wheelchair_control',executable='intent_mux',condition=IfCondition(legacy),parameters=[{'input_source':LaunchConfiguration('input_source'),'use_sim_time':True}],output='screen'),
        Node(package='wheelchair_control',executable='task_manager',condition=IfCondition(confirmed),parameters=[share+'/config/tasks.yaml',config,{'input_source':LaunchConfiguration('input_source'),'use_sim_time':True,'max_time':ParameterValue(LaunchConfiguration('task_max_time'),value_type=float),'max_distance':ParameterValue(LaunchConfiguration('task_max_distance'),value_type=float)}],output='screen'),
        Node(package='wheelchair_control',executable='direction_manager',condition=IfCondition(directional),parameters=[share+'/config/tasks.yaml',config,{'input_source':LaunchConfiguration('input_source'),'use_sim_time':True}],output='screen'),
        Node(package='wheelchair_control',executable='navigation_map',condition=IfCondition(directional),parameters=[nav2_config],output='screen'),
        Node(package='wheelchair_nav2',executable='wheelchair_costmap',condition=IfCondition(directional),parameters=[nav2_config],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='wheelchair/local_costmap',name='lifecycle_manager',condition=IfCondition(directional),parameters=[{'use_sim_time':True,'autostart':True,'node_names':['local_costmap']}],output='screen'),
        Node(package='wheelchair_nav2',executable='discrete_path_planner',condition=IfCondition(discrete),parameters=[config,nav2_config,search_params],output='screen'),
        Node(package='wheelchair_nav2',executable='smac_lattice_interface',condition=IfCondition(smac),parameters=[config,nav2_config,nav_share+'/config/smac_lattice.yaml',search_params,{'Lattice.lattice_filepath':nav_share+'/config/wheelchair_lattice.json'}]),
        Node(package='wheelchair_control',executable='executor',parameters=[config],output='screen'),
        Node(package='wheelchair_control',executable='safety_gate',remappings=[('/wheelchair/compute_discrete_path',PythonExpression(["'/wheelchair/smac/compute_discrete_path' if '",LaunchConfiguration('path_planner'),"' == 'smac_lattice' else '/wheelchair/compute_discrete_path'"])),('/wheelchair/path_planner_health',PythonExpression(["'/wheelchair/smac/path_planner_health' if '",LaunchConfiguration('path_planner'),"' == 'smac_lattice' else '/wheelchair/path_planner_health'"]))],parameters=[config,share+'/config/directional.yaml',{'mode':LaunchConfiguration('mode'),'path_planner_backend':PythonExpression(["'smac_lattice' if '",LaunchConfiguration('path_planner'),"' == 'smac_lattice' else 'nav2_dwb'"]),'navigation_backend':PythonExpression(["'nav2' if '",LaunchConfiguration('interaction_mode'),"' == 'directional_intent' else 'none'"])}],output='screen'),
        Node(package='eye_shared_control',executable='eye_intent',condition=IfCondition(PythonExpression(["'",LaunchConfiguration('input_source'),"' == 'eye'"])),parameters=[get_package_share_directory('eye_shared_control')+'/config/eye.yaml',{'use_sim_time':True,'interaction_mode':LaunchConfiguration('interaction_mode'),'eye_sample_mode':LaunchConfiguration('eye_sample_mode')}],output='screen')])
