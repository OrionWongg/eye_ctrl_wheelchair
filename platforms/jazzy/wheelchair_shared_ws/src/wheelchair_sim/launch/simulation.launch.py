import os
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, EmitEvent, SetEnvironmentVariable
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from wheelchair_sim.world import world, LIDAR_POSITION
from wheelchair_sim.scenes import obstacles,MESH_SCENES

def start(context):
    scene=LaunchConfiguration('scene').perform(context)
    sim_time=LaunchConfiguration('use_sim_time').perform(context).lower()=='true'
    first_person=LaunchConfiguration('first_person').perform(context).lower()=='true'
    folder=tempfile.mkdtemp(prefix='wheelchair_gazebo_')
    sdf=os.path.join(folder,'world.sdf')
    with open(sdf,'w') as f: f.write(world(scene,first_person=first_person))
    modern=os.environ.get('ROS_DISTRO')=='jazzy'
    command=['gz','sim'] if modern else ['ign','gazebo']
    bridge=[]
    for ros,gz,rt,gt,direction in [
      ('/clock','/clock','rosgraph_msgs/msg/Clock','Clock','GZ_TO_ROS'),
      ('/wheelchair/scan','/wheelchair/scan','sensor_msgs/msg/LaserScan','LaserScan','GZ_TO_ROS'),
      ('/wheelchair/odom','/wheelchair/odom','nav_msgs/msg/Odometry','Odometry','GZ_TO_ROS'),
      ('/tf','/wheelchair/sim/odom_tf','tf2_msgs/msg/TFMessage','Pose_V','GZ_TO_ROS'),
      ('/wheelchair/sim/cmd_vel','/wheelchair/sim/cmd_vel','geometry_msgs/msg/Twist','Twist','ROS_TO_GZ')]:
        bridge.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=('gz.msgs.' if modern else 'ignition.msgs.')+gt,direction=direction))
    if first_person:
        bridge.append(dict(ros_topic_name='/wheelchair/first_person/image_raw',gz_topic_name='/wheelchair/first_person/image_raw',ros_type_name='sensor_msgs/msg/Image',gz_type_name='gz.msgs.Image' if modern else 'ignition.msgs.Image',direction='GZ_TO_ROS'))
    for b in obstacles(scene):
        bridge.append(dict(ros_topic_name='/wheelchair/sim/contacts',gz_topic_name=f'/world/wheelchair_arena/model/{b.name}/link/wall_link/sensor/contact/contact',ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='gz.msgs.Contacts' if modern else 'ignition.msgs.Contacts',direction='GZ_TO_ROS'))
    if scene in MESH_SCENES:
        bridge.append(dict(ros_topic_name='/wheelchair/sim/contacts',gz_topic_name=f'/world/wheelchair_arena/model/{scene}/link/office/sensor/contact/contact',ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='gz.msgs.Contacts' if modern else 'ignition.msgs.Contacts',direction='GZ_TO_ROS'))
    config=os.path.join(folder,'bridge.yaml')
    with open(config,'w') as f: yaml.safe_dump(bridge,f)
    args=command+['-s','-r','-v','2',sdf]
    plugin_path='' if modern else get_package_prefix('wheelchair_scan_viz')+'/lib'
    model_path=get_package_share_directory('wheelchair_sim')+'/models'
    actions=[SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH' if modern else 'IGN_GAZEBO_RESOURCE_PATH',model_path+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')),SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH',plugin_path+':'+os.environ.get('IGN_GUI_PLUGIN_PATH','')),Node(package='wheelchair_sim',executable='drive_watchdog',output='screen'),ExecuteProcess(cmd=args,output='screen',on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))]),Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':config,'use_sim_time':sim_time}],output='screen'),Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--x',str(LIDAR_POSITION[0]),'--y',str(LIDAR_POSITION[1]),'--z',str(LIDAR_POSITION[2]),'--frame-id','base_link','--child-frame-id','base_scan'],parameters=[{'use_sim_time':sim_time}]),Node(package='wheelchair_sim',executable='obstacle_map',parameters=[{'scene':scene,'use_sim_time':sim_time}],output='screen')]
    if LaunchConfiguration('gui').perform(context).lower()=='true': actions.append(ExecuteProcess(cmd=command+['-g'] if modern else command+['-g','--gui-config',get_package_share_directory('wheelchair_sim')+'/config/'+('office_points.config' if scene in MESH_SCENES else 'scan_points.config')],output='screen',on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))]))
    if first_person and LaunchConfiguration('first_person_view').perform(context).lower()=='true':
        actions.append(Node(package='wheelchair_sim',executable='first_person_view',output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='First-person window closed'))]))
    return actions

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scene',default_value='willow_office'),DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('first_person',default_value='false',choices=['true','false']),DeclareLaunchArgument('first_person_view',default_value='true',choices=['true','false']),DeclareLaunchArgument('use_sim_time',default_value='true'),OpaqueFunction(function=start)])
