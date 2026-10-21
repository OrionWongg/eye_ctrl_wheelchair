"""Pure SDF generator: no model downloads, all geometry has physical collisions."""
from .scenes import obstacles,MESH_SCENES

def plugin(short, body=''):
    names={'physics':'Physics','user-commands':'UserCommands','scene-broadcaster':'SceneBroadcaster','sensors':'Sensors','contact':'Contact','diff-drive':'DiffDrive','odometry-publisher':'OdometryPublisher'}
    return f'<plugin filename="ignition-gazebo-{short}-system" name="ignition::gazebo::systems::{names[short]}">{body}</plugin>'
def geom_box(size): return f'<box><size>{size}</size></box>'
def shapes(name,geo,color='0.18 0.3 0.5 1',pose='0 0 0 0 0 0',friction=None):
    surface='' if friction is None else f'<surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction></surface>'
    return f'<collision name="{name}_collision"><pose>{pose}</pose><geometry>{geo}</geometry>{surface}</collision><visual name="{name}_visual"><pose>{pose}</pose><geometry>{geo}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'
def inertia(m,ix=.3,iy=.3,iz=.3): return f'<inertial><mass>{m}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz></inertia></inertial>'
LIDAR_POSITION = (.48, 0., .45)

def wheelchair(first_person=False):
    # Ground-projected base: physical envelope 1.00 x .60 x .85 m.
    # Planner envelope stays 1.00 x .70 m (5 cm extra per side).
    body='<model name="generic_wheelchair"><pose>0 0 0.005 0 0 0</pose><link name="base_link">'+inertia(70,5,6,5).replace('<inertial>', '<inertial><pose>.08 0 .45 0 0 0</pose>')
    for name,size,pose,color in [
        ('chassis','.65 .50 .16','-.075 0 .27 0 0 0','0.2 0.22 0.25 1'),
        ('seat','.50 .46 .08','-.05 0 .44 0 0 0','0.08 0.2 0.4 1'),
        ('backrest','.10 .46 .40','-.40 0 .65 0 0 0','0.08 0.2 0.4 1'),
        ('arm_left','.62 .06 .08','-.04 .27 .66 0 0 0','0.2 0.2 0.2 1'),
        ('arm_right','.62 .06 .08','-.04 -.27 .66 0 0 0','0.2 0.2 0.2 1'),
        ('footplate','.30 .48 .06','.40 0 .15 0 0 0','0.4 0.4 0.4 1')]:
        body+=shapes(name,geom_box(size),color,pose)
    if first_person:
        # Seated eye level, rigidly attached to the base; no added collision
        # geometry or mass. Camera +X points forward, positive pitch looks down.
        body+='''<sensor name="first_person" type="camera">
          <pose>-.05 0 1.2 0 .08 0</pose>
          <always_on>true</always_on><update_rate>20</update_rate>
          <topic>/wheelchair/first_person/image_raw</topic>
          <camera><horizontal_fov>1.57079632679</horizontal_fov>
            <image><width>960</width><height>540</height><format>R8G8B8</format></image>
            <clip><near>.05</near><far>80</far></clip>
          </camera></sensor>'''
    body+='</link>'
    for side,y in [('left',.25),('right',-.25)]:
        body+=f'<link name="{side}_wheel"><pose>0 {y} .20 1.57079632679 0 0</pose>'+inertia(3,.035,.035,.06)+shapes(side,'<cylinder><radius>.20</radius><length>.10</length></cylinder>','0.06 0.06 0.06 1',friction=1)+'</link>'
        body+=f'<joint name="{side}_wheel_joint" type="revolute"><parent>base_link</parent><child>{side}_wheel</child><axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'
    for side,y,x in [('left',.23,.38),('right',-.23,.38),('rear_left',.23,-.36),('rear_right',-.23,-.36)]:
        body+=f'<link name="caster_{side}"><pose>{x} {y} .065 0 0 0</pose>'+inertia(.3,.0005,.0005,.0005)+shapes('caster','<sphere><radius>.065</radius></sphere>','0.1 0.1 0.1 1',friction=.001)+'</link>'
        body+=f'<joint name="caster_{side}_joint" type="fixed"><parent>base_link</parent><child>caster_{side}</child></joint>'
    body+=f'<link name="base_scan"><pose>{LIDAR_POSITION[0]} {LIDAR_POSITION[1]} {LIDAR_POSITION[2]} 0 0 0</pose>'+inertia(.1,.001,.001,.001)+shapes('lidar','<cylinder><radius>.04</radius><length>.05</length></cylinder>','0.7 0.1 0.1 1')
    body+='''<sensor name="lidar" type="gpu_lidar"><always_on>true</always_on><update_rate>15</update_rate><topic>/wheelchair/scan</topic><ignition_frame_id>base_scan</ignition_frame_id><ray><scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159265359</min_angle><max_angle>3.14159265359</max_angle></horizontal></scan><range><min>.06</min><max>12</max><resolution>.01</resolution></range></ray></sensor></link><joint name="scan_joint" type="fixed"><parent>base_link</parent><child>base_scan</child></joint>'''
    body+=plugin('diff-drive','''<left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint><wheel_separation>.50</wheel_separation><wheel_radius>.20</wheel_radius><topic>/wheelchair/sim/cmd_vel</topic><odom_topic>/wheelchair/sim/wheel_odom_unused</odom_topic><tf_topic>/wheelchair/sim/wheel_tf_unused</tf_topic><max_linear_acceleration>.6</max_linear_acceleration><min_linear_acceleration>-.6</min_linear_acceleration><max_angular_acceleration>.8</max_angular_acceleration><min_angular_acceleration>-.8</min_angular_acceleration>''')
    body+=plugin('odometry-publisher','''<odom_frame>odom</odom_frame><robot_base_frame>base_link</robot_base_frame><odom_publish_frequency>30</odom_publish_frequency><odom_topic>/wheelchair/odom</odom_topic><tf_topic>/wheelchair/sim/odom_tf</tf_topic><dimensions>2</dimensions>''')
    return body+'</model>'
def world(scene, first_person=False):
    s='<?xml version="1.0"?><sdf version="1.8"><world name="wheelchair_arena">'
    for p in ['physics','user-commands','scene-broadcaster','contact']: s+=plugin(p)
    s+=plugin('sensors','<render_engine>ogre2</render_engine>')
    s+='''<physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics><light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>.9 .9 .9 1</diffuse><direction>-.5 .1 -.9</direction></light><scene><ambient>.6 .6 .6 1</ambient><background>.7 .8 .9 1</background></scene><model name="ground"><static>true</static><link name="ground_link">'''
    s+=shapes('ground','<plane><normal>0 0 1</normal><size>30 30</size></plane>','.6 .6 .6 1')+'</link></model>'
    for b in obstacles(scene):
        s+=f'<model name="{b.name}"><static>true</static><pose>{b.x} {b.y} {b.z_bottom+b.height/2} 0 0 0</pose><link name="wall_link">'+shapes('wall',geom_box(f'{b.sx} {b.sy} {b.height}'),b.color)
        s+='<sensor name="contact" type="contact"><always_on>true</always_on><update_rate>30</update_rate><contact><collision>wall_collision</collision></contact></sensor></link></model>'
    if scene in MESH_SCENES:
        s=s.replace('<size>30 30</size>','<size>100 100</size>')
        s=s.replace('<visual name="ground_visual"><pose>0 0 0 0 0 0</pose>',
                    '<visual name="ground_visual"><pose>0 0 -0.02 0 0 0</pose>')
        s+=f'<include><uri>model://{scene}</uri></include>'
    return s+wheelchair(first_person)+'</world></sdf>' 
