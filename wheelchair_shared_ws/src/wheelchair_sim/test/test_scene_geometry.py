"""Regression: oracle geometry must match physical scenes, including low obstacles."""
import xml.etree.ElementTree as ET
from wheelchair_sim.scenes import SCENES, obstacles, map_obstacles, MESH_SCENES
from wheelchair_sim.world import world

def test_map_matches_every_physical_obstacle():
    for scene in SCENES:
        if scene in MESH_SCENES:continue # Mesh-derived map is checked separately below.
        tree=ET.fromstring(world(scene))
        models={m.attrib['name']:m for m in tree.findall('./world/model')}
        for obstacle, mapped in zip(obstacles(scene),map_obstacles(scene)):
            model=models[obstacle.name]
            pose=list(map(float,model.findtext('pose').split()))
            size=list(map(float,model.findtext('./link/collision/geometry/box/size').split()))
            assert pose[:2]==[obstacle.x,obstacle.y]
            assert pose[2]==obstacle.z_bottom+obstacle.height/2
            assert size==[obstacle.sx,obstacle.sy,obstacle.height]
            assert all(mapped.x-mapped.sx/2<=x<=mapped.x+mapped.sx/2 and mapped.y-mapped.sy/2<=y<=mapped.y+mapped.sy/2 for x,y in obstacle.polygon())
        assert len(obstacles(scene))==len(map_obstacles(scene))

def test_ground_truth_odom_and_isolated_command():
    tree=ET.fromstring(world('straight_wall'))
    model=tree.find("./world/model[@name='generic_wheelchair']")
    odom=model.find("plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
    assert odom.findtext('odom_topic')=='/wheelchair/odom'
    drive=model.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    assert drive.findtext('odom_topic')!='/wheelchair/odom'
    assert drive.findtext('topic')=='/wheelchair/sim/cmd_vel'


def test_complex_scene_has_clear_start_and_full_furniture_map():
    from wheelchair_control.planner import Predictor,Config,State
    from wheelchair_control.protocol import STOP,FORWARD
    from wheelchair_sim.scenes import BOUNDARY
    boxes=map_obstacles('complex_indoor')
    assert len(boxes)>=20
    names={b.name for b in boxes}
    assert {'desk_top','desk_leg_0','chair_south_seat','low_box','door_south','door_north'}<=names
    p=Predictor(Config());polygons=[b.polygon() for b in boxes]
    assert p.check(State(),STOP,polygons,BOUNDARY).safe
    assert p.check(State(),FORWARD,polygons,BOUNDARY).safe


def test_office_extent_and_mesh_height_band():
    from wheelchair_sim.scenes import office_metadata,boundary
    from wheelchair_control.planner import Predictor,Config,State
    from wheelchair_control.geometry import IndexedPolygon,nearby_obstacles
    from wheelchair_control.protocol import STOP,FORWARD,LEFT,RIGHT
    meta=office_metadata()
    assert meta['mesh_dimensions_m'][0]>40 and meta['mesh_dimensions_m'][1]>25
    assert meta['license']=='BSD-3-Clause' and meta['source_triangles']==3790
    assert meta['height_band_m']==[.04,1.3]
    cfg=Config();p=Predictor(cfg)
    polygons=[IndexedPolygon(b.polygon()) for b in map_obstacles('clearpath_office')]
    assert len(polygons)>500
    for state in (State(),State(x=2.),State(x=5.,v=.372,current_action=FORWARD),State(x=-5.,y=2.)):
        local=nearby_obstacles(state,polygons,cfg)
        assert len(local)<len(polygons)
        for action in (STOP,FORWARD,LEFT,RIGHT):
            assert p.check(state,action,polygons,boundary('clearpath_office')).safe == p.check(state,action,local,boundary('clearpath_office')).safe


def test_chair_dimensions_collision_envelope_and_lidar_mount():
    import math
    from wheelchair_sim.world import LIDAR_POSITION
    model=ET.fromstring(world('straight_wall')).find("./world/model[@name='generic_wheelchair']")
    lo=[math.inf]*3;hi=[-math.inf]*3
    for link in model.findall('link'):
        lp=list(map(float,link.findtext('pose','0 0 0 0 0 0').split()))
        for collision in link.findall('collision'):
            cp=list(map(float,collision.findtext('pose','0 0 0 0 0 0').split()))
            geometry=collision.find('geometry')
            if geometry.find('box') is not None:
                half=[v/2 for v in map(float,geometry.findtext('box/size').split())]
            elif geometry.find('sphere') is not None:
                half=[float(geometry.findtext('sphere/radius'))]*3
            else:
                r=float(geometry.findtext('cylinder/radius'));h=float(geometry.findtext('cylinder/length'))/2
                half=[r,h,r] if 'wheel' in link.attrib['name'] else [r,r,h]
            for i in range(3):
                lo[i]=min(lo[i],lp[i]+cp[i]-half[i]);hi[i]=max(hi[i],lp[i]+cp[i]+half[i])
            visual=link.find("visual[@name='%s']"%collision.attrib['name'].replace('_collision','_visual'))
            assert ET.tostring(visual.find('geometry'))==ET.tostring(geometry)
            assert visual.findtext('pose')==collision.findtext('pose')
    assert all(abs(a-b)<1e-8 for a,b in zip(lo,[-.45,-.30,0.]))
    assert all(abs(a-b)<1e-8 for a,b in zip(hi,[.55,.30,.85]))
    lidar=model.find("link[@name='base_scan']")
    assert tuple(map(float,lidar.findtext('pose').split()[:3]))==LIDAR_POSITION==(.48,0.,.45)
    seat=model.find("link[@name='base_link']/collision[@name='seat_collision']")
    assert abs(float(seat.findtext('pose').split()[2])+float(seat.findtext('geometry/box/size').split()[2])/2-.48)<1e-8
    drive=model.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    left=float(model.find("link[@name='left_wheel']/pose").text.split()[1])
    right=float(model.find("link[@name='right_wheel']/pose").text.split()[1])
    assert left-right==float(drive.findtext('wheel_separation'))


def test_willow_mesh_scene_has_clear_spawn_and_source_scale():
    from pathlib import Path
    from wheelchair_sim.scenes import office_metadata,boundary
    from wheelchair_control.planner import Predictor,Config,State
    from wheelchair_control.geometry import IndexedObstacles
    meta=office_metadata('willow_office')
    assert 58.<meta['mesh_dimensions_m'][0]<60.
    assert 45.<meta['mesh_dimensions_m'][1]<47.
    assert meta['license']=='CC-BY-3.0'
    assert meta['source_triangles']==13348
    assert len(meta['rectangles'])>500
    scene=ET.fromstring(world('willow_office'))
    assert scene.findtext('./world/include/uri')=='model://willow_office'
    obstacles=IndexedObstacles([b.polygon() for b in map_obstacles('willow_office')])
    predictor=Predictor(Config())
    for action in range(4):
        assert predictor.check(State(),action,obstacles,boundary('willow_office')).safe
    # Source collision units are inches; derived collision vertices are metres.
    import wheelchair_sim.scenes as scenes
    model=Path(scenes.__file__).resolve().parents[1]/'models/willow_office'
    mesh=ET.parse(model/'meshes/office_collision.dae')
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    assert mesh.find('c:asset/c:unit',ns).get('meter')=='1'
