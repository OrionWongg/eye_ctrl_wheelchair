import math
import struct
from types import SimpleNamespace
import numpy as np
import pytest
from rclpy.clock import ClockType
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from wheelchair_hardware_bridge.scan_map import rasterize
from wheelchair_hardware_bridge.odom_relay import convert_odom
from wheelchair_hardware_bridge.cloud_to_scan import _iter_xyz
from wheelchair_control.nav2_transport import occupied_rectangles


def cell(grid, origin, x, y, resolution=.1):
    return grid[int(math.floor((y-origin[1])/resolution)),int(math.floor((x-origin[0])/resolution))]


def test_no_returns_never_establish_free_space():
    grid,origin=rasterize([math.inf]*720,-math.pi,math.pi/360,.15,8.,(0,0,0))
    assert cell(grid,origin,1.,0.)==-1
    assert cell(grid,origin,-1.,0.)==-1
    assert cell(grid,origin,0.,0.)==0  # measured chassis interior only
    rects=occupied_rectangles(grid.ravel(),100,100,.1,origin)
    assert sum(r.native.area for r in rects)==pytest.approx(np.sum(grid!=0)*.01)


def test_finite_wall_blocks_endpoint_and_occluded_space():
    ranges=np.full(720,2.)
    grid,origin=rasterize(ranges,-math.pi,math.pi/360,.15,8.,(0,0,0))
    assert cell(grid,origin,1.,0.)==0
    assert cell(grid,origin,2.,0.)==100
    assert cell(grid,origin,3.,0.)==-1


def test_angular_gap_not_interpolated_as_free():
    ranges=np.full(720,4.)
    ranges[340:381]=math.inf
    grid,origin=rasterize(ranges,-math.pi,math.pi/360,.15,8.,(0,0,0))
    assert cell(grid,origin,2.,0.)==-1
    assert cell(grid,origin,-2.,0.)==0


def test_scan_pose_places_grid_and_hits_in_odom():
    grid,origin=rasterize(np.full(720,2.),-math.pi,math.pi/360,.15,8.,(10,20,math.pi/2))
    assert cell(grid,origin,10.,21.)==0
    assert cell(grid,origin,10.,23.)==-1
    assert np.sum(grid==100)>50



def test_blind_band_clears_only_unknown_sides_and_rear():
    from wheelchair_hardware_bridge.scan_map import clear_blind_band
    cells=np.full((100,100),-1,dtype=np.int8);origin=(-5.,-5.)
    cells[int((.35+5)/.1),int((.05+5)/.1)]=100  # return beside the left side
    out=clear_blind_band(cells,origin,.1,(0,0,0),.10)
    assert cell(out,origin,.05,.35)==100       # occupied is never cleared
    assert cell(out,origin,-.15,.35)==0        # left side
    assert cell(out,origin,-.15,-.35)==0       # right side
    assert cell(out,origin,-.5,0.)==0          # rear
    assert cell(out,origin,.65,0.)==-1          # front must be observed
    assert cell(out,origin,0.,.55)==-1         # beyond the band


def test_blind_band_follows_pose_and_can_be_disabled():
    from wheelchair_hardware_bridge.scan_map import clear_blind_band
    cells=np.full((100,100),-1,dtype=np.int8);origin=(-4.,-3.)
    out=clear_blind_band(cells,origin,.1,(1.,2.,math.pi/2),.10)
    assert cell(out,origin,1.-.35,2.)==0       # body left is world -x
    assert cell(out,origin,1.,2.-.5)==0        # body rear is world -y
    assert cell(out,origin,1.,2.+.65)==-1       # body front is world +y
    assert np.all(clear_blind_band(cells,origin,.1,(1.,2.,0.),0.)==-1)


def test_fuse_keeps_returns_and_aligns_shifted_windows():
    from wheelchair_hardware_bridge.scan_map import fuse
    n=100
    a=np.full((n,n),-1,dtype=np.int8);a[30:50,:]=0;a[40,60]=100
    b=np.full((n,n),-1,dtype=np.int8);b[:,60:70]=0
    out=fuse([(0,0,a),(0,0,b)],0,0,n)
    assert out[35,40]==0 and out[10,65]==0     # free in either layer
    assert out[40,60]==100                     # a return beats a later free
    assert out[10,10]==-1                      # never observed
    out=fuse([(0,0,a)],1,0,n)                  # window moved one cell +x
    assert out[40,59]==100 and out[40,99]==-1
    out=fuse([(0,0,a)],70,0,n)                 # partial overlap only
    assert np.sum(out==0)==30*20 and np.all(out[:,30:]==-1)


def scan_map_node(published):
    from wheelchair_hardware_bridge.scan_map import ScanMap
    from rclpy.time import Time
    pose=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
    node=SimpleNamespace(resolution=.05,size=5.,max_age=.3,history_window=.25,blind_band=.10,
        session='test',sequence=0,layers=__import__('collections').deque(),last_stamp=None,pending=None,
        now=10.,grid_pub=SimpleNamespace(publish=lambda _:None),map_pub=SimpleNamespace(publish=published.append),
        tf=SimpleNamespace(lookup_transform=lambda *args:SimpleNamespace(transform=pose)),
        get_logger=lambda:SimpleNamespace(error=lambda *a:None,warning=lambda *a,**k:None))
    node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=node.now,clock_type=ClockType.ROS_TIME))
    node.publish=lambda:ScanMap.publish(node)
    return node


def full_scan(stamp,ranges):
    msg=LaserScan();msg.header.frame_id='base_link'
    msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int(round((stamp%1)*1e9))
    msg.angle_min=-math.pi;msg.angle_increment=math.pi/360;msg.range_min=.15;msg.range_max=8.
    msg.ranges=list(ranges);return msg


def test_scan_map_publishes_on_arrival_and_rejects_replays():
    from wheelchair_hardware_bridge.scan_map import ScanMap
    published=[];node=scan_map_node(published)
    ScanMap.receive(node,full_scan(9.9,[2.]*720));assert len(published)==1 and node.pending is None
    ScanMap.receive(node,full_scan(9.9,[2.]*720));assert len(published)==1   # same stamp
    ScanMap.receive(node,full_scan(9.95,[2.]*720));assert len(published)==2
    assert published[1].sequence==2 and published[1].complete


def test_scan_map_fusion_window_expires_old_free_space():
    from wheelchair_hardware_bridge.scan_map import ScanMap
    published=[];node=scan_map_node(published)
    left=np.full(720,math.inf);left[360:720]=2.   # only +y half observed
    right=np.full(720,math.inf);right[0:360]=2.
    ScanMap.receive(node,full_scan(9.90,left))
    ScanMap.receive(node,full_scan(9.95,right))
    assert len(node.layers)==2
    node.now=10.3
    ScanMap.receive(node,full_scan(10.2,right))   # 9.90 is now > 0.25 s old
    assert [round(l[0],2) for l in node.layers]==[9.95,10.2]


def test_stationary_footprint_not_blocked_by_blind_band_with_open_front():
    """End-to-end: unknown sides/rear no longer trip INITIAL_HARD_BOUNDARY."""
    from wheelchair_hardware_bridge.scan_map import rasterize,clear_blind_band
    from wheelchair_control.planner import Config
    from wheelchair_control.geometry import clearance,transform
    ranges=np.full(720,math.inf);ranges[260:460]=3.   # only a +-50 deg front cone sees returns
    res=.05;cells,origin=rasterize(ranges,-math.pi,math.pi/360,.15,8.,(0,0,0),res,10.)
    cfg=Config(footprint=((-.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)),margin=.06)
    n=cells.shape[0];w=n*res
    boundary=((origin[0],origin[1]),(origin[0]+w,origin[1]),(origin[0]+w,origin[1]+w),(origin[0],origin[1]+w))
    fp=transform(list(cfg.footprint),(0,0,0))
    before=occupied_rectangles(cells.ravel(),n,n,res,origin)
    assert clearance(fp,before,boundary,cfg.padding)<=cfg.padding
    after=occupied_rectangles(clear_blind_band(cells,origin,res,(0,0,0),.10).ravel(),n,n,res,origin)
    assert clearance(fp,after,boundary,cfg.padding)>cfg.padding


def test_odom_lever_arm_and_rotation():
    msg=Odometry();msg.pose.pose.orientation.w=1.;msg.twist.twist.angular.z=2.
    t=Transform();t.translation.x=1.;t.rotation.z=math.sqrt(.5);t.rotation.w=math.sqrt(.5)
    out=convert_odom(msg,t)
    assert out.pose.pose.position.x==pytest.approx(1.)
    assert out.pose.pose.orientation.z==pytest.approx(math.sqrt(.5))
    assert out.twist.twist.linear.x==pytest.approx(2.)
    assert out.twist.twist.linear.y==pytest.approx(0.,abs=1e-10)
    assert msg.pose.pose.position.x==0.  # source is unchanged


def test_cloud_layout_handles_big_endian_padding_and_field_offsets():
    cloud=PointCloud2(height=2,width=1,point_step=20,row_step=24,is_bigendian=True)
    cloud.fields=[PointField(name=n,offset=i,datatype=PointField.FLOAT32,count=1)
                  for n,i in [('x',4),('y',12),('z',16)]]
    data=bytearray(48)
    for row in range(2):
        for offset,value in [(4,1.+row),(12,2.+row),(16,3.+row)]:
            struct.pack_into('>f',data,row*24+offset,value)
    cloud.data=bytes(data)
    assert np.allclose(list(_iter_xyz(cloud)),[[1,2,3],[2,3,4]])


@pytest.mark.parametrize('dynamic', [False, True])
def test_map_updates_remain_validated_in_both_modes(dynamic):
    from wheelchair_interfaces.msg import ObstacleMap
    from wheelchair_hardware_bridge.scan_map import polygon
    from wheelchair_control.nodes import PlannerNode
    from wheelchair_control.protocol import StreamGuard
    from wheelchair_control.planner import Config
    reasons=[]
    clock=SimpleNamespace(nanoseconds=10_000_000_000)
    node=SimpleNamespace(cfg=Config(),allow_dynamic_map=dynamic,
        map_guard=StreamGuard(Config().map_age),map_geometry=None,core=SimpleNamespace(trip=reasons.append),
        get_clock=lambda:SimpleNamespace(now=lambda:clock))
    msg=ObstacleMap();msg.header.frame_id='odom';msg.header.stamp.sec=10
    msg.session_id='sensor';msg.sequence=1;msg.complete=True
    msg.known_free_boundary=polygon(((-5,-5),(5,-5),(5,5),(-5,5)))
    msg.obstacles=[polygon(((1,1),(2,1),(2,2),(1,2)))]
    PlannerNode.on_map(node,msg);assert node.map_valid
    reasons.clear();msg.sequence+=1;msg.obstacles[0].points[0].x=1.1
    PlannerNode.on_map(node,msg);assert node.map_valid
    assert 'PASSAGE_MAP_CHANGED' not in reasons
    msg.sequence+=1;msg.obstacles[0].points[0].x=float('nan')
    PlannerNode.on_map(node,msg);assert not node.map_valid
    msg.complete=False;msg.sequence+=1
    PlannerNode.on_map(node,msg);assert not node.map_valid


def test_odom_old_device_clock_is_rejected_not_restamped():
    from wheelchair_hardware_bridge.odom_relay import OdomRelay
    from rclpy.time import Time
    warnings=[]
    node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=1_790_000_000,clock_type=ClockType.ROS_TIME)),
                         get_logger=lambda:SimpleNamespace(warning=lambda *args,**kwargs:warnings.append(args)))
    msg=Odometry();msg.header.stamp.sec=4000
    assert not OdomRelay.fresh(node,msg)
    assert msg.header.stamp.sec==4000 and warnings
    msg.header.stamp.sec=1_790_000_000
    assert OdomRelay.fresh(node,msg)


@pytest.mark.parametrize('second', ['missing','stale','fresh'])
def test_scan_requires_both_current_sensors(monkeypatch,second):
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    from wheelchair_hardware_bridge import cloud_to_scan as module
    from wheelchair_hardware_bridge.cloud_to_scan import CloudToScan
    values={'ground_filter_enabled':False,'cloud_sync_window':.3,'target_frame':'base_link','height_min':-.15,
            'height_max':-.05,'range_min':.15,'range_max':8.,'angle_min':-math.pi,
            'angle_max':math.pi,'angular_resolution':.5}
    a=PointCloud2();a.header.stamp.sec=10
    b=PointCloud2();b.header.stamp.sec=9 if second=='stale' else 10
    published=[]
    node=SimpleNamespace(get_parameter=lambda name:Parameter(name,value=values[name]),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10,clock_type=ClockType.ROS_TIME)),
        _clouds={'a':a,'b':None if second=='missing' else b},
        _tf_buffer=None,_pub=SimpleNamespace(publish=published.append))
    monkeypatch.setattr(module,'_cloud_to_base',lambda *args:np.array([[2.,0.,-.1]]))
    CloudToScan._try_publish(node)
    assert bool(published)==(second=='fresh')
    if published:
        scan=published[0]
        assert scan.angle_max==pytest.approx(scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment)
        assert min(scan.ranges)==pytest.approx(2.)


@pytest.mark.parametrize('yaw_deg,side', [(0.,-.21),(-1.,.21)])
def test_inverted_mid360_tf_applied_once_before_height_filter(yaw_deg,side):
    """Known floor/obstacle points survive local -> base with roll=180 degrees."""
    from wheelchair_hardware_bridge.cloud_to_scan import _cloud_to_base
    roll,pitch,yaw=math.pi,math.radians(-1.),math.radians(yaw_deg)
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    t=Transform();t.translation.x=.28;t.translation.y=side;t.translation.z=-.10
    t.rotation.x=sr*cp*cy-cr*sp*sy;t.rotation.y=cr*sp*cy+sr*cp*sy
    t.rotation.z=cr*cp*sy-sr*sp*cy;t.rotation.w=cr*cp*cy+sr*sp*sy
    # Independent fixed-axis rotation, used only to construct raw sensor points.
    rx=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    ry=np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    expected=np.array([[2.,0.,-.503],[2.,0.,-.153],[1.,.7,.197]])
    raw=(expected-np.array([.28,side,-.10]))@(rz@ry@rx)
    assert raw[0,2]>0  # Ground is positive sensor Z for the inverted mount.
    cloud=PointCloud2(height=1,width=3,point_step=12,row_step=36)
    cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                  for i,n in enumerate(('x','y','z'))]
    cloud.data=np.asarray(raw,dtype='<f4').tobytes()
    cloud.header.frame_id='mid360_raw'
    calls=[]
    def lookup(*args,**kwargs):
        calls.append(args);return SimpleNamespace(transform=t)
    actual=_cloud_to_base(cloud,SimpleNamespace(lookup_transform=lookup),'base_link')
    assert np.allclose(actual,expected,atol=1e-6)
    assert calls[0][:2]==('base_link','mid360_raw')
    sliced=actual[(actual[:,2]>=-.203)&(actual[:,2]<=-.103)]
    assert len(sliced)==1 and np.allclose(sliced[0],expected[1],atol=1e-6)


def test_preflight_rejects_existing_map_even_with_same_node_names():
    from wheelchair_hardware_bridge.preflight import publisher_conflicts
    publisher=SimpleNamespace(node_namespace='/',node_name='scan_map')
    node=SimpleNamespace(get_publishers_info_by_topic=lambda topic:
                         [publisher,publisher] if topic=='/wheelchair/planning_map' else [])
    conflicts=publisher_conflicts(node)
    assert len(conflicts)==1 and '2 publisher(s)' in conflicts[0]
    node.get_publishers_info_by_topic=lambda topic:[]
    assert publisher_conflicts(node)==[]



def test_wider_height_band_keeps_nearest_obstacle_and_rejects_floor(monkeypatch):
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    from wheelchair_hardware_bridge import cloud_to_scan as module
    values={'ground_filter_enabled':False,'cloud_sync_window':.3,'target_frame':'base_link','height_min':-.203,
            'height_max':.397,'range_min':.15,'range_max':20.,'angle_min':-math.pi,
            'angle_max':math.pi,'angular_resolution':.5}
    cloud=PointCloud2();cloud.header.stamp.sec=10
    published=[]
    node=SimpleNamespace(get_parameter=lambda name:Parameter(name,value=values[name]),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10,clock_type=ClockType.ROS_TIME)),
        _clouds={'a':cloud,'b':cloud},_tf_buffer=None,
        _pub=SimpleNamespace(publish=published.append))
    # Ground at 0.6 m must not become a return; old-band nearer obstacle wins
    # over a more distant upper-band return. A different upper-band ray is added.
    points=np.array([[.6,0.,-.503],[2.,0.,-.153],[3.,0.,.197],[0.,2.,.197]])
    monkeypatch.setattr(module,'_cloud_to_base',lambda *args:points)
    module.CloudToScan._try_publish(node)
    assert len(published)==1
    scan=published[0]
    assert scan.ranges[360]==pytest.approx(2.)
    assert scan.ranges[540]==pytest.approx(2.)
    assert sum(math.isfinite(x) for x in scan.ranges)==2
