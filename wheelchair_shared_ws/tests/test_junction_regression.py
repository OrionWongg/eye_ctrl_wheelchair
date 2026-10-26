import math
from types import SimpleNamespace
import pytest
from wheelchair_control.navigation import NavigationScope
from wheelchair_control.passage import Passage
from wheelchair_control.planner import Planner, State
from wheelchair_control.protocol import Packet
from wheelchair_control.nav2_transport import Nav2PathClient
from test_passage import config, BOUND


@pytest.mark.parametrize('heading',[0.,math.pi/2,-math.pi/2])
def test_office_request_uses_centerline_independent_of_robot_yaw(heading):
    c,s=math.cos(heading),math.sin(heading)
    rotate=lambda x,y:(c*x-s*y,s*x+c*y)
    start=(*rotate(2.5849175,.8221296),heading+.3567718)
    goal=(*rotate(4.0656133,1.05),heading)
    owner=Planner(config());captured={}
    def request(*args,**kwargs):captured.update(kwargs);return 1
    owner.path_provider=SimpleNamespace(request=request)
    scope=NavigationScope(owner,Packet('test',1,0.,.25))
    scope.passage=Passage(rotate(0.,0.),heading,1.05,1.4,0.,10.,False,'corridor')
    scope.request(start,[goal],'CRUISE',.25)
    origin=captured['corridor_origin']
    lateral=lambda p,o:-(p[0]-o[0])*math.sin(o[2])+(p[1]-o[1])*math.cos(o[2])
    assert abs(lateral(goal,start))>.25  # Original failing constraint.
    assert abs(lateral(start,origin))<.25
    assert lateral(goal,origin)==pytest.approx(0.)


def test_grid_cache_preserves_unknown_and_invalidates_moved_map():
    from nav_msgs.msg import OccupancyGrid
    client=Nav2PathClient.__new__(Nav2PathClient)
    client.node=SimpleNamespace(cfg=SimpleNamespace(costmap_timeout=.5))
    client.ros_now=lambda:10.
    client.grid_signature=None
    m=OccupancyGrid();m.header.frame_id='odom';m.header.stamp.sec=10
    m.info.width=2;m.info.height=2;m.info.resolution=.05;m.info.origin.orientation.w=1.
    m.data=[100,0,-1,20]
    client.grid(m);first=client.obstacles
    m.data=[100,50,-1,99];client.grid(m)
    assert client.obstacles is first
    m.info.origin.position.x=.05;client.grid(m)
    assert client.obstacles is not first
    second=client.obstacles;m.data=[0,50,-1,99];client.grid(m)
    assert client.obstacles is not second


@pytest.mark.parametrize('x',[0.,.2,.8,1.2])
@pytest.mark.parametrize('w',[0.,.00001,.1])
def test_goal_feasibility_without_diagnostics_keeps_collision_result(x,w):
    from wheelchair_control.planner import Predictor
    from wheelchair_control.protocol import STOP,FORWARD
    from wheelchair_control.geometry import IndexedObstacles
    predictor=Predictor(config())
    obstacle=IndexedObstacles([((1.,-.8),(1.2,-.8),(1.2,.8),(1.,.8))])
    state=State(x=x,v=.372,w=w,current_action=FORWARD,observation_age=.15)
    detailed=predictor.check(state,STOP,obstacle,BOUND)
    feasibility=predictor.check(state,STOP,obstacle,BOUND,diagnostics=False)
    assert detailed.safe==feasibility.safe
    assert detailed.clearance==pytest.approx(feasibility.clearance)
    assert detailed.endpoint.pose==pytest.approx(feasibility.endpoint.pose)


def test_map_heartbeat_reuses_geometry_but_changes_are_validated():
    from geometry_msgs.msg import Point32,Polygon
    from wheelchair_interfaces.msg import ObstacleMap
    from wheelchair_control.nodes import PlannerNode
    from wheelchair_control.protocol import StreamGuard
    now=SimpleNamespace(nanoseconds=10_000_000_000)
    node=SimpleNamespace(cfg=config(),map_guard=StreamGuard(.5),
        map_geometry=None,allow_dynamic_map=False,core=SimpleNamespace(trip=lambda reason:None),
        get_clock=lambda:SimpleNamespace(now=lambda:now))
    msg=ObstacleMap();msg.header.frame_id='odom';msg.header.stamp.sec=10
    msg.complete=True;msg.session_id='test';msg.sequence=1
    poly=lambda pts:Polygon(points=[Point32(x=float(x),y=float(y)) for x,y in pts])
    msg.known_free_boundary=poly(BOUND)
    msg.obstacles=[poly(((1.,1.),(2.,1.),(2.,2.),(1.,2.)))]
    PlannerNode.on_map(node,msg);first=node.obstacles
    assert node.map_valid
    for _ in range(100):
        msg.sequence+=1;PlannerNode.on_map(node,msg)
        assert node.obstacles is first and node.map_valid
    msg.sequence+=1;msg.obstacles[0].points[0].x=1.2
    PlannerNode.on_map(node,msg)
    assert node.obstacles is not first and node.map_valid
    msg.sequence+=1;msg.obstacles[0].points[0].x=float('nan')
    PlannerNode.on_map(node,msg)
    assert not node.map_valid


def test_compressed_grid_exactly_preserves_random_occupied_cells():
    import numpy as np
    from wheelchair_control.nav2_transport import occupied_rectangles
    rng=np.random.default_rng(178)
    for height,width in ((1,1),(1,25),(20,1),(17,23)):
        cells=rng.choice([-1,0,50,99,100],size=(height,width))
        polygons=occupied_rectangles(cells.ravel(),width,height,1.,(0.,0.))
        restored=np.zeros_like(cells,dtype=bool)
        for poly in polygons:
            x0,x1,y0,y1=map(int,poly.aabb)
            assert not restored[y0:y1,x0:x1].any()
            restored[y0:y1,x0:x1]=True
        np.testing.assert_array_equal(restored,(cells<0)|(cells>=100))


@pytest.mark.parametrize('behavior',['stop','continue_straight'])
def test_corridor_exit_policy_and_release(behavior):
    from dataclasses import replace
    from test_passage import walls
    c=replace(config(),nav_corridor_end_behavior=behavior)
    owner=Planner(c);scope=NavigationScope(owner,Packet('test',1,0.,.25))
    obs=walls(1.4,length=2.)
    scope.boundary=BOUND
    scope.update_context(State(),obs,0.)
    assert scope.passage is not None
    goal=scope.cruise_goal((1.2,0.,0.),State(x=1.2),obs)
    if behavior=='stop':
        assert goal is None or goal[0]<2.
    else:
        assert goal[0]>3.
        scope.update_context(State(x=2.2),obs,1.)
        assert scope.passage is not None
        scope.update_context(State(x=2.8),obs,2.)
        assert scope.passage is None


def test_continue_straight_still_respects_front_wall_braking():
    from dataclasses import replace
    from test_passage import walls,box
    from wheelchair_control.geometry import IndexedObstacles
    from wheelchair_control.protocol import STOP,FORWARD
    c=replace(config(),nav_corridor_end_behavior='continue_straight')
    owner=Planner(c);scope=NavigationScope(owner,Packet('test',1,0.,.25))
    obs=IndexedObstacles([*walls(1.4,length=2.),box(2.5,2.7,-2.,2.)])
    scope.boundary=BOUND;scope.update_context(State(),obs,0.)
    goal=scope.cruise_goal((1.,0.,0.),State(x=1.),obs)
    assert goal is not None and goal[0]<2.
    stop=owner.predictor.check(State(x=goal[0],v=c.forward_speed,current_action=FORWARD),STOP,obs,BOUND)
    assert stop.safe


def test_reported_office_exit_pose_has_forward_goal_with_new_policy():
    from dataclasses import replace
    from wheelchair_sim.scenes import map_obstacles,boundary
    from wheelchair_control.geometry import IndexedObstacles
    obs=IndexedObstacles([b.polygon() for b in map_obstacles('clearpath_office')])
    state=State(x=9.30005,y=1.08833,yaw=.00293)
    for behavior in ('stop','continue_straight'):
        owner=Planner(replace(config(),nav_corridor_end_behavior=behavior))
        scope=NavigationScope(owner,Packet('test',1,0.,.25,
            reference_x=state.x,reference_y=state.y,reference_yaw=state.yaw))
        scope.boundary=boundary('clearpath_office')
        scope.update_context(state,obs,0.)
        goal=scope.cruise_goal(state.pose,state,obs)
        if behavior=='stop':assert goal is None
        else:assert goal==pytest.approx((11.30005,1.05,0.))
