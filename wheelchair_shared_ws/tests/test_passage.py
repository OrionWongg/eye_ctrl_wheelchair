import math
import pytest
from wheelchair_control.planner import Config,Planner,State,toward,integrate
from wheelchair_control.protocol import Packet,STOP,FORWARD,LEFT,RIGHT
from wheelchair_control.executor import Executor
from wheelchair_control.passage import detect
from wheelchair_control.geometry import IndexedObstacles

BOUND=[(-10,-10),(10,-10),(10,10),(-10,10)]
def box(x0,x1,y0,y1):return [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
def config():return Config(mode='corridor_assist',footprint=((-0.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)),margin=.06,comfort_distance=.15,corridor_heading_enter=math.radians(.8))
def walls(width,length=7):return IndexedObstacles([box(-1,length,width/2,width/2+.12),box(-1,length,-width/2-.12,-width/2)])
def authorize(core,state,obs):
    core.intent(Packet('test',1,0,.25,STOP,False),True);core.tick(state,obs,BOUND,0)
    assert core.ready
    core.intent(Packet('test',2,0,.25,FORWARD,True))

def test_soft_distance_does_not_lock_ready_but_hard_boundary_does():
    c=config();core=Planner(c)
    assert c.padding<.15
    obs=walls(.79);s=State()
    core.intent(Packet('test',1,0,.25,STOP,False),True);core.tick(s,obs,BOUND,0)
    assert core.ready # 95mm gap: inside comfort distance, outside hard envelope.
    core.tick(State(y=.015),obs,BOUND,1);assert not core.ready

@pytest.mark.parametrize('width',[.80,.90,1.,1.10])
@pytest.mark.parametrize('angle',[-10,-5,-3,0,3,5,10])
def test_parallel_geometry_matrix(width,angle):
    c=config();s=State(yaw=math.radians(angle));obs=walls(width)
    from wheelchair_control.planner import Predictor
    result=Predictor(c).check(s,STOP,obs,BOUND)
    # Compare the actual asymmetric full rectangle projection at the pose.
    from wheelchair_control.geometry import transform
    ys=[p[1] for p in transform(c.footprint,s.pose)]
    static_gap=width/2-max(abs(min(ys)),abs(max(ys)))
    assert result.safe==(static_gap>c.padding)
    if not result.safe:assert result.failure['initial'] and 'obstacle_id' in result.failure

def test_geometry_detection_and_authority():
    c=config();p,reason=detect(State(yaw=.05),walls(.9),c)
    assert not reason and p.width==pytest.approx(.9) and p.kind=='corridor'
    assert detect(State(yaw=math.pi/4),walls(.9),c)[0] is None
    assert detect(State(),IndexedObstacles([box(-1,5,.45,.55)]),c)[0] is None
    door,reason=detect(State(x=-1),IndexedObstacles([box(0,.3,.45,3),box(0,.3,-3,-.45)]),c)
    assert not reason and door.kind=='door'

def test_stop_and_explicit_turn_cancel_passage():
    core=Planner(config());obs=walls(1.1);s=State(yaw=.1)
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    assert core.local is not None
    core.intent(Packet('test',3,0,.25,STOP,False));assert core.local is None
    core.intent(Packet('test',4,0,.25,RIGHT,True))
    assert core.tick(s,obs,BOUND,.2).action in (STOP,RIGHT)

def test_repeated_micro_corrections_and_progress():
    c=config();core=Planner(c);obs=walls(.90);s=State(yaw=math.radians(5))
    authorize(core,s,obs);executor=Executor(c);executor.request(STOP,0)
    executor.tick(0,0,0,True,True);executor.tick(.2,0,0,True,True)
    actions=[];corrections=0
    for i in range(1,601):
        now=.2+i*.05
        stopped=abs(s.v)<.015 and abs(s.w)<.025
        d=core.tick(s,obs,BOUND,now,stopped=stopped)
        assert not core.latch,(d,s)
        executor.request(d.action,now);a,_=executor.tick(now,s.v,s.w,True,True);actions.append(a)
        tv,tw=c.velocity(a);v=toward(s.v,tv,.6*.05);w=toward(s.w,tw,.8*.05)
        moved=integrate(s,(s.v+v)/2,(s.w+w)/2,.05)
        s=State(moved.x,moved.y,moved.yaw,v,w,d.action if executor.phase=='STARTING' else a,.03,{'RUNNING':1,'STARTING':3,'STOPPED':0,'STOPPING':2,'FAULT':4}[executor.phase])
        if core.local:corrections=max(corrections,core.local.corrections)
        if s.x>2:break
    assert s.x>2 and corrections>=2,(s,corrections)
    assert set(actions)<={STOP,FORWARD,LEFT,RIGHT}

def test_straight_swept_hull_checks_middle_and_braking_tail():
    from wheelchair_control.planner import Predictor
    c=config();p=Predictor(c);s=State(v=.372,current_action=FORWARD,observation_age=.03)
    # Endpoint can be beyond a thin obstacle; continuous sweep must still reject.
    obstacle=IndexedObstacles([box(.9,.91,-.4,.4)])
    result=p.check(s,FORWARD,obstacle,BOUND,duration=3.)
    assert not result.safe and result.failure['time']>0
    assert result.failure['obstacle_id']
    # Compare the straight optimized branch to the sampled branch, forced by a
    # parallel fixed reference, on free space and a blocked path.
    for obs in (walls(.90),obstacle):
        fast=p.check(s,FORWARD,obs,BOUND,duration=3.)
        slow=p.check(s,FORWARD,obs,BOUND,duration=3.,reference=s.pose,swept_hull=False)
        assert fast.safe==slow.safe
        if fast.safe:assert fast.endpoint.pose==pytest.approx(slow.endpoint.pose)

def test_door_blocked_exit_and_branch_do_not_start_navigation():
    from wheelchair_control.passage import PassageController
    c=config();core=Planner(c);s=State()
    obs=IndexedObstacles([box(1.05,1.35,.45,4),box(1.05,1.35,-4,-.45),box(1.8,1.9,-.6,.6)])
    authorize(core,s,obs)
    core.tick(s,obs,BOUND,.1)
    d=core.tick(s,obs,BOUND,.15)
    assert d.action==STOP and core.latch
    assert d.reason.startswith('PASSAGE_BLOCKED')

def test_rviz_markers_replace_stale_decision():
    from builtin_interfaces.msg import Time
    from wheelchair_control.visualization import safety_markers
    from visualization_msgs.msg import Marker
    m=safety_markers(Time(sec=1),42,State(),config(),{})
    assert m.markers[0].action==Marker.DELETEALL
    assert m.markers[-1].text=='decision 42'

def test_open_room_preserves_user_forward_without_autonomous_turn():
    core=Planner(config());s=State(yaw=.6)
    authorize(core,s,IndexedObstacles([]))
    result=core.tick(s,IndexedObstacles([]),BOUND,.1)
    assert result.action==FORWARD and result.reason=='USER_FORWARD_NO_PASSAGE'

def test_observation_does_not_interrupt_executor_startup():
    core=Planner(config());obs=walls(.9);s=State()
    authorize(core,s,obs)
    assert core.tick(s,obs,BOUND,.1).action==STOP # initial geometry observation
    # While the executor is still STARTING / reporting stopped, identical
    # FORWARD renewals must not be replaced by periodic observation STOPs.
    for i in range(1,21):
        result=core.tick(s,obs,BOUND,.1+i*.05,stopped=True)
        assert result.action==FORWARD,result

def test_straight_hull_keeps_small_angular_feedback_residuals():
    from wheelchair_control.planner import Predictor
    c=config();p=Predictor(c);s=State(v=.372,w=1e-7,current_action=FORWARD,observation_age=.05)
    fast=p.check(s,FORWARD,walls(.9),BOUND,duration=4.)
    sampled=p.check(s,FORWARD,walls(.9),BOUND,duration=4.,swept_hull=False)
    assert fast.safe==sampled.safe
    assert fast.endpoint.pose==pytest.approx(sampled.endpoint.pose,abs=1e-12)
    assert fast.endpoint.yaw!=0 # no fabricated zero angular feedback

@pytest.mark.parametrize('heading',[0.,.6,math.pi/2])
def test_open_room_moving_forward_never_waits_for_observation_stop(heading):
    core=Planner(config());obs=IndexedObstacles([])
    authorize(core,State(yaw=heading),obs)
    for i in range(1,25):
        state=State(x=i*.01*math.cos(heading),y=i*.01*math.sin(heading),yaw=heading,
                    v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1)
        decision=core.tick(state,obs,BOUND,i*.05,stopped=False)
        assert decision.action==FORWARD and decision.reason=='USER_FORWARD_NO_PASSAGE'
        assert not core.latch

def test_open_room_motion_still_stops_for_obstacle():
    core=Planner(config());obs=IndexedObstacles([box(.9,1.,-2.,2.)])
    s=State(yaw=.01);authorize(core,s,obs)
    moving=State(v=.372,current_action=FORWARD,observation_age=.04,execution_phase=1)
    decision=core.tick(moving,obs,BOUND,.1,stopped=False)
    assert decision.action==STOP and core.latch

def test_tracked_confidence_extension_still_requires_stop():
    core=Planner(config());obs=walls(.9);s=State()
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    core.local.refresh_needed=True
    decision=core.tick(State(v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1),obs,BOUND,.2,stopped=False)
    assert decision.action==STOP and decision.reason=='PASSAGE_WAIT_OBSERVATION_STOP'

def side_door_map(both_sides=False,blocked=False):
    polygons=[box(-1,1.,-.57,-.45),box(2.,6.,-.57,-.45)]
    polygons+=([box(-1,1.,.45,.57),box(2.,6.,.45,.57)] if both_sides else [box(-1,6.,.45,.57)])
    if blocked:polygons.append(box(1.4,1.5,-.3,.3))
    return IndexedObstacles(polygons)

def test_side_door_keeps_main_axis_but_crossing_and_obstacle_do_not_bridge():
    c=config()
    straight,reason=detect(State(),side_door_map(),c)
    assert not reason and straight.end>2.5 and straight.center==pytest.approx(0.)
    for obs in (side_door_map(True),side_door_map(blocked=True)):
        passage,_=detect(State(),obs,c)
        assert passage.end<1.2

def test_explicit_turn_then_forward_does_not_snap_back_to_old_corridor():
    c=config();core=Planner(c);obs=walls(1.1);s=State()
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    assert core.local.passage.heading==0.
    core.intent(Packet('test',3,0,.25,RIGHT,True))
    core.tick(s,obs,BOUND,.2)
    # Stopping the manual rotation must not erase the human's route choice.
    core.intent(Packet('test',4,0,.25,STOP,False));core.tick(s,obs,BOUND,.3)
    core.intent(Packet('test',5,0,.25,FORWARD,True))
    d=core.tick(State(yaw=-.15),obs,BOUND,.4)
    assert d.action==FORWARD
    assert core.local.passage is None
    assert core.last_diagnostics['passage_detection']=='USER_HEADING_PRIORITY'

def test_human_turn_can_select_a_perpendicular_door():
    c=config();core=Planner(c);s=State()
    authorize(core,s,walls(1.1));core.tick(s,walls(1.1),BOUND,.1)
    core.intent(Packet('test',3,0,.25,RIGHT,True));core.tick(s,walls(1.1),BOUND,.2)
    core.intent(Packet('test',4,0,.25,FORWARD,True))
    # Door lies in the wall y=-1.2, selected by facing -Y, not by its existence.
    doorway=IndexedObstacles([box(-4,-.45,-1.35,-1.05),box(.45,4,-1.35,-1.05)])
    d=core.tick(State(yaw=-math.pi/2),doorway,BOUND,.3)
    assert d.action==STOP and core.local.passage.kind=='door'
    assert core.local.passage.heading==pytest.approx(-math.pi/2)
    assert core.local.passage.heading!=core.manual_departure_axis

def test_locked_axis_survives_later_observation_of_far_door_jamb():
    c=config()
    obs=IndexedObstacles([box(-1,7,.45,.57),box(-1,1.8,-.57,-.45),box(3.1,7,-.57,-.45)])
    first,_=detect(State(),obs,c)
    assert first.end<2.
    extended,_=detect(State(x=1.5),obs,c,first)
    assert extended.end>4. and extended.origin==first.origin
    assert extended.heading==first.heading and extended.center==first.center

@pytest.mark.parametrize('width,offset',[(.82,.01),(.85,.03),(.9,.05)])
@pytest.mark.parametrize('side',[-1,1])
def test_parallel_off_center_corridor_continues_when_centering_is_infeasible(width,offset,side):
    core=Planner(config());obs=walls(width);s=State(y=side*offset,observation_age=.03)
    assert core.predictor.check(s,FORWARD,obs,BOUND).safe
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    decision=core.tick(s,obs,BOUND,.15)
    assert decision.action==FORWARD and not core.latch
    assert core.last_diagnostics['alignment_deferred']=='NO_EXECUTABLE_CORRECTION'
    # Follow the same parallel line for 2 m, including confidence extensions.
    # Renewals must never add a stop just to retry the impossible centering turn.
    for i in range(1,109):
        moving=State(x=i*.05*core.cfg.forward_speed,y=s.y,v=core.cfg.forward_speed,
                     current_action=FORWARD,observation_age=.03,execution_phase=1)
        decision=core.tick(moving,obs,BOUND,.15+i*.05,stopped=False)
        assert decision.action==FORWARD and not core.latch,decision
        assert core.last_diagnostics['candidate']['safe']

def test_deferred_centering_still_stops_for_new_obstacle():
    core=Planner(config());obs=walls(.9);s=State(y=.05,observation_age=.03)
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    assert core.tick(s,obs,BOUND,.15).action==FORWARD
    blocked=IndexedObstacles([box(-1,7,.45,.57),box(-1,7,-.57,-.45),box(.75,.85,-.45,.45)])
    moving=State(y=.05,v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1)
    assert core.tick(moving,blocked,BOUND,.2,stopped=False).action==STOP
    assert core.local.alignment_unavailable_at is None
    decision=core.tick(s,blocked,BOUND,.3)
    assert decision.action==STOP and core.latch

def test_deferred_centering_reconsiders_heading_drift():
    core=Planner(config());obs=walls(.9);s=State(y=.05,observation_age=.03)
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    assert core.tick(s,obs,BOUND,.15).action==FORWARD
    moving=State(y=.05,yaw=.01,v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1)
    decision=core.tick(moving,obs,BOUND,.2,stopped=False)
    assert decision.action==STOP and decision.reason=='PASSAGE_STOP_TO_ALIGN'
    assert core.local.alignment_unavailable_at is None

def test_centering_fallback_cannot_cross_corridor_end():
    core=Planner(config());obs=walls(.9,length=.7);s=State(y=.05,observation_age=.03)
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    assert core.tick(s,obs,BOUND,.15).action==FORWARD
    moving=State(x=.35,y=.05,v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1)
    decision=core.tick(moving,obs,BOUND,.2,stopped=False)
    assert decision.action==STOP and decision.reason=='PASSAGE_END_OR_BRANCH'

def test_centering_fallback_still_checks_full_door_exit():
    core=Planner(config());s=State(x=-.1,y=.05,observation_age=.03)
    obs=IndexedObstacles([box(0,.3,.45,3),box(0,.3,-3,-.45),box(1.05,1.15,-.45,.45)])
    assert core.predictor.check(s,FORWARD,obs,BOUND).safe
    authorize(core,s,obs);core.tick(s,obs,BOUND,.1)
    assert core.local.passage.kind=='door'
    decision=core.tick(s,obs,BOUND,.15)
    assert core.last_diagnostics['alignment_deferred']=='NO_EXECUTABLE_CORRECTION'
    assert decision.action==STOP and core.latch
    assert decision.reason=='PASSAGE_BLOCKED_COLLISION_OR_UNKNOWN'
