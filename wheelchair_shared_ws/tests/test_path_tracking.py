import math
import pytest
from dataclasses import replace
from wheelchair_control.path_tracking import PathFollower,segments,direct_path
from wheelchair_control.direction_manager import TURN_AROUND,NAV_UTURN
from wheelchair_control.task_manager import Event
from wheelchair_control.planner import Planner,State,integrate,toward
from wheelchair_control.protocol import STOP,FORWARD,LEFT,RIGHT
from wheelchair_control.executor import Executor
from wheelchair_control.geometry import IndexedObstacles,clearance,transform,wrap
from wheelchair_control.nav2_transport import occupied_rectangles
from nav_fixtures import config,BOUND,walls
from nav_fixtures import nav_authorize
from test_directional import manager_start
from test_task_manager import feed


def simulate(poses,start,obstacles=(),limit=900):
    c=config();core=Planner(c);core.latch=False
    follower=PathFollower(core,poses,'test_path');state=start
    executor=Executor(c);executor.request(STOP,0);executor.tick(0,0,0,True,True);executor.tick(.2,0,0,True,True)
    actions=[];obs=IndexedObstacles(obstacles)
    for i in range(limit):
        now=.25+i*.05;core.last_diagnostics={}
        stopped=executor.phase=='STOPPED' and abs(state.v)<.015 and abs(state.w)<.025
        d=follower.tick(state,obs,BOUND,now,stopped)
        assert not core.latch,(d,state)
        bound=core.last_diagnostics.get('task_pulse_duration') if d.reason=='TASK_HEADING_MICRO_TURN' else None
        executor.request(d.action,now,bound);a,_=executor.tick(now,state.v,state.w,True,True)
        actions.append(a);tv,tw=c.velocity(a)
        v=toward(state.v,tv,(c.linear_brake if a==STOP else c.linear_accel)*.05)
        w=toward(state.w,tw,(c.angular_brake if a==STOP else c.angular_accel)*.05)
        moved=integrate(state,(state.v+v)/2,(state.w+w)/2,.05)
        state=State(moved.x,moved.y,moved.yaw,v,w,d.action if executor.phase=='STARTING' else a,.03,
            {'STOPPED':0,'RUNNING':1,'STARTING':3,'STOPPING':2,'FAULT':4}[executor.phase])
        assert clearance(transform(c.footprint,state.pose),obs,BOUND,c.padding)>c.padding
        if follower.done:return state,actions,follower
    pytest.fail(f'Path did not finish: {state} index={follower.index}')


@pytest.mark.parametrize('initial',[0.,math.radians(179),math.radians(-179)])
@pytest.mark.parametrize('sign',[-1,1])
def test_half_turn_crosses_pi_without_changing_direction(initial,sign):
    poses=[(0.,0.,initial+sign*math.pi*i/90) for i in range(91)]
    state,actions,follower=simulate(poses,State(yaw=initial,observation_age=.03,execution_phase=0))
    assert follower.done and abs(wrap(state.yaw-initial-sign*math.pi))<config().task_heading_tolerance
    assert set(actions)<={STOP,LEFT if sign>0 else RIGHT}
    assert abs(state.w)<.025 and abs(state.v)<.015


def test_many_collinear_points_are_one_continuous_motion():
    poses=[(i*.1,0.,0.) for i in range(21)]
    assert len(segments(poses))==1
    state,actions,_=simulate(poses,State(observation_age=.03,execution_phase=0))
    assert abs(state.x-2.)<config().nav_position_tolerance
    assert sum(a!=STOP and b==STOP for a,b in zip(actions,actions[1:]))==1


def test_non_axis_aligned_polyline_uses_same_tracker():
    poses=direct_path((0.,0.,0.),[(1.,1.,math.pi/4),(1.,2.,math.pi/2)])
    state,actions,_=simulate(poses,State(observation_age=.03,execution_phase=0))
    assert math.dist(state.pose[:2],(1.,2.))<.1
    assert abs(wrap(state.yaw-math.pi/2))<config().task_heading_tolerance
    assert {FORWARD,LEFT}.issubset(actions)


def test_final_straight_pose_corrects_drift_inside_drive_deadband():
    state,actions,follower=simulate([(0.,0.,0.),(1.,0.,0.)],
        State(yaw=.05,observation_age=.03,execution_phase=0))
    assert follower.done
    assert math.dist(state.pose[:2],(1.,0.))<.08
    assert abs(state.yaw)<=config().task_heading_tolerance
    assert RIGHT in actions


def test_straight_tracking_keeps_acceptable_offset_without_corrective_pulses():
    state,actions,follower=simulate([(0.,0.,0.),(1.,0.,0.)],
        State(y=.04,observation_age=.03,execution_phase=0))
    assert follower.done
    assert abs(state.x-1.)<config().nav_position_tolerance
    assert state.y==pytest.approx(.04)
    assert set(actions)<={STOP,FORWARD}


@pytest.mark.parametrize('path',[[(0,0,0),(-1,0,0)],[(0,0,0),(0,1,0)],[(0,0,0),(1,1,.3)]])
def test_unsupported_paths_are_rejected_instead_of_quantized(path):
    with pytest.raises(ValueError):segments(path)


def test_uturn_replaces_task_only_after_stopped_and_duplicates_do_not_reset():
    m=manager_start();feed(m,.6,v=.2)
    event=Event('source',2,.6,1.5,TURN_AROUND);m.on_event(event,.6,.6)
    assert not m.tick(.6,.6).enabled
    feed(m,1.1)
    assert m.tick(1.1,1.1).task_kind==NAV_UTURN
    task=m.active;m.on_event(event,1.2,1.2);assert m.active is task


def test_narrow_uturn_is_refused_before_moving():
    core=Planner(config());state=State();obs=walls(.9)
    nav_authorize(core,state,obs,NAV_UTURN)
    for i in range(40):
        d=core.tick(state,obs,BOUND,.1+i*.1)
        assert d.action==STOP
        if core.latch:break
    assert core.latch and d.reason=='TURN_ADJUST_HELP_REQUIRED:NO_SAFE_COMBINATION'


def test_grid_unknown_and_lethal_cells_keep_exact_cell_area():
    obs=occupied_rectangles([0,100,100,-1,0,100,100,-1],4,2,.1,(1.,2.))
    assert len(obs)==1
    assert obs[0].aabb==pytest.approx((1.1,1.4,2.,2.2))
    assert not occupied_rectangles([0,50,99,0],2,2,.1,(0.,0.))
