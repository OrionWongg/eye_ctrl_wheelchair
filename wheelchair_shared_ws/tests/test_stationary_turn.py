import math
from dataclasses import replace
import pytest
from wheelchair_control.direction_manager import (DirectionManager,PREFER_LEFT,PREFER_RIGHT,TURN_AROUND,
    START,STOP_EVENT,NAV_TURN_LEFT,NAV_TURN_RIGHT,NAV_TURN_AROUND,NAV_CRUISE)
from wheelchair_control.task_manager import Event
from wheelchair_control.planner import Planner,State,integrate,toward
from wheelchair_control.executor import Executor
from wheelchair_control.protocol import STOP,FORWARD
from wheelchair_control.geometry import IndexedObstacles,wrap
from nav_fixtures import config,BOUND,walls
from nav_fixtures import nav_authorize
from test_task_manager import feed


@pytest.mark.parametrize('event,kind',[(PREFER_LEFT,NAV_TURN_LEFT),(PREFER_RIGHT,NAV_TURN_RIGHT),(TURN_AROUND,NAV_TURN_AROUND)])
def test_stopped_gesture_authorizes_only_finite_turn(event,kind):
    m=DirectionManager();feed(m,0.)
    gesture=Event('source',1,0.,1.5,event)
    assert m.on_event(gesture,0.,0.) is None
    assert not m.tick(0.,0.).enabled
    feed(m,.5);out=m.tick(.5,.5)
    assert out.enabled and out.task_kind==kind and m.active.turn_only
    task=m.active
    m.on_event(gesture,.6,.6);assert m.active is task
    m.on_lower(dict(m.lower,task_id=task.task_id,reason='NAV_TURN_COMPLETED'),.7)
    assert m.active is None and not m.tick(.7,.7).enabled
    m.on_event(gesture,.8,.8)
    assert m.active is None and m.pending is None
    feed(m,1.);m.on_event(Event('source',2,1.,1.5,START),1.,1.)
    feed(m,1.5);assert m.tick(1.5,1.5).task_kind==NAV_CRUISE


@pytest.mark.parametrize('kind,angle',[(7,math.radians(20)),(8,-math.radians(20)),(9,math.pi)])
def test_standalone_turn_finishes_without_forward_motion(kind,angle):
    c=config();core=Planner(c);state=State(yaw=.2,observation_age=.03,execution_phase=0)
    obs=IndexedObstacles([]);packet=nav_authorize(core,state,obs,kind)
    executor=Executor(c);executor.request(STOP,0);executor.tick(0,0,0,True,True);executor.tick(.2,0,0,True,True)
    complete=False
    for i in range(1,650):
        now=.2+i*.05;core.intent(replace(packet,sequence=i+2,stamp=now,task_elapsed=now))
        stopped=executor.phase=='STOPPED' and abs(state.w)<.025
        d=core.tick(state,obs,BOUND,now,stopped=stopped)
        assert not core.latch,d
        assert d.action!=FORWARD
        duration=core.last_diagnostics.get('task_pulse_duration') if d.reason=='TASK_HEADING_MICRO_TURN' else None
        executor.request(d.action,now,duration);a,_=executor.tick(now,state.v,state.w,True,True)
        tv,tw=c.velocity(a);v=toward(state.v,tv,c.linear_accel*.05)
        w=toward(state.w,tw,(c.angular_brake if a==STOP else c.angular_accel)*.05)
        moved=integrate(state,(state.v+v)/2,(state.w+w)/2,.05)
        state=State(moved.x,moved.y,moved.yaw,v,w,d.action if executor.phase=='STARTING' else a,.03,
            {'STOPPED':0,'RUNNING':1,'STARTING':3,'STOPPING':2}[executor.phase])
        if d.reason=='NAV_TURN_COMPLETED':
            assert abs(wrap(state.yaw-.2-angle))<c.task_heading_tolerance
            assert state.x==pytest.approx(0.) and state.y==pytest.approx(0.)
            if complete:break
            complete=True
    assert complete


def test_stopped_uturn_keeps_narrow_corridor_collision_guard():
    core=Planner(config());obs=walls(.9);state=State()
    nav_authorize(core,state,obs,NAV_TURN_AROUND)
    for i in range(40):
        d=core.tick(state,obs,BOUND,.1+i*.1)
        assert d.action==STOP
        if core.latch:break
    assert core.latch and d.reason.startswith('TURN_ADJUST_HELP_REQUIRED:')


def test_stop_cancels_pending_turn_and_old_gesture_cannot_restart():
    m=DirectionManager();feed(m,0.)
    turn=Event('source',1,0.,1.5,TURN_AROUND);m.on_event(turn,0.,0.)
    m.on_event(Event('source',2,.1,1.5,STOP_EVENT),.1,.1)
    feed(m,.6);m.on_event(turn,.6,.6)
    assert not m.tick(.6,.6).enabled and m.pending is None


def test_start_during_standalone_turn_waits_for_stop_before_cruising():
    m=DirectionManager();feed(m,0.)
    m.on_event(Event('source',1,0.,1.5,PREFER_LEFT),0.,0.)
    feed(m,.5);m.tick(.5,.5)
    feed(m,.6,w=.2)
    m.on_event(Event('source',2,.6,1.5,START),.6,.6)
    assert not m.tick(.6,.6).enabled and not m.pending['turn_only']
    feed(m,1.1);out=m.tick(1.1,1.1)
    assert out.enabled and out.task_kind==NAV_CRUISE


def test_fresh_id_with_pre_stop_timestamp_cannot_authorize_turn():
    m=DirectionManager();feed(m,0.)
    m.on_event(Event('source',1,.1,1.5,STOP_EVENT),.1,.1)
    feed(m,.6)
    ack=m.on_event(Event('source',2,.05,1.5,TURN_AROUND),.6,.6)
    assert not ack.accepted and m.pending is None


@pytest.mark.parametrize('kind,sign',[(7,1),(8,-1)])
def test_continuous_turn_runs_while_held_without_target_or_pulses(kind,sign):
    from wheelchair_control.protocol import LEFT,RIGHT
    c=replace(config(),turn_mode='continuous');core=Planner(c);state=State(observation_age=.03,execution_phase=0)
    obs=IndexedObstacles([]);packet=nav_authorize(core,state,obs,kind)
    cmd=LEFT if sign>0 else RIGHT
    # Far beyond the fixed 20 deg target: still turning, never pulsed or completed.
    for i,yaw in enumerate([0.,.5,1.5,3.,-2.]):
        core.intent(replace(packet,sequence=i+3,stamp=i*.05,task_elapsed=i*.05))
        d=core.tick(State(yaw=yaw,w=sign*.8,current_action=cmd,observation_age=.03,execution_phase=1),
                    obs,BOUND,.1+i*.05,stopped=False)
        assert (d.action,d.reason)==(cmd,'NAV_CONTINUOUS_TURN') and not core.latch


def test_continuous_turn_waits_without_latch_when_blocked():
    from wheelchair_control.protocol import LEFT
    c=replace(config(),turn_mode='continuous');core=Planner(c);state=State(observation_age=.03,execution_phase=0)
    packet=nav_authorize(core,state,IndexedObstacles([]),7)
    # Footrest corner sweeps into this post during a left turn.
    post=IndexedObstacles([[(.50,.38),(.60,.38),(.60,.48),(.50,.48)]])
    d=core.tick(state,post,BOUND,.1)
    assert (d.action,d.reason)==(STOP,'NAV_TURN_WAIT_CLEARANCE') and not core.latch
    core.intent(replace(packet,sequence=4,stamp=.15,task_elapsed=.15))
    assert core.tick(state,IndexedObstacles([]),BOUND,.2).action==LEFT
