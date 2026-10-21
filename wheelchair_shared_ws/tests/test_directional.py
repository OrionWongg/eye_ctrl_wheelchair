import math
from dataclasses import replace
import pytest
from wheelchair_control.direction_manager import *
from wheelchair_control.direction_input import KeyEvents
from wheelchair_control.entries import openings, route_clear
from wheelchair_control.passage import detect
from wheelchair_control.planner import Planner, State, integrate, toward
from wheelchair_control.protocol import Packet, STOP, FORWARD
from wheelchair_control.executor import Executor
from wheelchair_control.geometry import IndexedObstacles, clearance, transform, wrap
from test_passage import config, box, BOUND, walls
from test_task_manager import feed


def door_map(side=1, gaps=((1.,2.4),), width=1.4):
    wall=width/2
    obs=[box(-3,10,-wall-.12,-wall)]
    end=-3.
    for a,b in gaps:
        obs.append(box(end,a,wall,wall+.12));end=b
    obs.append(box(end,10,wall,wall+.12))
    if side<0:obs=[[(x,-y) for x,y in reversed(o)] for o in obs]
    return IndexedObstacles(obs)


def nav_authorize(core,state,obs,kind=4):
    core.intent(Packet('manager',1,0.,.25,STOP,False),True)
    core.tick(state,obs,BOUND,0.)
    assert core.ready
    p=Packet('manager',2,0.,.25,FORWARD,True,task_id='nav',task_kind=kind,
             reference_x=state.x,reference_y=state.y,reference_yaw=state.yaw)
    core.intent(p)
    return p


@pytest.mark.parametrize('side',[-1,1])
@pytest.mark.parametrize('gaps',[((1.,2.4),),((.6,1.8),(2.2,3.6)),((1.5,2.5),)])
def test_entry_geometry_uses_stable_id_and_full_sweep(side,gaps):
    cfg=config();s=State();obs=door_map(side,gaps)
    passage,_=detect(s,obs,cfg)
    candidates=openings(s,obs,passage,side,cfg)
    assert candidates
    first=candidates[0]
    assert first.start==pytest.approx(gaps[0][0])
    assert first.end==pytest.approx(gaps[0][1])
    assert first.identity==openings(State(x=.1),IndexedObstacles(reversed(obs)),passage,side,cfg)[0].identity
    assert route_clear(s,first,obs,BOUND,cfg)
    assert not openings(State(x=gaps[-1][1]+.1),obs,passage,side,cfg)


def test_skip_unreachable_before_lock_but_never_switch_locked_entry():
    cfg=config();s=State();obs=door_map(gaps=((.7,1.7),(2.,3.4)))
    obs=IndexedObstacles([*obs,box(.7,1.7,.85,1.1)])
    core=Planner(cfg);nav_authorize(core,s,obs)
    core.tick(s,obs,BOUND,.1)
    assert core.task_scope.entry.start==2.
    entry=core.task_scope.entry
    blocked=IndexedObstacles([*obs,box(1.,1.1,-.4,.4)])
    # No planner re-selection, even with another opening available.
    core.tick(State(x=.3,v=.372,current_action=FORWARD),blocked,BOUND,.2,stopped=False)
    assert core.task_scope is None or core.task_scope.entry==entry


@pytest.mark.parametrize('side',[-1,1])
@pytest.mark.parametrize('start_x',[0.,1.35])
def test_complete_entry_and_continue_without_second_turn(side,start_x):
    c=config();core=Planner(c);obs=door_map(side);s=State(x=start_x,observation_age=.03,execution_phase=0)
    packet=nav_authorize(core,s,obs,4 if side>0 else 5)
    executor=Executor(c);executor.request(STOP,0.);executor.tick(0,0,0,True,True);executor.tick(.2,0,0,True,True)
    phases=[];ids=set();completed=False
    for i in range(1,601):
        now=.2+i*.05
        core.intent(replace(packet,sequence=i+2,stamp=now,task_elapsed=now))
        stopped=executor.phase=='STOPPED' and abs(s.v)<.015 and abs(s.w)<.025
        d=core.tick(s,obs,BOUND,now,stopped=stopped)
        assert not core.latch,(d,s,phases[-10:])
        nav=core.task_scope
        phases.append(nav.phase)
        if nav.entry:ids.add(nav.entry.identity)
        max_run=core.last_diagnostics.get('task_pulse_duration') if d.reason=='TASK_HEADING_MICRO_TURN' else None
        executor.request(d.action,now,max_run=max_run)
        a,_=executor.tick(now,s.v,s.w,True,True)
        tv,tw=c.velocity(a)
        v=toward(s.v,tv,(c.linear_brake if a==STOP else c.linear_accel)*.05)
        w=toward(s.w,tw,(c.angular_brake if a==STOP else c.angular_accel)*.05)
        moved=integrate(s,(s.v+v)/2,(s.w+w)/2,.05)
        s=State(moved.x,moved.y,moved.yaw,v,w,d.action if executor.phase=='STARTING' else a,.03,
                {'STOPPED':0,'RUNNING':1,'STOPPING':2,'STARTING':3,'FAULT':4}[executor.phase])
        assert clearance(transform(c.footprint,s.pose),obs,BOUND,c.padding)>c.padding
        if nav.phase=='CRUISE' and abs(s.y)>2.:
            completed=True;break
    assert completed,(s,phases[-10:])
    assert len(ids)==1 and 'ALIGN_ENTRY' in phases and 'TRAVERSE' in phases
    assert abs(wrap(s.yaw-side*math.pi/2))<c.task_heading_tolerance
    assert nav.side==0 and a==FORWARD


def test_search_budget_never_refreshed_by_renewals():
    c=config();core=Planner(c);obs=walls(1.4);s=State()
    p=nav_authorize(core,s,obs)
    for t in (.1,1.,4.,8.2):
        core.intent(replace(p,sequence=int(t*10)+3,task_elapsed=t))
        core.tick(s,obs,BOUND,t)
    assert core.task_scope.phase=='CRUISE' and core.task_scope.side==0


def manager_start():
    m=DirectionManager(session='manager');feed(m,0.)
    e=Event('source',1,0.,1.5,START)
    m.on_event(e,0.,0.);assert not m.tick(0.,0.).enabled
    feed(m,.5);assert m.tick(.5,.5).enabled
    return m


def test_start_stop_new_start_only_and_no_periodic_budget():
    m=DirectionManager();feed(m,0.)
    assert not m.on_event(Event('source',1,0.,1.5,CANCEL_PREFERENCE),0.,0.).accepted
    m=manager_start()
    for t in (1.,5.,11.,20.):
        feed(m,t);assert m.tick(t,t).enabled
    m.on_event(Event('source',3,20.,1.5,STOP_EVENT),20.,20.)
    feed(m,20.1);assert not m.tick(20.1,20.1).enabled
    assert not m.on_event(Event('source',4,19.9,1.5,START),20.1,20.1).accepted
    feed(m,21.);assert not m.tick(21.,21.).enabled


def test_w_and_repeated_side_preserve_task_and_opposite_stops_first():
    m=manager_start();feed(m,.6)
    e=Event('source',2,.6,1.5,PREFER_LEFT)
    assert m.on_event(e,.6,.6).accepted
    task=m.active
    for n,k in ((3,START),(4,PREFER_LEFT)):
        assert m.on_event(Event('source',n,.6,1.5,k),.6,.6).accepted
        assert m.active is task
    assert m.on_event(e,.7,.7).accepted and m.active is task
    feed(m,.8,v=.2)
    m.on_event(Event('source',5,.8,1.5,PREFER_RIGHT),.8,.8)
    assert m.active is None and not m.tick(.8,.8).enabled
    feed(m,1.3)
    assert m.tick(1.3,1.3).task_kind==NAV_RIGHT


@pytest.mark.parametrize('fault',['health','clock','planner','executor'])
def test_fault_clears_pending_and_recovery_requires_new_w(fault):
    m=manager_start()
    if fault=='health':m.tick(1.,1.)
    elif fault=='clock':m.set_clock_fault('CLOCK_PAUSED',.6)
    elif fault=='planner':m.on_lower(dict(session_id='new'),.6)
    else:m.on_execution(dict(session_id='new'),.6)
    m.set_clock_fault('',1.1);feed(m,1.1)
    assert not m.tick(1.1,1.1).enabled and m.pending is None


def test_terminal_repeat_requires_silence_after_completion():
    keys=KeyEvents(.6)
    assert keys.key('a',0.)==PREFER_LEFT
    for n in range(1,50):assert keys.key('a',n*.1) is None
    assert keys.key('a',5.6)==PREFER_LEFT
    assert keys.key('d',5.7)==PREFER_RIGHT
    assert keys.key(' ',5.8)==STOP_EVENT


def test_cancel_preference_uses_stopped_heading_and_does_not_need_new_start():
    m=manager_start();feed(m,.6,v=.2)
    m.on_event(Event('source',2,.6,1.5,PREFER_LEFT),.6,.6)
    m.on_event(Event('source',3,.7,1.5,CANCEL_PREFERENCE),.7,.7)
    assert not m.tick(.7,.7).enabled
    feed(m,1.2,pose=(.1,0.,.2))
    out=m.tick(1.2,1.2)
    assert out.enabled and out.task_kind==NAV_CRUISE and out.reference==(.1,0.,.2)


def test_pending_event_expires_and_old_ack_cannot_restart():
    m=manager_start();feed(m,.6,v=.2)
    e=Event('source',2,.6,1.5,CANCEL_PREFERENCE)
    m.on_event(e,.6,.6)
    feed(m,2.11,v=.2)
    assert not m.tick(2.11,2.11).enabled and m.pending is None
    feed(m,2.2)
    assert not m.on_event(e,2.2,2.2).accepted
    assert not m.tick(2.2,2.2).enabled


def test_completed_event_retry_and_mutated_id_never_create_another_turn():
    m=manager_start();feed(m,.6)
    e=Event('source',2,.6,1.5,PREFER_LEFT);m.on_event(e,.6,.6)
    task=m.active
    m.on_lower(dict(m.lower,task_id=task.task_id,reason='NAV_CRUISE'),.7)
    assert m.consumed_preference
    m.on_event(e,.7,.7)
    assert m.active is task
    assert not m.on_event(replace(e,kind=PREFER_RIGHT),.7,.7).accepted
    assert m.active is task
    # Only a fresh gesture can start a second maneuver.
    feed(m,.8)
    assert m.on_event(Event('source',3,.8,1.5,PREFER_LEFT),.8,.8).accepted
    assert m.active is not task


def test_open_interpretation_is_locked_even_if_corridor_appears():
    core=Planner(config());empty=IndexedObstacles([]);s=State(yaw=.05)
    nav_authorize(core,s,empty,3)
    assert core.tick(s,empty,BOUND,.1).action==FORWARD
    reference=core.task_scope.reference
    core.tick(s,walls(1.3),BOUND,.2)
    assert core.task_scope.passage is None and core.task_scope.reference==reference


def test_search_distance_expires_independently_of_time():
    core=Planner(config());obs=walls(1.4);s=State()
    nav_authorize(core,s,obs)
    core.tick(s,obs,BOUND,.1)
    for i in range(1,111):
        core.tick(State(x=i*.028,v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1),
                  obs,BOUND,.1+i*.05,stopped=False)
    assert core.task_scope.phase=='CRUISE' and core.task_scope.side==0


def test_straight_door_cruise_does_not_require_exit_confirmation():
    core=Planner(config());obs=IndexedObstacles([box(1.,1.3,.5,4.),box(1.,1.3,-4.,-.5)])
    nav_authorize(core,State(),obs,3)
    core.tick(State(),obs,BOUND,.1)
    assert core.task_scope.passage.kind=='door'
    for i in range(1,111):
        d=core.tick(State(x=i*.02,v=.372,current_action=FORWARD,observation_age=.03,execution_phase=1),
                    obs,BOUND,.1+i*.05,stopped=False)
        assert not core.latch,(i,d)
    assert d.action==FORWARD and core.task_scope.phase=='CRUISE'
