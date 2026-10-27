import math
from dataclasses import replace
import pytest
from wheelchair_control.direction_manager import *
from wheelchair_control.direction_input import KeyEvents
from wheelchair_control.planner import Planner, State, integrate, toward
from wheelchair_control.protocol import Packet, STOP, FORWARD
from wheelchair_control.executor import Executor
from wheelchair_control.geometry import IndexedObstacles, clearance, transform, wrap
from nav_fixtures import config, box, BOUND, walls
from test_task_manager import feed


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
