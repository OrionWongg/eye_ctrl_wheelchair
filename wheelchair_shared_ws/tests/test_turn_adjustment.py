import math
from dataclasses import replace
import pytest
from wheelchair_control.planner import Planner,State,toward,integrate
from wheelchair_control.protocol import STOP,FORWARD,LEFT,RIGHT
from wheelchair_control.executor import Executor
from wheelchair_control.geometry import IndexedObstacles,clearance,transform,wrap
from wheelchair_control.direction_input import navigation_notice
from nav_fixtures import config,BOUND,box,walls
from nav_fixtures import nav_authorize


def bay(blocked=False):
    obs=[box(-1,-.2,.43,.55),box(-1,-.2,-.55,-.43)]
    if blocked:obs.append(box(.79,.91,-.9,.9))
    return IndexedObstacles(obs)


@pytest.mark.parametrize('kind,target',[(7,math.radians(20)),(8,-math.radians(20)),(9,math.pi)])
def test_forward_adjustment_completes_original_heading_and_stays_stopped(kind,target):
    c=config();core=Planner(c);s=State(observation_age=.03);obs=bay()
    p=nav_authorize(core,s,obs,kind);ex=Executor(c);ex.request(STOP,0);ex.tick(0,0,0,True,True)
    forward=False;done=False;attempts=0
    for i in range(1,550):
        now=i*.05;core.intent(replace(p,sequence=i+3,task_elapsed=now))
        d=core.tick(s,obs,BOUND,now,stopped=ex.phase=='STOPPED')
        assert not core.latch,(d,s)
        attempts=core.task_scope.adjustment.attempts
        if done:assert d.action==STOP
        if d.action==FORWARD:forward=True
        pulse=core.last_diagnostics.get('task_pulse_duration') if d.reason=='TASK_HEADING_MICRO_TURN' else None
        ex.request(d.action,now,pulse);a,_=ex.tick(now,s.v,s.w,True,True)
        tv,tw=c.velocity(a);v=toward(s.v,tv,(c.linear_brake if a==STOP else c.linear_accel)*.05)
        w=toward(s.w,tw,(c.angular_brake if a==STOP else c.angular_accel)*.05)
        q=integrate(s,(s.v+v)/2,(s.w+w)/2,.05)
        s=State(q.x,q.y,q.yaw,v,w,a,.03)
        assert clearance(transform(c.footprint,s.pose),obs,BOUND,c.padding)>c.padding
        assert s.x>=-1e-6 and math.hypot(s.x,s.y)<=c.turn_adjust_distance+c.nav_position_tolerance
        if d.reason=='NAV_TURN_COMPLETED':
            if done:break
            done=True
    assert done and forward and 1<=attempts<=3
    assert abs(wrap(s.yaw-target))<c.task_heading_tolerance


@pytest.mark.parametrize('obs',[bay(True),walls(.9)])
def test_no_combination_stops_without_any_forward_motion_and_requests_help(obs):
    core=Planner(config());s=State();p=nav_authorize(core,s,obs,9)
    for i in range(50):
        d=core.tick(s,obs,BOUND,i*.1)
        assert d.action==STOP
        if core.latch:break
    assert core.latch and d.reason=='TURN_ADJUST_HELP_REQUIRED:NO_SAFE_COMBINATION'
    assert '需要协助' in navigation_notice(d.reason)
    core.intent(replace(p,sequence=99))
    assert core.tick(s,obs,BOUND,10).action==STOP


def test_stop_cancels_pending_search_and_new_events_do_not_replay_it():
    core=Planner(config());s=State();obs=bay();p=nav_authorize(core,s,obs,9)
    core.tick(s,obs,BOUND,0)
    assert core.task_scope.adjustment.active
    core.intent(replace(p,command=STOP,enabled=False,sequence=3))
    assert core.task_scope is None and core.tick(s,obs,BOUND,.2).action==STOP


def test_health_fault_never_becomes_a_position_adjustment():
    core=Planner(config());s=State();obs=bay();nav_authorize(core,s,obs,9)
    d=core.tick(s,obs,BOUND,0,healthy=False)
    assert core.latch and core.task_scope is None and d.action==STOP
    assert d.reason=='SENSOR_OR_EXECUTOR_FAULT'


def test_stopping_handshake_before_search():
    core=Planner(config());s=State();obs=bay();nav_authorize(core,s,obs,9)
    core.tick(s,obs,BOUND,0)
    for i in range(10):
        d=core.tick(s,obs,BOUND,.1+i*.05,stopped=False)
        assert d.action==STOP and core.task_scope.adjustment.attempts==0


def test_bad_path_with_detour_is_not_authorized():
    from wheelchair_control.path_tracking import direct_path
    core=Planner(config());s=State();obs=bay();nav_authorize(core,s,obs,9)
    core.tick(s,obs,BOUND,0);adj=core.task_scope.adjustment
    adj.origin=(0,0,0);adj.request_distance=.4
    path=direct_path((0,0,0),[(.2,.2,0),(.4,0,math.pi)])
    assert not adj.valid_path(path,s)


class BlockingProvider:
    last_report={}
    def __init__(self):self.requests=[];self.cancelled=[];self.wait=False
    def request(self,task_id,start,goals,max_length,**kwargs):
        self.requests.append((task_id,start,goals,max_length,kwargs));return len(self.requests)
    def poll(self,token):return ('WAIT',None) if self.wait else ('FAILED','NO_EXECUTABLE_PATH')
    def cancel(self,token):self.cancelled.append(token)


def test_existing_service_search_is_bounded_no_detour_and_stop_cancels_request():
    core=Planner(config());provider=BlockingProvider();core.path_provider=provider
    s=State();obs=bay();p=nav_authorize(core,s,obs,9)
    for i in range(12):
        core.tick(s,obs,BOUND,.1*i)
        if len(provider.requests)>1:break
    assert len(provider.requests)==2
    task,start,goals,budget,options=provider.requests[-1]
    assert task==p.task_id and options['allow_detour'] is False
    assert 0<goals[0][0]<=.6 and goals[0][1]==0 and goals[0][:2]==goals[1][:2]
    assert budget<=.61
    provider.wait=True
    core.intent(replace(p,command=STOP,enabled=False,sequence=3))
    assert 2 in provider.cancelled and core.task_scope is None


def test_service_failure_cannot_trigger_unlimited_search_or_motion():
    core=Planner(config());provider=BlockingProvider();core.path_provider=provider
    s=State();obs=bay();nav_authorize(core,s,obs,9)
    for i in range(60):
        d=core.tick(s,obs,BOUND,.1*i)
        assert d.action==STOP
        if core.latch:break
    assert core.latch and d.reason=='TURN_ADJUST_HELP_REQUIRED:NO_SAFE_COMBINATION'
    assert len(provider.requests)==4  # original turn + three candidate probes


def test_help_notice_is_distinct_from_request_acknowledgement():
    assert '需要协助' in navigation_notice('TURN_ADJUST_HELP_REQUIRED:NO_SAFE_COMBINATION')
    assert navigation_notice('NAV_TURN_COMPLETED') is None


def test_obstacle_appearing_during_chosen_translation_stops_and_requests_help():
    core=Planner(config());s=State();obs=bay();nav_authorize(core,s,obs,7)
    for i in range(30):
        d=core.tick(s,obs,BOUND,.1*i)
        if core.task_scope.adjustment.phase=='EXECUTE':break
    assert core.task_scope.adjustment.phase=='EXECUTE'
    # Initial footprint remains clear but no full forward stopping envelope fits.
    blocked=IndexedObstacles([*obs,box(.70,.8,-.9,.9)])
    d=core.tick(s,blocked,BOUND,3.)
    assert d.action==STOP and core.latch and d.reason.startswith('TURN_ADJUST_HELP_REQUIRED:')


def test_valid_path_accepts_tiny_alignment_turns_around_drive():
    # Logged 2026-10-27: service path [turn, DRIVE, -turn, TURN] was rejected.
    from types import SimpleNamespace
    from wheelchair_control.turn_adjustment import TurnAdjustment
    from wheelchair_control.planner import Config, State
    adj=TurnAdjustment(SimpleNamespace(cfg=Config(),uturn=False))
    adj.origin=(-0.1796,-0.3043,0.0049);adj.target=0.1508;adj.request_distance=0.54
    path=[(-0.18,-0.3056,0.0051),(-0.18,-0.3056,0.0073),(0.3604,-0.3017,0.0073),
          (0.3604,-0.3017,0.0049),(0.3604,-0.3017,0.1508)]
    assert adj.valid_path(path,State(-0.1799,-0.3057,0.0051))
    # A real extra turn is still rejected.
    bent=path[:3]+[(0.3604,-0.3017,-0.2),(0.3604,-0.3017,0.1508)]
    assert not adj.valid_path(bent,State(-0.1799,-0.3057,0.0051))
