"""Held W cruise must roll its path horizon forward without stopping."""
import math
import pytest
from dataclasses import replace
from wheelchair_control.planner import Planner,State,integrate,toward
from wheelchair_control.protocol import STOP,FORWARD
from wheelchair_control.executor import Executor
from wheelchair_control.geometry import IndexedObstacles
from wheelchair_control.path_tracking import direct_path
from nav_fixtures import config,BOUND,nav_authorize


class DelayedProvider:
    """Straight-line planner whose answer arrives after a fixed latency."""
    last_report={}
    def __init__(self,latency,first=0.):self.latency=latency;self.first=first;self.requests=[];self.clock=lambda:0.
    def request(self,task_id,start,goals,max_length,**kwargs):
        self.requests.append((self.clock(),start,goals));return len(self.requests)
    def poll(self,token):
        born,start,goals=self.requests[token-1]
        if self.clock()-born<(self.first if token==1 else self.latency):return 'WAIT',None
        return 'READY',direct_path(start,goals)
    def cancel(self,token):pass


def cruise(latency,duration=14.,obstacles=(),first=0.):
    # Hardware speed and a realistic feedback age shrink the stock margin.
    c=replace(config(),forward_speed=.534);core=Planner(c);provider=DelayedProvider(latency,first);core.path_provider=provider
    obs=IndexedObstacles(obstacles);state=State(observation_age=.1,execution_phase=0)
    nav_authorize(core,state,obs,3)
    executor=Executor(c);executor.request(STOP,0);executor.tick(0,0,0,True,True);executor.tick(.2,0,0,True,True)
    log=[];now=.25
    for i in range(int(duration/.05)):
        now=.25+i*.05;provider.clock=lambda:now
        stopped=executor.phase=='STOPPED' and abs(state.v)<.015 and abs(state.w)<.025
        d=core.tick(state,obs,BOUND,now,stopped=stopped)
        assert not core.latch,(d,state,i)
        executor.request(d.action,now);a,_=executor.tick(now,state.v,state.w,True,True)
        log.append((now,a,d.reason,state.x))
        tv,tw=c.velocity(a)
        v=toward(state.v,tv,(c.linear_brake if a==STOP else c.linear_accel)*.05)
        w=toward(state.w,tw,.8*.05)
        m=integrate(state,(state.v+v)/2,(state.w+w)/2,.05)
        state=State(m.x,m.y,m.yaw,v,w,a,.1,{'STOPPED':0,'RUNNING':1,'STARTING':3,'STOPPING':2,'FAULT':4}[executor.phase])
    return log,provider


def stops_after_start(log):
    first=next(i for i,row in enumerate(log) if row[1]==FORWARD)
    return [row for row in log[first:] if row[1]!=FORWARD]


@pytest.mark.parametrize('latency',[.05,.3,.6])
def test_held_cruise_never_stops_across_path_horizons(latency):
    log,provider=cruise(latency)
    assert log[-1][3]>4.5   # several 2 m horizons were crossed
    assert len(provider.requests)>=3
    assert stops_after_start(log)==[]


def test_cruise_requests_continuation_before_braking_distance():
    log,provider=cruise(.05,duration=8.)
    # Every continuation starts at the previous goal while the chair is still far from it.
    for born,start,goals in provider.requests[1:]:
        x=next(row[3] for row in log if row[0]>=born)
        assert start[0]-x>.5


def test_cruise_stops_safely_when_continuation_never_arrives():
    log,_=cruise(10.,duration=14.)   # first path is instant; continuations never arrive
    assert max(row[3] for row in log)<2.0+config().path_overrun+.5
    assert log[-1][1]==STOP
