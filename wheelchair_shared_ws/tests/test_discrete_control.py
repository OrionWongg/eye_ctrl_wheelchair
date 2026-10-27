import math
import pytest
from wheelchair_control.protocol import Packet,StreamGuard,ClockGuard,STOP,FORWARD,LEFT,RIGHT
from wheelchair_control.geometry import distance,transform,convex
from wheelchair_control.planner import Config,State,Planner,Predictor
from wheelchair_control.executor import Executor

BOUND=[(-10.,-10.),(10.,-10.),(10.,10.),(-10.,10.)]
def box(x,y,w=.1,h=.1):return [(x-w/2,y-h/2),(x+w/2,y-h/2),(x+w/2,y+h/2),(x-w/2,y+h/2)]
def authorize(core,cmd=FORWARD,state=None,obs=None):
    state=state or State();obs=obs or []
    core.intent(Packet('test',1,0,.25,STOP,False),True)
    core.tick(state,obs,BOUND,0,stopped=True)
    assert core.ready
    core.intent(Packet('test',2,0,.25,cmd,True))
    assert not core.latch

def test_guard_duplicate_replay_session_and_expiry():
    g=StreamGuard(.25)
    assert g.accept(Packet('a',1,5,.25),5,10)==(True,True,'')
    assert not g.accept(Packet('a',1,5,.25),5,10.1)[0]
    assert g.accept(Packet('a',2,5.1,.25),5.1,10.2)[0]
    assert g.fresh(10.3) and not g.fresh(10.46)
    assert g.accept(Packet('b',0,5.2,.25),5.2,10.4)[1]
    assert not g.accept(Packet('a',99,5.3,.25),5.3,10.5)[0]

@pytest.mark.parametrize('stamp,ttl,cmd',[(1,.25,0),(5,10,0),(6,.25,0),(5,.25,9),(math.nan,.25,0)])
def test_guard_bad_source(stamp,ttl,cmd):
    assert not StreamGuard(.25).accept(Packet('a',1,stamp,ttl,cmd),5,5)[0]

def test_pause_and_jump():
    c=ClockGuard();assert c.update(1,10)==''
    assert c.update(1,10.4)=='CLOCK_PAUSED'
    assert c.update(.5,10.5)=='CLOCK_JUMP'

def test_forward_is_fixed_or_zero():
    p=Predictor(Config())
    assert p.check(State(),FORWARD,[],BOUND).safe
    assert not p.check(State(),FORWARD,[box(.75,0,.1,2)],BOUND).safe
    core=Planner(Config());authorize(core)
    assert core.tick(State(),[box(.75,0,.1,2)],BOUND,1).action==STOP
    assert core.latch

def test_swept_polygon_rotation_not_point_check():
    p=Predictor(Config())
    # Footrest corner's braking sweep reaches a wall that is clear at initial pose.
    obs=[box(.52,.49,.12,.10)]
    assert p.check(State(),STOP,obs,BOUND).safe
    assert not p.check(State(),LEFT,obs,BOUND).safe

def test_momentum_and_angular_braking():
    p=Predictor(Config())
    # 0.372 m/s brakes in 0.138 m: clear at rest, blocked while moving.
    wall=[box(.75,0,.1,2.)]
    assert p.check(State(),STOP,wall,BOUND).safe
    assert not p.check(State(v=.372,current_action=FORWARD),STOP,wall,BOUND).safe
    obs=[box(.52,.49,.12,.10)]
    assert not p.check(State(w=.389,current_action=LEFT),STOP,obs,BOUND).safe

def test_unknown_boundary_is_not_free():
    small=[(-1,-1),(.7,-1),(.7,1),(-1,1)]
    assert not Predictor(Config()).check(State(),FORWARD,[],small).safe

def test_obstacle_cannot_be_outvoted_by_score():
    core=Planner(Config(mode='bounded_assist'));authorize(core)
    assert core.tick(State(),[box(.75,0,.1,3.)],BOUND,1).action==STOP

def test_latch_requires_disabled_stop_and_new_confirmation():
    core=Planner(Config());authorize(core)
    core.tick(State(),[box(.75,0,.1,3)],BOUND,1)
    for i in range(3,20):
        core.intent(Packet('test',i,0,.25,FORWARD,True))
        assert core.tick(State(),[],BOUND,i).action==STOP
    core.intent(Packet('test',20,0,.25,STOP,False))
    assert core.tick(State(),[],BOUND,20).action==STOP and core.ready
    core.intent(Packet('test',21,0,.25,FORWARD,True))
    assert core.tick(State(),[],BOUND,21).action==FORWARD

def test_explicit_turn_never_reversed_or_forward():
    for cmd in (LEFT,RIGHT):
        core=Planner(Config(mode='bounded_assist'));authorize(core,cmd)
        assert core.tick(State(),[],BOUND,1).action in (STOP,cmd)
        core=Planner(Config(mode='bounded_assist'));authorize(core,cmd)
        assert core.tick(State(),[box(0,.43,2,.1)],BOUND,1).action==STOP

def test_assist_limits_do_not_reset_each_cycle():
    core=Planner(Config(mode='bounded_assist'));authorize(core)
    core.tick(State(),[],BOUND,1)
    ref=core.reference
    for i in range(5):
        core.intent(Packet('test',3+i,0,.25,FORWARD,True))
        core.tick(State(x=i*.03,y=.1,yaw=.05),[],BOUND,1+i*.1)
        assert core.reference==ref
    assert core.tick(State(y=.251),[],BOUND,2).action==STOP

def test_assist_time_applies_after_turn_phase_completed():
    core=Planner(Config(mode='bounded_assist'));authorize(core)
    core.tick(State(),[],BOUND,1);core.assist_started=1.;core.assist_spent=True
    assert core.tick(State(),[],BOUND,3.1).action==STOP

def test_stop_and_new_direction_cancel_plan():
    core=Planner(Config(mode='bounded_assist'));authorize(core)
    core.phase='TURN';core.side=LEFT;core.assist_started=1.
    core.intent(Packet('test',3,0,.25,STOP,False))
    assert core.phase is None and core.tick(State(),[],BOUND,1.1).action==STOP
    core.phase='TURN';core.side=LEFT
    core.intent(Packet('test',4,0,.25,RIGHT,True))
    assert core.phase is None

def test_executor_constant_renewals_and_feedback_transition():
    e=Executor(Config());e.request(STOP,0);e.tick(0,0,0,True,True);e.tick(.2,0,0,True,True)
    e.request(FORWARD,.2)
    assert e.tick(.2,0,0,True,True)[0]==STOP
    assert e.tick(.4,0,0,True,True)[0]==FORWARD
    for i in range(5):
        e.request(FORWARD,.5+i*.05)
        assert e.tick(.5+i*.05,.372,0,True,True)[0]==FORWARD
    e.request(LEFT,1.)
    assert e.tick(1.1,.2,0,True,True)[0]==STOP
    assert e.tick(1.2,0,0,True,True)[0]==STOP
    e.tick(1.4,0,0,True,True)
    assert e.tick(1.6,0,0,True,True)[0]==LEFT

def test_executor_loss_rearm_and_pending_cancel():
    e=Executor(Config());e.request(STOP,0);e.tick(0,0,0,True,True);e.tick(.2,0,0,True,True)
    e.request(LEFT,.2);e.tick(.2,0,0,True,True)
    e.request(STOP,.25)
    assert e.tick(.5,0,0,True,True)[0]==STOP
    e.request(FORWARD,.6);e.tick(.6,0,0,True,True)
    assert e.tick(.8,0,0,True,True)[0]==FORWARD
    assert e.tick(1,.372,0,True,False)[0]==STOP
    e.request(FORWARD,1.1)
    assert e.tick(1.2,0,0,True,True)[0]==STOP

def test_executor_new_session_must_first_stop():
    e=Executor(Config());e.new_session(0);e.request(LEFT,0)
    for t in (0.,.2,.4,1.):
        e.request(LEFT,t)
        assert e.tick(t,0,0,True,True)[0]==STOP

@pytest.mark.parametrize('kwargs',[{'footprint':((0,0),(1,1),(0,1),(1,0))},{'linear_brake':0},{'mode':'scale_speed'}])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):Config(**kwargs)


def test_stop_timeout_reports_fault_even_user_stop():
    e=Executor(Config());e.request(STOP,0.)
    assert e.tick(3.,.2,0.,True,True)[0]==STOP
    assert e.fault and e.reason=='STOP_TIMEOUT'

def test_prediction_is_braking_distance_only():
    c=Config(forward_speed=.534,margin=.06);p=Predictor(c)
    # Forward from rest: the stop zone is swept only by v^2/(2*brake).
    r=p.check(State(),FORWARD,[],BOUND)
    assert r.endpoint.x==pytest.approx(.534**2/(2*c.linear_brake))
    assert r.duration==pytest.approx(.534/c.linear_brake)
    # run=0 brakes from the measured speed, not the commanded one.
    rolling=p.check(State(v=.2,current_action=FORWARD),FORWARD,[],BOUND,run=0.)
    assert rolling.endpoint.x==pytest.approx(.2**2/(2*c.linear_brake))
    # Stationary STOP checks the current pose only.
    assert p.check(State(),STOP,[],BOUND).poses==[(0.,0.,0.)]
    # Obstacle just beyond the braking distance plus margin is clear.
    front=.55+.534**2/(2*c.linear_brake)+c.margin
    assert p.check(State(),FORWARD,[box(front+.06,0,.1,2)],BOUND).safe
    assert not p.check(State(),FORWARD,[box(front+.04,0,.1,2)],BOUND).safe
