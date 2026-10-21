"""Regression at the user's actual office exit stop, 2026-10-13."""
import pytest
from dataclasses import replace
from wheelchair_control.planner import Planner,State
from wheelchair_control.protocol import Packet,STOP,FORWARD,LEFT,RIGHT
from wheelchair_control.geometry import IndexedPolygon,nearby_obstacles
from wheelchair_sim.scenes import map_obstacles,boundary
from test_passage import config
from wheelchair_control.executor import Executor

@pytest.fixture
def exit_stop():
    core=Planner(config())
    state=State(x=10.906735444657267,y=.873704798336599,yaw=-.039533730318275256,observation_age=.03)
    obs=nearby_obstacles(state,[IndexedPolygon(b.polygon()) for b in map_obstacles('clearpath_office')],core.cfg)
    bound=boundary('clearpath_office')
    core.intent(Packet('user',1,0,.25,STOP,False),True)
    core.tick(state,obs,bound,0)
    assert core.ready
    return core,state,obs,bound

def test_actual_exit_supports_small_left_turn_but_not_large_turn(exit_stop):
    core,s,obs,bound=exit_stop
    assert not core.predictor.check(s,LEFT,obs,bound).safe
    assert core.predictor.check(s,LEFT,obs,bound,duration=.1).safe
    core.intent(Packet('user',2,0,.25,LEFT,True))
    assert core.tick(s,obs,bound,.1).reason=='USER_MICRO_TURN_PREPARE'
    assert core.tick(s,obs,bound,.15).action==LEFT
    assert not core.latch

def test_actual_exit_never_substitutes_left_for_unsafe_right(exit_stop):
    core,s,obs,bound=exit_stop
    core.intent(Packet('user',2,0,.25,RIGHT,True))
    d=core.tick(s,obs,bound,.1)
    assert d.action==STOP and d.reason=='BLOCKED_TURN' and core.latch

@pytest.mark.parametrize('command',[STOP,FORWARD,RIGHT])
def test_new_user_command_cancels_pending_micro_turn(exit_stop,command):
    core,s,obs,bound=exit_stop
    core.intent(Packet('user',2,0,.25,LEFT,True));core.tick(s,obs,bound,.1)
    core.intent(Packet('user',3,0,.25,command,command!=STOP))
    assert core.manual_turn is None
    assert core.tick(s,obs,bound,.15).action in (STOP,command)

def test_pulse_times_from_running_and_waits_for_stop(exit_stop):
    core,s,obs,bound=exit_stop
    core.intent(Packet('user',2,0,.25,LEFT,True));core.tick(s,obs,bound,.1)
    # Startup delay does not consume commanded motion time.
    s.execution_phase=3;s.current_action=LEFT
    assert core.tick(s,obs,bound,.3).action==LEFT
    s.execution_phase=1
    assert core.tick(s,obs,bound,.4).action==LEFT
    assert core.tick(s,obs,bound,.51).reason=='USER_MICRO_TURN_BRAKE'
    assert core.tick(s,obs,bound,.56,stopped=False).reason=='USER_MICRO_TURN_SETTLE'
    s.execution_phase=0;s.current_action=STOP
    # A zero-motion pulse cannot loop forever, nor execute old motion on W.
    assert core.tick(s,obs,bound,.6).reason=='USER_MICRO_TURN_SETTLE'
    assert core.tick(s,obs,bound,.75).reason=='USER_TURN_LIMIT_REACHED'
    assert not core.latch
    assert core.tick(s,obs,bound,10).action==STOP
    core.intent(Packet('user',3,0,.25,FORWARD,True))
    assert core.tick(s,obs,bound,10.1).action==FORWARD

def test_pulse_timeout_and_health_failure_cancel(exit_stop):
    core,s,obs,bound=exit_stop
    core.intent(Packet('user',2,0,.25,LEFT,True));core.tick(s,obs,bound,.1)
    assert core.tick(s,obs,bound,3).reason=='MANUAL_TURN_TIMEOUT'
    assert core.manual_turn is None and core.latch

def test_micro_turn_still_checks_new_geometry(exit_stop):
    core,s,obs,bound=exit_stop
    core.intent(Packet('user',2,0,.25,LEFT,True));core.tick(s,obs,bound,.1)
    # Newly unknown boundary must cancel an already-approved pulse.
    small=[(10.5,.7),(11.2,.7),(11.2,1.),(10.5,1.)]
    assert core.tick(s,obs,small,.2).action==STOP
    assert core.latch and core.manual_turn is None

def test_starting_feedback_bounds_start_time_without_removing_lease_or_braking(exit_stop):
    core,s,obs,bound=exit_stop
    s=replace(s,execution_phase=3,current_action=LEFT,observation_age=.09)
    assert not core.predictor.check(s,LEFT,obs,bound,duration=0.).safe
    fresh=replace(s,pending_start_age=.01)
    bounded=core.predictor.check(fresh,LEFT,obs,bound,duration=0.)
    assert bounded.safe
    # Same acceleration exposure on an observation with matching age, with
    # the extra old-map interval retained as stationary/coasting time.
    expected=core.predictor.check(replace(s,observation_age=.01),LEFT,obs,bound,duration=0.)
    assert bounded.endpoint.pose==pytest.approx(expected.endpoint.pose)
    assert bounded.duration==pytest.approx(expected.duration+.08)
    assert bounded.endpoint.w==0. and bounded.endpoint.yaw>s.yaw
    # Old/missing feedback and actual RUNNING must retain the original bound.
    for changed in (replace(s,pending_start_age=.09),replace(s,pending_start_age=.01,execution_phase=1)):
        assert not core.predictor.check(changed,LEFT,obs,bound,duration=0.).safe

def test_executor_bounds_pulse_even_if_planner_keeps_renewing():
    ex=Executor(config());ex.request(STOP,0)
    ex.tick(0,0,0,True,True);ex.tick(.2,0,0,True,True)
    ex.request(LEFT,.2,max_run=.1)
    assert ex.tick(.2,0,0,True,True)[0]==STOP
    assert ex.tick(.35,0,0,True,True)[0]==LEFT
    ex.request(LEFT,.40,max_run=1.)
    assert ex.tick(.46,0,.08,True,True)[0]==STOP
    assert ex.reason=='RUN_LIMIT_REACHED'
    # Dropping the bound on a renewal cannot restart or prolong this episode.
    for t in (.5,.7,1.):
        ex.request(LEFT,t)
        assert ex.tick(t,0,0,True,True)[0]==STOP
    ex.request(STOP,1.1);ex.tick(1.1,0,0,True,True)
    ex.request(LEFT,1.2,max_run=.1);ex.tick(1.2,0,0,True,True)
    assert ex.tick(1.35,0,0,True,True)[0]==LEFT

def test_bounded_prediction_matches_legacy_until_cap_then_still_brakes(exit_stop):
    core,s,obs,bound=exit_stop
    pending=replace(s,current_action=LEFT,execution_phase=3,observation_age=.1,pending_start_age=.08)
    assert not core.predictor.check(pending,LEFT,obs,bound,duration=0.).safe
    bounded=core.predictor.check(pending,LEFT,obs,bound,duration=0.,max_run=.1)
    assert bounded.safe and bounded.endpoint.w==0.
    # A bounded action cannot escape an initially unsafe footprint.
    unsafe=replace(pending,y=.8)
    assert not core.predictor.check(unsafe,LEFT,obs,bound,duration=0.,max_run=.1).safe

def test_bounded_episode_keeps_braking_through_transit_phase():
    from wheelchair_control.planner import Predictor
    c=config();p=Predictor(c);s=State(observation_age=0.)
    bound=[(-5,-5),(5,-5),(5,5),(-5,5)]
    r=p.check(s,LEFT,[],bound,duration=.1,max_run=.1)
    # Conservative runtime includes executor scheduling and downstream transit.
    # Once it expires the base brakes continuously, even if the planner's
    # generic lease/transit phases have not all ended.
    exposure=.1+c.period+c.transport_delay+c.period+c.step
    peak=c.angular_accel*exposure
    maximum=.5*c.angular_accel*exposure**2+peak**2/(2*c.angular_brake)
    assert r.safe and abs(r.endpoint.w)<1e-12
    assert r.endpoint.yaw<=maximum+1e-9
    legacy=p.check(s,LEFT,[],bound,duration=.1)
    assert r.endpoint.yaw<legacy.endpoint.yaw
