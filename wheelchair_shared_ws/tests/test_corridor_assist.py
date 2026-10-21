"""Regression for the office corridor stop at (2.93485, 0.97368)."""
import pytest
from wheelchair_control.planner import Config,Planner,Predictor,State
from wheelchair_control.protocol import Packet,STOP,FORWARD,LEFT,RIGHT
from wheelchair_control.geometry import IndexedPolygon,nearby_obstacles
from wheelchair_sim.scenes import map_obstacles,boundary

POSE=State(x=2.9348465168514086,y=.9736821013766448,yaw=.29968090579863527,observation_age=.05)
REFERENCE=(2.8325045531343336,.9422187278698693,.3740525872945759)
@pytest.fixture
def corridor():
    core=Planner(Config(mode='bounded_assist'))
    obs=nearby_obstacles(POSE,[IndexedPolygon(b.polygon()) for b in map_obstacles('clearpath_office')],core.cfg)
    bound=boundary('clearpath_office')
    core.intent(Packet('user',1,0,.25,STOP,False),True)
    core.tick(POSE,obs,bound,0)
    core.intent(Packet('user',2,0,.25,FORWARD,True))
    core.reference=REFERENCE;core.last_command=FORWARD
    core.assist_started=1.;core.assist_spent=True
    return core,obs,bound

def test_fresh_confirmation_recovers_from_expired_assist_without_resetting_reference(corridor):
    core,obs,bound=corridor
    assert core.tick(POSE,obs,bound,4).reason=='ASSIST_TIME_LIMIT'
    for sequence in range(3,7):
        core.intent(Packet('user',sequence,0,.25,FORWARD,True))
        assert core.tick(POSE,obs,bound,5).action==STOP
        assert core.assist_started==1.
    core.intent(Packet('user',7,0,.25,STOP,False))
    core.tick(POSE,obs,bound,6);assert core.ready
    assert core.assist_started==1. # STOP alone is not a new grant.
    core.intent(Packet('user',8,0,.25,FORWARD,True))
    assert core.reference==REFERENCE
    assert core.assist_started is None and not core.assist_spent
    assert core.tick(POSE,obs,bound,7).action==FORWARD

def test_confirmation_cannot_reset_spatial_bounds(corridor):
    core,obs,bound=corridor
    core.tick(POSE,obs,bound,4)
    core.intent(Packet('user',3,0,.25,STOP,False));core.tick(POSE,obs,bound,5)
    core.intent(Packet('user',4,0,.25,FORWARD,True))
    assert core.reference==REFERENCE
    outside=State(x=POSE.x,y=POSE.y,yaw=REFERENCE[2]-.27,observation_age=.05)
    assert core.tick(outside,obs,bound,6).reason=='ASSIST_BOUND'

@pytest.mark.parametrize('command',[LEFT,RIGHT])
def test_user_turn_is_not_replaced_or_locked_by_old_assist_timer(corridor,command):
    core,obs,bound=corridor
    core.tick(POSE,obs,bound,4)
    core.intent(Packet('user',3,0,.25,STOP,False));core.tick(POSE,obs,bound,5)
    core.intent(Packet('user',4,0,.25,command,True))
    assert core.tick(POSE,obs,bound,6).action==command

def test_time_limit_still_applies_to_continuous_authorization(corridor):
    core,obs,bound=corridor
    for seq in range(3,6):core.intent(Packet('user',seq,0,.25,FORWARD,True))
    assert core.assist_started==1. and core.assist_spent
    assert core.tick(POSE,obs,bound,4).reason=='ASSIST_TIME_LIMIT'
