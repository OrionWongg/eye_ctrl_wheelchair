"""Held keyboard directions must stay continuous across planning horizons."""
import pytest
from wheelchair_control.executor import Executor
from wheelchair_control.planner import Config
from wheelchair_control.protocol import STOP, FORWARD, LEFT, RIGHT


def running(action):
    executor = Executor(Config())
    executor.request(STOP, 0.)
    executor.tick(0., 0., 0., True, True)
    executor.tick(.2, 0., 0., True, True)
    executor.request(action, .2)
    executor.tick(.2, 0., 0., True, True)
    assert executor.tick(.4, 0., 0., True, True)[0] == action
    return executor


@pytest.mark.parametrize('action', [FORWARD, LEFT, RIGHT])
def test_held_direction_remains_continuous_until_release(action):
    executor = running(action)
    for i in range(1, 201):
        now = .4 + i*.05
        executor.request(action, now)
        assert executor.tick(now, .12, .09, True, True)[0] == action
        assert executor.phase == 'RUNNING'
    executor.request(STOP, 10.5)
    assert executor.tick(10.5, .12, .09, True, True)[0] == STOP


@pytest.mark.parametrize('action', [FORWARD, LEFT, RIGHT])
@pytest.mark.parametrize('healthy, fresh', [(False, True), (True, False)])
def test_continuous_hold_still_stops_on_lost_feedback_or_lease(action, healthy, fresh):
    executor = running(action)
    assert executor.tick(.45, .12, .09, healthy, fresh)[0] == STOP
    assert executor.fault
