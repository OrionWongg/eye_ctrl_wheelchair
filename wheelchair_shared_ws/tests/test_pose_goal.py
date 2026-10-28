import pytest
from wheelchair_control.pose_goal import PoseGoal, PoseGoalScope
from wheelchair_control.direction_manager import DirectionManager
from wheelchair_control.planner import Planner, State
from wheelchair_control.protocol import Packet, FORWARD
from wheelchair_control.geometry import IndexedObstacles
from nav_fixtures import config, BOUND


def ready():
    c = DirectionManager()
    c.lower = dict(intent_session_id=c.session, state=0, requires_rearm=False)
    c.execution = dict(state=0, stopped=True, session_id='executor')
    c.pose = (0., 0., 0.)
    c.v = c.w = 0.
    c.lower_at = c.execution_at = c.pose_at = 1.
    return c


def test_idle_goal_and_explicit_cancel_never_resume():
    c, g = ready(), PoseGoal()
    assert g.accept((2., 0., 0.), 1., c)[0]
    assert g.tick(1., c).enabled
    g.cancel('KEYBOARD_TAKEOVER')
    assert g.tick(1., c) is None


@pytest.mark.parametrize('case', ['moving', 'pending', 'stale', 'rearm'])
def test_reject_without_disturbing_keyboard(case):
    c, g = ready(), PoseGoal()
    if case == 'moving': c.execution['stopped'] = False
    if case == 'pending': c.pending = {'event': 'keyboard'}
    if case == 'stale': c.pose_at = -10.
    if case == 'rearm': c.lower['requires_rearm'] = True
    pending = c.pending
    assert not g.accept((2., 0., 0.), 1., c)[0]
    assert c.pending is pending


@pytest.mark.parametrize('case', ['blocked', 'complete', 'timeout', 'restart', 'stale'])
def test_goal_terminates_without_retry(case):
    c, g = ready(), PoseGoal(timeout=.1)
    assert g.accept((2., 0., 0.), 1., c)[0]
    c.lower['task_id'] = g.active[0]
    if case == 'blocked': c.lower.update(state=2, reason='PATH_NO_EXECUTABLE_PATH')
    if case == 'complete': c.lower['reason'] = 'NAV_GOAL_COMPLETED'
    if case == 'restart': c.execution['session_id'] = 'new'
    if case == 'stale': c.pose_at = -10.
    assert not g.tick(1.2 if case == 'timeout' else 1., c).enabled
    assert g.tick(1.2, c) is None


def test_fixed_goal_finishes_without_cruise_extension():
    owner = Planner(config())
    packet = Packet('test', 1, 0., .25, FORWARD, True,
                    task_id='goal', task_kind=10, reference_x=1.)
    scope = PoseGoalScope(owner, packet)
    obs = IndexedObstacles([])
    assert scope.tick(packet, State(), obs, BOUND, 0., True).action == FORWARD
    result = scope.tick(packet, State(x=1.), obs, BOUND, 1., True)
    assert result.reason == 'NAV_GOAL_COMPLETED'
    assert scope.tick(packet, State(x=1.), obs, BOUND, 2., True).action == 0
