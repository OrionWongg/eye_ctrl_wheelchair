"""Finite RViz goals sharing the existing intent publisher and safety gate."""
import math
import uuid
from .task_manager import Output
from .navigation import NavigationScope
from .planner import Decision
from .protocol import STOP

POSE_GOAL = 10


class PoseGoal:
    def __init__(self, timeout=120.):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('goal_timeout must be positive')
        self.timeout = timeout
        self.active = None
        self.reason = 'IDLE'

    def cancel(self, reason):
        self.active = None
        self.reason = reason

    def accept(self, pose, now, core, keyboard_pending=False):
        if not all(math.isfinite(v) for v in pose):
            return False, 'INVALID_GOAL'
        if keyboard_pending or core.active or core.pending or not core.stopped():
            return False, 'KEYBOARD_BUSY_OR_MOVING'
        if core.clock_fault or not core.links_healthy(now) or core.lower.get('requires_rearm', True):
            return False, 'CONTROL_NOT_READY'
        self.active = ('rviz:' + uuid.uuid4().hex, tuple(pose), now)
        self.reason = 'GOAL_ACCEPTED'
        self.sessions = (core.lower_session, core.execution.get('session_id'))
        self.last_pose = core.pose
        return True, self.reason

    def tick(self, now, core):
        if self.active is None:
            return None
        ident, pose, started = self.active
        reason = None
        if core.clock_fault or not core.links_healthy(now):
            reason = 'GOAL_LINK_FAULT'
        elif self.sessions != (core.lower_session, core.execution.get('session_id')):
            reason = 'GOAL_SESSION_CHANGED'
        elif math.dist(core.pose[:2], self.last_pose[:2]) > .5:
            reason = 'GOAL_ODOM_DISCONTINUITY'
        elif now-started >= self.timeout:
            reason = 'GOAL_TIMEOUT'
        elif core.lower.get('requires_rearm'):
            reason = 'GOAL_REARM_REQUIRED'
        elif core.lower.get('task_id') == ident:
            lower_reason = core.lower.get('reason', '')
            if lower_reason == 'NAV_GOAL_COMPLETED':
                reason = 'GOAL_COMPLETED'
            elif core.lower.get('state') in (2, 3):
                reason = 'GOAL_FAILED:' + lower_reason
        if reason:
            self.cancel(reason)
            return Output(reason=reason)
        self.last_pose = core.pose
        return Output(1, True, ident, POSE_GOAL, pose, pose[2], 'RVIZ_GOAL')


class PoseGoalScope(NavigationScope):
    """A fixed terminal pose; never extend into cruise after reaching it."""
    def step(self, state, obstacles, boundary, now, stopped):
        if self.phase == 'COMPLETED':
            return Decision(STOP, 'PASS', 'NAV_GOAL_COMPLETED')
        if self.phase == 'INITIALIZE':
            self.phase = 'GOAL'
            self.request(state.pose, [self.reference], 'GOAL')
        failure = self.poll(state, now)
        if failure:
            return failure
        if self.follower is None:
            return Decision(STOP, 'ASSISTING', 'PATH_WAIT_PLAN')
        if self.follower.blocked:
            return self.owner.trip('GOAL_PATH_BLOCKED')
        decision = self.follower.tick(state, obstacles, boundary, now, stopped)
        if self.follower.done:
            self.phase = 'COMPLETED'
            return Decision(STOP, 'PASS', 'NAV_GOAL_COMPLETED')
        return decision
