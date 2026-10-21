"""One bounded forward-then-turn fallback using the existing path service.

Normal turns are unchanged. No reverse, detour, global preflight or new sensor
policy. The fallback is only entered after a geometric stationary-turn failure.
"""
import math
from .geometry import wrap,transform,clearance
from .planner import Decision
from .protocol import STOP,FORWARD
from .path_tracking import PathFollower,direct_path,segments

PLAN_BLOCKED=frozenset(('NO_EXECUTABLE_PATH','GOAL_COLLISION','LOCKED_ROUTE_BLOCKED'))
TURN_BLOCKED=frozenset(('PATH_NO_EXECUTABLE_PATH','PATH_GOAL_COLLISION','PATH_LOCKED_ROUTE_BLOCKED',
    'TASK_HEADING_BLOCKED','TASK_HEADING_NO_AUTHORIZED_MICRO_TURN',
    'TASK_HEADING_ACTION_UNEXECUTABLE','NAV_NO_PROGRESS'))


class TurnAdjustment:
    def __init__(self,nav):
        self.nav=nav;self.cfg=nav.cfg;self.active=False;self.used=False
        self.phase='WAIT_STOPPED';self.pending=None;self.attempts=0
        self.started=None;self.stop_since=None;self.origin=None;self.target=None
        self.distance=0.;self.request_distance=0.;self.follower=None
        self.progress_pose=None;self.progress_at=None;self.limit_braking=False

    def begin(self,reason):
        nav=self.nav
        if (reason not in TURN_BLOCKED or self.active or self.used or nav.entry is not None
                or nav.phase not in ('OPEN_HEADING','TURN_AROUND') or nav.goal is None):return False
        self.target=nav.goal[2]
        nav.cancel();nav.follower=None
        self.active=True;self.used=True;self.phase='WAIT_STOPPED'
        return True

    def cancel(self):
        if self.pending is not None and self.nav.provider:self.nav.provider.cancel(self.pending)
        self.pending=None

    def fail(self,reason):
        self.cancel()
        return self.nav.owner.trip('TURN_ADJUST_HELP_REQUIRED:'+reason)

    def point(self,distance,heading):
        x,y,h=self.origin
        return (x+distance*math.cos(h),y+distance*math.sin(h),heading)

    def request(self,state,distance,phase):
        self.phase=phase;self.request_distance=distance
        goals=[self.point(distance,self.origin[2]),self.point(distance,self.target)]
        self.nav.goal=goals[-1]
        if self.nav.provider:
            self.pending=self.nav.provider.request(self.nav.task_id,state.pose,goals,
                self.cfg.turn_adjust_distance+.01,0.,allow_detour=False)
        else:self.pending=direct_path(state.pose,goals)

    def pure_plan(self,path,obstacles,boundary):
        """Unit-test substitute for service footprint checks; never used by ROS."""
        c=self.cfg
        def checked(value):
            for part in segments(value):
                length=math.dist(part.start[:2],part.end[:2])
                n=max(1,math.ceil(max(length,abs(part.angle))/.025))
                for i in range(n+1):
                    t=i/n
                    pose=(part.start[0]+(part.end[0]-part.start[0])*t,
                          part.start[1]+(part.end[1]-part.start[1])*t,
                          part.start[2]+part.angle*t if part.kind=='TURN' else part.start[2])
                    if clearance(transform(c.footprint,pose),obstacles,boundary,c.padding)<=c.padding:return False
            return True
        if checked(path):return 'READY',path
        # The existing service tries both directions of a 180-degree turn.
        if self.nav.uturn and abs(abs(wrap(self.target-self.origin[2]))-math.pi)<1e-6:
            pivot=self.point(self.request_distance,self.origin[2])
            reverse=[self.origin,pivot]+[(pivot[0],pivot[1],pivot[2]-math.pi*i/126) for i in range(1,127)]
            if checked(reverse):return 'READY',reverse
        return 'FAILED','NO_EXECUTABLE_PATH'

    def valid_path(self,path,state):
        try:parts=segments(path)
        except ValueError:return False
        if len(parts)!=2 or [p.kind for p in parts]!=['DRIVE','TURN']:return False
        a,b=parts;goal=self.point(self.request_distance,self.target)
        if math.dist(path[0][:2],state.pose[:2])>.005:return False
        if abs(wrap(path[0][2]-state.yaw))>.005:return False
        if math.dist(a.end[:2],goal[:2])>.005 or math.dist(b.end[:2],goal[:2])>.005:return False
        if abs(wrap(b.end[2]-self.target))>.005:return False
        if abs(wrap(a.start[2]-self.origin[2]))>.005:return False
        expected=abs(wrap(self.target-self.origin[2]))
        if abs(abs(b.angle)-expected)>.005:return False
        if not self.nav.uturn and b.angle*wrap(self.target-self.origin[2])<=0:return False
        return math.dist(a.start[:2],a.end[:2])<=self.cfg.turn_adjust_distance+.005

    def tick(self,state,obstacles,boundary,now,stopped):
        c=self.cfg;nav=self.nav
        if self.started is None:self.started=now
        nav.owner.last_diagnostics['turn_adjustment']=dict(phase=self.phase,attempts=self.attempts,
            max_attempts=c.turn_adjust_attempts,selected_distance=self.distance,target_yaw=self.target)
        budget=c.uturn_timeout if nav.uturn else c.maneuver_timeout
        if (now-self.started>=c.turn_adjust_timeout or
                nav.locked_at is not None and now-nav.locked_at>=budget):return self.fail('TIMEOUT')
        if self.phase=='WAIT_STOPPED':
            if not stopped:self.stop_since=None
            elif self.stop_since is None:self.stop_since=now
            if self.stop_since is None or now-self.stop_since<c.stop_dwell:
                return Decision(STOP,'ASSISTING','TURN_ADJUST_WAIT_STOPPED')
            self.origin=state.pose;self.phase='SELECT'
            return Decision(STOP,'ASSISTING','TURN_ADJUST_SEARCH')
        if self.phase=='SELECT':
            if self.attempts>=c.turn_adjust_attempts:return self.fail('NO_SAFE_COMBINATION')
            self.attempts+=1
            self.distance=c.turn_adjust_distance*self.attempts/c.turn_adjust_attempts
            # First test the near edge of the stopping tolerance. Then ask for
            # the actual complete route. Execution still rechecks every action.
            probe=self.distance-c.nav_position_tolerance
            if probe<=0:return Decision(STOP,'ASSISTING','TURN_ADJUST_SEARCH')
            self.request(state,probe,'CHECK_STOP_TOLERANCE')
            return Decision(STOP,'ASSISTING','TURN_ADJUST_SEARCH')
        if self.phase in ('CHECK_STOP_TOLERANCE','PLAN_COMBINATION'):
            if not stopped:return self.fail('MOVED_DURING_SEARCH')
            status,value=(nav.provider.poll(self.pending) if nav.provider
                          else self.pure_plan(self.pending,obstacles,boundary))
            if status=='WAIT':return Decision(STOP,'ASSISTING','TURN_ADJUST_SEARCH')
            self.pending=None
            if status=='FAILED':
                if value in PLAN_BLOCKED:
                    self.phase='SELECT';return Decision(STOP,'ASSISTING','TURN_ADJUST_SEARCH')
                # Transport/health/protocol failures are not geometric retries.
                return nav.owner.trip('PATH_'+str(value))
            if not self.valid_path(value,state):return nav.owner.trip('INVALID_TURN_ADJUST_PATH')
            if self.phase=='CHECK_STOP_TOLERANCE':
                self.request(state,self.distance,'PLAN_COMBINATION')
                return Decision(STOP,'ASSISTING','TURN_ADJUST_SEARCH')
            self.follower=PathFollower(nav.owner,value,nav.task_id+'/adjust')
            self.progress_pose=state.pose;self.progress_at=now;self.phase='EXECUTE'
            return Decision(STOP,'ASSISTING','TURN_ADJUST_ROUTE_READY')
        # Exactly one chosen combination is executed. If it becomes blocked,
        # stop and ask for help rather than moving again to another candidate.
        if math.dist(state.pose[:2],self.origin[:2])>c.turn_adjust_distance+c.nav_position_tolerance:
            return self.fail('DISTANCE_LIMIT')
        movement=math.dist(state.pose[:2],self.progress_pose[:2])+c.radius*abs(wrap(state.yaw-self.progress_pose[2]))
        if movement>=.015:self.progress_pose=state.pose;self.progress_at=now
        if now-self.progress_at>=c.progress_timeout+c.startup_delay+c.stop_dwell:return self.fail('NO_PROGRESS')
        if self.limit_braking and not stopped:
            return Decision(STOP,'ASSISTING','TURN_ADJUST_BRAKE')
        self.limit_braking=False
        d=self.follower.tick(state,obstacles,boundary,now,stopped)
        if nav.owner.latch:return d
        if self.follower.blocked:return self.fail('COMBINATION_BLOCKED')
        if self.follower.done:
            self.active=False
            if nav.turn_only:
                nav.phase='COMPLETED';nav.locked_at=None
                return Decision(STOP,'PASS','NAV_TURN_COMPLETED')
            nav.follower=self.follower;nav.finish(state)
            return Decision(STOP,'ASSISTING','NAV_HEADING_COMPLETED')
        # The follower already checks its conservative minimum-action horizon.
        # Bound this renewed command's actual TTL + transport + full stop too,
        # rather than demanding a new minimum episode at every renewal.
        if d.action==FORWARD:
            bound=nav.owner.predictor.check(state,FORWARD,obstacles,boundary,duration=0.)
            if not bound.safe:return self.fail('STOPPING_ENVELOPE')
            poses=bound.poses
            if not poses or any(math.dist(p[:2],self.origin[:2])>c.turn_adjust_distance+c.nav_position_tolerance for p in poses):
                if stopped:return self.fail('PREDICTED_DISTANCE_LIMIT')
                self.limit_braking=True
                return Decision(STOP,'ASSISTING','TURN_ADJUST_BRAKE')
        return d
