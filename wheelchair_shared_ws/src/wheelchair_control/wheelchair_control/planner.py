"""Fixed-action safety gate; simulator oracle collision map is required.
No speed scaling. All returns are STOP/FORWARD/LEFT/RIGHT.
"""
from dataclasses import dataclass,field
from shapely.geometry import MultiPoint
import math
from .protocol import STOP,FORWARD,LEFT,RIGHT
from .geometry import transform,clearance,wrap,convex,IndexedObstacles,near_boundary

@dataclass
class Config:
    footprint: tuple = ((-.45,-.35),(.55,-.35),(.55,.35),(-.45,.35))
    forward_speed: float = .372
    left_speed: float = .389
    right_speed: float = .403
    linear_accel: float = .6
    angular_accel: float = .8
    linear_brake: float = .5
    angular_brake: float = .7
    sensor_age: float = .25
    map_age: float = 2.
    odom_age: float = .15
    action_ttl: float = .15
    transport_delay: float = .05
    period: float = .05
    startup_delay: float = .12
    minimum_action: float = .30
    # The only stop-zone inflation: footprint + margin.
    margin: float = .06
    assist_angle: float = math.radians(15)
    assist_lateral: float = .25
    assist_duration: float = 2.
    assist_prediction: float = 3.0
    turn_segment: float = .30
    stop_dwell: float = .15
    micro_turn_min: float = .10
    path_correction_angle: float = math.radians(6)
    path_heading_deadband: float = math.radians(2.)
    turn_min_improvement: float = math.radians(.4)
    task_heading_tolerance: float = math.radians(2.)
    task_heading_limit: float = math.radians(25.)
    open_area_heading_bias: float = math.radians(20.)
    maneuver_timeout: float = 20.
    progress_timeout: float = 3.
    turn_adjust_distance: float = .60
    turn_adjust_attempts: int = 3
    turn_adjust_timeout: float = 20.
    nav_position_tolerance: float = .06
    goal_check_period: float = .25
    nav_turn_envelope: float = math.radians(5.)
    path_lookahead: float = 2.0
    uturn_timeout: float = 35.
    path_replan_distance: float = .55
    path_timeout: float = 1.0
    costmap_timeout: float = .5
    context_assist_enter: float = .45
    context_assist_leave: float = .65
    context_stable_time: float = .5
    mode: str = 'stop_only'
    # Stopped A/D turn: 'fixed' = one bounded heading task; 'continuous' =
    # turn while the key is held (hold-to-run input only), stop on release.
    turn_mode: str = 'fixed'
    # WHILL Model F: rear-wheel drive, front casters.  ICC is at the rear axle
    # midpoint, approximately this far forward of base_link.  During a LEFT/RIGHT
    # action the chassis arcs forward at v = icc_x * w; set 0.0 to disable.
    # Measured from /odin1/odometry bag (lever-arm corrected to base_link):
    #   overall median 0.138 m (left 0.050, right 0.159 — asymmetric due to IMU offset)
    icc_x: float = 0.138
    def __post_init__(self):
        if not convex(list(self.footprint)): raise ValueError('Footprint must be a nondegenerate convex polygon')
        for k,v in self.__dict__.items():
            if isinstance(v,(int,float)) and (not math.isfinite(v) or v<=0): raise ValueError(k+' must be positive')
        if (not isinstance(self.turn_adjust_attempts,int) or self.turn_adjust_attempts>3 or
                self.turn_adjust_distance>.60):raise ValueError('Invalid turn adjustment limits')
        if self.mode not in ('stop_only','bounded_assist'): raise ValueError('Invalid mode')
        if self.turn_mode not in ('fixed','continuous'): raise ValueError('Invalid turn mode')
        if self.turn_segment<self.minimum_action: raise ValueError('Turn segment shorter than actuator minimum')
        if self.open_area_heading_bias>math.pi/2: raise ValueError('Open-area bias cannot authorize a backward target')
        if self.nav_turn_envelope>math.pi/12: raise ValueError('Navigation turn envelope must be at most 15 degrees')
        if self.context_assist_leave<=self.context_assist_enter: raise ValueError('Context hysteresis requires leave > enter')
    def velocity(self,action):
        # LEFT/RIGHT: ICC offset produces forward translation during turns.
        # v = icc_x * w so the predictor sweeps the actual arc, not an in-place circle.
        return ((0.,0.),(self.forward_speed,0.),
                (self.icc_x*self.left_speed, self.left_speed),
                (self.icc_x*self.right_speed,-self.right_speed))[action]
    @property
    def radius(self): return max(math.hypot(x,y) for x,y in self.footprint)
    @property
    def reaction(self): return self.sensor_age+self.odom_age+self.transport_delay+self.period
    @property
    def padding(self): return self.margin

@dataclass
class State:
    x: float=0.
    y: float=0.
    yaw: float=0.
    v: float=0.
    w: float=0.
    current_action: int=STOP
    observation_age: float=None
    execution_phase: int=None
    pending_start_age: float=None
    feedback_age: float=None
    @property
    def pose(self): return self.x,self.y,self.yaw

@dataclass
class Result:
    safe: bool
    endpoint: State
    clearance: float
    reason: str=''
    poses: list=field(default_factory=list)
    duration: float=0.
    failure: dict=field(default_factory=dict)
    def summary(self):
        return dict(safe=self.safe,reason=self.reason,clearance=self.clearance,
                    duration=self.duration,stop_pose=self.endpoint.pose if self.safe else None,prediction_endpoint=self.endpoint.pose,
                    failure=self.failure,poses=self.poses)

def toward(value,target,delta):
    return min(target,value+delta) if value<target else max(target,value-delta)

def integrate(s,v,w,dt):
    mid=s.yaw+w*dt/2
    return State(s.x+v*math.cos(mid)*dt,s.y+v*math.sin(mid)*dt,s.yaw+w*dt,v,w,s.current_action,s.observation_age)

class Predictor:
    """Braking-distance check only.

    The stop zone (footprint + margin) is swept from the current pose while
    braking to rest from the larger of the measured and commanded speed on each
    axis. ``run`` bounds how long the action may still accelerate (0 = brake
    from the measured speed only; None = full commanded speed). No reaction
    time, lease, startup or sampling inflation is added.
    """
    BRAKE_STEP=.05  # seconds between swept braking poses
    def __init__(self,cfg): self.cfg=cfg
    def check(self,state,action,obstacles,boundary,reference=None,run=None):
        c=self.cfg;pad=c.padding
        if not isinstance(obstacles,IndexedObstacles):obstacles=IndexedObstacles(obstacles)
        tv,tw=c.velocity(action)
        if run is not None:
            tv=math.copysign(min(abs(tv),abs(state.v)+c.linear_accel*run),tv)
            tw=math.copysign(min(abs(tw),abs(state.w)+c.angular_accel*run),tw)
        v=tv if abs(tv)>=abs(state.v) else state.v
        w=tw if abs(tw)>=abs(state.w) else state.w
        duration=max(abs(v)/c.linear_brake,abs(w)/c.angular_brake)
        n=max(1,math.ceil(duration/self.BRAKE_STEP));dt=duration/n
        s=State(state.x,state.y,state.yaw,v,w,STOP,state.observation_age)
        poses=[s.pose]
        for _ in range(n if duration>0 else 0):
            nv=math.copysign(max(0.,abs(s.v)-c.linear_brake*dt),s.v)
            nw=math.copysign(max(0.,abs(s.w)-c.angular_brake*dt),s.w)
            s=integrate(s,(s.v+nv)/2,(s.w+nw)/2,dt);s.v,s.w=nv,nw
            poses.append(s.pose)
        s.v=s.w=0.;s.current_action=STOP
        current=transform(list(c.footprint),poses[0])
        gap=clearance(current,obstacles,boundary,pad)
        if gap<=pad:
            reason='MAP_BOUNDARY_PROXIMITY' if near_boundary(current,boundary,pad) else 'INITIAL_HARD_BOUNDARY'
            return Result(False,s,gap,reason,poses,duration,dict(time=0.,pose=poses[0],initial=True,hard_margin=pad))
        # Consecutive footprint hulls cover the swept braking area.
        for i,(a,b) in enumerate(zip(poses,poses[1:])):
            hull=list(MultiPoint(transform(list(c.footprint),a)+transform(list(c.footprint),b)).convex_hull.exterior.coords)[:-1]
            g=clearance(hull,obstacles,boundary,pad);gap=min(gap,g)
            if g<=pad:
                return Result(False,s,gap,'COLLISION_OR_UNKNOWN',poses,duration,
                              dict(time=(i+1)*dt,pose=b,initial=False,hard_margin=pad))
        if reference is not None:
            for i,pose in enumerate(poses):
                dx,dy=pose[0]-reference[0],pose[1]-reference[1]
                lateral=-math.sin(reference[2])*dx+math.cos(reference[2])*dy
                if abs(wrap(pose[2]-reference[2]))>c.assist_angle+1e-9 or abs(lateral)>c.assist_lateral:
                    return Result(False,s,gap,'ASSIST_BOUND',poses,duration,
                                  dict(time=i*dt,pose=pose,constraint='ASSIST_BOUND',reference=reference))
        return Result(True,s,gap,'',poses,duration,{})

@dataclass
class Decision:
    action: int
    state: str
    reason: str

class Planner:
    def __init__(self,cfg):
        self.cfg=cfg;self.predictor=Predictor(cfg)
        self.reference=None;self.last_command=STOP
        self.assist_started=None;self.assist_spent=False
        self.side=None;self.phase=None;self.turn_origin=None
        self.target_angle=None;self.turn_started=None;self.stopped_since=None;self.assist_pending_since=None
        self.latch=True;self.acknowledged=False;self.ready=False;self.ack_sequence=-1
        self.reason='STARTUP';self.current_intent=None
        self.last_diagnostics={};self.task_scope=None
    def cancel(self):
        if self.task_scope is not None and hasattr(self.task_scope,'cancel'):self.task_scope.cancel()
        self.task_scope=None
        self.side=None;self.phase=None;self.turn_origin=None;self.target_angle=None;self.turn_started=None;self.stopped_since=None
    def trip(self,reason):
        scope=self.task_scope
        if not self.latch and scope is not None and hasattr(scope,'adjustment'):
            from .turn_adjustment import TURN_BLOCKED
            if scope.adjustment.active and reason in TURN_BLOCKED:
                reason='TURN_ADJUST_HELP_REQUIRED:'+reason
            elif scope.adjustment.begin(reason):
                return Decision(STOP,'ASSISTING','TURN_ADJUST_WAIT_STOPPED')
        # Only hard faults (sensor/executor/clock) require manual rearm.
        # Geometry blocks (obstacle, boundary) clear automatically once the
        # footprint is free again — no key release or R-press needed.
        hard=any(x in reason for x in ('STALE','FAULT','CLOCK','INVALID','REPLAY','ODOM','TF'))
        self.latch=hard;self.acknowledged=not hard;self.ready=not hard;self.reason=reason
        self.cancel()
        return Decision(STOP,'FAULT' if hard else 'BLOCKED',reason)
    def intent(self,packet,session_changed=False):
        if (self.task_scope is not None and packet.task_id==self.task_scope.task_id
                and packet.valid and packet.enabled and packet.command!=STOP
                and self.task_scope.signature!=self.task_scope.fields(packet)):
            self.trip('INVALID_TASK_SCOPE');self.current_intent=packet
            return
        if session_changed:
            # Session change means a node restarted; clear state silently without
            # locking. The geometric checks will gate motion if the new session's
            # first frame is unsafe.
            self.cancel();self.reference=None;self.last_command=STOP
            self.reason='SESSION_CHANGED'
        if not packet.valid:
            self.trip('INVALID_INPUT')
        elif not packet.enabled and packet.command==STOP:
            self.acknowledged=True;self.ack_sequence=packet.sequence
            self.cancel()
        elif self.latch and packet.enabled and packet.command!=STOP:
            if self.ready and packet.sequence>self.ack_sequence:
                self.latch=False;self.ready=False;self.reason='USER_REARMED'
            # Until a fresh disabled STOP has been acknowledged, heartbeat cannot unlock.
        if self.current_intent and packet.command!=self.current_intent.command:
            self.cancel()
        # A NEW confirmed authorization gets its own intervention time budget.
        # Merely renewing FORWARD, sending STOP, or receiving invalid input does
        # not replenish it. Preserve the spatial reference: confirmation cannot
        # ratchet the cumulative angle/lateral limits into an autonomous detour.
        if (not self.latch and packet.valid and packet.enabled and packet.command!=STOP
                and self.current_intent is not None and not self.current_intent.enabled):
            self.cancel();self.assist_started=None;self.assist_spent=False
            self.assist_pending_since=None
        if self.task_scope is not None and packet.task_id!=self.task_scope.task_id:
            self.cancel()
        self.current_intent=packet
    def tick(self,state,obstacles,boundary,now,healthy=True,stopped=True):
        p=self.current_intent
        self.last_diagnostics={}
        if not healthy: return self.trip('SENSOR_OR_EXECUTOR_FAULT')
        footprint_world=transform(list(self.cfg.footprint),state.pose)
        stationary_clear=clearance(footprint_world,obstacles,boundary,self.cfg.padding)>self.cfg.padding
        if not stationary_clear:
            # Distinguish boundary-edge overhang (robot at map edge / unknown area)
            # from genuine obstacle contact so the operator sees a meaningful reason.
            reason='MAP_BOUNDARY_PROXIMITY' if near_boundary(footprint_world,boundary,self.cfg.padding) else 'INITIAL_HARD_BOUNDARY'
            return self.trip(reason)
        self.ready=self.latch and self.acknowledged and stopped and stationary_clear
        if self.latch: return Decision(STOP,'BLOCKED',self.reason+(':READY_TO_REARM' if self.ready else ''))
        if p is None or not p.valid or not p.enabled or p.command==STOP:
            self.cancel()
            return Decision(STOP,'PASS','USER_STOP')
        if p.task_id:
            from .task_scope import TaskScope
            from .navigation import NavigationScope
            if self.task_scope is None:
                self.cancel()
                self.task_scope=NavigationScope(self,p) if p.task_kind in (3,4,5,6,7,8,9) else TaskScope(self,p)
                self.reference=self.task_scope.reference
            return self.task_scope.tick(p,state,obstacles,boundary,now,stopped)
        if p.task_kind:
            return self.trip('INVALID_TASK_ID')
        cmd=p.command
        # Reference is not reset by a safety STOP or rearm; only a deliberate LEFT/RIGHT
        # followed by FORWARD begins a new forward episode.
        if cmd==FORWARD and (self.reference is None or self.last_command in (LEFT,RIGHT)):
            self.reference=state.pose;self.assist_started=None;self.assist_spent=False
        self.last_command=cmd
        if cmd!=FORWARD:
            r=self.predictor.check(state,cmd,obstacles,boundary)
            self.last_diagnostics={'candidate':r.summary()}
            return Decision(cmd,'PASS','USER_REQUEST') if r.safe else self.trip('BLOCKED_TURN')
        reference=self.reference if self.cfg.mode=='bounded_assist' else None
        if reference is not None:
            rel=wrap(state.yaw-reference[2]);lat=-math.sin(reference[2])*(state.x-reference[0])+math.cos(reference[2])*(state.y-reference[1])
            if abs(rel)>self.cfg.assist_angle or abs(lat)>self.cfg.assist_lateral:
                # Drifted outside assist bounds — reset reference to current pose
                # so the next forward attempt uses the new heading, not a latch.
                self.reference=state.pose;self.assist_started=None;self.assist_spent=False
        if self.assist_started is not None and now-self.assist_started>self.cfg.assist_duration:
            return self.trip('ASSIST_TIME_LIMIT')
        if self.phase is not None:
            return self.assist_tick(state,obstacles,boundary,now,stopped)
        direct=self.predictor.check(state,FORWARD,obstacles,boundary,reference=reference)
        self.last_diagnostics={'forward_safe':direct.safe,'clearance':direct.clearance,'candidate':direct.summary()}
        if direct.safe:return Decision(FORWARD,'PASS','USER_REQUEST')
        if self.cfg.mode=='stop_only':return self.trip('BLOCKED_FORWARD')
        # No assist_spent or assist_already_used gate: let the planner retry
        # avoidance each time the user holds forward.
        # No assistance may begin unless residual momentum can be stopped safely.
        stopping=self.predictor.check(state,STOP,obstacles,boundary,reference=reference)
        if not stopping.safe:return self.trip('STOPPING_ENVELOPE_RISK')
        candidates=[]
        self.last_diagnostics['assist_candidates']=[]
        for side in (LEFT,RIGHT):
            turn=self.predictor.check(stopping.endpoint,side,obstacles,boundary,reference=reference)
            if not turn.safe:
                self.last_diagnostics['assist_candidates'].append({'side':side,'safe':False,'reason':turn.reason})
                continue
            forward=self.predictor.check(turn.endpoint,FORWARD,obstacles,boundary,reference=reference)
            total=turn.duration+forward.duration
            self.last_diagnostics['assist_candidates'].append({'side':side,'safe':forward.safe and total<=self.cfg.assist_prediction,'reason':forward.reason,'clearance':forward.clearance,'prediction_after_stop':total})
            if not forward.safe or total>self.cfg.assist_prediction:continue
            progress=(forward.endpoint.x-state.x)*math.cos(reference[2])+(forward.endpoint.y-state.y)*math.sin(reference[2])
            if progress<.04:continue
            # Hard constraints above; clearance cannot compensate for a collision.
            score=progress+.2*min(forward.clearance,1.)-.15*abs(wrap(turn.endpoint.yaw-reference[2]))-.02*2
            candidates.append((score,side,turn.endpoint.yaw))
        if not candidates:return self.trip('BLOCKED_FORWARD')
        candidates.sort(reverse=True)
        # If both sides score within margin, prefer the one the user last turned
        # toward; if no prior preference, pick the better score.
        if len(candidates)>1 and abs(candidates[0][0]-candidates[1][0])<.025:
            if self.last_command==LEFT:candidates=[c for c in candidates if c[1]==LEFT] or candidates
            elif self.last_command==RIGHT:candidates=[c for c in candidates if c[1]==RIGHT] or candidates
        _,self.side,self.target_angle=candidates[0]
        self.phase='STOP_BEFORE_TURN';self.assist_pending_since=now;self.assist_started=None;self.assist_spent=True
        return Decision(STOP,'ASSISTING','ASSIST_STOP_BEFORE_TURN')
    def assist_tick(self,state,obstacles,boundary,now,stopped):
        c=self.cfg
        # Removed ASSIST_TIME_LIMIT and ASSIST_STOP_TIMEOUT: avoidance maneuvers
        # are allowed to take as long as the geometry requires.
        if self.phase in ('STOP_BEFORE_TURN','STOP_AFTER_TURN'):
            if not self.predictor.check(state,STOP,obstacles,boundary,reference=self.reference).safe:
                return self.trip('STOPPING_ENVELOPE_RISK')
            if not stopped:self.stopped_since=None
            elif self.stopped_since is None:self.stopped_since=now
            elif now-self.stopped_since>=c.stop_dwell:
                if self.phase=='STOP_BEFORE_TURN':
                    self.phase='TURN';self.assist_started=now;self.turn_origin=state.yaw;self.turn_started=now;self.stopped_since=None
                else:
                    self.cancel()
                    r=self.predictor.check(state,FORWARD,obstacles,boundary,reference=self.reference)
                    return Decision(FORWARD,'ASSISTING','ASSIST_FORWARD') if r.safe else self.trip('ASSIST_NO_FORWARD_CLEARANCE')
            return Decision(STOP,'ASSISTING','ASSIST_'+self.phase)
        # Rolling check sweeps the angular braking footprint.
        r=self.predictor.check(state,self.side,obstacles,boundary,reference=self.reference,run=0.)
        turned=abs(wrap(state.yaw-self.turn_origin))
        target=abs(wrap(self.target_angle-self.turn_origin))
        predicted_stop=abs(state.w)*(c.transport_delay+c.period+c.action_ttl)+state.w*state.w/(2*c.angular_brake)
        if (not r.safe or turned+predicted_stop>=target):
            self.phase='STOP_AFTER_TURN';self.stopped_since=None
            return Decision(STOP,'ASSISTING','ASSIST_BRAKE_TURN')
        return Decision(self.side,'ASSISTING','ASSIST_BOUNDED_TURN')
