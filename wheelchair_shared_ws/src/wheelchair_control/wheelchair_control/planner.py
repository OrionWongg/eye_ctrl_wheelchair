"""Fixed-action safety gate; simulator oracle collision map is required.
No speed scaling. All returns are STOP/FORWARD/LEFT/RIGHT.
"""
from dataclasses import dataclass,field
from shapely.geometry import MultiPoint
import math
from .protocol import STOP,FORWARD,LEFT,RIGHT
from .geometry import transform,clearance,wrap,convex,IndexedObstacles,collision_diagnostic

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
    margin: float = .06
    model_error: float = .025
    step: float = .025
    assist_angle: float = math.radians(15)
    assist_lateral: float = .25
    assist_duration: float = 2.
    assist_prediction: float = 3.0
    turn_segment: float = .30
    stop_dwell: float = .15
    comfort_distance: float = .06
    micro_turn_min: float = .10
    corridor_heading_limit: float = math.radians(25)
    corridor_correction_angle: float = math.radians(6)
    corridor_heading_enter: float = math.radians(2.)
    corridor_min_improvement: float = math.radians(.4)
    corridor_center_tolerance: float = .035
    corridor_lateral_limit: float = .25
    corridor_max_width: float = 1.5
    passage_entry_distance: float = 1.5
    corridor_side_opening_max: float = 1.5
    nav_corridor_end_behavior: str = 'stop'
    task_heading_tolerance: float = math.radians(2.)
    task_heading_limit: float = math.radians(25.)
    intent_search_distance: float = 3.
    intent_search_timeout: float = 8.
    entry_lookahead: float = 3.
    open_area_heading_bias: float = math.radians(20.)
    maneuver_timeout: float = 20.
    progress_timeout: float = 3.
    turn_adjust_distance: float = .60
    turn_adjust_attempts: int = 3
    turn_adjust_timeout: float = 20.
    nav_position_tolerance: float = .06
    entry_exit_clearance: float = .20
    entry_scan_period: float = .25
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
    def __post_init__(self):
        if not convex(list(self.footprint)): raise ValueError('Footprint must be a nondegenerate convex polygon')
        for k,v in self.__dict__.items():
            if isinstance(v,(int,float)) and (not math.isfinite(v) or v<=0): raise ValueError(k+' must be positive')
        if (not isinstance(self.turn_adjust_attempts,int) or self.turn_adjust_attempts>3 or
                self.turn_adjust_distance>.60):raise ValueError('Invalid turn adjustment limits')
        if self.mode not in ('stop_only','bounded_assist','corridor_assist'): raise ValueError('Invalid mode')
        if self.nav_corridor_end_behavior not in ('stop','continue_straight'):
            raise ValueError('Invalid navigation corridor end behavior')
        if self.turn_segment<self.minimum_action: raise ValueError('Turn segment shorter than actuator minimum')
        if self.open_area_heading_bias>math.pi/2: raise ValueError('Open-area bias cannot authorize a backward target')
        if self.nav_turn_envelope>math.pi/12: raise ValueError('Navigation turn envelope must be at most 15 degrees')
        if self.context_assist_leave<=self.context_assist_enter: raise ValueError('Context hysteresis requires leave > enter')
    def velocity(self,action):
        return ((0.,0.),(self.forward_speed,0.),(0.,self.left_speed),(0.,-self.right_speed))[action]
    @property
    def radius(self): return max(math.hypot(x,y) for x,y in self.footprint)
    @property
    def reaction(self): return self.sensor_age+self.odom_age+self.transport_delay+self.period
    @property
    def padding(self):
        return self.margin+self.model_error+(self.forward_speed+self.radius*max(self.left_speed,self.right_speed))*self.step/2

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
    def __init__(self,cfg): self.cfg=cfg
    def check(self,state,action,obstacles,boundary,duration=None,reference=None,reaction=True,swept_hull=True,max_run=None,diagnostics=True):
        c=self.cfg;s=State(**state.__dict__);poses=[];gap=math.inf;elapsed=0.
        self.last_failure={};times=[]
        # A bounded executor episode cannot accelerate indefinitely during an
        # old observation, startup or lease renewal. Include executor scheduling
        # and downstream STOP transit in the maximum acceleration exposure.
        run_remaining=None if max_run is None else max_run+c.period+c.transport_delay+c.period
        run_started=False
        # A monotone, zero-angular-velocity straight rollout sweeps exactly the
        # convex hull of its endpoint footprints. Preserve the same dynamics and
        # sampling inflation, but avoid hundreds of duplicate polygon queries.
        fast_straight=(swept_hull and action in (STOP,FORWARD) and abs(state.w)<1e-4 and state.v>=0 and state.current_action in (STOP,FORWARD))
        if not isinstance(obstacles,IndexedObstacles):obstacles=IndexedObstacles(obstacles)
        pad=c.margin+c.model_error+(max(abs(s.v),c.forward_speed)+c.radius*max(abs(s.w),c.left_speed,c.right_speed))*c.step/2
        def inspect():
            nonlocal gap
            poses.append(s.pose);times.append(elapsed)
            # All geometry and reference bounds are constant for this rollout.
            # Identical consecutive poses already passed inspection; keep the
            # time samples, but do not repeat the same native geometry work.
            if len(poses)>1 and poses[-1]==poses[-2]:return True,''
            g=math.inf if fast_straight else clearance(transform(list(c.footprint),s.pose),obstacles,boundary,pad)
            gap=min(gap,g)
            if g<=pad:
                # Always compute the diagnostic at the initial pose to distinguish
                # boundary-edge overhang from genuine obstacle contact.
                if diagnostics or elapsed==0:
                    self.last_failure=collision_diagnostic(transform(list(c.footprint),s.pose),obstacles,boundary,pad)
                else:
                    self.last_failure={}
                self.last_failure.update(time=elapsed,pose=s.pose,initial=elapsed==0,
                    hard_margin=c.margin,model_error=c.model_error,sampling_error=pad-c.margin-c.model_error,
                    comfort_distance=c.comfort_distance)
                if elapsed==0:
                    reason='MAP_BOUNDARY_PROXIMITY' if self.last_failure.get('obstacle_id')=='known_free_boundary' else 'INITIAL_HARD_BOUNDARY'
                else:
                    reason='COLLISION_OR_UNKNOWN'
                return False,reason
            if reference is not None:
                dx,dy=s.x-reference[0],s.y-reference[1]
                lateral=-math.sin(reference[2])*dx+math.cos(reference[2])*dy
                if abs(wrap(s.yaw-reference[2]))>c.assist_angle+1e-9 or abs(lateral)>c.assist_lateral:
                    self.last_failure=dict(time=elapsed,pose=s.pose,initial=elapsed==0,constraint='ASSIST_BOUND',reference=reference,lateral_error=lateral,angle_error=wrap(s.yaw-reference[2]))
                    return False,'ASSIST_BOUND'
            return True,''
        def finish(ok,why):
            nonlocal gap
            if fast_straight:
                initial=transform(list(c.footprint),poses[0])
                def swept(i):
                    # Keep tiny odometry angular residuals in the geometry instead
                    # of dropping to hundreds of map queries. The hull of EVERY
                    # sampled footprint is conservative; no residual is zeroed.
                    points=(initial+transform(list(c.footprint),poses[i]) if abs(state.w)<1e-9
                            else [point for pose in poses[:i+1] for point in transform(list(c.footprint),pose)])
                    return list(MultiPoint(points).convex_hull.exterior.coords)[:-1]
                gap=clearance(swept(len(poses)-1),obstacles,boundary,pad)
                if gap<=pad:
                    if not diagnostics:return Result(False,s,gap,'COLLISION_OR_UNKNOWN',poses,elapsed,{})
                    lo,hi=0,len(poses)-1
                    while lo<hi:
                        mid=(lo+hi)//2
                        if clearance(swept(mid),obstacles,boundary,pad)<=pad:hi=mid
                        else:lo=mid+1
                    self.last_failure=collision_diagnostic(transform(list(c.footprint),poses[lo]),obstacles,boundary,pad)
                    self.last_failure.update(time=times[lo],pose=poses[lo],initial=lo==0,hard_margin=c.margin,
                        model_error=c.model_error,sampling_error=pad-c.margin-c.model_error,comfort_distance=c.comfort_distance)
                    if lo==0:
                        ok=False;why='MAP_BOUNDARY_PROXIMITY' if self.last_failure.get('obstacle_id')=='known_free_boundary' else 'INITIAL_HARD_BOUNDARY'
                    else:
                        ok=False;why='COLLISION_OR_UNKNOWN'
            return Result(ok,s,gap,why,poses,elapsed,self.last_failure.copy())
        def phase(seconds,target=None,braking=False):
            nonlocal s,elapsed,run_remaining,run_started
            n=max(1,math.ceil(seconds/c.step));dt=seconds/n
            for _ in range(n):
                effective=target;is_braking=braking
                if run_remaining is not None and action in (LEFT,RIGHT):
                    if target==c.velocity(action):run_started=True
                    if run_started:
                        if run_remaining<=0:
                            # The executor's bounded episode has already forced
                            # STOP. Braking must continue through the transit
                            # phase too; a target=None phase cannot restart a
                            # fictional coast after that deadline.
                            effective=(0.,0.);is_braking=True
                        run_remaining-=dt
                if effective is None: v,w=s.v,s.w
                else:
                    av=c.linear_brake if is_braking else c.linear_accel
                    aw=c.angular_brake if is_braking else c.angular_accel
                    v=toward(s.v,effective[0],av*dt);w=toward(s.w,effective[1],aw*dt)
                # Use average segment velocity for trapezoidal acceleration integration.
                moved=integrate(s,(s.v+v)/2,(s.w+w)/2,dt)
                s=State(moved.x,moved.y,moved.yaw,v,w,s.current_action,s.observation_age);elapsed+=dt
                ok,why=inspect()
                if not ok: return False,why
            return True,''
        def brake():
            duration=max(abs(s.v)/c.linear_brake,abs(s.w)/c.angular_brake)
            result=phase(duration,(0.,0.),True)
            s.current_action=STOP
            return result
        ok,why=inspect()
        if not ok: return finish(False,why)
        phases=[]
        # The latest observation may be old: continue current momentum before acting.
        reaction_time=(c.reaction if s.observation_age is None else s.observation_age+c.transport_delay+c.period) if reaction else 0.
        if (reaction and s.execution_phase==3 and s.pending_start_age is not None
                and math.isfinite(s.pending_start_age) and s.pending_start_age>=0
                and abs(s.v)<.015 and abs(s.w)<.025):
            # Timestamped STARTING feedback proves output was STOP until that
            # sample. Older scan/map ages must not invent earlier acceleration.
            # Retain residual momentum, then allow immediate startup after the
            # feedback sample, including all transport/control delay.
            target=c.velocity(s.current_action)
            possible_start=min(reaction_time,s.pending_start_age+c.transport_delay+c.period)
            ok,why=phase(reaction_time-possible_start)
            if not ok:return finish(False,why)
            ok,why=phase(possible_start,target)
        else:
            ok,why=phase(reaction_time,c.velocity(s.current_action) if s.current_action!=STOP else None)
        if not ok: return finish(False,why)
        if action==STOP:
            ok,why=brake()
            return finish(ok,why)
        target=c.velocity(action)
        incompatible=(abs(s.v)>.005 and action!=FORWARD) or (abs(s.w)>.005 and (action==FORWARD or s.w*target[1]<0))
        if incompatible:
            ok,why=brake()
            if not ok:return finish(False,why)
        if abs(s.v)<.005 and abs(s.w)<.005:
            ok,why=phase(c.startup_delay,(0.,0.))
            if not ok:return finish(False,why)
        hold=c.minimum_action if duration is None else duration
        ok,why=phase(hold+c.action_ttl,target)
        if not ok:return finish(False,why)
        # Final command can still be in transit; only then guarantee braking.
        ok,why=phase(c.transport_delay+c.period)
        if not ok:return finish(False,why)
        ok,why=brake()
        return finish(ok,why)

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
        self.last_diagnostics={};self.local=None;self.manual_heading=False;self.manual_departure_axis=None
        self.manual_turn=None;self.task_scope=None
    def cancel(self):
        if self.task_scope is not None and hasattr(self.task_scope,'cancel'):self.task_scope.cancel()
        self.task_scope=None
        self.local=None
        self.manual_turn=None
        self.side=None;self.phase=None;self.turn_origin=None;self.target_angle=None;self.turn_started=None;self.stopped_since=None
    def trip(self,reason):
        scope=self.task_scope
        if not self.latch and scope is not None and hasattr(scope,'adjustment'):
            from .turn_adjustment import TURN_BLOCKED
            if scope.adjustment.active and reason in TURN_BLOCKED:
                reason='TURN_ADJUST_HELP_REQUIRED:'+reason
            elif scope.adjustment.begin(reason):
                return Decision(STOP,'ASSISTING','TURN_ADJUST_WAIT_STOPPED')
        self.latch=True;self.acknowledged=False;self.ready=False;self.reason=reason
        self.cancel()
        return Decision(STOP,'FAULT' if any(x in reason for x in ('STALE','FAULT','CLOCK','SESSION','INVALID','REPLAY','ODOM','TF','MAP')) else 'BLOCKED',reason)
    def intent(self,packet,session_changed=False):
        if (self.task_scope is not None and packet.task_id==self.task_scope.task_id
                and packet.valid and packet.enabled and packet.command!=STOP
                and self.task_scope.signature!=self.task_scope.fields(packet)):
            self.trip('INVALID_TASK_SCOPE');self.current_intent=packet
            return
        # An explicit turn is a human route choice. Capture the old passage axis
        # before cancelling it, so the next FORWARD cannot snap back onto it.
        if packet.valid and packet.enabled and packet.command in (LEFT,RIGHT):
            self.manual_heading=True
            if self.local is not None and self.local.passage is not None:
                self.manual_departure_axis=self.local.passage.heading
            elif self.manual_departure_axis is None and self.reference is not None:
                self.manual_departure_axis=round(self.reference[2]/(math.pi/2))*(math.pi/2)
        if session_changed:
            self.trip('INTENT_SESSION_CHANGED');self.reference=None;self.last_command=STOP
            self.manual_heading=False;self.manual_departure_axis=None
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
            diag=collision_diagnostic(footprint_world,obstacles,boundary,self.cfg.padding)
            self.last_diagnostics['current_geometry']=diag
            # Distinguish boundary-edge overhang (robot at map edge / unknown area)
            # from genuine obstacle contact so the operator sees a meaningful reason.
            reason='MAP_BOUNDARY_PROXIMITY' if diag.get('obstacle_id')=='known_free_boundary' else 'INITIAL_HARD_BOUNDARY'
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
            if self.cfg.mode=='corridor_assist':
                return self.manual_turn_tick(state,cmd,obstacles,boundary,now,stopped)
            r=self.predictor.check(state,cmd,obstacles,boundary)
            self.last_diagnostics={'candidate':r.summary()}
            return Decision(cmd,'PASS','USER_REQUEST') if r.safe else self.trip('BLOCKED_TURN')
        if self.cfg.mode=='corridor_assist':
            from .passage import PassageController
            if self.local is None:self.local=PassageController(self)
            return self.local.tick(state,obstacles,boundary,now,stopped)
        reference=self.reference if self.cfg.mode=='bounded_assist' else None
        if reference is not None:
            rel=wrap(state.yaw-reference[2]);lat=-math.sin(reference[2])*(state.x-reference[0])+math.cos(reference[2])*(state.y-reference[1])
            if abs(rel)>self.cfg.assist_angle or abs(lat)>self.cfg.assist_lateral:return self.trip('ASSIST_BOUND')
        if self.assist_started is not None and now-self.assist_started>self.cfg.assist_duration:
            return self.trip('ASSIST_TIME_LIMIT')
        if self.phase is not None:
            return self.assist_tick(state,obstacles,boundary,now,stopped)
        direct=self.predictor.check(state,FORWARD,obstacles,boundary,reference=reference)
        self.last_diagnostics={'forward_safe':direct.safe,'clearance':direct.clearance,'candidate':direct.summary()}
        if direct.safe:return Decision(FORWARD,'PASS','USER_REQUEST')
        if self.cfg.mode=='stop_only':return self.trip('BLOCKED_FORWARD')
        if self.assist_spent:return self.trip('ASSIST_ALREADY_USED')
        # No assistance may begin unless residual momentum can be stopped safely.
        stopping=self.predictor.check(state,STOP,obstacles,boundary,reference=reference)
        if not stopping.safe:return self.trip('STOPPING_ENVELOPE_RISK')
        candidates=[]
        self.last_diagnostics['assist_candidates']=[]
        for side in (LEFT,RIGHT):
            turn=self.predictor.check(stopping.endpoint,side,obstacles,boundary,duration=self.cfg.turn_segment,reference=reference,reaction=False)
            if not turn.safe:
                self.last_diagnostics['assist_candidates'].append({'side':side,'safe':False,'reason':turn.reason})
                continue
            forward=self.predictor.check(turn.endpoint,FORWARD,obstacles,boundary,duration=2*self.cfg.minimum_action,reference=reference,reaction=False)
            total=turn.duration+forward.duration
            self.last_diagnostics['assist_candidates'].append({'side':side,'safe':forward.safe and total<=self.cfg.assist_prediction,'reason':forward.reason,'clearance':forward.clearance,'prediction_after_stop':total})
            if not forward.safe or total>self.cfg.assist_prediction:continue
            progress=(forward.endpoint.x-state.x)*math.cos(reference[2])+(forward.endpoint.y-state.y)*math.sin(reference[2])
            if progress<.04:continue
            # Hard constraints above; clearance cannot compensate for a collision.
            score=progress+.2*min(forward.clearance,1.)-.15*abs(wrap(turn.endpoint.yaw-reference[2]))-.02*2
            candidates.append((score,side,turn.endpoint.yaw))
        if not candidates:return self.trip('NO_USEFUL_SAFE_ASSIST')
        candidates.sort(reverse=True)
        if len(candidates)>1 and abs(candidates[0][0]-candidates[1][0])<.025:
            return self.trip('AMBIGUOUS_SIDE_CHOOSE_DIRECTION')
        _,self.side,self.target_angle=candidates[0]
        self.phase='STOP_BEFORE_TURN';self.assist_pending_since=now;self.assist_started=None;self.assist_spent=True
        return Decision(STOP,'ASSISTING','ASSIST_STOP_BEFORE_TURN')
    def manual_turn_tick(self,state,cmd,obstacles,boundary,now,stopped):
        """Preserve the requested direction, using stopped micro-turns near walls.

        Each pulse retains the full lease-loss and braking envelope. A held
        request may continue after settling only when another pulse is safe.
        Reaching its geometric limit leaves STOP until a different user command.
        """
        pulse=self.manual_turn;c=self.cfg
        if pulse is not None:
            self.last_diagnostics['manual_turn_phase']=pulse['phase']
            if pulse['phase']=='LIMIT':return Decision(STOP,'PASS','USER_TURN_LIMIT_REACHED')
            if now-pulse['started']>c.assist_duration:return self.trip('MANUAL_TURN_TIMEOUT')
            if pulse['phase']=='TURN':
                r=self.predictor.check(state,cmd,obstacles,boundary,duration=0.,max_run=c.micro_turn_min)
                self.last_diagnostics['candidate']=r.summary()
                running=state.execution_phase==1 and state.current_action==cmd
                if state.execution_phase is None:running=abs(state.w)>.02
                if running and pulse['motion_at'] is None:pulse['motion_at']=now
                elapsed=0. if pulse['motion_at'] is None else now-pulse['motion_at']
                if not r.safe or elapsed>=c.micro_turn_min:
                    pulse['phase']='SETTLE';pulse['brake_at']=now
                    return Decision(STOP,'ASSISTING','USER_MICRO_TURN_BRAKE')
                return Decision(cmd,'ASSISTING','USER_MICRO_TURN')
            # The stopped bit may still describe startup, before the last turn
            # command reached the base. Wait out transit and a fresh stop dwell;
            # STARTING/RUNNING feedback is never completion of this pulse.
            if (not stopped or state.execution_phase not in (None,0) or
                    now-pulse['brake_at']<c.transport_delay+c.stop_dwell):
                return Decision(STOP,'ASSISTING','USER_MICRO_TURN_SETTLE')
            # A human-requested pulse may make a useful sub-degree adjustment.
            # The corridor optimizer's minimum scoring gain is not a manual
            # motion limit. Stop retrying only when there was no resolved turn.
            if abs(wrap(state.yaw-pulse['yaw']))<1e-4:
                pulse['phase']='LIMIT'
                return Decision(STOP,'PASS','USER_TURN_LIMIT_REACHED')
        full=self.predictor.check(state,cmd,obstacles,boundary)
        self.last_diagnostics['candidate']=full.summary()
        if full.safe:
            self.manual_turn=None
            return Decision(cmd,'PASS','USER_REQUEST')
        # Stop existing momentum before starting or sizing a short pulse.
        if not stopped:return Decision(STOP,'ASSISTING','USER_TURN_WAIT_STOPPED')
        short=self.predictor.check(state,cmd,obstacles,boundary,duration=c.micro_turn_min,max_run=c.micro_turn_min)
        self.last_diagnostics['candidate']=short.summary()
        if not short.safe:
            if pulse is None:return self.trip('BLOCKED_TURN')
            pulse['phase']='LIMIT'
            return Decision(STOP,'PASS','USER_TURN_LIMIT_REACHED')
        self.manual_turn=dict(phase='TURN',started=now,motion_at=None,yaw=state.yaw)
        self.last_diagnostics['planned_turn_duration']=c.micro_turn_min
        return Decision(STOP,'ASSISTING','USER_MICRO_TURN_PREPARE')

    def assist_tick(self,state,obstacles,boundary,now,stopped):
        c=self.cfg
        if self.assist_started is not None and now-self.assist_started>c.assist_duration:return self.trip('ASSIST_TIME_LIMIT')
        if self.assist_started is None and now-self.assist_pending_since>2.:return self.trip('ASSIST_STOP_TIMEOUT')
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
        # Rolling check includes delayed stop and full angular braking footprint.
        r=self.predictor.check(state,self.side,obstacles,boundary,duration=0.,reference=self.reference)
        turned=abs(wrap(state.yaw-self.turn_origin))
        target=abs(wrap(self.target_angle-self.turn_origin))
        predicted_stop=abs(state.w)*(c.transport_delay+c.period+c.action_ttl)+state.w*state.w/(2*c.angular_brake)
        if (not r.safe or turned+predicted_stop>=target):
            self.phase='STOP_AFTER_TURN';self.stopped_since=None
            return Decision(STOP,'ASSISTING','ASSIST_BRAKE_TURN')
        return Decision(self.side,'ASSISTING','ASSIST_BOUNDED_TURN')
