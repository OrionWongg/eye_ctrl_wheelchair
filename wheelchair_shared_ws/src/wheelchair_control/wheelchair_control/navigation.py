"""Intent goals and context; every movement is executed by one PathFollower."""
import math
import time
from .planner import Decision,State
from .protocol import STOP,FORWARD,LEFT,RIGHT
from .task_scope import TaskScope
from .path_tracking import PathFollower,direct_path
from .geometry import wrap,transform,clearance


class NavigationScope:
    fields=staticmethod(TaskScope.fields)
    def __init__(self,owner,packet):
        self.owner=owner;self.cfg=owner.cfg;self.task_id=packet.task_id
        self.signature=self.fields(packet)
        self.reference=(packet.reference_x,packet.reference_y,packet.reference_yaw)
        self.side={4:1,5:-1,7:1,8:-1}.get(packet.task_kind,0);self.uturn=packet.task_kind in (6,9)
        self.turn_only=packet.task_kind in (7,8,9)
        self.phase='INITIALIZE';self.started=None;self.locked_at=None
        self.distance=0.;self.previous=self.reference
        self.follower=None;self.goal=None;self.goals=[]
        self.provider=getattr(owner,'path_provider',None);self.pending=None;self.pending_kind=None
        self.last_context=-math.inf
        self.progress_at=None;self.progress_pose=None
        self.context='ROAM';self.context_candidate=None;self.context_since=None
        self.no_forward_space=False;self.replanning=False
        self.open_task=False;self.next_goal_check=-math.inf
        from .turn_adjustment import TurnAdjustment
        self.adjustment=TurnAdjustment(self)

    def cancel(self):
        self.adjustment.cancel()
        if self.follower is not None and hasattr(self.follower,"cancel"):self.follower.cancel()
        if self.provider and self.pending is not None:self.provider.cancel(self.pending)
        self.pending=None

    def semantic(self,obstacles):return getattr(self.owner,'semantic_obstacles',obstacles)

    def request(self,start,goals,kind):
        self.goals=list(goals);self.goal=tuple(goals[-1]);self.pending_kind=kind
        if self.provider:
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip([start,*goals],goals))
            self.pending=self.provider.request(self.task_id,start,goals,min(10.,max(3.,length*2.5)),
                allow_detour=not self.turn_only)
        else:self.pending=direct_path(start,goals)

    def poll(self,state,now):
        if self.pending is None:return None
        status,value=self.provider.poll(self.pending) if self.provider else ('READY',self.pending)
        if status=='WAIT':return None
        kind=self.pending_kind;self.pending=None
        if status=='FAILED':
            return self.owner.trip('PATH_'+str(value))
        if self.provider is None and kind in ('UTURN','HEADING'):
            def safe(path):
                return all(clearance(transform(self.cfg.footprint,p),self.check_obstacles,self.boundary,self.cfg.padding)>self.cfg.padding for p in path)
            if not safe(value):
                if kind=='UTURN':
                    value=[(state.x,state.y,state.yaw-math.pi*i/180) for i in range(181)]
                if kind!='UTURN' or not safe(value):return self.owner.trip('PATH_NO_EXECUTABLE_PATH')
        try:
            if kind=='APPEND' and self.follower and not self.follower.done:
                self.follower.append(value)
            else:
                self.follower=(self.owner.continuous_controller.follower(self.owner,value,self.task_id)
                               if getattr(self.owner,"continuous_controller",None) else PathFollower(self.owner,value,self.task_id))
                self.follower.open_ended=self.phase=='CRUISE'
        except ValueError:
            return self.owner.trip('INVALID_EXECUTABLE_PATH')
        self.replanning=False
        self.progress_at,self.progress_pose=now,state.pose
        return None

    def update_context(self,state,obstacles,now):
        if now-self.last_context<self.cfg.goal_check_period:return
        self.last_context=now
        # Display-only hysteresis between open roaming and obstacle proximity.
        gap=clearance(transform(self.cfg.footprint,state.pose),obstacles,self.boundary,0.)
        threshold=self.cfg.context_assist_leave if self.context=='ASSIST' else self.cfg.context_assist_enter
        wanted='ASSIST' if gap<threshold else 'ROAM'
        if wanted!=self.context_candidate:self.context_candidate=wanted;self.context_since=now
        elif wanted!=self.context and now-self.context_since>=self.cfg.context_stable_time:self.context=wanted

    def cruise_goal(self,start,state,obstacles):
        c=self.cfg;h=self.reference[2]
        target=(start[0]+c.path_lookahead*math.cos(h),start[1]+c.path_lookahead*math.sin(h),h)
        self.no_forward_space=False
        length=math.dist(start[:2],target[:2])
        # Place cruise endpoints where a full-speed stop still fits. A wall at
        # the nominal lookahead must not prevent all earlier safe progress.
        for remaining in [length-i*.25 for i in range(math.ceil(length/.25))]:
            if remaining<.12:continue
            ratio=remaining/length
            candidate=(start[0]+(target[0]-start[0])*ratio,start[1]+(target[1]-start[1])*ratio,h)
            future=State(x=candidate[0],y=candidate[1],yaw=h,v=c.forward_speed,
                         current_action=FORWARD,observation_age=state.observation_age)
            if self.owner.predictor.check(future,STOP,obstacles,self.boundary).safe:return candidate
        self.no_forward_space=True
        return None

    def cruise(self,state,obstacles,boundary,now,stopped):
        if self.follower is None and self.pending is None:
            goal=self.cruise_goal(state.pose,state,obstacles)
            if goal is None:return self.owner.trip('NAV_NO_FORWARD_SPACE')
            self.request(state.pose,[goal],'CRUISE')
        failure=self.poll(state,now)
        if failure:return failure
        if self.follower is None:return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        if self.follower.blocked:
            if not stopped:return Decision(STOP,'ASSISTING','PATH_WAIT_REPLAN_STOP')
            if self.pending is None:
                goal=self.follower.last_goal;self.follower=None
                self.request(state.pose,[goal],'CRUISE')
            return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        # Keep a planning-latency budget of committed path beyond braking
        # distance. Only a failed attempt is throttled; a pending request
        # already prevents duplicates.
        if (self.pending is None and now>=self.next_goal_check and
                self.follower.remaining(state)<self.follower.replan_distance(state)):
            start=self.follower.last_goal
            goal=self.cruise_goal(start,state,obstacles)
            if goal is not None:self.request(start,[goal],'APPEND')
            else:self.next_goal_check=now+self.cfg.goal_check_period
        self.follower.extending=self.pending is not None and self.pending_kind=='APPEND'
        d=self.follower.tick(state,obstacles,boundary,now,stopped)
        if self.adjustment.active or self.owner.latch:return d
        if self.follower.done:
            if self.no_forward_space:return self.owner.trip('NAV_NO_FORWARD_SPACE')
            self.follower=None
        if d.action==FORWARD:d.reason='NAV_CRUISE'
        return d

    def tick(self,packet,state,obstacles,boundary,now,stopped):
        if (self.signature!=self.fields(packet) or packet.command!=FORWARD or
            not all(math.isfinite(v) for v in (*self.reference,packet.task_elapsed)) or packet.task_elapsed<0):
            return self.owner.trip('INVALID_NAVIGATION_SCOPE')
        self.boundary=boundary
        self.check_obstacles=obstacles
        if self.started is None:
            self.started=now-packet.task_elapsed;self.progress_at,self.progress_pose=now,state.pose
        self.distance+=math.dist(self.previous[:2],state.pose[:2]);self.previous=state.pose
        begin=time.monotonic()
        # Pure turn tasks use collision validation, not cruise context.
        if not (self.turn_only or self.uturn):self.update_context(state,obstacles,now)
        context_done=time.monotonic()
        d=(self.adjustment.tick(state,obstacles,boundary,now,stopped) if self.adjustment.active
           else self.step(state,obstacles,boundary,now,stopped))
        self.owner.last_diagnostics['navigation_timing_ms']={
            'context':(context_done-begin)*1000,'step':(time.monotonic()-context_done)*1000}
        if self.provider:self.owner.last_diagnostics['path_search']=self.provider.last_report
        self.owner.last_diagnostics['navigation']=dict(phase=('TURN_ADJUST_'+self.adjustment.phase if self.adjustment.active else self.phase),preference=self.side,
            context=self.context,
            interpretation='uturn' if self.uturn else 'open_heading' if self.open_task else 'cruise',
            remaining_time=max(0.,(self.cfg.uturn_timeout if self.uturn else self.cfg.maneuver_timeout)-(now-self.locked_at)) if self.locked_at is not None else 0.,
            goal=self.goal,reference=self.reference,backend=getattr(self.provider,'backend','nav2_dwb') if self.provider else 'unit_direct')
        return d

    def step(self,state,obstacles,boundary,now,stopped):
        c=self.cfg
        if self.phase=='COMPLETED':return Decision(STOP,'PASS','NAV_TURN_COMPLETED')
        if self.phase=='INITIALIZE':
            if self.uturn:
                self.phase='TURN_AROUND';self.locked_at=now
                self.request(state.pose,[(state.x,state.y,state.yaw+math.pi)],'UTURN')
            elif self.turn_only and c.turn_mode=='continuous':
                # No target angle or time budget: the held key is the lease.
                self.phase='CONTINUOUS_TURN'
            elif self.turn_only:
                self.phase='OPEN_HEADING';self.open_task=True;self.locked_at=now
                self.request(state.pose,[(state.x,state.y,state.yaw+self.side*c.open_area_heading_bias)],'HEADING')
            elif not self.side:self.phase='CRUISE'
            else:
                self.phase='OPEN_HEADING';self.open_task=True;self.locked_at=now
                self.request(state.pose,[(state.x,state.y,self.reference[2]+self.side*c.open_area_heading_bias)],'HEADING')
        if self.locked_at is not None:
            timeout=c.uturn_timeout if self.uturn else c.maneuver_timeout
            if now-self.locked_at>=timeout:return self.owner.trip('NAV_MANEUVER_TIMEOUT')
            movement=math.dist(state.pose[:2],self.progress_pose[:2])+abs(wrap(state.yaw-self.progress_pose[2]))*c.radius
            if movement>=.015:self.progress_at,self.progress_pose=now,state.pose
            if now-self.progress_at>c.progress_timeout+c.stop_dwell+c.startup_delay and self.pending is None:
                return self.owner.trip('NAV_NO_PROGRESS')
        if self.phase=='CONTINUOUS_TURN':return self.continuous_turn(state,obstacles,boundary)
        if self.phase=='CRUISE':return self.cruise(state,obstacles,boundary,now,stopped)
        failure=self.poll(state,now)
        if failure:return failure
        if self.follower is None:return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        if self.follower.blocked:
            if not stopped:return Decision(STOP,'ASSISTING','PATH_WAIT_REPLAN_STOP')
            # Replan to the same locked goal; no event/search budget is reset.
            if self.pending is None:
                goals=[p.end for p in self.follower.parts[self.follower.index:]]
                self.follower=None;self.request(state.pose,goals[:8],'REPLAN')
            return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        d=self.follower.tick(state,obstacles,boundary,now,stopped)
        if self.adjustment.active or self.owner.latch:return d
        if self.follower.done:
            if self.turn_only:
                self.phase='COMPLETED';self.locked_at=None
                return Decision(STOP,'PASS','NAV_TURN_COMPLETED')
            self.finish(state)
            return Decision(STOP,'ASSISTING','NAV_HEADING_COMPLETED')
        if d.action==FORWARD:d.reason='NAV_'+self.phase
        return d

    def continuous_turn(self,state,obstacles,boundary):
        # Full-speed braking sweep is checked every cycle; a blocked turn stops
        # without latching and resumes by itself once the sweep is clear.
        cmd=LEFT if self.side>0 else RIGHT
        result=self.owner.predictor.check(state,cmd,obstacles,boundary)
        self.owner.last_diagnostics['candidate']=result.summary()
        if result.safe:
            controller=getattr(self.owner,'continuous_controller',None)
            if controller:return controller.checked(self.owner,state,obstacles,boundary,(0.,.821 if cmd==LEFT else -.800),'NAV_CONTINUOUS_TURN')
            return Decision(cmd,'ASSISTING','NAV_CONTINUOUS_TURN')
        return Decision(STOP,'ASSISTING','NAV_TURN_WAIT_CLEARANCE')

    def finish(self,state):
        heading=self.follower.last_goal[2] if self.follower is not None else state.yaw
        self.phase='CRUISE';self.side=0;self.uturn=False;self.open_task=False
        self.locked_at=None;self.reference=(state.x,state.y,heading);self.goal=None
        self.last_context=-math.inf
        if self.follower and self.follower.done:self.follower=None
