"""Intent goals and context; every movement is executed by one PathFollower."""
import math
import time
from .planner import Decision,State
from .protocol import STOP,FORWARD
from .task_scope import TaskScope
from .path_tracking import PathFollower,direct_path
from .passage import detect
from .geometry import wrap,transform,clearance
from .entries import openings,route_clear,coordinates


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
        self.entry=None;self.candidate_entry=None;self.candidates=[]
        self.passage=None;self.follower=None;self.goal=None;self.goals=[]
        self.provider=getattr(owner,'path_provider',None);self.pending=None;self.pending_kind=None
        self.last_scan=-math.inf;self.last_context=-math.inf
        self.progress_at=None;self.progress_pose=None
        self.context='ROAM';self.context_candidate=None;self.context_since=None
        self.end_of_corridor=False;self.replanning=False;self.search_expired=False
        self.open_task=False;self.next_goal_check=-math.inf
        from .turn_adjustment import TurnAdjustment
        self.adjustment=TurnAdjustment(self)

    def cancel(self):
        self.adjustment.cancel()
        if self.provider and self.pending is not None:self.provider.cancel(self.pending)
        self.pending=None

    def semantic(self,obstacles):return getattr(self.owner,'semantic_obstacles',obstacles)

    def request(self,start,goals,kind,band=0.):
        self.goals=list(goals);self.goal=tuple(goals[-1]);self.pending_kind=kind
        if self.provider:
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip([start,*goals],goals))
            self.pending=self.provider.request(self.task_id,start,goals,min(10.,max(3.,length*2.5)),band,
                allow_detour=kind!='ENTRY' and self.entry is None and not self.turn_only,
                corridor_origin=self.corridor_origin() if band else None)
        else:self.pending=direct_path(start,goals)

    def corridor_origin(self):
        p=self.passage
        return (p.origin[0]-p.center*math.sin(p.heading),
                p.origin[1]+p.center*math.cos(p.heading),p.heading)

    def poll(self,state,now):
        if self.pending is None:return None
        status,value=self.provider.poll(self.pending) if self.provider else ('READY',self.pending)
        if status=='WAIT':return None
        kind=self.pending_kind;self.pending=None
        if status=='FAILED':
            if kind=='ENTRY' and value in ('NO_EXECUTABLE_PATH','GOAL_COLLISION','LOCKED_ROUTE_BLOCKED'):
                self.candidate_entry=None
                return None
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
            else:self.follower=PathFollower(self.owner,value,self.task_id)
        except ValueError:
            return self.owner.trip('INVALID_EXECUTABLE_PATH')
        self.replanning=False
        if kind=='ENTRY':
            self.entry=self.candidate_entry;self.candidate_entry=None
            self.locked_at=now;self.phase='APPROACH_ENTRY'
        self.progress_at,self.progress_pose=now,state.pose
        return None

    def update_context(self,state,obstacles,now):
        if now-self.last_context<self.cfg.entry_scan_period:return
        self.last_context=now
        geometry=self.semantic(obstacles)
        observed,_=detect(state,geometry,self.cfg,self.passage)
        if self.passage is None and self.phase!='INITIALIZE':
            if observed is not None and abs(wrap(observed.heading-self.reference[2]))>self.cfg.task_heading_tolerance:
                observed=None
        if self.passage is not None and (self.passage.kind=='door' or
                self.cfg.nav_corridor_end_behavior=='continue_straight'):
            p=self.passage
            behind=min((x-p.origin[0])*math.cos(p.heading)+(y-p.origin[1])*math.sin(p.heading)
                       for x,y in transform(self.cfg.footprint,state.pose))>p.end+self.cfg.padding
            if behind:self.passage=None;observed=None
        if observed is not None:self.passage=observed
        # Topology remains locked for an active maneuver; the same tracker and
        # costmap costs apply in both contexts. Hysteresis avoids UI/control churn.
        gap=clearance(transform(self.cfg.footprint,state.pose),obstacles,self.boundary,0.)
        threshold=self.cfg.context_assist_leave if self.context=='ASSIST' else self.cfg.context_assist_enter
        wanted='ASSIST' if self.passage is not None or self.entry is not None or gap<threshold else 'ROAM'
        if wanted!=self.context_candidate:self.context_candidate=wanted;self.context_since=now
        elif wanted!=self.context and now-self.context_since>=self.cfg.context_stable_time:self.context=wanted

    def cruise_goal(self,start,state,obstacles):
        c=self.cfg;h=self.reference[2]
        target=(start[0]+c.path_lookahead*math.cos(h),start[1]+c.path_lookahead*math.sin(h),h)
        self.end_of_corridor=False
        if self.passage is not None:
            p=self.passage
            axis=coordinates(*p.origin,p.heading)[1]+p.center
            current=coordinates(start[0],start[1],p.heading)[0]
            end=coordinates(*p.origin,p.heading)[0]+p.end
            limit=(end-c.radius-c.padding if p.kind!='door' and
                   c.nav_corridor_end_behavior=='stop' else math.inf)
            forward=min(current+c.path_lookahead,limit)
            if forward-current<.12:
                self.end_of_corridor=True;return None
            target=(forward*math.cos(p.heading)-axis*math.sin(p.heading),
                    forward*math.sin(p.heading)+axis*math.cos(p.heading),p.heading)
        length=math.dist(start[:2],target[:2])
        heading=target[2]
        # Place cruise endpoints where a full-speed stop still fits. A wall at
        # the nominal lookahead must not prevent all earlier safe progress.
        for remaining in [length-i*.25 for i in range(math.ceil(length/.25))]:
            if remaining<.12:continue
            ratio=remaining/length
            candidate=(start[0]+(target[0]-start[0])*ratio,start[1]+(target[1]-start[1])*ratio,heading)
            future=State(x=candidate[0],y=candidate[1],yaw=heading,v=c.forward_speed,
                         current_action=FORWARD,observation_age=state.observation_age)
            if self.owner.predictor.check(future,STOP,obstacles,self.boundary,diagnostics=False).safe:return candidate
        self.end_of_corridor=True
        return None

    def cruise(self,state,obstacles,boundary,now,stopped):
        if self.follower is None and self.pending is None:
            goal=self.cruise_goal(state.pose,state,obstacles)
            if goal is None:return self.owner.trip('PASSAGE_END_OR_BRANCH')
            self.request(state.pose,[goal],'CRUISE',self.cfg.corridor_lateral_limit if self.passage else 0.)
        failure=self.poll(state,now)
        if failure:return failure
        if self.follower is None:return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        if self.follower.blocked:
            if not stopped:return Decision(STOP,'ASSISTING','PATH_WAIT_REPLAN_STOP')
            if self.pending is None:
                goal=self.follower.last_goal;self.follower=None
                self.request(state.pose,[goal],'CRUISE',self.cfg.corridor_lateral_limit if self.passage else 0.)
            return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        if self.pending is None and now>=self.next_goal_check and self.follower.remaining(state)<self.cfg.path_replan_distance:
            self.next_goal_check=now+self.cfg.entry_scan_period
            start=self.follower.last_goal
            goal=self.cruise_goal(start,state,obstacles)
            if goal is not None:self.request(start,[goal],'APPEND',self.cfg.corridor_lateral_limit if self.passage else 0.)
        d=self.follower.tick(state,obstacles,boundary,now,stopped)
        if self.adjustment.active or self.owner.latch:return d
        if self.follower.done:
            if self.end_of_corridor:return self.owner.trip('PASSAGE_END_OR_BRANCH')
            self.follower=None
        if d.action==FORWARD:d.reason='NAV_SEEK_ENTRY' if self.side else 'NAV_SEARCH_EXPIRED' if self.search_expired else 'NAV_CRUISE'
        return d

    def start_entry_search(self,state,obstacles,now):
        c=self.cfg
        if not self.candidates and now-self.last_scan>=c.entry_scan_period:
            self.last_scan=now
            self.candidates=openings(state,self.semantic(obstacles),self.passage,self.side,c)
        while self.candidates:
            entry=self.candidates.pop(0)
            stop=self.owner.predictor.check(state,STOP,obstacles,self.boundary)
            if coordinates(stop.endpoint.x,stop.endpoint.y,entry.heading)[0]>(entry.start+entry.end)/2+c.nav_position_tolerance:continue
            # Pure unit tests have no Nav2 process. Real execution always uses
            # the costmap service's bounded trajectory search for reachability.
            if self.provider is None and not route_clear(state,entry,obstacles,self.boundary,c):continue
            self.candidate_entry=entry
            self.cancel();self.follower=None
            exit_goal=(entry.exit[0]+.5*math.cos(entry.exit[2]),entry.exit[1]+.5*math.sin(entry.exit[2]),entry.exit[2])
            self.request(state.pose,[entry.approach,exit_goal],'ENTRY')
            return

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
        # Pure turn tasks use collision validation, not corridor/entry goals.
        if not (self.turn_only or self.uturn):self.update_context(state,obstacles,now)
        context_done=time.monotonic()
        d=(self.adjustment.tick(state,obstacles,boundary,now,stopped) if self.adjustment.active
           else self.step(state,obstacles,boundary,now,stopped))
        self.owner.last_diagnostics['navigation_timing_ms']={
            'context':(context_done-begin)*1000,'step':(time.monotonic()-context_done)*1000}
        if self.provider:self.owner.last_diagnostics['path_search']=self.provider.last_report
        self.owner.last_diagnostics['navigation']=dict(phase=('TURN_ADJUST_'+self.adjustment.phase if self.adjustment.active else self.phase),preference=self.side,
            entry_id=self.entry.identity if self.entry else '',context=self.context,
            interpretation='uturn' if self.uturn else 'entry' if self.entry else 'open_heading' if self.open_task else 'cruise',
            remaining_time=max(0.,(self.cfg.uturn_timeout if self.uturn else self.cfg.maneuver_timeout)-(now-self.locked_at)) if self.locked_at is not None
                else max(0.,self.cfg.intent_search_timeout-(now-self.started)) if self.side else 0.,
            remaining_distance=max(0.,self.cfg.intent_search_distance-self.distance) if self.phase=='SEEK_ENTRY' else 0.,
            goal=self.goal,reference=self.reference,backend=getattr(self.provider,'backend','nav2_dwb') if self.provider else 'unit_direct')
        return d

    def step(self,state,obstacles,boundary,now,stopped):
        c=self.cfg
        if self.phase=='COMPLETED':return Decision(STOP,'PASS','NAV_TURN_COMPLETED')
        if self.phase=='INITIALIZE':
            if self.uturn:
                self.phase='TURN_AROUND';self.locked_at=now
                self.request(state.pose,[(state.x,state.y,state.yaw+math.pi)],'UTURN')
            elif self.turn_only:
                self.phase='OPEN_HEADING';self.open_task=True;self.locked_at=now
                self.request(state.pose,[(state.x,state.y,state.yaw+self.side*c.open_area_heading_bias)],'HEADING')
            elif not self.side:self.phase='CRUISE'
            elif self.passage is None:
                self.phase='OPEN_HEADING';self.open_task=True;self.locked_at=now
                self.request(state.pose,[(state.x,state.y,self.reference[2]+self.side*c.open_area_heading_bias)],'HEADING')
            else:self.phase='SEEK_ENTRY'
        if self.locked_at is not None:
            timeout=c.uturn_timeout if self.uturn else c.maneuver_timeout
            if now-self.locked_at>=timeout:return self.owner.trip('NAV_MANEUVER_TIMEOUT')
            movement=math.dist(state.pose[:2],self.progress_pose[:2])+abs(wrap(state.yaw-self.progress_pose[2]))*c.radius
            if movement>=.015:self.progress_at,self.progress_pose=now,state.pose
            if now-self.progress_at>c.progress_timeout+c.stop_dwell+c.startup_delay and self.pending is None:
                return self.owner.trip('NAV_NO_PROGRESS')
        if self.phase=='SEEK_ENTRY':
            if now-self.started>=c.intent_search_timeout or self.distance>=c.intent_search_distance:
                self.cancel();self.candidate_entry=None;self.side=0;self.phase='CRUISE';self.search_expired=True
            elif self.pending_kind=='ENTRY' and self.pending is not None:
                failure=self.poll(state,now)
                if failure:return failure
                if self.pending is not None:return Decision(STOP,'ASSISTING','PATH_CHECK_ENTRY')
            if self.phase=='SEEK_ENTRY' and self.pending is None:
                self.start_entry_search(state,obstacles,now)
                if self.pending is not None:
                    failure=self.poll(state,now)
                    if failure:return failure
                    if self.pending is not None:return Decision(STOP,'ASSISTING','PATH_CHECK_ENTRY')
            if self.phase=='SEEK_ENTRY':return self.cruise(state,obstacles,boundary,now,stopped)
        if self.phase=='CRUISE':return self.cruise(state,obstacles,boundary,now,stopped)
        failure=self.poll(state,now)
        if failure:return failure
        if self.follower is None:return Decision(STOP,'ASSISTING','PATH_WAIT_PLAN')
        if self.entry:
            if self.passed(state):
                self.finish(state)
                return self.cruise(state,obstacles,boundary,now,stopped)
            if self.follower.current and self.follower.current.kind=='TURN':self.phase='ALIGN_ENTRY'
            elif self.follower.index>0:
                _,lat=coordinates(state.x,state.y,self.entry.heading)
                self.phase='EXIT' if self.side*(lat-self.entry.outer)>0 else 'TRAVERSE'
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
            if self.entry:return self.owner.trip('NAV_ENTRY_NOT_TRAVERSED')
            if self.turn_only:
                self.phase='COMPLETED';self.locked_at=None
                return Decision(STOP,'PASS','NAV_TURN_COMPLETED')
            self.finish(state)
            return Decision(STOP,'ASSISTING','NAV_HEADING_COMPLETED')
        if d.action==FORWARD:d.reason='NAV_'+self.phase
        return d

    def passed(self,state):
        distances=[self.side*(coordinates(x,y,self.entry.heading)[1]-self.entry.outer) for x,y in transform(self.cfg.footprint,state.pose)]
        return min(distances)>self.cfg.padding+self.cfg.entry_exit_clearance and abs(wrap(state.yaw-self.entry.exit[2]))<=self.cfg.task_heading_tolerance

    def finish(self,state):
        heading=self.follower.last_goal[2] if self.follower is not None else state.yaw
        self.phase='CRUISE';self.side=0;self.uturn=False;self.open_task=False
        self.locked_at=None;self.entry=None;self.reference=(state.x,state.y,heading);self.goal=None
        self.passage=None;self.last_context=-math.inf
        if self.follower and self.follower.done:self.follower=None
