"""Conservative local straight-passage assistance on an observed polygon map.
No route selection: one fixed axis and band per uninterrupted FORWARD request.
"""
import math
from dataclasses import dataclass,replace
import shapely
import numpy as np
from shapely.geometry import LineString,Polygon
from .geometry import wrap,IndexedObstacles,polygon_bounds
from .protocol import STOP,FORWARD,LEFT,RIGHT

@dataclass
class Passage:
    origin: tuple
    heading: float
    center: float
    width: float
    entry: float
    end: float
    closed_end: bool
    kind: str
    def coordinates(self,state):
        dx,dy=state.x-self.origin[0],state.y-self.origin[1]
        c,s=math.cos(self.heading),math.sin(self.heading)
        return dx*c+dy*s,-dx*s+dy*c-self.center
    def as_dict(self):return vars(self).copy()

def rectangle_sections(obstacles, origin, heading, longitudinal, lateral):
    """Intersect axis-aligned rectangles with the same 36 scan segments.

    Unioning polygons before intersecting a line is equivalent to merging their
    clipped 1D intervals afterwards. Avoid the expensive whole-map GEOS union.
    Non-rectangular maps retain the general geometry implementation below.
    """
    bounds=[]
    for obstacle in obstacles:
        if len(obstacle)!=4:return None
        x0,x1,y0,y1=polygon_bounds(obstacle)
        if set(obstacle)!={(x0,y0),(x0,y1),(x1,y0),(x1,y1)}:return None
        bounds.append((x0,x1,y0,y1))
    if not bounds:return [[] for _ in longitudinal]
    c,s=math.cos(heading),math.sin(heading)
    # The detector supports orthogonal axes. Keep the exact general path if a
    # caller supplies a non-orthogonal locked passage.
    if min(abs(c),abs(s))>1e-12:return None
    b=np.asarray(bounds)
    xs=b[:,:2]-origin[0];ys=b[:,2:]-origin[1]
    along_lo=np.min(c*xs,axis=1)+np.min(s*ys,axis=1)
    along_hi=np.max(c*xs,axis=1)+np.max(s*ys,axis=1)
    side_lo=np.maximum(np.min(-s*xs,axis=1)+np.min(c*ys,axis=1),lateral-2.)
    side_hi=np.minimum(np.max(-s*xs,axis=1)+np.max(c*ys,axis=1),lateral+2.)
    result=[]
    for at in longitudinal:
        mask=(along_lo<=at)&(at<=along_hi)&(side_lo<=side_hi)
        merged=[]
        for lo,hi in sorted(zip(side_lo[mask],side_hi[mask])):
            if merged and lo<=merged[-1][1]:
                merged[-1]=(merged[-1][0],max(hi,merged[-1][1]))
            else:merged.append((float(lo),float(hi)))
        result.append(merged)
    return result


def detect(state,obstacles,cfg,locked=None,allow_side_openings=True):
    """First release: axis-aligned straight wall pairs in the known static map.
    Unsupported directions, openings and nonparallel pairs are refused explicitly.
    Finite lookahead is a confidence boundary, not an assumed free destination.
    """
    heading=locked.heading if locked else round(state.yaw/(math.pi/2))*(math.pi/2)
    if abs(wrap(heading-state.yaw))>cfg.corridor_heading_limit:return None,'PASSAGE_HEADING_OUTSIDE_AUTHORITY'
    origin=locked.origin if locked else (state.x,state.y)
    c,s=math.cos(heading),math.sin(heading)
    at=(state.x-origin[0])*c+(state.y-origin[1])*s
    lateral=-(state.x-origin[0])*s+(state.y-origin[1])*c
    longitudinal_samples=[at+i*.1 for i in range(-5,31)]
    rectangular=rectangle_sections(obstacles,origin,heading,longitudinal_samples,lateral)
    if rectangular is None:
        shapes=[getattr(o,'native',None) or Polygon(o) for o in obstacles]
        union=shapely.union_all(shapes) if shapes else Polygon()
    sections=[];cross_sections=[]
    # Scan from behind the front axle to the complete lookahead limit.
    for index,longitudinal in enumerate(longitudinal_samples):
        if rectangular is not None:
            intervals=rectangular[index]
        else:
            x=origin[0]+c*longitudinal-s*lateral;y=origin[1]+s*longitudinal+c*lateral
            hit=union.intersection(LineString([(x+s*2,y-c*2),(x-s*2,y+c*2)]))
            intervals=[]
            geoms=list(hit.geoms) if hasattr(hit,'geoms') else [hit]
            for g in geoms:
                if g.is_empty:continue
                if not hasattr(g,'coords'):continue
                values=[-(px-origin[0])*s+(py-origin[1])*c for px,py in g.coords]
                if values:intervals.append((min(values),max(values)))
        left=min((a for a,b in intervals if a>lateral+1e-6),default=None)
        right=max((b for a,b in intervals if b<lateral-1e-6),default=None)
        occupied=any(a-1e-6<=lateral<=b+1e-6 for a,b in intervals)
        cross_sections.append((longitudinal,left,right,occupied))
        if occupied:sections.append(None);continue
        if left is None or right is None or not .55<left-right<cfg.corridor_max_width:sections.append(None);continue
        jambs=(any(a<=left+1e-6 and b>=lateral+1.5 for a,b in intervals)
               and any(b>=right-1e-6 and a<=lateral-1.5 for a,b in intervals))
        sections.append((longitudinal,(left+right)/2,left-right,jambs))
    runs=[];run=[]
    for section in sections+[None]:
        if section is None or (run and (abs(section[1]-run[-1][1])>.025 or abs(section[2]-run[-1][2])>.05)):
            if len(run)>=2:runs.append(run)
            run=[]
        if section is not None:run.append(section)
    # A short unilateral doorway does not end a straight corridor. Bridge only
    # when the opposite wall stays continuous AND the original pair returns.
    # Both-side openings, corners, blocked centerlines and long gaps stay breaks.
    joined=[]
    for run in runs:
        if joined and allow_side_openings:
            before=joined[-1];center0,width0=before[-1][1:3]
            gap=run[0][0]-before[-1][0]
            middle=[r for r in cross_sections if before[-1][0]<r[0]<run[0][0]]
            if middle and gap<=cfg.corridor_side_opening_max+1e-9 and abs(run[0][1]-center0)<.025 and abs(run[0][2]-width0)<.05:
                upper,lower=center0+width0/2,center0-width0/2
                left_kept=all(not blocked and left is not None and abs(left-upper)<.025
                              and (right is None or right<=lower+.025) for _,left,right,blocked in middle)
                right_kept=all(not blocked and right is not None and abs(right-lower)<.025
                               and (left is None or left>=upper-.025) for _,left,right,blocked in middle)
                if left_kept or right_kept:
                    before.extend((x,center0,width0,False) for x,_,_,_ in middle)
                    before.extend(run);continue
        joined.append(run)
    runs=joined
    runs=[r for r in runs if r[-1][0]>=at-.1 and r[0][0]<=at+cfg.passage_entry_distance]
    if locked:
        runs=[r for r in runs if abs(r[0][1]-locked.center)<.025 and abs(r[0][2]-locked.width)<.05]
        if not allow_side_openings:
            # A later wall pair on the same axis cannot erase an intervening
            # branch from a confirmed task's boundary.
            runs=[r for r in runs if r[0][0]<=locked.end+.05 and r[-1][0]>=locked.entry-.05]
    if not runs:return None,'PASSAGE_UNCERTAIN_OR_BRANCH'
    run=min(runs,key=lambda r:abs(max(at,r[0][0])-at))
    center=run[0][1];width=min(v[2] for v in run)
    entry=run[0][0];end=run[-1][0];closed=end<at+2.95
    if abs(center-lateral)>cfg.corridor_lateral_limit:return None,'PASSAGE_LATERAL_OUTSIDE_AUTHORITY'
    kind=locked.kind if locked else ('door' if closed and end-entry<1.0 and all(v[3] for v in run) else 'corridor')
    if locked:
        entry=locked.entry;center=locked.center;width=min(width,locked.width)
    return Passage(origin,heading,center,width,entry,end,closed,kind),''

class PassageController:
    def __init__(self,owner,task_scoped=False):
        self.owner=owner;self.cfg=owner.cfg;self.passage=None
        self.task_scoped=task_scoped;self.discovery_done=False
        self.phase='APPROACH';self.side=None;self.started=None;self.motion_at=None
        self.duration=0.;self.target=None;self.turn_origin=None;self.last_turn=None;self.request_heading=None
        self.last_detect=-math.inf;self.refresh_needed=False;self.corrections=0;self.stagnant=0;self.defer_until=None;self.turn_blocked_reason=None
        self.alignment_unavailable_at=None
    def fail(self,reason):return self.owner.trip(reason)
    def tick(self,state,obstacles,boundary,now,stopped):
        from .planner import Decision
        c=self.cfg;p=self.owner.predictor
        if self.request_heading is None:self.request_heading=state.yaw
        if abs(wrap(state.yaw-self.request_heading))>c.corridor_heading_limit:
            return self.fail('PASSAGE_AUTHORITY_LIMIT')
        # The tracked axis/center is never refreshed from the changing vehicle yaw.
        # Initial discovery runs alongside direct driving in open space. Extending
        # an existing confidence interval still waits stopped; its original end
        # remains a bound until then. Collision checks run on every motion cycle.
        if (self.passage is None and not (self.task_scoped and self.discovery_done)) or self.refresh_needed:
            # No passage is a normal open-room state, not a request to stop.
            # Previously every successful direct FORWARD caused the next moving
            # tick to STOP here, making the chair crawl indefinitely. Continue
            # observing while moving and validate FORWARD against the fresh map.
            # Only extending an already tracked confidence interval waits stopped.
            if self.refresh_needed and not stopped:
                return Decision(STOP,'ASSISTING','PASSAGE_WAIT_OBSERVATION_STOP')
            observed,reason=detect(state,obstacles,c,self.passage,allow_side_openings=not self.task_scoped)
            self.discovery_done=True
            self.last_detect=now;self.refresh_needed=False
            if observed is not None and self.passage is None and self.owner.manual_heading and not self.task_scoped:
                old_axis=self.owner.manual_departure_axis
                same_old_axis=old_axis is not None and abs(math.sin(observed.heading-old_axis))<.1
                not_aligned=old_axis is None and abs(wrap(observed.heading-state.yaw))>math.radians(5)
                if same_old_axis or not_aligned:
                    observed=None;reason='USER_HEADING_PRIORITY'
            if observed is None:
                if self.passage is None:
                    # Outside an identified passage, preserve direct joystick
                    # semantics. Never invent a corridor or select a turn.
                    direct=p.check(state,FORWARD,obstacles,boundary)
                    self.owner.last_diagnostics.update(passage_detection=reason,candidate=direct.summary())
                    return Decision(FORWARD,'PASS','USER_SELECTED_HEADING' if reason=='USER_HEADING_PRIORITY' else 'USER_FORWARD_NO_PASSAGE') if direct.safe else self.fail('PASSAGE_UNCERTAIN_BLOCKED')
                return self.fail(reason)
            self.passage=observed
            # Keep map extraction and candidate search in separate stopped cycles.
            # Never extend the 50ms control deadline to hide planning work.
            return Decision(STOP,'ASSISTING','PASSAGE_OBSERVED')
        if self.passage is None:
            direct=p.check(state,FORWARD,obstacles,boundary)
            self.owner.last_diagnostics.update(passage_detection='TASK_OPEN_SCOPE_LOCKED',candidate=direct.summary())
            return Decision(FORWARD,'PASS','TASK_CONTINUE_OPEN') if direct.safe else self.fail('TASK_CONTINUE_BLOCKED')
        passage=self.passage;along,lateral=passage.coordinates(state)
        heading_error=wrap(state.yaw-passage.heading)
        diag=self.owner.last_diagnostics
        diag.update(passage=passage.as_dict(),passage_phase=self.phase,
                    lateral_error=lateral,heading_error=heading_error,corrections=self.corrections,
                    requested_heading=self.request_heading,manual_departure_axis=self.owner.manual_departure_axis)
        if passage.width<=2*(max(abs(y) for x,y in c.footprint)+c.padding):
            return self.fail('PASSAGE_TOO_NARROW')
        if abs(lateral)>c.corridor_lateral_limit:return self.fail('PASSAGE_LATERAL_LIMIT')
        # Never select a branch. Door exit is a completed local task; a corridor
        # end/opening requires a new human choice, even if the floor is free.
        if along>passage.end+(c.radius if passage.kind=='door' else 0):
            self.phase='EXIT';diag['passage_phase']='EXIT'
            return self.fail('PASSAGE_EXIT_CONFIRM_DIRECTION')
        if self.phase=='TURN':
            if now-self.started>c.assist_duration:return self.fail('PASSAGE_CORRECTION_TIMEOUT')
            result=p.check(state,self.side,obstacles,boundary,duration=0.)
            diag['candidate']=result.summary()
            running=(state.execution_phase==1 and state.current_action==self.side)
            if state.execution_phase is None:running=abs(state.w)>.02 # standalone model tests
            # Time from executor RUNNING, the same event used in calibration.
            # Waiting for a later odometry velocity threshold lengthened short
            # commands and caused overshoot/left-right oscillation.
            if self.motion_at is None and running:self.motion_at=now
            elapsed=0 if self.motion_at is None else now-self.motion_at
            remaining=wrap(self.target-state.yaw)
            target_reached=(self.side==LEFT and remaining<=0) or (self.side==RIGHT and remaining>=0)
            turned=abs(wrap(state.yaw-self.turn_origin))
            brake_angle=abs(state.w)*(c.transport_delay+c.period)+state.w*state.w/(2*c.angular_brake)
            future_authorized=all(abs(wrap(pose[2]-self.request_heading))<=c.corridor_heading_limit for pose in result.poses)
            if not result.safe or not future_authorized or turned+brake_angle>=c.corridor_correction_angle or elapsed>=self.duration or target_reached:
                self.turn_blocked_reason='DIRECTION_AUTHORITY' if not future_authorized else 'GEOMETRY' if not result.safe else None
                self.phase='SETTLE';return Decision(STOP,'ASSISTING','PASSAGE_BRAKE_TURN')
            return Decision(self.side,'ASSISTING','PASSAGE_MICRO_TURN')
        if self.phase=='SETTLE':
            if now-self.started>c.assist_duration:return self.fail('PASSAGE_CORRECTION_TIMEOUT')
            if not stopped:return Decision(STOP,'ASSISTING','PASSAGE_WAIT_STOPPED')
            actual_turn=abs(wrap(state.yaw-self.turn_origin))
            self.stagnant=self.stagnant+1 if actual_turn<c.corridor_min_improvement else 0
            diag['last_actual_turn_rad']=actual_turn
            if self.stagnant>=2:
                direct=p.check(state,FORWARD,obstacles,boundary)
                if not direct.safe:return self.fail('PASSAGE_ACTION_UNEXECUTABLE')
                # A refused micro-turn must not lock an independently safe user
                # FORWARD request. Do not repeatedly retry the same zero-motion
                # action: defer it until meaningful progress, or until a new
                # authorization if its directional authority is exhausted.
                self.defer_until=math.inf if self.turn_blocked_reason=='DIRECTION_AUTHORITY' else along+.30
                self.stagnant=0
            self.phase='TRAVERSE'
            if actual_turn>=c.corridor_min_improvement:self.corrections+=1
        # Desired heading returns to the passage axis as the lateral offset falls.
        desired=passage.heading+max(-c.corridor_correction_angle,min(c.corridor_correction_angle,-lateral*1.5))
        error=wrap(desired-state.yaw)
        need=abs(error)>c.corridor_heading_enter
        # A parallel, off-center chair may fit while every minimum turn fails.
        # Centering is a preference: do not keep stopping to retry an unchanged
        # infeasible maneuver. Reconsider as soon as lateral/heading drift or a
        # newly blocked forward envelope makes the previous fallback obsolete.
        if self.alignment_unavailable_at is not None:
            old_lateral,old_yaw=self.alignment_unavailable_at
            unchanged=(abs(lateral-old_lateral)<.005 and
                       abs(wrap(state.yaw-old_yaw))<c.corridor_min_improvement)
            if unchanged and p.check(state,FORWARD,obstacles,boundary).safe:
                need=False;diag['alignment_deferred']='NO_EXECUTABLE_CORRECTION'
            else:self.alignment_unavailable_at=None
        # Heading hysteresis cannot postpone a correction until the forward
        # stopping envelope is already blocked. A small error is meaningful when
        # it predicts a hard-clearance loss, even before the nominal threshold.
        if not need and abs(error)>c.corridor_min_improvement:
            need=not p.check(state,FORWARD,obstacles,boundary).safe
        if self.defer_until is not None and along<self.defer_until:
            need=False;diag['alignment_deferred']=self.turn_blocked_reason or 'MINIMUM_ACTION_UNEXECUTABLE'
        if not need and abs(lateral)<c.corridor_center_tolerance:self.last_turn=None
        if need:
            if not stopped:return Decision(STOP,'ASSISTING','PASSAGE_STOP_TO_ALIGN')
            side=LEFT if error>0 else RIGHT
            # Hysteresis prevents a tiny error from flipping the previous choice.
            if self.last_turn not in (None,side) and abs(error)<c.corridor_heading_enter*1.5:
                need=False
            else:
                candidates=[]
                for duration in (c.micro_turn_min,c.micro_turn_min*2,c.micro_turn_min*3):
                    # At zero angular momentum this is the exact angular part of
                    # Predictor's hold/lease/transit/brake envelope. Reject an
                    # over-budget duration before expensive map queries; accepted
                    # candidates still receive the complete swept safety check.
                    if abs(state.w)<1e-6 and state.current_action in (STOP,FORWARD):
                        total=duration+c.action_ttl;speed=abs(c.velocity(side)[1])
                        ramp=min(total,speed/c.angular_accel);peak=min(speed,c.angular_accel*total)
                        envelope=.5*c.angular_accel*ramp*ramp+speed*max(0.,total-ramp)+peak*(c.transport_delay+c.period)+peak*peak/(2*c.angular_brake)
                        if envelope>c.corridor_correction_angle:continue
                    result=p.check(state,side,obstacles,boundary,duration=duration)
                    if not result.safe:continue
                    turn=abs(wrap(result.endpoint.yaw-state.yaw))
                    # Use nominal actively-stopped motion for progress scoring;
                    # the full lease-loss envelope above remains the HARD check.
                    # Treating that worst-case endpoint as the desired endpoint
                    # wrongly rejects useful sub-degree corrections.
                    velocity=min(abs(c.velocity(side)[1]),c.angular_accel*duration)
                    nominal=.5*velocity*duration+velocity*(c.transport_delay+c.period)+velocity*velocity/(2*c.angular_brake)
                    nominal_yaw=state.yaw+(nominal if side==LEFT else -nominal)
                    improvement=abs(error)-abs(wrap(desired-nominal_yaw))
                    if turn>c.corridor_correction_angle or improvement<c.corridor_min_improvement:continue
                    if any(abs(wrap(pose[2]-self.request_heading))>c.corridor_heading_limit for pose in result.poses):continue
                    # Hard-stop-safe turn first; roll forward and back toward the
                    # axis only as a scored candidate. Replan after each stopped turn.
                    nominal_stop=replace(result.endpoint,yaw=nominal_yaw)
                    forward=p.check(nominal_stop,FORWARD,obstacles,boundary,duration=.30)
                    if not forward.safe:continue
                    _,end_lateral=passage.coordinates(forward.endpoint)
                    # Also retain a feasible return-to-parallel option after the
                    # lateral transfer; only the first turn is ever authorized.
                    back_side=RIGHT if wrap(forward.endpoint.yaw-passage.heading)>0 else LEFT
                    if abs(lateral)>c.corridor_center_tolerance:
                        back=p.check(forward.endpoint,back_side,obstacles,boundary,duration=c.micro_turn_min)
                        if not back.safe:continue
                    score=improvement-abs(end_lateral)*.5+.1*min(forward.clearance,c.comfort_distance)-.02
                    candidates.append((score,duration,result,nominal_yaw))
                if not candidates:
                    direct=p.check(state,FORWARD,obstacles,boundary)
                    diag['candidate']=direct.summary()
                    if not direct.safe:return self.fail('PASSAGE_NO_EXECUTABLE_CORRECTION')
                    self.alignment_unavailable_at=(lateral,state.yaw)
                    diag['alignment_deferred']='NO_EXECUTABLE_CORRECTION'
                    # Fall through ALL passage checks, including the complete
                    # door traverse, finite confidence boundary and authority.
                else:
                    _,self.duration,result,nominal_yaw=max(candidates,key=lambda r:r[0])
                    self.side=side;self.started=now;self.motion_at=None
                    self.target=nominal_yaw;self.turn_origin=state.yaw;self.last_turn=side;self.phase='TURN'
                    diag['candidate']=result.summary();diag['planned_turn_duration']=self.duration;diag['nominal_target_yaw']=nominal_yaw
                    return Decision(STOP,'ASSISTING','PASSAGE_ALIGN')
        self.phase='APPROACH' if passage.kind=='door' and along+c.radius<passage.entry else 'TRAVERSE'
        diag['passage_phase']=self.phase
        duration=None
        if passage.kind=='door':
            # Validate the entire remaining doorway traverse plus braking before
            # entering; this is a local corridor check, not a queued motion task.
            duration=max(c.minimum_action,(passage.end+c.radius-along)/c.forward_speed)
        result=p.check(state,FORWARD,obstacles,boundary,duration=duration)
        diag['candidate']=result.summary()
        # A finite corridor confidence interval must cover the stopping endpoint.
        stop_along,_=passage.coordinates(result.endpoint)
        if passage.kind!='door' and stop_along>passage.end:
            # Re-observe further ahead without changing the selected axis/center.
            # This distinguishes a side door whose far jamb was outside the old
            # lookahead from a real ending. No turning branch is selected here.
            observed,reason=detect(state,obstacles,c,passage,allow_side_openings=not self.task_scoped)
            if observed is not None and observed.end>passage.end+.05:
                self.passage=passage=observed
                stop_along,_=passage.coordinates(result.endpoint)
                diag['passage']=passage.as_dict();diag['observation_extended']=True
            if stop_along>passage.end:return self.fail('PASSAGE_END_OR_BRANCH')
        if not result.safe:return self.fail('PASSAGE_BLOCKED_'+result.reason)
        if any(abs(wrap(yaw-self.request_heading))>c.corridor_heading_limit for x,y,yaw in result.poses):
            return self.fail('PASSAGE_PREDICTED_DIRECTION_LIMIT')
        if any(abs(-(x-passage.origin[0])*math.sin(passage.heading)+(y-passage.origin[1])*math.cos(passage.heading)-passage.center)>c.corridor_lateral_limit for x,y,yaw in result.poses):
            return self.fail('PASSAGE_PREDICTED_LATERAL_LIMIT')
        return Decision(FORWARD,'ASSISTING','PASSAGE_TRAVERSE')
