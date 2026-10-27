"""One feedback controller for executable straight/rotate paths, including U-turns.

Path samples describe poses, never continuous commands to quantize. Consecutive
collinear translations and same-direction rotations are merged before execution.
"""
from dataclasses import dataclass, replace
from types import SimpleNamespace
import math
from .geometry import wrap
from .planner import Decision
from .protocol import Packet, STOP, FORWARD, LEFT, RIGHT
from .task_scope import TaskScope, ADJUST_HEADING


@dataclass
class Segment:
    start: tuple
    end: tuple
    kind: str
    angle: float = 0.


def segments(poses):
    if not poses or any(len(p)!=3 or not all(math.isfinite(x) for x in p) for p in poses):
        raise ValueError('Invalid path poses')
    result=[]
    for a,b in zip(poses,poses[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        distance=math.hypot(dx,dy);angle=wrap(b[2]-a[2])
        if distance<1e-5 and abs(angle)<1e-5:continue
        if distance<1e-5:
            part=Segment(tuple(a),tuple(b),'TURN',angle)
        else:
            if abs(angle)>.005 or abs(wrap(math.atan2(dy,dx)-a[2]))>.005:
                raise ValueError('Path requires unsupported arc, reverse or lateral motion')
            part=Segment(tuple(a),tuple(b),'DRIVE')
        if (result and part.kind==result[-1].kind and
                ((part.kind=='TURN' and part.angle*result[-1].angle>0) or
                 (part.kind=='DRIVE' and abs(wrap(part.start[2]-result[-1].end[2]))<.001))):
            result[-1].end=part.end;result[-1].angle+=part.angle
        else:result.append(part)
    return result


def direct_path(start, goals):
    """Pure-test provider; ROS uses the Nav2 service, never this fallback."""
    path=[tuple(start)]
    for goal in goals:
        x,y,yaw=path[-1];dx,dy=goal[0]-x,goal[1]-y
        if math.hypot(dx,dy)>1e-5:
            h=math.atan2(dy,dx)
            delta=wrap(h-yaw)
            for i in range(1,max(1,math.ceil(abs(delta)/.1))+1):
                n=max(1,math.ceil(abs(delta)/.1));path.append((x,y,yaw+delta*i/n))
            path.append((goal[0],goal[1],h));x,y,yaw=path[-1]
        delta=wrap(goal[2]-yaw);n=max(1,math.ceil(abs(delta)/.1))
        for i in range(1,n+1):path.append((x,y,yaw+delta*i/n))
    return path


class PathTurn(TaskScope):
    def __init__(self,owner,state,angle,task_id):
        self.direction=1 if angle>0 else -1
        self.total=abs(angle)
        self.progress=0.
        self.last_yaw=state.yaw
        cfg=replace(owner.cfg,task_heading_limit=self.total+owner.cfg.nav_turn_envelope,
                    assist_duration=owner.cfg.maneuver_timeout)
        proxy=SimpleNamespace(cfg=cfg,predictor=owner.predictor,last_diagnostics={},trip=owner.trip)
        packet=Packet('path',0,0.,.15,LEFT if angle>0 else RIGHT,True,task_id=task_id,
                      task_kind=ADJUST_HEADING,reference_x=state.x,reference_y=state.y,
                      reference_yaw=state.yaw,target_yaw=state.yaw+angle)
        super().__init__(proxy,packet)

    def heading_remaining(self,state,direction):return self.total-self.progress
    def brake_threshold(self,remaining):return remaining+self.owner.cfg.task_heading_tolerance
    def pulse_durations(self,remaining):
        # Near the endpoint, avoid spending a real-time cycle checking a large
        # episode before both useful small ones. Every retained candidate still
        # receives the braking-distance collision check.
        return (.20,.10) if remaining<math.radians(5) else (.30,.20,.10)
    def within_heading_scope(self,yaw):
        predicted=self.progress+self.direction*wrap(yaw-self.last_yaw)
        return -self.owner.cfg.task_heading_tolerance<=predicted<=self.total+self.owner.cfg.nav_turn_envelope

    def step(self,state,obstacles,boundary,now,stopped):
        self.progress+=self.direction*wrap(state.yaw-self.last_yaw)
        self.last_yaw=state.yaw
        self.owner.last_diagnostics={}
        if not self.within_heading_scope(state.yaw):return self.owner.trip('PATH_TURN_SCOPE_LIMIT')
        if self.phase=='COMPLETED':return Decision(STOP,'PASS','TASK_HEADING_COMPLETED')
        return self.heading_tick(state,obstacles,boundary,now,stopped,self.direction)


class PathFollower:
    def __init__(self,owner,poses,task_id):
        self.owner=owner;self.cfg=owner.cfg;self.task_id=task_id
        self.parts=segments(poses);self.index=0;self.turn=None
        self.braking=False;self.done=not self.parts;self.blocked=False
        self.correction=False;self.last_goal=tuple(poses[-1]);self.last_action=STOP

    @property
    def current(self):return self.parts[self.index] if self.index<len(self.parts) else None

    def append(self,poses):
        if math.dist(self.last_goal[:2],poses[0][:2])>.005 or abs(wrap(self.last_goal[2]-poses[0][2]))>.005:
            raise ValueError('Discontinuous appended path')
        added=segments(poses)
        for part in added:
            if self.parts and self.parts[-1].kind==part.kind=='DRIVE' and abs(wrap(self.parts[-1].end[2]-part.start[2]))<.001:
                self.parts[-1].end=part.end
            else:self.parts.append(part)
        self.last_goal=tuple(poses[-1])

    def remaining(self,state):
        part=self.current
        if part is None:return 0.
        return math.dist(state.pose[:2],part.end[:2])+sum(math.dist(p.start[:2],p.end[:2]) for p in self.parts[self.index+1:])

    def advance(self):
        self.index+=1;self.turn=None;self.braking=False
        self.done=self.index>=len(self.parts)

    def tick(self,state,obstacles,boundary,now,stopped):
        self.owner.last_diagnostics['path_tracking']=dict(segment=self.index,segments=len(self.parts),
            kind=self.current.kind if self.current else 'DONE',remaining=self.remaining(state))
        part=self.current;c=self.cfg
        if self.turn is not None:
            # heading_tick checks the identical stopping envelope before every
            # turn decision. Avoid evaluating that rollout twice per cycle.
            d=self.turn.step(state,obstacles,boundary,now,stopped)
            self.owner.last_diagnostics.update(self.turn.owner.last_diagnostics)
            if d.reason=='TASK_HEADING_COMPLETED':
                self.turn=None
                if self.correction:self.correction=False
                else:self.advance()
                return Decision(STOP,'ASSISTING','PATH_ALIGNED')
            return d
        stop=self.owner.predictor.check(state,STOP,obstacles,boundary)
        if not stop.safe:return self.owner.trip('PATH_STOPPING_ENVELOPE_RISK')
        if self.done:return Decision(STOP,'PASS','PATH_COMPLETED')
        if part.kind=='TURN':
            if not stopped:return Decision(STOP,'ASSISTING','PATH_STOP_BEFORE_TURN')
            # Preserve the planned direction at the +/-pi branch cut.
            residual=wrap(part.start[2]-state.yaw)
            self.turn=PathTurn(self.owner,state,part.angle+residual,self.task_id+'/turn')
            return Decision(STOP,'ASSISTING','PATH_PREPARE_TURN')
        h=part.start[2];dx,dy=part.end[0]-state.x,part.end[1]-state.y
        along=dx*math.cos(h)+dy*math.sin(h)
        lateral=-dx*math.sin(h)+dy*math.cos(h)
        if along < -c.nav_position_tolerance:return self.owner.trip('PATH_ENDPOINT_OVERSHOOT')
        feedback_age=state.feedback_age if state.feedback_age is not None else state.observation_age
        delay=c.transport_delay+c.period+(feedback_age or 0.)
        peak=min(c.forward_speed,max(0.,state.v)+c.linear_accel*delay)
        brake=(max(0.,state.v)+peak)*delay/2+peak*peak/(2*c.linear_brake)
        if self.braking or along<=max(c.nav_position_tolerance,brake):
            self.braking=True
            if not stopped:return Decision(STOP,'ASSISTING','PATH_BRAKE')
            if along<=c.nav_position_tolerance:
                if abs(lateral)>c.nav_position_tolerance:return self.owner.trip('PATH_ENDPOINT_LATERAL_ERROR')
                # A final straight segment can accumulate heading drift within
                # the drive deadband. Finish its requested pose before reporting
                # completion to the entry/goal predicate.
                error=wrap(part.end[2]-state.yaw)
                if self.index==len(self.parts)-1 and abs(error)>c.task_heading_tolerance:
                    self.turn=PathTurn(self.owner,state,error,self.task_id+'/final_heading')
                    self.correction=True
                    return Decision(STOP,'ASSISTING','PATH_CORRECT_FINAL_HEADING')
                self.advance()
                return Decision(STOP,'ASSISTING','PATH_WAYPOINT_REACHED')
            self.braking=False
        projected_lateral=lateral-along*math.tan(wrap(state.yaw-h))
        # Do not stop and turn to eliminate a residual already inside the
        # endpoint tolerance. Shrinking lookahead otherwise amplifies a harmless
        # offset into a late correction and another final-heading correction.
        correction=0. if abs(projected_lateral)<=c.nav_position_tolerance else math.atan2(lateral,max(.4,along))
        aim=h+max(-c.path_correction_angle,min(c.path_correction_angle,correction))
        error=wrap(aim-state.yaw)
        if abs(error)>max(c.task_heading_tolerance*1.5,c.path_heading_deadband):
            if not stopped:return Decision(STOP,'ASSISTING','PATH_STOP_TO_CORRECT')
            self.turn=PathTurn(self.owner,state,error,self.task_id+'/correction');self.correction=True
            return Decision(STOP,'ASSISTING','PATH_CORRECT')
        result=self.owner.predictor.check(state,FORWARD,obstacles,boundary)
        self.owner.last_diagnostics['candidate']=result.summary()
        if not result.safe:
            self.blocked=True
            return Decision(STOP,'ASSISTING','PATH_REPLAN_REQUIRED')
        self.blocked=False
        return Decision(FORWARD,'ASSISTING','PATH_FORWARD')
