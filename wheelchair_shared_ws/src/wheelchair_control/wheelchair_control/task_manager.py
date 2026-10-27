"""Protocol-v1 finite task authorization. Pure, explicit-clock state machine.

Input health is a lease on availability, never a motion command. Only a fresh,
unconsumed candidate/event can create a task; all terminal paths send disabled
STOP through the existing safety gate. No ROS or hardware dependencies.
"""
from dataclasses import dataclass, replace
from collections import OrderedDict
import math
import uuid

CONTINUE, ADJUST_HEADING, CANCEL = 1, 2, 3
IDLE, ACCEPTED, RUNNING, STOPPING, COMPLETED, BLOCKED, FAULT = range(7)
STOP, FORWARD, LEFT, RIGHT = range(4)

@dataclass(frozen=True)
class Candidate:
    session: str
    sequence: int
    stamp: float
    ttl: float
    candidate_id: str
    version: int
    kind: int
    relative_yaw: float
    stable: bool
    confirmable: bool
    context_id: str
    context_version: int
    protocol_version: int = 1

@dataclass(frozen=True)
class Event:
    session: str
    event_id: int
    stamp: float
    ttl: float
    kind: int
    candidate_id: str = ''
    candidate_version: int = 0
    context_id: str = ''
    context_version: int = 0
    relative_yaw: float = 0.
    protocol_version: int = 1

@dataclass(frozen=True)
class Health:
    session: str
    sequence: int
    stamp: float
    ttl: float
    valid: bool
    source_age: float
    sample_sequence: int = 0
    protocol_version: int = 1

@dataclass(frozen=True)
class Ack:
    session: str
    event_id: int
    accepted: bool
    task_id: str
    reason: str

@dataclass
class Task:
    task_id: str
    event: Event
    started: float
    reference: tuple
    target_yaw: float
    distance: float = 0.
    motion_started: bool = False
    finished: float = None
    turn_only: bool = False

@dataclass(frozen=True)
class Output:
    command: int = STOP
    enabled: bool = False
    task_id: str = ''
    task_kind: int = 0
    reference: tuple = (0., 0., 0.)
    target_yaw: float = 0.
    reason: str = 'IDLE'

@dataclass
class TaskConfig:
    max_time: float = 10.
    max_distance: float = 3.
    max_event_ttl: float = 2.
    source_timeout: float = .25
    status_timeout: float = .35
    odom_timeout: float = .15
    max_adjust_angle: float = math.radians(25)
    linear_brake: float = .5
    angular_brake: float = .7
    stop_delay: float = .25
    budget_stop_delay: float = .50
    stop_dwell: float = .15
    period: float = .05
    forward_speed: float = .372
    angular_speed: float = .403

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('task limits must be finite and positive')

class TaskManager:
    def __init__(self, cfg=None, session=None):
        self.cfg=cfg or TaskConfig();self.session=session or str(uuid.uuid4())
        self.context_version=1;self.source_session=None;self.retired=set()
        self.health_sequence=-1;self.health_deadline=-math.inf;self.source_valid=False
        self.sample_sequence=-1;self.candidate=None;self.candidate_deadline=-math.inf
        self.sample_at=-math.inf;self.sample_age=math.inf
        self.candidate_sequence=-1;self.high_event={};self.events=OrderedDict();self.acks=[]
        self.consumed=set();self.pending=None;self.active=None;self.last_task=None
        self.state=IDLE;self.terminal=IDLE;self.reason='STARTUP'
        self.lower=None;self.lower_at=-math.inf;self.lower_session=None
        self.execution=None;self.execution_at=-math.inf;self.pose=None;self.pose_at=-math.inf
        self.v=self.w=0.;self.metrics=[];self.clock_fault=False;self.last_fault=None
        self.stopping_at=None;self.stop_event=None;self.last_status_distance=0.;self.last_status_time=0.

    def metric(self, stage, now, event=None, **details):
        event=event or (self.active.event if self.active else self.stop_event)
        self.metrics.append(dict(stage=stage,monotonic_s=now,source_session_id=event.session if event else '',
                                 event_id=event.event_id if event else 0,candidate_id=event.candidate_id if event else '',
                                 task_id=(self.active or self.last_task).task_id if self.active or self.last_task else '',**details))

    @staticmethod
    def valid_time(stamp, ttl, ros_now, maximum):
        return (all(math.isfinite(v) for v in (stamp,ttl,ros_now)) and
                0<ttl<=maximum+1e-9 and -.05<=ros_now-stamp<ttl)

    def healthy(self, now):
        return self.source_valid and now<self.health_deadline and not self.clock_fault

    def links_healthy(self, now):
        return (self.lower is not None and now-self.lower_at<self.cfg.status_timeout and
                self.execution is not None and now-self.execution_at<self.cfg.status_timeout and
                self.pose is not None and now-self.pose_at<self.cfg.odom_timeout and
                self.lower.get('intent_session_id')==self.session and
                self.lower.get('state')!=3 and self.execution.get('state')!=4)

    def stopped(self):
        return (self.execution is not None and self.execution.get('stopped',False) and
                self.execution.get('state')==0 and abs(self.v)<.015 and abs(self.w)<.025)

    def can_accept(self, now):
        return self.healthy(now) and self.links_healthy(now)

    def _ack(self, event, accepted, reason, task_id=''):
        ack=Ack(event.session,event.event_id,accepted,task_id,reason)
        self.acks.append(ack)
        key=(event.session,event.event_id)
        self.events[key]=(event,ack)
        while len(self.events)>2048:self.events.popitem(last=False)
        return ack

    def stop(self, reason, now, terminal=BLOCKED, invalidate_context=True, event=None):
        self.stop_event=event or (self.active.event if self.active else self.pending['event'] if self.pending else None)
        if self.pending is not None:
            self._ack(self.pending['event'],False,reason)
        if self.active is not None:
            self.last_task=self.active
            self.last_task.finished=now if self.stopped() else None
            self.last_status_distance=max(0.,self.cfg.max_distance-self.active.distance)
            self.last_status_time=max(0.,self.cfg.max_time-(now-self.active.started))
        self.active=None;self.pending=None
        self.reason=reason;self.terminal=terminal
        self.state=terminal if self.stopped() else STOPPING
        self.stopping_at=now
        if invalidate_context:
            self.context_version+=1;self.candidate=None;self.candidate_deadline=-math.inf
        self.metric('stop_recognized',now,reason=reason)

    def invalidate(self, reason, now):
        # A stream of identical invalid heartbeats must not churn contexts.
        if self.last_fault!=reason or self.active is not None or self.pending is not None:
            self.stop(reason,now,FAULT)
        self.last_fault=reason

    def set_clock_fault(self, reason, now):
        self.clock_fault=reason or ''
        if reason:self.invalidate(reason,now)

    def on_health(self, health, now, ros_now):
        valid=(health.protocol_version==1 and bool(health.session) and len(health.session)<=128 and
               health.session not in self.retired and health.sequence>=0 and
               self.valid_time(health.stamp,health.ttl,ros_now,self.cfg.source_timeout) and
               math.isfinite(health.source_age) and health.source_age>=0)
        if not valid:
            self.source_valid=False;self.invalidate('INVALID_INPUT_HEALTH',now);return False
        if health.session!=self.source_session:
            if self.source_session is not None:
                self.retired.add(self.source_session);self.stop('INPUT_SESSION_CHANGED',now,FAULT)
            self.source_session=health.session;self.health_sequence=-1;self.sample_sequence=-1
            self.candidate=None;self.candidate_sequence=-1
        if health.sequence<=self.health_sequence or health.sample_sequence<self.sample_sequence:
            self.source_valid=False;self.invalidate('REPLAY_INPUT_HEALTH',now);return False
        self.health_sequence=health.sequence
        age=max(0.,ros_now-health.stamp)
        sample_age=health.source_age+age
        if health.sample_sequence==self.sample_sequence:
            sample_age=max(sample_age,self.sample_age+now-self.sample_at)
        else:
            self.sample_sequence=health.sample_sequence;self.sample_at=now;self.sample_age=sample_age
        remaining=min(health.ttl-age,self.cfg.source_timeout-sample_age)
        self.health_deadline=now+max(0.,remaining)
        self.source_valid=health.valid and remaining>0
        if self.clock_fault:
            self.source_valid=False
            return False
        if not self.source_valid:self.invalidate('INPUT_INVALID_OR_STALE',now)
        else:self.last_fault=None
        return self.source_valid

    def on_candidate(self, candidate, now, ros_now):
        if (candidate.protocol_version!=1 or candidate.session!=self.source_session or
                candidate.sequence<=self.candidate_sequence):return False
        self.candidate_sequence=candidate.sequence
        self.candidate=candidate
        valid=self.valid_time(candidate.stamp,candidate.ttl,ros_now,self.cfg.max_event_ttl)
        self.candidate_deadline=now+max(0.,candidate.ttl-max(0.,ros_now-candidate.stamp)) if valid else -math.inf
        return valid

    def candidate_reason(self, event, now):
        c=self.candidate
        if event.context_id!=self.session or event.context_version!=self.context_version:return 'CONTEXT_CHANGED'
        if c is None or now>=self.candidate_deadline:return 'CANDIDATE_EXPIRED_OR_MISSING'
        if not c.stable or not c.confirmable:return 'CANDIDATE_WITHDRAWN'
        if (c.session,c.candidate_id,c.version,c.context_id,c.context_version,c.kind,c.relative_yaw)!=(
                event.session,event.candidate_id,event.candidate_version,event.context_id,event.context_version,event.kind,event.relative_yaw):
            return 'CANDIDATE_CHANGED'
        if (event.session,event.candidate_id,event.candidate_version) in self.consumed:return 'CANDIDATE_CONSUMED'
        return ''

    def on_event(self, event, now, ros_now):
        key=(event.session,event.event_id)
        prior=self.events.get(key)
        if prior is not None:
            if prior[0]!=event:
                ack=Ack(event.session,event.event_id,False,'','EVENT_ID_REUSED');self.acks.append(ack);return ack
            if prior[1] is not None:self.acks.append(prior[1])
            return prior[1]
        if (event.protocol_version!=1 or not event.session or len(event.session)>128 or event.session in self.retired or
                event.event_id<0 or not self.valid_time(event.stamp,event.ttl,ros_now,self.cfg.max_event_ttl)):
            return self._ack(event,False,'INVALID_OR_EXPIRED_EVENT')
        if event.event_id<=self.high_event.get(event.session,-1):return self._ack(event,False,'OUT_OF_ORDER_EVENT')
        self.high_event[event.session]=event.event_id
        self.metric('event_received',now,event)
        if event.kind==CANCEL:
            self.stop('USER_CANCEL',now,IDLE,event=event)
            return self._ack(event,True,'CANCELLED')
        if event.kind not in (CONTINUE,ADJUST_HEADING) or not math.isfinite(event.relative_yaw):
            return self._ack(event,False,'INVALID_TASK_KIND_OR_ANGLE')
        if (event.kind==CONTINUE and event.relative_yaw!=0.) or (event.kind==ADJUST_HEADING and not 0<abs(event.relative_yaw)<=self.cfg.max_adjust_angle):
            return self._ack(event,False,'ANGLE_OUTSIDE_AUTHORITY')
        reason=self.candidate_reason(event,now)
        if reason:return self._ack(event,False,reason)
        if self.active is not None or self.pending is not None:
            self.stop('TASK_REPLACED',now,IDLE,invalidate_context=False)
        self.pending=dict(event=event,deadline=now+event.ttl-max(0.,ros_now-event.stamp),
                          candidate_deadline=self.candidate_deadline,reference=self.pose,
                          task_id=self.session+':'+event.session+':'+str(event.event_id),stop_issued_at=None)
        self.events[key]=(event,None)
        self.state=STOPPING;self.reason='WAIT_STOPPED_AND_READY'
        self.metric('stop_wait_started',now,event)
        return None

    def on_lower(self, data, now):
        session=data.get('session_id')
        if self.lower_session is not None and session!=self.lower_session:
            # Planner restarted: stop the current task without a FAULT latch.
            self.stop('PLANNER_SESSION_CHANGED',now,IDLE)
        self.lower_session=session;self.lower=dict(data);self.lower_at=now

    def on_execution(self, data, now):
        if self.execution is not None and data.get('session_id')!=self.execution.get('session_id'):
            self.stop('EXECUTOR_SESSION_CHANGED',now,IDLE)
        self.execution=dict(data);self.execution_at=now

    def on_pose(self, pose, v, w, now):
        if not all(math.isfinite(x) for x in (*pose,v,w,now)):
            self.invalidate('INVALID_ODOM',now);self.pose_at=-math.inf;return
        tracked=self.active or (self.last_task if self.state==STOPPING else None)
        if self.pose is not None and tracked is not None:
            distance=math.hypot(pose[0]-self.pose[0],pose[1]-self.pose[1])
            if distance>.5:
                self.invalidate('ODOM_DISCONTINUITY',now)
            else:tracked.distance+=distance
        self.pose=tuple(pose);self.v=v;self.w=w;self.pose_at=now

    def tick(self, now, ros_now):
        if self.pending is not None:
            pending=self.pending;e=pending['event']
            reason=('EVENT_EXPIRED_WAITING_STOP' if now>=pending['deadline'] or now>=pending['candidate_deadline']
                    else self.candidate_reason(e,now))
            if reason:self.stop(reason,now,BLOCKED)
            elif pending['stop_issued_at'] is None:
                pending['stop_issued_at']=now
                return Output(reason='WAIT_STOPPED_AND_READY')
            elif (self.stopped() and now-pending['stop_issued_at']>=self.cfg.stop_delay+self.cfg.stop_dwell and
                  min(self.lower_at,self.execution_at,self.pose_at)>pending['stop_issued_at']+self.cfg.stop_delay and
                  (self.lower.get('ready_to_rearm') or not self.lower.get('requires_rearm',True))):
                reference=pending['reference'] if e.kind==CONTINUE else self.pose
                target=reference[2]+e.relative_yaw
                self.active=Task(pending['task_id'],e,now,reference,target)
                self.pending=None;self.state=ACCEPTED;self.reason='TASK_ACCEPTED'
                self.consumed.add((e.session,e.candidate_id,e.candidate_version))
                self._ack(e,True,'ACCEPTED',self.active.task_id)
                self.metric('stop_wait_ended',now,e);self.metric('ack',now,e,accepted=True)
        if self.active is not None:
            task=self.active;elapsed=now-task.started
            if self.lower.get('task_id')==task.task_id:
                reason=self.lower.get('reason','')
                if reason=='TASK_HEADING_COMPLETED' and self.stopped():
                    self.stop(reason,now,COMPLETED)
                elif self.lower.get('requires_rearm'):
                    complete=reason.startswith(('PASSAGE_EXIT_CONFIRM_DIRECTION','PASSAGE_END_OR_BRANCH','TASK_RANGE_COMPLETE'))
                    self.stop(reason,now,COMPLETED if complete else BLOCKED)
            if self.active is not None:
                # Removed max_time/max_distance budget limits: the geometric
                # collision gate in the planner is the real obstacle authority;
                # time/distance caps cut off valid motion in long corridors.
                if not task.motion_started and (abs(self.v)>.015 or abs(self.w)>.025):
                    task.motion_started=True;self.state=RUNNING;self.reason='TASK_RUNNING';self.metric('motion_started',now)
        if self.active is None:
            if self.pending is None and self.state==STOPPING and self.stopped():
                if self.last_task is not None and self.last_task.finished is None:self.last_task.finished=now
                self.state=self.terminal;self.metric('stopped',now,reason=self.reason)
            return Output(reason=self.reason)
        t=self.active
        command=FORWARD if t.event.kind==CONTINUE else LEFT if t.event.relative_yaw>0 else RIGHT
        return Output(command,True,t.task_id,t.event.kind,t.reference,t.target_yaw,self.reason)

    def status(self, now):
        task=self.active or self.last_task
        if self.pending is not None:
            e=self.pending['event']
            return dict(task_id=self.pending['task_id'],source_session_id=e.session,event_id=e.event_id,
                state=self.state,kind=e.kind,reference_yaw=self.pending['reference'][2],target_yaw=0.,
                remaining_time=self.cfg.max_time,remaining_distance=self.cfg.max_distance,
                executed_action=self.execution.get('executed_action',0) if self.execution else 0,
                reason=self.reason,can_accept=self.can_accept(now),context_id=self.session,context_version=self.context_version)
        return dict(task_id=task.task_id if task else self.pending['task_id'] if self.pending else '',
                    source_session_id=task.event.session if task else self.source_session or '',
                    event_id=task.event.event_id if task else self.pending['event'].event_id if self.pending else 0,
                    state=self.state,kind=task.event.kind if task else 0,reference_yaw=task.reference[2] if task else 0.,
                    target_yaw=task.target_yaw if task else 0.,
                    remaining_time=max(0.,self.cfg.max_time-((task.finished if task.finished is not None else now)-task.started)) if task else 0.,
                    remaining_distance=max(0.,self.cfg.max_distance-task.distance) if task else 0.,
                    executed_action=self.execution.get('executed_action',0) if self.execution else 0,
                    reason=self.reason,can_accept=self.can_accept(now),context_id=self.session,context_version=self.context_version)
