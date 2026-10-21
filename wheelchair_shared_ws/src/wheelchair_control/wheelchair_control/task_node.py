"""ROS v1 task manager; sole legacy-adapter producer in intent_confirm mode."""
import json
import math
import time
from dataclasses import fields
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Empty
from wheelchair_interfaces.msg import (CandidateStatus,IntentEvent,InputHealth,IntentAck,TaskStatus,
    ControlIntent,ControlStatus,ExecutionStatus,ObstacleMap,DirectionEvent,NavigationIntentStatus)
from .protocol import StreamGuard,Packet,ClockGuard
from .task_manager import TaskManager,TaskConfig,Candidate,Event,Health,CANCEL,FAULT

QOS=QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE)
EVENT_QOS=QoSProfile(depth=10,durability=DurabilityPolicy.VOLATILE)
def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

class TaskManagerNode(Node):
    def __init__(self, directional=False):
        super().__init__('wheelchair_task_manager')
        self.directional=directional
        self.navigation={};self.navigation_task=''
        source=self.declare_parameter('input_source','eye').value
        if source not in ('eye','keyboard'):raise ValueError('input_source must be eye or keyboard')
        cfg=TaskConfig()
        for f in fields(cfg):setattr(cfg,f.name,float(self.declare_parameter(f.name,getattr(cfg,f.name)).value))
        left=float(self.declare_parameter('left_speed',.389).value)
        right=float(self.declare_parameter('right_speed',.403).value)
        lease=float(self.declare_parameter('action_ttl',.15).value)
        transport=float(self.declare_parameter('transport_delay',.05).value)
        if any(not math.isfinite(v) or v<=0 for v in (left,right,lease,transport)):
            raise ValueError('invalid shared motion model')
        cfg.angular_speed=max(cfg.angular_speed,left,right)
        cfg.stop_delay=max(cfg.stop_delay,lease+transport+cfg.period)
        cfg.budget_stop_delay=max(cfg.budget_stop_delay,.25+lease+transport+cfg.period)
        cfg.__post_init__()
        if directional:
            from .direction_manager import DirectionManager
            self.core=DirectionManager(cfg)
        else:self.core=TaskManager(cfg)
        self.add_on_set_parameters_callback(lambda _:SetParametersResult(successful=False,reason='Restart stopped to change task configuration'))
        self.clock_guard=ClockGuard();self.seq=0;self.status_seq=0;self.events=[]
        self.lower_guard=StreamGuard(cfg.status_timeout);self.exec_guard=StreamGuard(cfg.status_timeout)
        self.map_signature=None;self.last_pose_stamp=-math.inf;self.last_stop=True
        self.command=self.create_publisher(ControlIntent,'/wheelchair/intent',QOS)
        self.ack=self.create_publisher(IntentAck,'/wheelchair/intent_ack',EVENT_QOS)
        self.status=self.create_publisher(NavigationIntentStatus if directional else TaskStatus,
            '/wheelchair/navigation_status' if directional else '/wheelchair/task_status',QOS)
        self.metrics=self.create_publisher(String,'/wheelchair/task_latency_events',EVENT_QOS)
        self.trace=self.create_publisher(String,'/wheelchair/task_trace',QOS)
        prefix='/wheelchair/'+source
        if not directional:self.create_subscription(CandidateStatus,prefix+'/candidate',self.candidate,QOS)
        self.create_subscription(InputHealth,prefix+'/health',self.health,QOS)
        self.create_subscription(DirectionEvent if directional else IntentEvent,
            prefix+('/direction_event' if directional else '/event'),self.event,EVENT_QOS)
        if directional:self.create_subscription(String,'/wheelchair/decision_trace',self.decision_trace,QOS)
        self.create_subscription(ControlStatus,'/wheelchair/control_status',self.lower,QOS)
        self.create_subscription(ExecutionStatus,'/wheelchair/executor_status',self.execution,QOS)
        self.create_subscription(Odometry,'/wheelchair/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(ObstacleMap,'/wheelchair/obstacle_map',self.map,QOS)
        self.create_subscription(Empty,'/wheelchair/emergency_stop',self.cancel,QOS)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ros_now(self):return self.get_clock().now().nanoseconds/1e9
    def candidate(self,m):
        self.core.on_candidate(Candidate(m.source_session_id,m.sequence,stamp(m),m.ttl_ms/1000,m.candidate_id,
            m.candidate_version,m.kind,m.relative_yaw,m.stable,m.confirmable,m.context_id,m.context_version,m.protocol_version),time.monotonic(),self.ros_now())
    def health(self,m):
        self.core.on_health(Health(m.source_session_id,m.sequence,stamp(m),m.ttl_ms/1000,m.source_valid,
            m.source_age_ms/1000,m.sample_sequence,m.protocol_version),time.monotonic(),self.ros_now())
    def event(self,m):
        if self.directional:
            from .direction_manager import STOP_EVENT
            event=Event(m.source_session_id,m.event_id,stamp(m),m.ttl_ms/1000,m.kind,protocol_version=m.protocol_version)
            if m.kind==STOP_EVENT:
                self.events.clear()
                self.core.on_event(event,time.monotonic(),self.ros_now())
                return
            if len(self.events)>=10:
                self.core.invalidate('EVENT_QUEUE_OVERFLOW',time.monotonic());self.events.clear()
            else:self.events.append(event)
            return
        event=Event(m.source_session_id,m.event_id,stamp(m),m.ttl_ms/1000,m.kind,m.candidate_id,
                    m.candidate_version,m.context_id,m.context_version,m.relative_yaw,m.protocol_version)
        if len(self.events)>=10:
            self.core.invalidate('EVENT_QUEUE_OVERFLOW',time.monotonic());self.events.clear()
        self.events.append(event)
    def decision_trace(self,m):
        try:
            data=json.loads(m.data)
            if data.get('session')!=self.core.lower_session:return
            if not -.05<=self.ros_now()-data['time']<self.core.cfg.status_timeout:return
            self.navigation=data.get('candidate',{}).get('navigation',{})
            self.navigation_task=data.get('task_id','')
        except (ValueError,KeyError,TypeError):return
    def lower(self,m):
        now=time.monotonic()
        ok,_,_=self.lower_guard.accept(Packet(m.session_id,m.sequence,stamp(m),self.core.cfg.status_timeout),self.ros_now(),now)
        if not ok:self.core.lower_at=-math.inf;return
        self.core.on_lower(dict(session_id=m.session_id,intent_session_id=m.intent_session_id,task_id=m.task_id,
            state=m.state,reason=m.reason,requires_rearm=m.requires_rearm,ready_to_rearm=m.ready_to_rearm),now-max(0.,self.ros_now()-stamp(m)))
    def execution(self,m):
        now=time.monotonic()
        ok,_,_=self.exec_guard.accept(Packet(m.session_id,m.sequence,stamp(m),self.core.cfg.status_timeout),self.ros_now(),now)
        if not ok:self.core.execution_at=-math.inf;return
        self.core.on_execution(dict(session_id=m.session_id,state=m.state,executed_action=m.executed_action,stopped=m.stopped),now-max(0.,self.ros_now()-stamp(m)))
    def odom(self,m):
        t=stamp(m);now=time.monotonic();age=self.ros_now()-t
        if m.header.frame_id!='odom' or m.child_frame_id!='base_link' or not 0<=age<self.core.cfg.odom_timeout:return
        if t<=self.last_pose_stamp:return
        self.last_pose_stamp=t
        p=m.pose.pose.position;q=m.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.05:
            self.core.invalidate('INVALID_ODOM',now);return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.core.on_pose((p.x,p.y,yaw),m.twist.twist.linear.x,m.twist.twist.angular.z,now-age)
    def map(self,m):
        if not m.complete:return
        signature=(tuple((p.x,p.y) for p in m.known_free_boundary.points),
                   tuple(tuple((p.x,p.y) for p in poly.points) for poly in m.obstacles))
        if self.map_signature is not None and self.map_signature!=signature:self.core.stop('TASK_CONTEXT_MAP_CHANGED',time.monotonic(),FAULT)
        self.map_signature=signature
    def cancel(self,_):
        self.events.clear();self.core.stop('USER_CANCEL',time.monotonic(),0)
    def tick(self):
        now=time.monotonic();ros_now=self.ros_now()
        fault=self.clock_guard.update(ros_now,now)
        if fault=='CLOCK_JUMP':self.last_pose_stamp=-math.inf
        if ros_now<=0:fault='CLOCK_UNAVAILABLE'
        self.core.set_clock_fault(fault,now)
        events,self.events=self.events,[]
        # Same-cycle cancellations win even if a confirmation arrived first.
        for event in events if self.directional else sorted(events,key=lambda e:e.kind!=CANCEL):self.core.on_event(event,now,ros_now)
        output=self.core.tick(now,ros_now)
        m=ControlIntent();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='base_link'
        self.seq+=1;m.session_id=self.core.session;m.sequence=self.seq
        m.command=output.command;m.enabled=output.enabled;m.valid=True;m.ttl_ms=250;m.reason=output.reason
        m.task_id=output.task_id;m.task_kind=output.task_kind
        m.reference_x,m.reference_y,m.reference_yaw=output.reference;m.target_yaw=output.target_yaw
        if self.directional and self.core.active:m.task_elapsed=max(0.,now-self.core.active.started)
        self.command.publish(m)
        if not output.enabled and not self.last_stop:self.core.metric('stop_published',now,reason=output.reason)
        self.last_stop=not output.enabled
        for ack in self.core.acks:
            a=IntentAck();a.header=m.header;a.protocol_version=1;a.manager_session_id=self.core.session
            a.source_session_id=ack.session;a.event_id=ack.event_id;a.status=1 if ack.accepted else 2
            a.task_id=ack.task_id;a.reason=ack.reason;self.ack.publish(a)
        self.core.acks.clear()
        self.status_seq+=1
        if self.directional:
            s=NavigationIntentStatus();s.header=m.header;s.manager_session_id=self.core.session;s.sequence=self.status_seq
            task=self.core.active
            pending=self.core.pending
            event=task.event if task else pending['event'] if pending else None
            s.source_session_id=event.session if event else self.core.source_session or ''
            s.event_id=event.event_id if event else 0
            s.task_id=task.task_id if task else pending['task_id'] if pending else ''
            nav=self.navigation if task and self.navigation_task==task.task_id else {}
            s.phase=nav.get('phase','WAIT_STOPPED' if pending else 'STARTING' if task else 'STOPPED' if self.core.state==0 else 'BLOCKED')
            s.preference=int(nav.get('preference',0));s.entry_id=nav.get('entry_id','')
            s.remaining_time=float(nav.get('remaining_time',0.));s.remaining_distance=float(nav.get('remaining_distance',0.))
            goal=nav.get('goal')
            if goal:s.local_goal.x,s.local_goal.y,s.local_goal.theta=map(float,goal)
            s.executed_action=self.core.execution.get('executed_action',0) if self.core.execution else 0
            s.can_start=self.core.can_accept(now);s.reason=self.core.lower.get('reason','') if task and self.core.lower else self.core.reason
            self.status.publish(s)
            self.trace.publish(String(data=json.dumps(dict(nav,monotonic_s=now,ros_time=ros_now,phase=s.phase,task_id=s.task_id,event_id=s.event_id))))
            self.core.metrics.clear()
            return
        s=TaskStatus();s.header=m.header;s.protocol_version=1;s.manager_session_id=self.core.session;s.sequence=self.status_seq
        for key,value in self.core.status(now).items():setattr(s,key,value)
        self.status.publish(s)
        self.trace.publish(String(data=json.dumps(dict(monotonic_s=now,ros_time=ros_now,**self.core.status(now),output=vars(output)))))
        for event in self.core.metrics:self.metrics.publish(String(data=json.dumps(event)))
        self.core.metrics.clear()

def main(directional=False):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=TaskManagerNode(directional)
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():
            node.core.stop('MANAGER_SHUTDOWN',time.monotonic(),0);node.events.clear();node.tick()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

def directional_main():
    main(directional=True)
