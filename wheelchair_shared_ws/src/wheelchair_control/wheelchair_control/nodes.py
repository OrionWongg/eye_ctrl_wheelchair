"""ROS wiring for the discrete planner and simulation executor.
Simulation uses a complete oracle; hardware supplies a conservative scan map with unknown cells blocked.
"""
import json
import math
import time
import uuid
from dataclasses import fields
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.time import Time
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty,String
from tf2_ros import Buffer,TransformListener
from wheelchair_interfaces.msg import ControlIntent,DiscreteAction,ControlStatus,ExecutionStatus,ObstacleMap
from .protocol import Packet,StreamGuard,ClockGuard,STOP,LEFT,RIGHT
from .geometry import convex,IndexedPolygon,nearby_obstacles
from .planner import Config,Planner,State,Decision
from .executor import Executor
from visualization_msgs.msg import MarkerArray
from .visualization import safety_markers

SENSOR_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
COMMAND_QOS=QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE)
def source_seconds(stamp):return stamp.sec+stamp.nanosec/1e9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def packet(msg,action=False):
    return Packet(msg.session_id,msg.sequence,source_seconds(msg.header.stamp),msg.ttl_ms/1000.,msg.action if action else msg.command,True if action else msg.enabled,True if action else msg.valid,msg.reason,
        **({} if action else {name:getattr(msg,name,default) for name,default in
           (('task_id',''),('task_kind',0),('reference_x',0.),('reference_y',0.),
            ('reference_yaw',0.),('target_yaw',0.),('task_elapsed',0.))}))
def sim_time(node):return node.get_clock().now().nanoseconds/1e9

def config(node):
    c=Config()
    for f in fields(c):
        val=getattr(c,f.name)
        if f.name=='footprint':val=[float(v) for p in val for v in p]
        node.declare_parameter(f.name,val)
    values={f.name:node.get_parameter(f.name).value for f in fields(c)}
    flat=values['footprint']
    if len(flat)%2:raise ValueError('footprint must contain xy pairs')
    values['footprint']=tuple(zip(flat[::2],flat[1::2]))
    result=Config(**values)
    # Changing safety budgets while moving is not supported. Restart disarmed to apply.
    node.add_on_set_parameters_callback(lambda params:SetParametersResult(successful=False,reason='Restart disarmed to change safety parameters'))
    return result

class PlannerNode(Node):
    def __init__(self):
        super().__init__('wheelchair_safety_gate')
        backend=self.declare_parameter('navigation_backend','none').value
        self.declare_parameter('path_planner_backend','nav2_dwb')
        self.allow_dynamic_map=self.declare_parameter('allow_dynamic_map',False).value
        self.cfg=config(self);self.core=Planner(self.cfg)
        if backend not in ('none','nav2'):raise ValueError('Invalid navigation backend')
        self.path_client=None
        if backend=='nav2':
            from .nav2_transport import Nav2PathClient
            self.path_client=Nav2PathClient(self)
            self.core.path_provider=self.path_client
        self.session=str(uuid.uuid4());self.sequence=0
        self.intent_guard=StreamGuard(.25);self.map_guard=StreamGuard(self.cfg.map_age);self.exec_guard=StreamGuard(.25)
        self.clock_guard=ClockGuard()
        self.state=State();self.odom_received=self.scan_received=self.map_received=self.executor_received=-math.inf
        self.odom_stamp=self.scan_stamp=0.
        self.obstacles=[];self.boundary=[];self.map_geometry=None;self.map_valid=False;self.scan_valid=False
        self.exec_status=None;self.scan_msg=None;self.last_clock_fault='';self.last_log=None;self.last_action=STOP;self.last_tick=None;self.previous_callback_ms=0.;self.map_callback_ms=0.
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self)
        self.actions=self.create_publisher(DiscreteAction,'/wheelchair/action_safe',COMMAND_QOS)
        self.status=self.create_publisher(ControlStatus,'/wheelchair/control_status',COMMAND_QOS)
        self.trace=self.create_publisher(String,'/wheelchair/decision_trace',1)
        self.markers=self.create_publisher(MarkerArray,'/wheelchair/safety_markers',1)
        self.create_subscription(ControlIntent,'/wheelchair/intent',self.on_intent,COMMAND_QOS)
        self.create_subscription(String,'/wheelchair/input_fault',lambda m:self.core.trip('INPUT_FAULT:'+m.data),COMMAND_QOS)
        self.create_subscription(Empty,'/wheelchair/emergency_stop',lambda _:self.core.trip('USER_EMERGENCY_STOP'),COMMAND_QOS)
        self.create_subscription(Odometry,'/wheelchair/odom',self.on_odom,SENSOR_QOS)
        self.create_subscription(LaserScan,'/wheelchair/scan',self.on_scan,SENSOR_QOS)
        self.create_subscription(ObstacleMap,'/wheelchair/obstacle_map',self.on_map,COMMAND_QOS)
        self.create_subscription(ExecutionStatus,'/wheelchair/executor_status',self.on_executor,COMMAND_QOS)
        self.create_timer(self.cfg.period,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def on_intent(self,msg):
        if msg.header.frame_id!='base_link':
            self.core.trip('INVALID_INTENT_FRAME');return
        p=packet(msg);ok,changed,reason=self.intent_guard.accept(p,sim_time(self),time.monotonic())
        if not ok:self.core.trip(reason);return
        self.core.intent(p,changed)
    def on_odom(self,msg):
        self.odom_received=-math.inf
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        vals=(p.x,p.y,p.z,q.x,q.y,q.z,q.w,msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if not all(math.isfinite(x) for x in vals):return
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        if abs(sum(x*x for x in (q.x,q.y,q.z,q.w))-1.)>.05:return
        self.state=State(p.x,p.y,yaw(q),msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        self.odom_received=time.monotonic();self.odom_stamp=source_seconds(msg.header.stamp)
    def on_scan(self,msg):
        self.scan_valid=False;self.scan_msg=msg
        vals=(msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
        if not all(math.isfinite(x) for x in vals) or len(msg.ranges)<180:return
        if msg.angle_increment<=0 or abs(len(msg.ranges)*msg.angle_increment-2*math.pi)>.06:return
        if not 0<=msg.range_min<msg.range_max:return
        if any(math.isnan(r) or (r>0. and r<msg.range_min) for r in msg.ranges):return
        self.scan_valid=True;self.scan_received=time.monotonic();self.scan_stamp=source_seconds(msg.header.stamp)
    def on_map(self,msg):
        begin=time.monotonic()
        self.map_valid=False
        if msg.header.frame_id!='odom' or not msg.complete:return
        p=Packet(msg.session_id,msg.sequence,source_seconds(msg.header.stamp),self.cfg.sensor_age)
        ok,changed,reason=self.map_guard.accept(p,sim_time(self),time.monotonic())
        if not ok:self.core.trip('MAP_'+reason);return
        # Compare exact geometry in native serialization before expanding
        # thousands of ROS point objects on every static-map heartbeat.
        geometry_msg=ObstacleMap()
        # Three-byte strings plus NUL keep every following CDR field aligned.
        # Empty strings leave unspecified padding bytes in serialized output.
        geometry_msg.header.frame_id='map';geometry_msg.session_id='map'
        geometry_msg.obstacles=msg.obstacles
        geometry_msg.known_free_boundary=msg.known_free_boundary
        geometry=serialize_message(geometry_msg)
        if geometry!=self.map_geometry:
            shape=tuple(tuple((p.x,p.y) for p in poly.points) for poly in msg.obstacles)
            boundary=tuple((p.x,p.y) for p in msg.known_free_boundary.points)
            if self.map_geometry is not None and self.cfg.mode=='corridor_assist' and not self.allow_dynamic_map:self.core.trip('PASSAGE_MAP_CHANGED')
            if not convex(boundary) or any(not convex(o) for o in shape):return
            self.obstacles=[IndexedPolygon(o) for o in shape]
            for i,o in enumerate(self.obstacles):o.obstacle_id=f'map_obstacle_{i}'
            self.boundary=boundary
            self.map_geometry=geometry
        if changed:self.core.trip('MAP_SESSION_CHANGED')
        self.map_valid=True;self.map_received=time.monotonic()
        self.map_callback_ms=(self.map_received-begin)*1000
    def on_executor(self,msg):
        if msg.executed_action not in range(4) or msg.state not in range(5):
            self.exec_guard.deadline=-math.inf;self.core.trip('EXECUTOR_INVALID');return
        p=Packet(msg.session_id,msg.sequence,source_seconds(msg.header.stamp),.25)
        ok,changed,reason=self.exec_guard.accept(p,sim_time(self),time.monotonic())
        if not ok:self.core.trip('EXECUTOR_'+reason);return
        if changed:self.core.trip('EXECUTOR_SESSION_CHANGED')
        self.exec_status=msg;self.executor_received=time.monotonic()
    def fresh(self,receipt,stamp,max_age,now,ros_now):
        return now-receipt<max_age and -.05<=ros_now-stamp<max_age
    def tick(self):
        now=time.monotonic();ros_now=sim_time(self)
        interval_ms=(now-self.last_tick)*1000 if self.last_tick is not None else None
        self.last_tick=now;stages={}
        self.core.last_diagnostics={}
        clock_fault=self.clock_guard.update(ros_now,now)
        fault=clock_fault
        if self.path_client is not None:
            if not self.path_client.healthy(now):fault=fault or 'NAV2_MAP_OR_PLANNER_STALE'
            if self.path_client.changed:
                self.path_client.changed=False;fault=fault or 'NAV2_SESSION_CHANGED'
        if not self.fresh(self.odom_received,self.odom_stamp,self.cfg.odom_age,now,ros_now):fault=fault or 'ODOM_STALE'
        elif not self.scan_valid or not self.fresh(self.scan_received,self.scan_stamp,self.cfg.sensor_age,now,ros_now):fault=fault or 'SCAN_STALE_OR_INVALID'
        elif not self.map_valid or not self.map_guard.fresh(now):fault=fault or 'MAP_STALE_OR_INCOMPLETE'
        elif self.scan_msg is not None:
            try:
                self.tf.lookup_transform('base_link',self.scan_msg.header.frame_id,Time.from_msg(self.scan_msg.header.stamp))
            except Exception:fault=fault or 'TF_FAULT'
        if not self.intent_guard.fresh(now):fault=fault or 'INTENT_STALE'
        if not self.exec_guard.fresh(now):fault=fault or 'EXECUTOR_STALE'
        elif self.exec_status.state==ExecutionStatus.FAULT:fault=fault or 'EXECUTOR_FAULT'
        stopped=bool(self.exec_status and self.exec_status.stopped and abs(self.state.v)<.015 and abs(self.state.w)<.025)
        self.state.observation_age=max(0.,now-self.odom_received,ros_now-self.odom_stamp,now-self.scan_received,ros_now-self.scan_stamp,now-self.map_received)
        if self.path_client is not None and self.path_client.healthy(now):
            self.state.observation_age=max(self.state.observation_age,ros_now-self.path_client.grid_stamp)
            self.state.feedback_age=max(0.,now-self.odom_received,ros_now-self.odom_stamp)
        if not math.isfinite(self.state.observation_age):self.state.observation_age=None
        self.state.current_action=self.exec_status.executed_action if self.exec_status else STOP
        self.state.execution_phase=self.exec_status.state if self.exec_status else None
        self.state.pending_start_age=None
        # Pending startup can begin within the reaction budget. During braking,
        # retain measured momentum; never pretend a new direction already executes.
        if self.exec_status and self.exec_status.state==ExecutionStatus.STARTING and abs(self.state.v)<.015 and abs(self.state.w)<.025:
            self.state.current_action=self.last_action
            if self.exec_status.executed_action==STOP:
                self.state.pending_start_age=max(0.,now-self.executor_received,
                                                ros_now-source_seconds(self.exec_status.header.stamp))
        if fault:
            decision=self.core.trip(fault)
        else:
            stage=time.monotonic()
            self.core.semantic_obstacles=nearby_obstacles(self.state,self.obstacles,self.cfg)
            combined=self.obstacles+(self.path_client.obstacles if self.path_client is not None else [])
            local_obstacles=nearby_obstacles(self.state,combined,self.cfg)
            stages['obstacles']=(time.monotonic()-stage)*1000;stage=time.monotonic()
            decision=self.core.tick(self.state,local_obstacles,self.boundary,now,stopped=stopped)
            stages['planner']=(time.monotonic()-stage)*1000
            # Report if even the measured current momentum cannot stop within known space.
            if decision.action==STOP and (abs(self.state.v)>.015 or abs(self.state.w)>.025):
                if not self.core.predictor.check(self.state,STOP,local_obstacles,self.boundary).safe:
                    decision=self.core.trip('STOPPING_ENVELOPE_RISK')
        compute_ms=(time.monotonic()-now)*1000
        if compute_ms>self.cfg.period*1000:
            decision=self.core.trip('PLANNING_TIMEOUT_FAULT')
        self.sequence+=1;stamp=self.get_clock().now().to_msg();p=self.core.current_intent
        action=DiscreteAction();action.header.stamp=stamp;action.header.frame_id='base_link'
        action.session_id=self.session;action.sequence=self.sequence
        action.intent_session_id=p.session if p else '';action.intent_sequence=p.sequence if p else 0
        action.action=decision.action;action.ttl_ms=round(self.cfg.action_ttl*1000);action.reason=decision.reason
        if decision.reason=='TASK_HEADING_MICRO_TURN':
            action.max_run_ms=round(self.core.last_diagnostics['task_pulse_duration']*1000)
        else:
            action.max_run_ms=round(self.cfg.micro_turn_min*1000) if decision.reason=='USER_MICRO_TURN' else 0
        self.actions.publish(action);self.last_action=decision.action
        published=time.monotonic()
        status=ControlStatus();status.header=action.header;status.session_id=self.session;status.sequence=self.sequence
        status.intent_session_id=action.intent_session_id;status.action_session_id=self.session
        status.requested_command=p.command if p else STOP;status.executed_action=self.exec_status.executed_action if self.exec_status else STOP
        status.state={'PASS':0,'ASSISTING':1,'BLOCKED':2,'FAULT':3}[decision.state]
        status.task_id=p.task_id if p else ''
        status.reason=decision.reason;status.requires_rearm=self.core.latch;status.ready_to_rearm=self.core.ready
        self.status.publish(status)
        trace={'task_id':p.task_id if p else '', 'time':ros_now,'intent_source_time':p.stamp if p else None,'intent_enabled':p.enabled if p else False,'intent_valid':p.valid if p else False,'intent_sequence':action.intent_sequence,'intent':status.requested_command,'action':decision.action,'executed':status.executed_action,'state':decision.state,'reason':decision.reason,'requires_rearm':self.core.latch,'odom':self.state.__dict__,'reference':self.core.reference,'candidate':self.core.last_diagnostics,'compute_ms':compute_ms,'decision_sequence':self.sequence,'mode':self.cfg.mode,'session':self.session,'source_ages':{'odom':ros_now-self.odom_stamp if math.isfinite(self.odom_received) else None,'scan':ros_now-self.scan_stamp if math.isfinite(self.scan_received) else None,'map_receipt':now-self.map_received if math.isfinite(self.map_received) else None,'executor_receipt':now-self.executor_received if math.isfinite(self.executor_received) else None},'geometry_budget':{'footprint':self.cfg.footprint,'hard_margin':self.cfg.margin,'model_error':self.cfg.model_error,'nominal_sampling_error':self.cfg.padding-self.cfg.margin-self.cfg.model_error,'comfort_distance':self.cfg.comfort_distance},'executor_reason':self.exec_status.reason if self.exec_status else None}
        trace['timing']={'timer_interval_ms':interval_ms,'stages_ms':stages,
            'previous_callback_ms':self.previous_callback_ms,'map_callback_ms':self.map_callback_ms,
            'action_publish_ros':source_seconds(stamp),'action_publish_monotonic':published,
            'publish_elapsed_ms':(published-now)*1000,
            'costmap_callback_ms':self.path_client.grid_compute_ms if self.path_client else None}
        self.trace.publish(String(data=json.dumps(trace)))
        # RViz work follows the safety action and runs at 5 Hz.
        if self.sequence%4==0 and self.markers.get_subscription_count():
            self.markers.publish(safety_markers(stamp,self.sequence,self.state,self.cfg,self.core.last_diagnostics))
        key=(decision.action,decision.reason,self.core.ready)
        if key!=self.last_log:self.get_logger().info(json.dumps(trace));self.last_log=key
        self.previous_callback_ms=(time.monotonic()-now)*1000

class ExecutorNode(Node):
    def __init__(self):
        super().__init__('wheelchair_executor')
        self.cfg=config(self);self.core=Executor(self.cfg)
        self.guard=StreamGuard(self.cfg.action_ttl);self.clock_guard=ClockGuard()
        self.session=str(uuid.uuid4());self.seq=0;self.odom_received=-math.inf;self.odom_stamp=0.;self.v=self.w=0.;self.last_tick=None;self.action_timing={};self.last_timing_reason=None
        self.drive=self.create_publisher(Twist,'/wheelchair/sim/drive',COMMAND_QOS)
        self.status=self.create_publisher(ExecutionStatus,'/wheelchair/executor_status',COMMAND_QOS)
        self.timing=self.create_publisher(String,'/wheelchair/executor_timing',1)
        self.create_subscription(DiscreteAction,'/wheelchair/action_safe',self.on_action,COMMAND_QOS)
        self.create_subscription(Odometry,'/wheelchair/odom',self.on_odom,SENSOR_QOS)
        self.create_timer(self.cfg.period,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def on_action(self,msg):
        now=time.monotonic();ros_now=sim_time(self)
        previous=self.action_timing.get('receive_monotonic')
        self.action_timing=dict(sequence=msg.sequence,session=msg.session_id,
            source_stamp=source_seconds(msg.header.stamp),receive_ros=ros_now,receive_monotonic=now,
            source_age_ms=(ros_now-source_seconds(msg.header.stamp))*1000,
            receive_interval_ms=(now-previous)*1000 if previous is not None else None)
        ok,changed,reason=self.guard.accept(packet(msg,True),ros_now,now)
        self.action_timing['accepted']=ok;self.action_timing['guard_reason']=reason
        if not ok:
            self.core.new_session(now);self.core.reason=reason;return
        if changed:
            self.core.new_session(now)
            if msg.action!=STOP:return
        allowed_limits={round(self.cfg.micro_turn_min*1000)}  # existing manual pulse
        allowed_limits.update(round(duration*1000) for duration in (.10,.20,.30)
                              if self.cfg.micro_turn_min-1e-9<=duration<=self.cfg.minimum_action+1e-9)
        if msg.max_run_ms and (msg.action not in (LEFT,RIGHT) or msg.max_run_ms not in allowed_limits):
            self.core.new_session(now);self.core.reason='INVALID_RUN_LIMIT';return
        self.core.request(msg.action,now,msg.max_run_ms/1000 if msg.max_run_ms else None)
    def on_odom(self,msg):
        self.odom_received=-math.inf
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        self.v,self.w=msg.twist.twist.linear.x,msg.twist.twist.angular.z
        if not math.isfinite(self.v) or not math.isfinite(self.w):return
        self.odom_received=time.monotonic();self.odom_stamp=source_seconds(msg.header.stamp)
    def tick(self):
        now=time.monotonic();ros_now=sim_time(self)
        interval_ms=(now-self.last_tick)*1000 if self.last_tick is not None else None
        self.last_tick=now
        clock_fault=self.clock_guard.update(ros_now,now)
        healthy=not clock_fault and now-self.odom_received<self.cfg.odom_age and -.05<=ros_now-self.odom_stamp<self.cfg.odom_age
        a,stopped=self.core.tick(now,self.v,self.w,healthy,self.guard.fresh(now))
        msg=Twist();msg.linear.x,msg.angular.z=self.cfg.velocity(a);self.drive.publish(msg)
        status=ExecutionStatus();status.header.stamp=self.get_clock().now().to_msg();status.header.frame_id='base_link'
        self.seq+=1;status.session_id=self.session;status.sequence=self.seq
        status.action_session_id=self.guard.session or '';status.action_sequence=max(0,self.guard.sequence)
        status.executed_action=a;status.state={'STOPPED':0,'RUNNING':1,'STOPPING':2,'STARTING':3,'FAULT':4}[self.core.phase]
        status.stopped=stopped;status.reason=clock_fault or self.core.reason;self.status.publish(status)
        if self.seq%4==0 or status.reason!=self.last_timing_reason:
            timing=dict(self.action_timing,timer_interval_ms=interval_ms,time=ros_now,
                receipt_age_ms=(now-self.action_timing['receive_monotonic'])*1000 if self.action_timing else None,
                callback_ms=(time.monotonic()-now)*1000,reason=status.reason)
            self.timing.publish(String(data=json.dumps(timing)))
            if status.reason!=self.last_timing_reason:self.get_logger().info(json.dumps(timing))
            self.last_timing_reason=status.reason

def run(cls):
    from .clock_worker import ClockWorker
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=cls();clock_worker=ClockWorker(node)
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if isinstance(node,ExecutorNode) and rclpy.ok():node.drive.publish(Twist())
        clock_worker.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
def planner_main():run(PlannerNode)
def executor_main():run(ExecutorNode)
