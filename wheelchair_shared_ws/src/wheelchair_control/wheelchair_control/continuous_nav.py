"""Downstream arbitration: existing intent/path -> full-speed clear-space or native DWB.
No eye/keyboard protocol or planner algorithm lives here.
"""
import math
from dataclasses import replace
from types import SimpleNamespace
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from .planner import Decision,Predictor,State
from .protocol import STOP,FORWARD,LEFT,RIGHT
from .geometry import wrap,transform,clearance

CENTER=(-.266192,.078385)

def center_pose(p):
    x,y,h=p;c=math.cos(h);s=math.sin(h)
    return (x+c*CENTER[0]-s*CENTER[1],y+s*CENTER[0]+c*CENTER[1],h)

class ContinuousNavigation:
    def __init__(self,node):
        self.node=node;self.active=None;self.command=None
        self.client=ActionClient(node,FollowPath,'/wheelchair/dwb_preview/follow_path')
        node.create_subscription(Twist,'/wheelchair/dwb_preview/cmd_vel',self.receive,1)
        self.last_goal_send=None

    def receive(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not self.active or self.active.mode!='DWB' or self.active.handle is None:return
        if not all(math.isfinite(x) for x in values) or msg.linear.x<0 or any(abs(x)>1e-6 for x in values[1:5]):
            self.command=None;return
        if msg.linear.x>.301 or abs(msg.angular.z)>.401:self.command=None;return
        self.command=(msg.linear.x,msg.angular.z)

    def follower(self,owner,poses,task_id):
        if self.active:self.active.cancel()
        value=ContinuousFollower(self,owner,poses,task_id);self.active=value;return value

    def checked(self,owner,state,obstacles,boundary,velocity,reason):
        # Reuse the existing swept braking checker at the same center as DWB.
        cfg=replace(owner.cfg,footprint=tuple((x-CENTER[0],y-CENTER[1]) for x,y in owner.cfg.footprint))
        x,y,h=center_pose(state.pose)
        current=State(x,y,h,state.v-state.w*CENTER[1],state.w,state.current_action,state.observation_age)
        result=Predictor(cfg).check(current,FORWARD,obstacles,boundary,velocity=velocity)
        owner.last_diagnostics['candidate']=result.summary()
        if not result.safe:return Decision(STOP,'BLOCKED','CONTINUOUS_BRAKING_BLOCKED')
        v,w=velocity
        action=FORWARD if v>1e-6 else LEFT if w>1e-6 else RIGHT if w< -1e-6 else STOP
        return Decision(action,'ASSISTING' if reason.startswith('DWB') else 'PASS',reason,velocity)

class ContinuousFollower:
    def __init__(self,adapter,owner,poses,task_id):
        self.adapter=adapter;self.owner=owner;self.cfg=owner.cfg;self.task_id=task_id
        self.poses=list(map(tuple,poses));self.last_goal=self.poses[-1]
        self.done=False;self.blocked=False;self.open_ended=False;self.extending=False
        self.parts=[SimpleNamespace(end=self.last_goal)];self.index=0
        self.mode='DIRECT';self.handle=None;self.pending=False;self.generation=0
        self.result=None;self.cancel_future=None
        self.legacy_turn=None
        if all(math.dist(p[:2],self.poses[0][:2])<.01 for p in self.poses):
            from .path_tracking import PathFollower
            self.legacy_turn=PathFollower(owner,poses,task_id)

    def cancel(self):
        self.generation+=1;self.adapter.command=None
        if self.handle:self.cancel_future=self.handle.cancel_goal_async()
        self.handle=None;self.mode='DIRECT';self.pending=False

    def append(self,poses):
        if math.dist(self.last_goal[:2],poses[0][:2])>.01:raise ValueError('Discontinuous path')
        self.poses.extend(map(tuple,poses[1:]));self.last_goal=self.poses[-1];self.parts=[SimpleNamespace(end=self.last_goal)]
        if self.mode=='DWB':self.cancel();self.mode='DWB'

    def remaining(self,state):
        index=min(range(len(self.poses)),key=lambda i:math.dist(state.pose[:2],self.poses[i][:2]))
        return math.dist(state.pose[:2],self.poses[index][:2])+sum(math.dist(a[:2],b[:2]) for a,b in zip(self.poses[index:],self.poses[index+1:]))

    def replan_distance(self,state):return max(self.cfg.path_replan_distance,1.0)

    def send(self):
        if self.cancel_future is not None and not self.cancel_future.done():return
        if self.pending or self.handle or not self.adapter.client.server_is_ready():return
        goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
        path=Path();path.header.frame_id='odom';path.header.stamp=self.adapter.node.get_clock().now().to_msg()
        for pose in self.poses:
            x,y,h=center_pose(pose);p=PoseStamped();p.header=path.header
            p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);path.poses.append(p)
        goal.path=path;self.pending=True;self.result=None;generation=self.generation
        future=self.adapter.client.send_goal_async(goal)
        def accepted(f):
            try:handle=f.result()
            except Exception:self.result='FAILED';self.pending=False;return
            if generation!=self.generation:
                if handle.accepted:handle.cancel_goal_async()
                return
            self.pending=False
            if not handle.accepted:self.result='FAILED';return
            self.handle=handle;self.adapter.command=None
            def complete(r):
                if generation!=self.generation:return
                try:self.result='DONE' if r.result().status==4 else 'FAILED'
                except Exception:self.result='FAILED'
                self.handle=None;self.adapter.command=None
            handle.get_result_async().add_done_callback(complete)
        future.add_done_callback(accepted)

    def tick(self,state,obstacles,boundary,now,stopped):
        goal=self.last_goal
        if self.result=='DONE':self.done=True;return Decision(STOP,'PASS','PATH_COMPLETED')
        if self.result=='FAILED':self.blocked=True;return Decision(STOP,'BLOCKED','DWB_PATH_FAILED')
        scope=self.owner.task_scope
        # Cruise can pass through unchanged only when its path and heading are
        # straight ahead, clearance is open, and a full-speed stop fits.
        near=clearance(transform(self.cfg.footprint,state.pose),obstacles,boundary,0.)
        if self.legacy_turn is not None and self.mode=='DIRECT' and near>self.cfg.context_assist_leave:
            decision=self.legacy_turn.tick(state,obstacles,boundary,now,stopped)
            self.done=self.legacy_turn.done;self.blocked=self.legacy_turn.blocked
            return decision
        h=state.yaw;dx=goal[0]-state.x;dy=goal[1]-state.y
        aligned=abs(wrap(math.atan2(dy,dx)-h))<.06 and abs(wrap(goal[2]-h))<.06
        nearest=min(range(len(self.poses)),key=lambda i:math.dist(state.pose[:2],self.poses[i][:2]))
        straight=all(abs(wrap(p[2]-h))<.06 and abs(-(p[0]-state.x)*math.sin(h)+(p[1]-state.y)*math.cos(h))<.10 for p in self.poses[nearest:])
        direct=(getattr(scope,'phase','')=='CRUISE' and self.open_ended and aligned and straight
                and near>self.cfg.context_assist_leave and math.hypot(dx,dy)>.6
                and (math.hypot(dx,dy)>self.replan_distance(state) or self.extending))
        if direct:
            if self.mode=='DWB':self.cancel()
            if self.cancel_future is not None and not self.cancel_future.done():return Decision(STOP,'ASSISTING','DWB_CANCEL_WAIT')
            decision=self.adapter.checked(self.owner,state,obstacles,boundary,(.548,0.),'NAV_OPEN_FULL_FORWARD')
            if decision.action!=STOP:return decision
        if self.mode!='DWB':self.mode='DWB';self.adapter.command=None
        self.send()
        if self.adapter.command is None:
            return Decision(STOP,'ASSISTING','DWB_WAIT_COMMAND')
        return self.adapter.checked(self.owner,state,obstacles,boundary,self.adapter.command,'DWB_CONTINUOUS_TRACK')
