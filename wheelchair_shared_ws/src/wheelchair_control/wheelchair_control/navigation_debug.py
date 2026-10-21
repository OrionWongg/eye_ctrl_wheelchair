"""Read-only RViz diagnostics, rendered outside the safety control process."""
import json
import math
import time
from collections import deque
from types import SimpleNamespace

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .visualization import safety_markers


class NavigationDebug(Node):
    def __init__(self):
        super().__init__('wheelchair_navigation_debug')
        self.trace=None;self.received=None;self.plan_at=None;self.last_key=None
        self.poses=deque(maxlen=3000);self.odom_stamp=None;self.session=None
        self.last_fault=None;self.fault_key=None
        self.pub=self.create_publisher(MarkerArray,'/wheelchair/debug/markers',1)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trail_pub=self.create_publisher(Path,'/wheelchair/debug/executed_path',latched)
        self.plan_pub=self.create_publisher(Path,'/wheelchair/debug/local_plan',latched)
        self.create_subscription(String,'/wheelchair/decision_trace',self.decision,1)
        self.create_subscription(Odometry,'/wheelchair/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(Path,'/wheelchair/local_plan',self.plan,1)
        self.create_timer(.2,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Read-only diagnostics: waiting for /wheelchair/decision_trace')

    def decision(self,msg):
        try:
            data=json.loads(msg.data)
            s=data['odom'];budget=data['geometry_budget']
            if not all(math.isfinite(float(s[k])) for k in ('x','y','yaw')):return
            if len(budget['footprint'])<3:return
        except (ValueError,KeyError,TypeError):return
        if self.session is not None and self.session!=data.get('session'):
            self.poses.clear();self.plan_at=None;self.last_fault=None;self.fault_key=None
            empty=Path();empty.header.frame_id='odom';self.plan_pub.publish(empty)
        self.session=data.get('session');self.trace=data;self.received=time.monotonic()
        if data.get('state') in ('BLOCKED','FAULT'):
            reason=data.get('reason','').split(':READY_TO_REARM')[0]
            if reason!=self.fault_key or (self.last_fault and data.get('compute_ms',0)>self.last_fault[2]):
                self.last_fault=(reason,data.get('decision_sequence'),data.get('compute_ms',0),time.monotonic())
            self.fault_key=reason
        else:self.fault_key=None
        key=(data.get('state'),data.get('reason'),data.get('executor_reason'))
        if key!=self.last_key:
            self.get_logger().info(f'{key[0]}: {key[1]} | executor: {key[2]} | compute {data.get("compute_ms",0):.1f} ms')
            self.last_key=key

    def odom(self,msg):
        if msg.header.frame_id!='odom':return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.odom_stamp is not None and stamp<self.odom_stamp:self.poses.clear()
        self.odom_stamp=stamp
        p=msg.pose.pose
        if not all(math.isfinite(v) for v in (p.position.x,p.position.y,p.orientation.z,p.orientation.w)):return
        if self.poses:
            last=self.poses[-1].pose
            if math.hypot(p.position.x-last.position.x,p.position.y-last.position.y)<.02:return
        sample=PoseStamped();sample.header=msg.header;sample.pose=p;self.poses.append(sample)

    def plan(self,msg):
        if msg.header.frame_id!='odom':return
        self.plan_at=time.monotonic();self.plan_pub.publish(msg)

    def publish(self):
        stamp=self.get_clock().now().to_msg()
        d=self.trace
        output=MarkerArray()
        if d is not None:
            s=d['odom'];b=d['geometry_budget']
            cfg=SimpleNamespace(footprint=b['footprint'],padding=b['hard_margin']+b['model_error']+b['nominal_sampling_error'],comfort_distance=b['comfort_distance'])
            state=SimpleNamespace(x=s['x'],y=s['y'],pose=(s['x'],s['y'],s['yaw']))
            stale=time.monotonic()-self.received>.5
            output=safety_markers(stamp,d.get('decision_sequence',0),state,cfg,d.get('candidate',{}))
            text=output.markers[-1];text.pose.position.x+=.8;text.pose.position.y+=.8;text.scale.z=.14
            nav=d.get('candidate',{}).get('navigation',{});search=d.get('candidate',{}).get('path_search',{})
            def ms(value):return 'N/A' if value is None else f'{value*1000:.0f} ms'
            ages=d.get('source_ages',{})
            plan_age='not received (wait for next plan)' if self.plan_at is None else f'{time.monotonic()-self.plan_at:.1f}s ago'
            text.text=(f'{"STALE TRACE / last received: " if stale else ""}{d.get("state")} | {d.get("reason")}\n'
                f'executor: {d.get("executor_reason")} | rearm: {d.get("requires_rearm")}\n'
                f'control: {d.get("compute_ms",0):.1f} / 50 ms | trace age: {ms(time.monotonic()-self.received)}\n'
                f'phase: {nav.get("phase","-")} | context: {nav.get("context","-")}\n'
                f'search: {search.get("reason","-")} {search.get("compute_ms",0):.1f} ms | last plan: {plan_age}\n'
                f'ages at decision: odom {ms(ages.get("odom"))}, scan {ms(ages.get("scan"))}, map receipt {ms(ages.get("map_receipt"))}')
            if self.last_fault:
                reason,seq,compute,received=self.last_fault
                text.text+=f'\nlast fault: {reason} | seq {seq} | peak {compute:.1f} ms | {time.monotonic()-received:.1f}s ago'
            if stale or d.get('state') in ('BLOCKED','FAULT'):text.color.r=1.;text.color.g=.3;text.color.b=.2
            goal=nav.get('goal')
            if goal and len(goal)==3 and all(math.isfinite(v) for v in goal):
                marker=Marker();marker.header.frame_id='odom';marker.header.stamp=stamp;marker.ns='navigation_goal';marker.id=2000
                marker.type=Marker.ARROW;marker.pose.position.x=goal[0];marker.pose.position.y=goal[1];marker.pose.position.z=.12
                marker.pose.orientation.z=math.sin(goal[2]/2);marker.pose.orientation.w=math.cos(goal[2]/2)
                marker.scale.x=.5;marker.scale.y=.09;marker.scale.z=.09
                marker.color.r=1.;marker.color.g=.6;marker.color.a=1.;output.markers.append(marker)
            if stale:
                for marker in output.markers[:-1]:
                    if marker.type!=Marker.TEXT_VIEW_FACING and marker.action==Marker.ADD:marker.color.a=.25
        else:
            text=Marker();text.header.frame_id='odom';text.ns='waiting';text.id=1000;text.type=Marker.TEXT_VIEW_FACING
            text.pose.orientation.w=1.;text.pose.position.z=1.;text.scale.z=.2
            text.color.r=1.;text.color.g=.6;text.color.a=1.;text.text='WAITING FOR DECISION TRACE - check simulation and ROS_DOMAIN_ID'
            output.markers.append(text)
        # Keep the last diagnostics visible when simulated time stops. The wall
        # timer updates the STALE label; old measurements are never marked live.
        for marker in output.markers:marker.lifetime.sec=0;marker.lifetime.nanosec=0
        self.pub.publish(output)
        trail=Path();trail.header.frame_id='odom';trail.header.stamp=stamp;trail.poses=list(self.poses)
        self.trail_pub.publish(trail)


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=NavigationDebug()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
