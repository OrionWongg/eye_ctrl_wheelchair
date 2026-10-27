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
from .planner import Config
from ament_index_python.packages import get_package_share_directory
from pathlib import Path as FilePath
import yaml


class NavigationDebug(Node):
    def __init__(self):
        super().__init__('wheelchair_navigation_debug')
        self.trace=None;self.received=None;self.plan_at=None;self.last_key=None
        self.poses=deque(maxlen=3000);self.odom_stamp=None;self.session=None
        self.last_fault=None;self.fault_key=None
        values={}
        for package,name in [('wheelchair_control','narrow_control.yaml'),
                             ('wheelchair_hardware_bridge','hardware_timing.yaml'),
                             ('wheelchair_hardware_bridge','nav2_hardware.yaml')]:
            path=FilePath(get_package_share_directory(package))/'config'/name
            values.update(yaml.safe_load(path.read_text()).get('/**',{}).get('ros__parameters',{}))
        points=values['footprint'];values['footprint']=tuple(zip(points[::2],points[1::2]))
        self.fallback_cfg=Config(**{k:v for k,v in values.items() if k in Config.__dataclass_fields__})
        self.body_mask=self.declare_parameter('body_mask',[-.42,.63,-.40,.40]).value
        from wheelchair_hardware_bridge.scan_map import BODY, SELF_ECHO
        self.scan_body=BODY;self.self_echo=SELF_ECHO
        bridge=yaml.safe_load((FilePath(get_package_share_directory('wheelchair_hardware_bridge'))/'config'/'hardware_bridge.yaml').read_text())
        self.blind_band=bridge['scan_map']['ros__parameters'].get('blind_band',.1)
        self.pub=self.create_publisher(MarkerArray,'/wheelchair/debug/markers',1)
        layers=('hard','navigation_goal',
                'perception_body_mask_not_safety','perception_self_echo_not_safety',
                'perception_blind_band_ideal_not_safety','status_text')
        self.layer_pubs={name:self.create_publisher(
            MarkerArray,'/wheelchair/debug/'+name,1) for name in layers}

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
            dot=abs(sum(getattr(p.orientation,k)*getattr(last.orientation,k) for k in ('x','y','z','w')))
            if math.hypot(p.position.x-last.position.x,p.position.y-last.position.y)<.02 and dot>math.cos(.01):return
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
            cfg=SimpleNamespace(footprint=b['footprint'],padding=b['hard_margin'])
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
            if self.poses:
                p=self.poses[-1].pose;q=p.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                state=SimpleNamespace(x=p.position.x,y=p.position.y,pose=(p.position.x,p.position.y,yaw))
                output=safety_markers(stamp,0,state,self.fallback_cfg,{})
            text=Marker();text.header.frame_id='odom';text.ns='waiting';text.id=1000;text.type=Marker.TEXT_VIEW_FACING
            text.pose.orientation.w=1.;text.pose.position.z=1.;text.scale.z=.2
            text.color.r=1.;text.color.g=.6;text.color.a=1.;text.text='WAITING FOR CONTROL STACK\nOutline: installed default config only; no live planner goal'
            output.markers.append(text)
        # Keep the last diagnostics visible when simulated time stops. The wall
        # timer updates the STALE label; old measurements are never marked live.
        from geometry_msgs.msg import Point
        mask=Marker();mask.header.frame_id='base_link';mask.header.stamp=stamp
        mask.ns='perception_body_mask_not_safety';mask.id=3000;mask.type=Marker.LINE_STRIP
        mask.pose.orientation.w=1.;mask.scale.x=.012;mask.color.r=.8;mask.color.b=1.;mask.color.a=.9
        x0,x1,y0,y1=self.body_mask
        mask.points=[Point(x=float(x),y=float(y),z=.02) for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]]
        output.markers.append(mask)
        # Perception exclusion/clearing regions are different from collision margins.
        echo=Marker();echo.header=mask.header;echo.ns='perception_self_echo_not_safety';echo.id=3001
        echo.type=Marker.LINE_STRIP;echo.pose.orientation.w=1.;echo.scale.x=.012
        echo.color.r=1.;echo.color.g=.3;echo.color.b=.7;echo.color.a=.8
        x0,x1,y0,y1=self.self_echo
        echo.points=[Point(x=float(x),y=float(y),z=.025) for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]]
        output.markers.append(echo)
        from shapely.geometry import box
        x0,x1,y0,y1=self.scan_body
        # Ideal continuous limit; actual clearing is quantized to cells and only
        # applies to UNKNOWN cells whose centres are behind the front edge.
        band=box(x0,y0,x1,y1).buffer(self.blind_band).intersection(box(x0-self.blind_band,y0-self.blind_band,x1,y1+self.blind_band))
        outline=Marker();outline.header=mask.header;outline.ns='perception_blind_band_ideal_not_safety';outline.id=3002
        outline.type=Marker.LINE_STRIP;outline.pose.orientation.w=1.;outline.scale.x=.01
        outline.color.g=1.;outline.color.b=1.;outline.color.a=.7
        outline.points=[Point(x=float(x),y=float(y),z=.03) for x,y in band.exterior.coords]
        output.markers.append(outline)
        for marker in output.markers:marker.lifetime.sec=0;marker.lifetime.nanosec=0
        self.pub.publish(output)
        for name,publisher in self.layer_pubs.items():
            layer=MarkerArray()
            clear=Marker();clear.action=Marker.DELETEALL;layer.markers.append(clear)
            layer.markers.extend(marker for marker in output.markers
                if marker.action!=Marker.DELETEALL and
                ((name=='status_text' and marker.type==Marker.TEXT_VIEW_FACING) or
                 (name!='status_text' and marker.ns==name)))
            publisher.publish(layer)
        trail=Path();trail.header.frame_id='odom';trail.header.stamp=stamp;trail.poses=list(self.poses)
        self.trail_pub.publish(trail)


def main(args=None):
    rclpy.init(args=args)
    node=NavigationDebug()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
