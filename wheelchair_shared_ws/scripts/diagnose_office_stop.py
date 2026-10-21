#!/usr/bin/python3
"""Drive the SIMULATION through a bounded maneuver and capture timing evidence."""
import json,time,uuid,math,argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from wheelchair_interfaces.msg import ControlIntent,ControlStatus,ExecutionStatus,DiscreteAction
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
p=argparse.ArgumentParser();p.add_argument('--output',default='office-stop-before.json');p.add_argument('--seconds',type=float,default=15);p.add_argument('--command',type=int,default=2);p.add_argument('--expect-pass',action='store_true');p.add_argument('--expect-blocked',action='store_true');args=p.parse_args()
rclpy.init();n=Node('office_stop_diagnostic',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=n.create_publisher(ControlIntent,'/wheelchair/keyboard_intent',1)
motion_started=None;motion_ended=None
session=str(uuid.uuid4());seq=0;events=[];last={};state={};contacts=0;start=time.monotonic()
def record(key,m):
 now=time.monotonic();stamp=getattr(m,'header',None);age=None if stamp is None else n.get_clock().now().nanoseconds/1e9-(stamp.stamp.sec+stamp.stamp.nanosec/1e9)
 row={'t':now-start,'topic':key,'gap':now-last.get(key,now),'age':age,'reason':getattr(m,'reason',''),'state':getattr(m,'state',None)}
 if key=='odom':
  q=m.pose.pose.orientation;row['pose']=[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))];row['velocity']=[m.twist.twist.linear.x,m.twist.twist.angular.z]
 events.append(row);last[key]=now;state[key]=m
n.create_subscription(ControlStatus,'/wheelchair/control_status',lambda m:record('status',m),10)
n.create_subscription(ExecutionStatus,'/wheelchair/executor_status',lambda m:record('executor',m),10)
n.create_subscription(DiscreteAction,'/wheelchair/action_safe',lambda m:record('action',m),10)
n.create_subscription(Odometry,'/wheelchair/odom',lambda m:record('odom',m),qos_profile_sensor_data)
n.create_subscription(String,'/wheelchair/decision_trace',lambda m:events.append({'t':time.monotonic()-start,'topic':'trace','value':json.loads(m.data)}),10)
def on_contact(m):
 global contacts
 contacts+=sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in m.contacts)
n.create_subscription(Contacts,'/wheelchair/sim/contacts',on_contact,10)
def spin(seconds,command=0):
 global seq
 end=time.monotonic()+seconds;sent=0
 while time.monotonic()<end:
  now=time.monotonic()
  if now-sent>=.04:
   m=ControlIntent();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link';m.session_id=session;seq+=1;m.sequence=seq;m.command=command;m.enabled=command!=0;m.valid=True;m.ttl_ms=250;m.reason='SIM_DIAGNOSTIC';pub.publish(m);sent=now
  rclpy.spin_once(n,timeout_sec=.003)
try:
 end=time.monotonic()+10
 while time.monotonic()<end:
  spin(.1)
  if 'status'in state and state['status'].intent_session_id==session and state['status'].ready_to_rearm:break
 else:raise RuntimeError('No ready handshake')
 spin(3.0)
 print('Starting maneuver',args.command,flush=True);motion_started=time.monotonic()-start;spin(args.seconds,args.command);motion_ended=time.monotonic()-start
finally:
 spin(1.5)
 active=[e for e in events if motion_started is not None and motion_started+.5<e['t']<(motion_ended or float('inf'))]
 faults=[e for e in active if e['topic'] in ('status','executor') and e['state']==(3 if e['topic']=='status' else 4)]
 traces=[e['value'] for e in active if e['topic']=='trace']
 poses=[e['pose'] for e in active if e['topic']=='odom']
 summary={'command':args.command,'seconds':args.seconds,'faults_during_motion':len(faults),'contacts':contacts,'start_pose':poses[0] if poses else None,'end_pose':poses[-1] if poses else None,'max_compute_ms':max((e.get('compute_ms',0) for e in traces),default=0)}
 out={'summary':summary,'contacts':contacts,'events':events};path=Path(__file__).resolve().parents[1]/'artifacts'/args.output;path.write_text(json.dumps(out))
 for key in ['status','executor','action','odom']:
  rows=[r for r in events if r['topic']==key];print(key,'max_gap',round(max((r['gap'] for r in rows),default=0),3),'reasons',sorted(set(r['reason'] for r in rows)),flush=True)
 print('summary',summary,'saved',path,flush=True);n.destroy_node();rclpy.shutdown()
 if args.expect_pass:
  assert not faults and contacts==0 and poses,'Motion diagnostic failed'
  assert any(e['topic']=='executor' and e['state']==1 for e in active),'Executor never ran'
  assert all(e['state']==0 for e in active if e['topic']=='status'),'Unexpected blocked motion'
 if args.expect_blocked:
  assert not faults and contacts==0,'Fault or contact before obstacle stop'
  assert any(e['topic']=='status' and e['state']==2 for e in active),'Did not block at obstacle'
  assert all(abs(v)<.025 for v in [e for e in events if e['topic']=='odom'][-1]['velocity']),'Robot did not stop'
