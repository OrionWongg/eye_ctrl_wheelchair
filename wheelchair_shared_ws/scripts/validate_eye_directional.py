#!/usr/bin/env python3
"""Stamped synthetic eye samples through the real directional/Nav2 simulator."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid
from process_cleanup import stop_group


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case',choices=['gestures','obstacle'],default='gestures')
    ap.add_argument('--output',type=Path,default=Path('artifacts/eye-direction-20261014'))
    args=ap.parse_args();folder=args.output/args.case;folder.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='93',ROS_LOCALHOST_ONLY='1',IGN_PARTITION='eye_direction_'+uuid.uuid4().hex[:10])
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from wheelchair_interfaces.msg import EyeSample,NavigationIntentStatus,DirectionEvent,IntentAck
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Contacts
    from std_msgs.msg import String
    log=(folder/'launch.log').open('w');trace=(folder/'trace.jsonl').open('w')
    child=subprocess.Popen(['ros2','launch','wheelchair_control','shared.launch.py',
        'gui:=false','scene:=clearpath_office','input_source:=eye','interaction_mode:=directional_intent',
        'eye_sample_mode:=stamped'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('eye_direction_validation',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(EyeSample,'/eye_tracking/samples',1)
    keyboard=node.create_publisher(DirectionEvent,'/wheelchair/keyboard/direction_event',10)
    data=dict(status=None,feedback={},pose=(0.,0.,0.),v=0.,w=0.,contacts=0,events=[],faults=[],max_compute_ms=0.)
    source=uuid.uuid4().hex;sequence=0;next_sample=0.;testing=False
    def record(kind,**fields):trace.write(json.dumps(dict(kind=kind,mono=time.monotonic(),**fields))+'\n')
    def status(m):data['status']=m;record('status',phase=m.phase,reason=m.reason)
    def feedback(m):data['feedback']=json.loads(m.data)
    def odom(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        data.update(pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z)
        record('odom',pose=data['pose'],v=data['v'],w=data['w'])
    def decision(m):
        d=json.loads(m.data);record('decision',**d);data['max_compute_ms']=max(data['max_compute_ms'],d['compute_ms'])
        if testing and (d['reason']=='PLANNING_TIMEOUT_FAULT' or d.get('executor_reason')=='ACTION_STALE'):data['faults'].append(d['reason'])
    def event(m):
        data['events'].append((m.event_id,m.kind));record('eye_event',event=m.event_id,command=m.kind)
    node.create_subscription(NavigationIntentStatus,'/wheelchair/navigation_status',status,1)
    node.create_subscription(String,'/wheelchair/eye_feedback',feedback,1)
    node.create_subscription(String,'/wheelchair/decision_trace',decision,1)
    node.create_subscription(DirectionEvent,'/wheelchair/eye/direction_event',event,10)
    node.create_subscription(IntentAck,'/wheelchair/intent_ack',lambda m:record('ack',event=m.event_id,status=m.status,reason=m.reason),10)
    node.create_subscription(Odometry,'/wheelchair/odom',odom,qos_profile_sensor_data)
    node.create_subscription(Contacts,'/wheelchair/sim/contacts',lambda m:data.update(contacts=data['contacts']+sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in m.contacts)),qos_profile_sensor_data)
    def spin(seconds,predicate=None,h=0.,closed=False,valid=True,send=True):
        nonlocal sequence,next_sample
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            now=time.monotonic()
            if send and now>=next_sample:
                sequence+=1;m=EyeSample();ns=time.time_ns();m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
                m.source_session_id=source;m.sequence=sequence;m.horizontal=float(h);m.closed=closed;m.valid=valid
                pub.publish(m);next_sample=now+.025
            rclpy.spin_once(node,timeout_sec=.005)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def stopped():return abs(data['v'])<.015 and abs(data['w'])<.025
    def confirm(h=0.):
        assert spin(3,lambda:data['feedback'].get('stable') and (data['feedback'].get('candidate_kind')==1 if h==0 else data['feedback'].get('candidate_kind')==2 and data['feedback'].get('candidate_relative_yaw',0)*h<0),h=h),'no stable candidate'
        for _ in range(2):spin(.12,h=h,closed=True);spin(.12,h=h)
    def moving():return abs(data['v'])>.1 or abs(data['w'])>.1
    result=dict(case=args.case,status='FAIL',scene='clearpath_office',synthetic_only=True)
    try:
        assert spin(22,lambda:data['status'] is not None and data['status'].can_start),'startup not ready'
        spin(1.);testing=True
        assert stopped(),'gaze alone moved'
        # Keyboard motion is not subscribed in eye mode.
        k=DirectionEvent();k.header.stamp=node.get_clock().now().to_msg();k.source_session_id='keyboard';k.event_id=1;k.protocol_version=1;k.kind=1;k.ttl_ms=1500;keyboard.publish(k)
        spin(.4);assert stopped(),'unselected keyboard started motion'
        spin(.12,closed=True);spin(.65);assert stopped(),'single blink started motion'
        if args.case=='gestures':
            initial=data['pose'];confirm(-.8)
            assert spin(8,lambda:data['status'].reason=='NAV_TURN_COMPLETED' and stopped(),h=-.8),'left gesture did not finish'
            delta=math.atan2(math.sin(data['pose'][2]-initial[2]),math.cos(data['pose'][2]-initial[2]))
            assert .28<delta<.40,delta
            confirm(.8)
            assert spin(8,lambda:data['status'].reason=='NAV_TURN_COMPLETED' and stopped(),h=.8),'right gesture did not finish'
            assert abs(data['pose'][2]-initial[2])<.06,data['pose']
            confirm();assert spin(6,moving),'center double blink did not start'
            spin(.9,closed=True);assert spin(3,stopped,closed=True),'long close failed to stop'
            spin(.8);assert stopped(),'open eyes resumed old task'
            confirm();assert spin(6,moving),'new confirmation did not restart'
            spin(.2,valid=False);assert spin(3,stopped,valid=False),'invalid tracking failed to stop'
            spin(.8);assert stopped(),'valid frames resumed old task'
            confirm();assert spin(6,moving),'restart after invalid failed'
            assert spin(3,stopped,send=False),'source timeout failed to stop'
            spin(.8);assert stopped(),'source recovery resumed old task'
        else:
            initial=data['pose'];confirm();assert spin(6,moving),'no eye-authorized motion'
            assert spin(25,lambda:data['status'].phase=='BLOCKED' and stopped()),'obstacle was not safely stopped at'
            assert math.dist(initial[:2],data['pose'][:2])>.5,'no progress before obstacle'
            pose=data['pose'];spin(1.)
            assert math.dist(pose[:2],data['pose'][:2])<.02,'blocked motion restarted'
        assert data['contacts']==0,'physical contact'
        assert not data['faults'],data['faults']
        result['status']='PASS'
    except Exception as exc:result['error']=str(exc)
    finally:
        spin(.2,valid=False)
        result.update(final_pose=data['pose'],contacts=data['contacts'],max_compute_ms=data['max_compute_ms'],runtime_faults=data['faults'],events=sorted(set(data['events'])))
        node.destroy_node();rclpy.shutdown();stop_group(child);log.close();trace.close()
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if result['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
