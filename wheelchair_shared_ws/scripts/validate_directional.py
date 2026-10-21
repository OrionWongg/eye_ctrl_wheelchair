#!/usr/bin/env python3
"""Keyboard-first Gazebo acceptance. Only isolated, owned processes are stopped."""
import argparse
import json
import math
import os
import re
from pathlib import Path
import subprocess
import time
import uuid
from process_cleanup import stop_group
from validate_intent_tasks import owned_pid

CASES=('early_left','near_left','right','two_left','blocked_first','no_entry','behind_entry',
       'open_area','repeat','replace','w_during_turn','stop','branch','input_exit',
       'manager_exit','planner_exit','executor_exit','narrow_turn','straight_door','stop_turn','stop_traverse','keyboard_exit','uturn_open','uturn_refused','uturn_repeat','uturn_stop','nav2_exit','costmap_exit','path_detour','uturn_right_clear')


def run_case(name,root,scene_override=None,exercise=None,launch_args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from wheelchair_interfaces.msg import DirectionEvent,InputHealth,NavigationIntentStatus,IntentAck,ExecutionStatus
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ros_gz_interfaces.msg import Contacts
    scene=('uturn_right_clear' if name=='uturn_right_clear' else 'straight_wall' if name in ('uturn_open','uturn_repeat','uturn_stop') else 'passage_090' if name=='uturn_refused' else 'path_obstacle' if name=='path_detour' else 'door_100' if name=='straight_door' else 'side_door' if name=='right' else 'intent_two_left' if name=='two_left' else
           'intent_blocked_first' if name=='blocked_first' else 'intent_behind' if name=='behind_entry' else 'passage_110' if name=='no_entry' else
           'straight_wall' if name=='open_area' else 'passage_branch' if name=='branch' else
           'intent_narrow_turn' if name=='narrow_turn' else 'intent_left')
    if scene_override is not None:scene=scene_override
    folder=root/name;folder.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID='94',ROS_LOCALHOST_ONLY='1',IGN_PARTITION='wheelchair_direction_'+uuid.uuid4().hex[:10])
    os.environ['ROS_DOMAIN_ID']='94';os.environ['ROS_LOCALHOST_ONLY']='1'
    log=(folder/'gazebo.log').open('w');timeline=(folder/'trace.jsonl').open('w')
    process=subprocess.Popen(['ros2','launch','wheelchair_control','shared.launch.py','gui:=false',
        'interaction_mode:=directional_intent','input_source:=keyboard','scene:='+scene]+list(launch_args or []),
        stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('direction_validation',parameter_overrides=[Parameter('use_sim_time',value=True)])
    ep=node.create_publisher(DirectionEvent,'/wheelchair/keyboard/direction_event',10)
    hp=node.create_publisher(InputHealth,'/wheelchair/keyboard/health',1)
    data=dict(status=None,pose=(0.,0.,0.),v=0.,w=0.,contacts=0,health=True,phase=[],entries=set(),samples=[],reasons=[],compute=[])
    source=uuid.uuid4().hex;sequence=0;event_id=0;next_health=0.
    result=dict(case=name,status='FAIL',scene=scene,simulated_only=True)
    keyboard_process=None
    def record(kind,**values):timeline.write(json.dumps(dict(values,monotonic_s=time.monotonic(),kind=kind))+'\n')
    def status(m):
        data['status']=m;data['phase'].append(m.phase);data['reasons'].append(m.reason)
        if m.entry_id:data['entries'].add(m.entry_id)
        record('status',phase=m.phase,reason=m.reason,entry=m.entry_id,event=m.event_id,task=m.task_id)
    def odom(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        data['pose']=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        data['v']=m.twist.twist.linear.x;data['w']=m.twist.twist.angular.z
        data['samples'].append(data['pose']);record('odom',pose=data['pose'],v=data['v'],w=data['w'])
    def contact(m):data['contacts']+=sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in m.contacts)
    def trace(m):
        d=json.loads(m.data)
        if event_id and (d['reason']=='PLANNING_TIMEOUT_FAULT' or d.get('executor_reason')=='ACTION_STALE'):
            data.setdefault('runtime_faults',[]).append((d['time'],d['reason'],d.get('executor_reason')))
        data['compute'].append(d['compute_ms']);data.setdefault('actions',[]).append(d['action']);
        search=d.get('candidate',{}).get('path_search',{})
        if search:data.setdefault('path_searches',[]).append(search)
        gap=d.get('candidate',{}).get('candidate',{}).get('clearance')
        if gap is not None and math.isfinite(gap):data.setdefault('clearances',[]).append(gap)
        record('decision',**d)
    node.create_subscription(NavigationIntentStatus,'/wheelchair/navigation_status',status,1)
    node.create_subscription(Odometry,'/wheelchair/odom',odom,qos_profile_sensor_data)
    node.create_subscription(Contacts,'/wheelchair/sim/contacts',contact,qos_profile_sensor_data)
    node.create_subscription(String,'/wheelchair/decision_trace',trace,1)
    node.create_subscription(String,'/wheelchair/executor_timing',lambda m:record('executor_timing',**json.loads(m.data)),10)
    node.create_subscription(IntentAck,'/wheelchair/intent_ack',lambda m:record('ack',event=m.event_id,status=m.status,reason=m.reason),10)
    def spin(seconds,predicate=None):
        nonlocal sequence,next_health
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_health and data['health']:
                sequence+=1;h=InputHealth();h.header.stamp=node.get_clock().now().to_msg()
                h.protocol_version=1;h.source_session_id=source;h.sequence=h.sample_sequence=sequence
                h.source_valid=True;h.source_age_ms=0.;h.reception_timed=True;h.ttl_ms=250;hp.publish(h);next_health=now+.05
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def event(kind):
        nonlocal event_id
        event_id+=1;m=DirectionEvent();m.header.stamp=node.get_clock().now().to_msg();m.protocol_version=1
        m.source_session_id=source;m.event_id=event_id;m.kind=kind;m.ttl_ms=1500;ep.publish(m)
        record('event',event=event_id,command=kind)
        return m
    def stopped():return abs(data['v'])<.015 and abs(data['w'])<.025
    ready_since=None
    def startup_ready():
        nonlocal ready_since
        if data['status'] is None or not data['status'].can_start:
            ready_since=None
            return False
        if ready_since is None:ready_since=time.monotonic()
        # Gazebo's first rendered scans can pause briefly during startup. Arm
        # only after continuous readiness; runtime stale-input guards stay intact.
        return time.monotonic()-ready_since>=1.
    try:
        assert spin(18,startup_ready),'startup not stably ready'
        if exercise is not None:
            from types import SimpleNamespace
            exercise(SimpleNamespace(data=data,spin=spin,event=event,stopped=stopped,record=record))
            assert data['contacts']==0,'physical collision'
            result['status']='PASS'
            return result
        if name=='keyboard_exit':
            # Exercise the actual interactive executable in a controlling PTY,
            # including terminal foreground health and its X -> STOP path.
            data['health']=False
            keyboard_process=subprocess.Popen(['script','-q','-c',str(Path('scripts/direction_keyboard.sh').resolve()),'/dev/null'],
                stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                env=dict(env,WHEELCHAIR_ROS_DOMAIN_ID='94'))
            assert spin(6,lambda:data['status'].source_session_id!=source and data['status'].can_start),'keyboard heartbeat not ready'
            keyboard_process.stdin.write(b'w');keyboard_process.stdin.flush()
            assert spin(4,lambda:abs(data['v'])>.1),'interactive W did not start'
            keyboard_process.stdin.write(b'a');keyboard_process.stdin.flush()
            assert spin(3,lambda:data['status'].phase=='APPROACH_ENTRY'),'interactive A did not select entry'
            keyboard_process.stdin.write(b'x');keyboard_process.stdin.flush()
            assert spin(4,lambda:keyboard_process.poll() is not None and stopped()),'keyboard X did not exit and stop'
            pose=data['pose'];spin(.5);assert math.dist(pose[:2],data['pose'][:2])<.03
            assert data['contacts']==0
            result['status']='PASS'
            return result
        event(2);spin(.2);assert stopped(),'A alone started chair'
        event(1)
        assert spin(4,lambda:abs(data['v'])>.1 or (name=='path_detour' and abs(data['w'])>.1)),'W did not start'
        if name.startswith('uturn_'):
            event(6)
            if name=='uturn_repeat':
                for _ in range(3):spin(.1);event(6)
            if name=='uturn_refused':
                assert spin(5,lambda:data['status'].phase=='BLOCKED'),'narrow U-turn was not refused'
                assert abs(data['pose'][2])<.1
            elif name=='uturn_stop':
                assert spin(6,lambda:abs(data['w'])>.1),'U-turn did not rotate'
                event(5);assert spin(4,stopped),'U-turn STOP did not brake'
                heading=data['pose'][2];spin(.5);assert abs(data['pose'][2]-heading)<.03
            else:
                assert spin(22,lambda:data['status'].phase=='CRUISE' and abs(data['pose'][2])>3.05),'U-turn did not complete'
                assert abs(abs(data['pose'][2])-math.pi)<.04
                if name=='uturn_right_clear':assert next(p[2] for p in data['samples'] if abs(p[2])>.2)<0,'did not choose the feasible right pivot'
                x=data['pose'][0];assert spin(4,lambda:data['pose'][0]<x-.15),'did not cruise after U-turn'
            assert data['contacts']==0;result['status']='PASS';return result
        if name in ('nav2_exit','costmap_exit'):
            executable='discrete_path_planner' if name=='nav2_exit' else 'wheelchair_costmap'
            os.kill(owned_pid(process.pid,executable),9)
            assert spin(4,stopped),'Nav2 component loss failed to stop'
            pose=data['pose'];spin(.5);assert math.dist(pose[:2],data['pose'][:2])<.03
            assert data['contacts']==0;result['status']='PASS';return result
        if name=='path_detour':
            assert spin(35,lambda:data['pose'][0]>2.1),'generic path did not pass obstacle'
            assert max(abs(p[1]) for p in data['samples'])>.45,'no lateral avoidance'
            assert data['contacts']==0;result['status']='PASS';return result
        if name=='near_left':assert spin(6,lambda:data['pose'][0]>1.2),'did not approach'
        if name=='behind_entry':
            event(2);spin(9.)
            assert max(p[0] for p in data['samples'])>2.,'no forward search'
            assert not data['entries'] and abs(data['pose'][2])<.15,'selected an entry behind'
        elif name=='branch':
            assert spin(8,lambda:data['status'].phase=='BLOCKED'),'ambiguous branch did not stop'
        elif name in ('input_exit','manager_exit','planner_exit','executor_exit'):
            event(2);spin(.4)
            if name=='input_exit':data['health']=False
            else:
                executable={'manager_exit':'direction_manager','planner_exit':'safety_gate','executor_exit':'executor'}[name]
                os.kill(owned_pid(process.pid,executable),9)
            assert spin(4,stopped),'process loss failed to stop'
            pose=data['pose'];spin(.5);assert math.dist(pose[:2],data['pose'][:2])<.03
        elif name=='straight_door':
            assert spin(12,lambda:data['pose'][0]>2.4 and data['status'].phase=='CRUISE'),'straight door did not continue'
            assert not data['entries'] and abs(data['pose'][2])<.1
        elif name in ('stop','stop_turn','stop_traverse'):
            event(2)
            if name=='stop':spin(.6)
            elif name=='stop_turn':
                assert spin(12,lambda:data['status'].phase=='ALIGN_ENTRY' and abs(data['w'])>.1),'no angular motion'
            else:
                assert spin(20,lambda:data['status'].phase=='TRAVERSE' and abs(data['v'])>.1),'no traversing motion'
            event(5)
            assert spin(3,stopped),'STOP did not brake'
            pose=data['pose'];event(2);spin(1.);assert math.dist(pose[:2],data['pose'][:2])<.03,'direction restarted STOP'
        elif name=='replace':
            event(2);spin(.4);old=data['status'].task_id;event(3)
            assert spin(3,lambda:data['status'].task_id!=old and data['status'].preference==-1),'replacement failed'
            assert data['pose'][1]<.2,'continued old left maneuver'
        elif name in ('no_entry','narrow_turn'):
            event(2);spin(9.)
            assert not data['entries'],'unreachable/nonexistent entry locked'
            assert abs(data['pose'][2])<.15,'unrequested rotation'
            assert data['status'].preference==0 or data['status'].phase=='BLOCKED'
        elif name=='open_area':
            event(2)
            assert spin(9,lambda:data['status'].phase=='CRUISE' and data['pose'][2]>.28),'finite open heading failed'
            yaw=data['pose'][2];spin(.5);assert abs(data['pose'][2]-yaw)<.08
        else:
            m=event(3 if name=='right' else 2)
            if name=='repeat':
                for _ in range(5):ep.publish(m);event(2);spin(.1)
            if name=='w_during_turn':
                assert spin(10,lambda:data['status'].phase=='ALIGN_ENTRY'),'no turn phase'
                task=data['status'].task_id;event(1);spin(.2);assert data['status'].task_id==task,'W reset turn'
            assert spin(23,lambda:data['status'].phase=='CRUISE' and abs(data['pose'][1])>1.6),'entry did not complete'
            assert len(data['entries'])==1,'entry changed after lock'
            assert 'APPROACH_ENTRY' in data['phase'] and 'ALIGN_ENTRY' in data['phase'] and 'TRAVERSE' in data['phase']
            x,y,yaw=data['pose'];assert y<0 if name=='right' else y>0
            if name=='two_left':assert x<1.8,'did not use nearest opening'
            if name=='blocked_first':assert x>2.,'did not skip blocked opening'
            spin(.5);assert data['status'].preference==0,'preference was not consumed'
        assert data['contacts']==0,'physical collision'
        result['status']='PASS'
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if keyboard_process is not None:stop_group(keyboard_process)
        if rclpy.ok():event(5);spin(.3)
        from wheelchair_sim.scenes import map_obstacles,boundary
        from wheelchair_control.geometry import transform
        from shapely.geometry import Polygon
        import shapely
        obstacles=shapely.union_all([Polygon(b.polygon()) for b in map_obstacles(scene)])
        edge=Polygon(boundary(scene)).boundary
        footprint=((-0.45,-.30),(.55,-.30),(.55,.30),(-.45,.30))
        gaps=[min(Polygon(transform(footprint,p)).distance(obstacles),Polygon(transform(footprint,p)).distance(edge)) for p in data['samples']]
        result['minimum_actual_clearance_m']=min(gaps,default=None)
        result.update(final_pose=data['pose'],contacts=data['contacts'],entries=sorted(data['entries']),
                      phases=list(dict.fromkeys(data['phase'])),last_reasons=data['reasons'][-8:],
                      max_compute_ms=max(data['compute'],default=0.),samples=len(data['samples']),
                      path_length_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(data['samples'],data['samples'][1:])),
                      action_switches=sum(a!=b for a,b in zip(data.get('actions',[]),data.get('actions',[])[1:])),
                      stop_transitions=sum(a!=0 and b==0 for a,b in zip(data.get('actions',[]),data.get('actions',[])[1:])),
                      minimum_predicted_clearance_m=min(data.get('clearances',[]),default=None),input_events=event_id)
        searches=data.get('path_searches',[])
        result['max_path_search_ms']=max((s.get('compute_ms',0.) for s in searches),default=0.)
        result['max_path_search_expansions']=max((s.get('expansions',0) for s in searches),default=0)
        node.destroy_node();rclpy.try_shutdown();stop_group(process);log.close();timeline.close()
        shutdown_log=(folder/'gazebo.log').read_text()
        costmap_exit=re.findall(r'\[wheelchair_costmap-\d+\]: process has died .*?exit code (-?\d+)',shutdown_log)
        costmap_clean=bool(re.search(r'\[wheelchair_costmap-\d+\]: process has finished cleanly',shutdown_log))
        result['costmap_clean_shutdown']=costmap_clean
        result['costmap_exit_codes']=[int(code) for code in costmap_exit]
        if name!='costmap_exit' and not costmap_clean:
            result['status']='FAIL'
            result['error']=result.get('error','Costmap did not shut down cleanly')
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',choices=(*CASES,'all'),default='all')
    p.add_argument('--output',default='artifacts/directional');args=p.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    results=[]
    for name in CASES if args.case=='all' else (args.case,):
        result=run_case(name,root);results.append(result);print(json.dumps(result),flush=True)
    (root/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    raise SystemExit(0 if all(r['status']=='PASS' for r in results) else 1)

if __name__=='__main__':main()
