#!/usr/bin/env python3
"""Existing monocular detector -> eye direction adapter, with an absent test center.
No simulator, motion executor or chassis is started. User reference files are untouched.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
from process_cleanup import stop_group


def main():
    os.environ.update(ROS_DOMAIN_ID='97',ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from wheelchair_interfaces.msg import EyeSample,DirectionEvent,NavigationIntentStatus,InputHealth
    from eye_shared_control.node import EyeIntentNode
    root=Path(__file__).resolve().parents[1]
    output=root/'artifacts/eye-3d-interface-20261014';output.mkdir(parents=True,exist_ok=True)
    eyetle=Path(os.environ.get('EYETLE_WORKSPACE',root.parent/'projects/EYETLE'))
    rclpy.init(args=['--ros-args','-p','interaction_mode:=directional_intent',
                    '-p','eye_sample_mode:=stamped','-p','use_sim_time:=false'])
    adapter=EyeIntentNode();probe=Node('eye_3d_transport_probe')
    executor=SingleThreadedExecutor();executor.add_node(adapter);executor.add_node(probe)
    samples=[];events=[];health=[]
    probe.create_subscription(EyeSample,'/eye_tracking/samples',lambda m:samples.append(dict(
        session=m.source_session_id,sequence=m.sequence,valid=m.valid,closed=m.closed,horizontal=m.horizontal)),10)
    probe.create_subscription(DirectionEvent,'/wheelchair/eye/direction_event',lambda m:events.append(m.kind),10)
    probe.create_subscription(InputHealth,'/wheelchair/eye/health',lambda m:health.append(m.source_valid),10)
    status=probe.create_publisher(NavigationIntentStatus,'/wheelchair/navigation_status',1)
    log=(output/'live-transport.log').open('w');child=None;result=dict(status='FAIL',no_motion_process=True)
    try:
        with tempfile.TemporaryDirectory(prefix='eye-3d-unconfirmed-') as tmp:
            env=dict(os.environ,EYE_ROS_DOMAIN_ID='97',EYE_WHEELCHAIR_SETUP=str(root/'install/setup.bash'))
            child=subprocess.Popen([str(eyetle/'start_ritnet.sh'),
                '--center-file',str(Path(tmp)/'absent-center.json'),'--publish-stamped','--no-display',
                '--max-frames','90','--report',str(output/'live-detector.json')],env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+45;seq=0;next_status=0.
            while time.monotonic()<deadline and child.poll() is None:
                if time.monotonic()>=next_status:
                    seq+=1;m=NavigationIntentStatus();m.header.stamp=probe.get_clock().now().to_msg()
                    m.manager_session_id='isolated-test-manager';m.sequence=seq;m.phase='STOPPED';m.can_start=True
                    status.publish(m);next_status=time.monotonic()+.05
                executor.spin_once(timeout_sec=.01)
            for _ in range(20):executor.spin_once(timeout_sec=.01)
            assert child.poll()==0,'detector failed or timed out; see live-transport.log'
            assert len(samples)>=20,'insufficient EyeSample messages'
            assert all(not m['valid'] for m in samples),'unconfirmed 3D reference emitted valid input'
            assert all(b['sequence']>a['sequence'] and b['session']==a['session'] for a,b in zip(samples,samples[1:])), 'source ordering changed'
            assert events and set(events)<={5},'unconfirmed 3D input authorized movement'
            assert health and not any(health),'unconfirmed 3D source declared healthy'
            result['status']='PASS'
    except Exception as exc:result['error']=str(exc)
    finally:
        if child is not None:stop_group(child)
        executor.shutdown();adapter.destroy_node();probe.destroy_node();rclpy.shutdown();log.close()
        result.update(samples=len(samples),valid_samples=sum(m['valid'] for m in samples),direction_events=events)
        (output/'live-transport-result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    return 0 if result['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
