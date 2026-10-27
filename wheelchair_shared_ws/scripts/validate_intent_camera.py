#!/usr/bin/python3
"""Real camera -> intent-confirm -> simulation transport smoke, no human labels.

This is NOT the human control/usability acceptance. No images are saved here.
Owns only its isolated domain-99 simulator/tracker processes.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='99'
    assert os.environ.get('IGN_PARTITION')=='wheelchair_intent_camera_validation'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from wheelchair_interfaces.msg import EyeSample,InputHealth,TaskStatus,IntentEvent
    root=Path(__file__).resolve().parents[1];out=root/'artifacts/intent-camera';out.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,WHEELCHAIR_ROS_DOMAIN_ID='99',WHEELCHAIR_MODE='stop_only',
             WHEELCHAIR_INTERACTION_MODE='intent_confirm',WHEELCHAIR_GUI='false',WHEELCHAIR_SCENE='straight_wall')
    log=(out/'runtime.log').open('w')
    rclpy.init();node=Node('camera_intent_transport_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
    samples=[];health=[];tasks=[];events=[]
    def sample(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        samples.append(dict(session=m.source_session_id,sequence=m.sequence,source_wall=stamp,
                            receive_wall=time.time(),age_s=time.time()-stamp,valid=m.valid,closed=m.closed))
    node.create_subscription(EyeSample,'/eye_tracking/samples',sample,qos_profile_sensor_data)
    node.create_subscription(InputHealth,'/wheelchair/eye/health',lambda m:health.append(dict(valid=m.source_valid,reception_timed=m.reception_timed,source_age_ms=m.source_age_ms,sample_sequence=m.sample_sequence)),10)
    node.create_subscription(TaskStatus,'/wheelchair/task_status',lambda m:tasks.append(dict(task_id=m.task_id,state=m.state,reason=m.reason)),10)
    node.create_subscription(IntentEvent,'/wheelchair/eye/event',lambda m:events.append(dict(event_id=m.event_id,kind=m.kind)),10)
    process=subprocess.Popen(['/usr/bin/python3',str(root/'scripts/run_eye_sim.py'),'--eyes','both','--no-display',
                              '--max-frames','150','--report',str(out/'tracker.json')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report=dict(status='FAIL',validation='camera_transport_only',human_labeled_trials=0)
    try:
        deadline=time.monotonic()+90
        while time.monotonic()<deadline and process.poll() is None:rclpy.spin_once(node,timeout_sec=.02)
        assert process.poll() is not None,'Camera transport run did not exit in 90 s'
        assert process.returncode==0,('Camera/launch failed',process.returncode)
        assert len(samples)>=20,('Too few stamped camera frames',len(samples))
        assert all(b['sequence']>a['sequence'] and b['source_wall']>=a['source_wall'] for a,b in zip(samples,samples[1:])), 'Source replay/time reversal'
        assert any(not h['reception_timed'] and h['sample_sequence']>0 for h in health),'Stamped health adapter not observed'
        assert tasks,'Task status link not observed'
        report.update(status='PASS',stamped_samples=len(samples),valid_camera_samples=sum(s['valid'] for s in samples),
                      sample_age_min_s=min(s['age_s'] for s in samples),sample_age_max_s=max(s['age_s'] for s in samples),
                      motion_events=sum(e['kind'] in (1,2) for e in events),
                      note='Source camera-read timestamp, not hardware exposure. No human intent accuracy or usability claim.')
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'observations.json').write_text(json.dumps(dict(samples=samples,health=health,tasks=tasks,events=events)))
        print(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown();log.close()

if __name__=='__main__':main()
