#!/usr/bin/env python3
"""Publish synthetic eye events to an isolated, already running simulation.

This verifies ROS and physics, not recognition accuracy on a person.
Start a fresh simulation and an eye console before running this script.
"""
import json
import math
from pathlib import Path
import time
import uuid

import cv2
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from wheelchair_sim.first_person_view import decode_image


def main():
    rclpy.init()
    node=rclpy.create_node('whill_synthetic_eye_validation')
    latest={}
    node.create_subscription(Odometry,'/wheelchair/odom',lambda msg:latest.update(odom=msg),10)
    node.create_subscription(Image,'/wheelchair/first_person/image_raw',lambda msg:latest.update(image=msg),10)
    pub=node.create_publisher(String,'/eye_tracking/fusion_samples',1)
    session=str(uuid.uuid4())
    sequence=0

    def stream(seconds,horizontal=0.,event_id=None,valid=True):
        nonlocal sequence
        end=time.monotonic()+seconds
        event_end=time.monotonic()
        while time.monotonic()<end:
            now=time.monotonic()
            sequence+=1
            event=None
            if event_id is not None and now-event_end<.15:
                event=dict(id=event_id,start=event_end-.12,end=event_end,durations=[.12,.12])
            payload=dict(schema=1,session=session,sequence=sequence,stamp_ns=time.time_ns(),
                source_t=now,horizontal=horizontal,fresh=True,calibrated=True,open=valid,
                motion_valid=valid,gesture_continuity=True,long_close=False,blink_event=event)
            pub.publish(String(data=json.dumps(payload)))
            until=time.monotonic()+.05
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.01)

    def pose():
        p=latest['odom'].pose.pose
        q=p.orientation
        return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    try:
        stream(2.)
        assert 'odom' in latest and 'image' in latest,'Simulation IO missing'
        start=pose()
        stream(.2,event_id=1)
        stream(.15)
        one_blink=pose()
        stream(.2,event_id=2)
        stream(1.8)
        forward=pose()
        stream(1.6,horizontal=-.9)
        turn=pose()
        stream(1.,valid=False)
        stopped=pose()
        stream(.8)
        recovered_eyes=pose()
        result=dict(input='SYNTHETIC schema=1 samples; not human eye recognition',
            start=start,after_one_blink=one_blink,forward=forward,turn=turn,stopped=stopped,
            after_valid_input_without_blink=recovered_eyes,
            first_blink_distance=math.dist(start[:2],one_blink[:2]),
            forward_distance=math.dist(one_blink[:2],forward[:2]),
            turn_radians=abs(turn[2]-forward[2]),
            stopped_drift=math.dist(stopped[:2],recovered_eyes[:2]))
        result['success']=(result['first_blink_distance']<.02 and result['forward_distance']>.15
                           and result['turn_radians']>.25 and result['stopped_drift']<.02)
        out=Path(__file__).resolve().parents[1]/'reports'
        out.mkdir(exist_ok=True)
        (out/'eye-sim-transport.json').write_text(json.dumps(result,indent=2)+'\n')
        cv2.imwrite('/tmp/whill-simulation-first-person.png',decode_image(latest['image']))
        print(json.dumps(result,indent=2))
        assert result['success']
    finally:
        stream(.2,valid=False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
