"""Isolated ROS transport check; do not run in a domain with a real controller."""
import json
import os
from pathlib import Path
import time
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from eye_turtle_control.controller import EyeTurtleController,Motion
from blink_events import BlinkEvents

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='159':raise RuntimeError('Run this transport check only in isolated ROS_DOMAIN_ID=159')
    rclpy.init(args=['--ros-args','-p','blink_input_mode:=events','-p','blink_cooldown_ms:=120.0'])
    controller=EyeTurtleController();source=rclpy.create_node('fusion_smoke_source')
    pub=source.create_publisher(String,'/eye_tracking/fusion_samples',1)
    executor=SingleThreadedExecutor();executor.add_node(controller);executor.add_node(source)
    matcher=BlinkEvents(2);session=str(uuid.uuid4());sequence=0;events=0
    def emit(states,fresh=True):
        nonlocal sequence,events
        now=time.monotonic();event,long=matcher.update(states,now,fresh);events+=int(event is not None);sequence+=1
        opened=states==['open']*2
        d=dict(schema=1,session=session,sequence=sequence,stamp_ns=source.get_clock().now().nanoseconds,
            source_t=now,horizontal=0.,motion_valid=opened and fresh,fresh=fresh,calibrated=True,
            open=opened,gesture_continuity=matcher.continuity,long_close=long,blink_event=event)
        pub.publish(String(data=json.dumps(d)))
        deadline=time.monotonic()+.02
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.002)
    def phase(states,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:emit(states)
    try:
        deadline=time.monotonic()+5
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        if not pub.get_subscription_count():raise RuntimeError('No fusion subscription discovered')
        phase(['open']*2,.25)
        for _ in range(2):
            phase(['closed','open'],.08);phase(['closed']*2,.04);phase(['open','closed'],.08);phase(['open']*2,.14)
        active=controller.motion==Motion.FORWARD
        emit(['open']*2,False)
        stopped=controller.motion==Motion.STOPPED
        result=dict(events=events,enabled_after_double_blink=active,stopped_on_invalid=stopped,domain=159)
        Path(__file__).resolve().parents[1].joinpath('model-results/fusion-event-transport.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
        assert events==2 and active and stopped,result
    finally:
        controller.pause('TEST_FINISHED');controller.control_loop()
        executor.shutdown();source.destroy_node();controller.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
