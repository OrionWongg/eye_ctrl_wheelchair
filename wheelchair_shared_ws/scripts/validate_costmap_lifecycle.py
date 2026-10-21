#!/usr/bin/env python3
"""Exercise the standalone costmap lifecycle and owned-process signal exits."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from lifecycle_msgs.srv import ChangeState
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from process_cleanup import stop_group


def main():
    os.environ['ROS_DOMAIN_ID']='95'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    folder=Path('artifacts/nav2-costmap-lifecycle');folder.mkdir(parents=True,exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('costmap_lifecycle_validation')
    clock_pub=node.create_publisher(Clock,'/clock',10)
    def clock_tick():
        message=Clock();stamp=time.monotonic()
        message.clock.sec=int(stamp);message.clock.nanosec=int((stamp-int(stamp))*1e9)
        clock_pub.publish(message)
    node.create_timer(.02,clock_tick)
    client=node.create_client(ChangeState,'/wheelchair/local_costmap/local_costmap/change_state')
    results=[]
    tf_log=(folder/'tf.log').open('w')
    tf=subprocess.Popen(['ros2','run','tf2_ros','static_transform_publisher',
        '--frame-id','odom','--child-frame-id','base_link'],stdout=tf_log,stderr=subprocess.STDOUT,start_new_session=True)
    def transition(identifier):
        request=ChangeState.Request();request.transition.id=identifier
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        assert future.done() and future.result().success,f'transition {identifier} failed'
    try:
        for index,sig in enumerate((signal.SIGINT,signal.SIGTERM,signal.SIGTERM,signal.SIGINT)):
            with (folder/f'run-{index}.log').open('w') as log:
                process=subprocess.Popen(['install/wheelchair_nav2/lib/wheelchair_nav2/wheelchair_costmap',
                    '--ros-args','-p','use_sim_time:=true','-p','global_frame:=odom',
                    '-p','robot_base_frame:='+('missing_frame' if index==3 else 'base_link'),'-p','plugins:=[static_layer,obstacle_layer,inflation_layer]',
                    '-p','static_layer.plugin:=nav2_costmap_2d::StaticLayer',
                    '-p','obstacle_layer.plugin:=nav2_costmap_2d::ObstacleLayer',
                    '-p','obstacle_layer.observation_sources:=scan',
                    '-p','obstacle_layer.scan.topic:=/scan',
                    '-p','obstacle_layer.scan.data_type:=LaserScan',
                    '-p','inflation_layer.plugin:=nav2_costmap_2d::InflationLayer'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                result=dict(signal=sig.name,lifecycle_shutdown=index==2,interrupt_activation=index==3,status='FAIL')
                try:
                    assert client.wait_for_service(timeout_sec=5.),'service unavailable'
                    if index==3:
                        transition(1)
                        request=ChangeState.Request();request.transition.id=3
                        client.call_async(request)
                        time.sleep(.2)
                    else:
                        for _ in range(2):
                            transition(1);transition(3);time.sleep(.2)
                            transition(4);transition(2)
                        transition(1);transition(3)
                        if index==2:transition(7)
                    # No more spins: /clock is frozen while the costmap exits.
                    process.send_signal(sig)
                    result['exit_code']=process.wait(timeout=5.)
                    assert result['exit_code']==0,'abnormal process exit'
                    result['status']='PASS'
                except Exception as exc:
                    result['error']=str(exc)
                finally:
                    stop_group(process)
                results.append(result)
                print(json.dumps(result),flush=True)
                time.sleep(.3)
    finally:
        stop_group(tf);tf_log.close();node.destroy_node();rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    return 0 if len(results)==4 and all(r['status']=='PASS' for r in results) else 1

if __name__=='__main__':raise SystemExit(main())
