#!/usr/bin/python3
"""Exercise the actual keyboard adapter against isolated Gazebo, no desktop keys."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from wheelchair_control.inputs import Keyboard

assert os.environ.get('ROS_DOMAIN_ID')=='98' and os.environ.get('IGN_PARTITION')=='wheelchair_keyboard_test'
root=Path(__file__).resolve().parents[1]
log=open(root/'artifacts/direct-keyboard.log','w')
p=subprocess.Popen(['ros2','launch','wheelchair_control','shared.launch.py','gui:=false','scene:=straight_wall'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],signal_handler_options=SignalHandlerOptions.NO)
k=Keyboard();probe=Node('direct_keyboard_probe');executor=SingleThreadedExecutor();executor.add_node(k);executor.add_node(probe)
data={'odom':None,'contacts':0}
probe.create_subscription(Odometry,'/wheelchair/odom',lambda m:data.update(odom=m),qos_profile_sensor_data)
def contact(m):data['contacts']+=sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in m.contacts)
probe.create_subscription(Contacts,'/wheelchair/sim/contacts',contact,10)
def spin(seconds,key=None):
    end=time.monotonic()+seconds;last=0.
    while time.monotonic()<end:
        assert p.poll() is None,'Simulation exited'
        if key and time.monotonic()-last>.1:k.key(key);last=time.monotonic()
        executor.spin_once(timeout_sec=.01)
def wait_ready():
    end=time.monotonic()+35
    while time.monotonic()<end:
        spin(.1)
        if k.ready():return
    raise AssertionError(k.reason)
try:
    wait_ready();k.key('w');spin(8.,'w')
    assert data['odom'].pose.pose.position.x>.3,'Direct W never moved'
    assert k.locked and not k.armed,'Wall did not latch keyboard'
    stopped_x=data['odom'].pose.pose.position.x
    spin(1.,'w');assert abs(data['odom'].pose.pose.position.x-stopped_x)<.01
    wait_ready();k.key('r');assert not k.armed,'Recovery must not execute stale W'
    k.key('a');spin(7.,'a')
    assert k.armed and not k.locked and data['odom'].twist.twist.angular.z>.2,'Unexpected five-second stop'
    k.key(' ');spin(1.2);wait_ready();assert not k.locked
    k.key('a');spin(.8);assert k.armed and data['odom'].twist.twist.angular.z>.1,'User STOP required R'
    k.key(' ');spin(1.2)
    assert data['contacts']==0
    result={'status':'PASS','direct_W_without_R':True,'held_W_cannot_unlock_obstacle':True,'R_cancels_old_direction':True,'turn_continues_over_5_seconds':True,'manual_stop_then_direct_key':True,'robot_contacts':0}
    (root/'artifacts/result-direct-keyboard.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
finally:
    k.key(' ')
    try:spin(.15)
    except Exception:pass
    executor.shutdown();k.destroy_node();probe.destroy_node();rclpy.shutdown()
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=12)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    log.close()
