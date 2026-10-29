"""Exercise actual ROS transport and existing HID generation without hardware."""
import time
from types import SimpleNamespace
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from ipad_mouse_velocity import MouseController
from whill_controller.controller_node import WhillControllerNode

rclpy.init();node=rclpy.create_node('test_mouse_transport')
p={'steering wheel center':(-16,-19),'left':(-24,-19),'left-forward':(-22,-25),'forward':(-16,-27),'right-forward':(-10,-25),'right':(-8,-19)}
obj=SimpleNamespace(dragging=True,full_points=p,calibration_coordinates=dict(p),keys_down=set(),movement_keys={'w'},
    feedback=SimpleNamespace(get=lambda:False),actual=np.zeros(2),calibrating=False,locked_current_coordinates=p['steering wheel center'],
    keyboard_auto_drag=True,click_held=False,link_status='',get_clock=node.get_clock,blockers=lambda:[],
    mapping_pub=SimpleNamespace(publish=lambda m:None),publish_event=lambda *a,**k:None,_set_display=lambda t:None,
    toggle_keyboard_arm=lambda: (_ for _ in ()).throw(AssertionError('Unexpected disarm')))
packets=[]
obj._write_command=lambda payload,**kw:packets.append(payload) or True
obj.vector=lambda:(0,-1) if 'w' in obj.keys_down else (0,0)
obj.move_locked_to_active_target=lambda:WhillControllerNode.move_locked_to_active_target(obj)
obj.update_keyboard_target=lambda:WhillControllerNode.update_keyboard_target(obj)
obj.stop_mouse=lambda:None
sub=node.create_subscription(TwistStamped,'/test_mouse/cmd_vel',lambda m:MouseController.on_cmd_vel(obj,m),1)
pub=node.create_publisher(TwistStamped,'/test_mouse/cmd_vel',1)
end=time.monotonic()+3
while pub.get_subscription_count()==0 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
assert pub.get_subscription_count()>0
msg=TwistStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='wheelchair_control_center_estimated';msg.twist.linear.x=.548
pub.publish(msg)
end=time.monotonic()+2
while not packets and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
assert packets==[b'[',b'wwwwwwww'],packets
obj.keys_down.clear();assert obj.update_keyboard_target()
assert packets[-2:]==[b']',b'ssssssss'],packets
print('PASS: ROS cmd_vel -> existing calibrated HID: press [, forward w*8, release ], center s*8. No serial opened.')
node.destroy_node();rclpy.shutdown()
