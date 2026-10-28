"""Exercise navigation against the real iPad keyboard drag path, without hardware."""
from pathlib import Path
import time
import json
from types import SimpleNamespace
from std_msgs.msg import String
import rclpy
from geometry_msgs.msg import TwistStamped
from whill_controller.controller_node import WhillControllerNode


def test_navigation_drag_stop_watchdog_and_replay():
    calibration=Path(__file__).parents[1]/'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args','-p','hardware_enabled:=false','-p','whill_mode:=true',
                    '-p','keyboard_auto_drag:=true','-p','navigation_control_enabled:=true',
                    '-p',f'calibration_file:={calibration}'])
    node=None
    try:
        node=WhillControllerNode()
        end=time.monotonic()+3.
        while time.monotonic()<end:node.root.update();time.sleep(.005)
        assert not node.calibrating
        writes=[];original=node._write_command
        def record(payload,**kwargs):writes.append(payload);return original(payload,**kwargs)
        node._write_command=record
        def command(v=0.,w=0.):
            msg=TwistStamped();msg.header.frame_id='base_link'
            msg.header.stamp=node.get_clock().now().to_msg()
            msg.twist.linear.x=v;msg.twist.angular.z=w
            node.on_navigation_command(msg);return msg
        forwarded=[]
        node.create_subscription(String, '/wheelchair/keyboard/gui_key',
                                 lambda m: forwarded.append(json.loads(m.data)), 10)
        def key(k):
            node.on_key_press(SimpleNamespace(keysym=k))
            end=time.monotonic()+.03
            while time.monotonic()<end:
                node.root.update();time.sleep(.005)
        key('w');assert not forwarded and not writes
        key('space');assert forwarded[-1]['key']==' ' and not writes
        command(.372);assert not writes  # Local Arm remains mandatory.
        command()
        node.toggle_keyboard_arm()
        for k in ('W','a','d','u','c','s','space'):
            key(k)
            assert forwarded[-1]['key']==(' ' if k=='space' else k.lower())
            assert not writes  # GUI gestures never directly drive the iPad.
            command()  # Keep the navigation watchdog supplied during this test.
        command(.372);assert writes==[b'[',b'w'*8] and node.click_held
        for _ in range(4):command(.372)
        assert writes==[b'[',b'w'*8]  # Refresh without cumulative pointer drift.
        command();assert writes[-2:]==[b']',b's'*8] and not node.click_held
        assert node.keyboard_armed  # A normal stop does not require re-arming.
        command(w=.389);assert node.click_held
        node.navigation_received_at=time.monotonic()-.21
        node.update_navigation_drive()
        assert not node.click_held and not node.keyboard_armed and writes[-1]==b']'
        count=len(writes);command(.372);assert len(writes)==count
        node.toggle_keyboard_arm();msg=command(.372);assert node.click_held
        node.on_navigation_command(msg)  # Duplicate timestamp is not a new lease.
        assert not node.click_held and not node.keyboard_armed
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()


def test_hold_navigation_releases_locally_and_rejects_late_drive(monkeypatch):
    calibration=Path(__file__).parents[1]/'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args','-p','hardware_enabled:=false','-p','whill_mode:=true',
                    '-p','keyboard_auto_drag:=true','-p','navigation_control_enabled:=true',
                    '-p','navigation_hold_to_run:=true','-p',f'calibration_file:={calibration}'])
    node=None
    try:
        node=WhillControllerNode()
        end=time.monotonic()+3.
        while time.monotonic()<end:node.root.update();time.sleep(.005)
        writes=[];original=node._write_command
        def record(payload,**kwargs):writes.append(payload);return original(payload,**kwargs)
        node._write_command=record
        def command(v=0.):
            msg=TwistStamped();msg.header.frame_id='base_link'
            msg.header.stamp=node.get_clock().now().to_msg();msg.twist.linear.x=v
            node.on_navigation_command(msg)
        physically_down=set()
        monkeypatch.setattr(node.physical_keyboard,'is_down',lambda key:key in physically_down)
        command();node.toggle_keyboard_arm()
        command(.372);assert not writes  # A planner command alone cannot drive.
        physically_down.add('w')
        node.on_key_press(SimpleNamespace(keysym='w'))
        command(.372);assert node.click_held and writes==[b'[',b'w'*8]
        for _ in range(20):command(.372)
        assert writes==[b'[',b'w'*8]  # Continuous press, no click/re-drag pulses.
        # Delayed auto-repeat press used to arrive AFTER the 12 ms release timer.
        node.on_key_release(SimpleNamespace(keysym='w'))
        end=time.monotonic()+.06
        while time.monotonic()<end:
            command(.372);node.root.update();time.sleep(.005)
        assert node.click_held and writes==[b'[',b'w'*8]
        node.on_key_press(SimpleNamespace(keysym='w'))
        assert node.navigation_keys_held=={'w'} and node.click_held
        physically_down.clear()
        node.on_key_release(SimpleNamespace(keysym='w'))
        end=time.monotonic()+.03
        while time.monotonic()<end:node.root.update();time.sleep(.002)
        assert not node.click_held and writes[-2:]==[b']',b's'*8]
        count=len(writes);command(.372)
        assert len(writes)==count  # Late upstream motion cannot undo key-up.
        assert node.keyboard_armed
        physically_down.add('w')
        node.on_key_press(SimpleNamespace(keysym='w'));command(.372)
        assert node.click_held
        node.on_key_press(SimpleNamespace(keysym='s'))
        assert not node.click_held
        node.on_key_press(SimpleNamespace(keysym='w'));command(.372)
        assert not node.click_held  # OS repeats cannot cancel an explicit stop.
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()


def test_rviz_lease_drives_without_key_and_stop_revokes_it():
    calibration=Path(__file__).parents[1]/'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args','-p','hardware_enabled:=false','-p','whill_mode:=true',
                    '-p','keyboard_auto_drag:=true','-p','navigation_control_enabled:=true',
                    '-p','navigation_hold_to_run:=true','-p','rviz_goal_authority_enabled:=true',
                    '-p',f'calibration_file:={calibration}'])
    node=None
    try:
        node=WhillControllerNode()
        end=time.monotonic()+3.
        while time.monotonic()<end:node.root.update();time.sleep(.005)
        def command(v=0.):
            msg=TwistStamped();msg.header.frame_id='base_link'
            msg.header.stamp=node.get_clock().now().to_msg();msg.twist.linear.x=v
            node.on_navigation_command(msg)
        def authority():
            node.on_goal_authority(String(data=json.dumps(dict(
                stamp=node.get_clock().now().nanoseconds/1e9,task_id='rviz:test'))))
        command();node.toggle_keyboard_arm()
        command(.372);assert not node.click_held
        authority();command(.372)
        assert node.click_held and not node.navigation_keys_held
        node.on_key_press(SimpleNamespace(keysym='s'))
        assert not node.click_held
        authority();command(.372)
        assert not node.click_held  # Late authorization cannot undo local stop.
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()


def test_real_x11_state_ignores_delayed_repeat_and_detects_keyup():
    """XTest changes the X server bitmap, unlike Tk event_generate/key handlers."""
    import ctypes
    from ctypes.util import find_library
    calibration=Path(__file__).parents[1]/'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args','-p','hardware_enabled:=false','-p','whill_mode:=true',
                    '-p','keyboard_auto_drag:=true','-p','navigation_control_enabled:=true',
                    '-p','navigation_hold_to_run:=true','-p',f'calibration_file:={calibration}'])
    node=None;xtst=None;code=None
    try:
        node=WhillControllerNode()
        end=time.monotonic()+3.
        while time.monotonic()<end:node.root.update();time.sleep(.005)
        keyboard=node.physical_keyboard
        xtst=ctypes.CDLL(find_library('Xtst') or 'libXtst.so.6')
        xtst.XTestFakeKeyEvent.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]
        keyboard.x11.XSync.argtypes=[ctypes.c_void_p,ctypes.c_int]
        code=keyboard.keycodes['w']
        def physical(down):
            xtst.XTestFakeKeyEvent(keyboard.display,code,int(down),0)
            keyboard.x11.XSync(keyboard.display,0)
        def command(v=0.):
            msg=TwistStamped();msg.header.frame_id='base_link'
            msg.header.stamp=node.get_clock().now().to_msg();msg.twist.linear.x=v
            node.on_navigation_command(msg)
        command();node.toggle_keyboard_arm();node.root.focus_force();node.root.update()
        physical(True);assert keyboard.is_down('w')
        node.root.update();command(.372)
        assert node.navigation_keys_held=={'w'} and node.click_held
        # A fake release followed by a late repeat press must remain held.
        node.on_key_release(SimpleNamespace(keysym='w'))
        end=time.monotonic()+.12
        while time.monotonic()<end:
            command(.372);node.root.update();time.sleep(.005)
        assert node.click_held and node.navigation_keys_held=={'w'}
        node.on_key_press(SimpleNamespace(keysym='w'))
        # Even if Tk loses the true release event, periodic physical polling stops.
        physical(False);assert not keyboard.is_down('w')
        node.navigation_hold_tick()
        assert not node.navigation_keys_held and not node.click_held
        command(.372);assert not node.click_held
    finally:
        if node is not None:
            if xtst is not None and code is not None and node.physical_keyboard.display:
                xtst.XTestFakeKeyEvent(node.physical_keyboard.display,code,0,0)
                node.physical_keyboard.x11.XSync(node.physical_keyboard.display,0)
            node.close();node.destroy_node()
        rclpy.try_shutdown()
