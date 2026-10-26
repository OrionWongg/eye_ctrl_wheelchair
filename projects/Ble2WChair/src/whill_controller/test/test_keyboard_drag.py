"""Exercise actual Tk key handlers without opening a hardware serial port."""
from pathlib import Path
from types import SimpleNamespace
import time

import rclpy
from whill_controller.controller_node import WhillControllerNode


def test_keyboard_drag_lifecycle():
    calibration = Path(__file__).parents[1] / 'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args', '-p', 'hardware_enabled:=false',
                     '-p', 'whill_mode:=true', '-p', 'keyboard_auto_drag:=true',
                     '-p', f'calibration_file:={calibration}'])
    node = None
    try:
        node = WhillControllerNode()
        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                node.root.update()
                time.sleep(.005)
        pump(3.)
        assert not node.calibrating
        writes = []
        write = node._write_command
        def record(payload, **kwargs):
            writes.append(payload)
            return write(payload, **kwargs)
        node._write_command = record
        key = lambda name: SimpleNamespace(keysym=name)
        node.on_key_press(key('w'))
        assert writes == []  # Disarmed.
        node.toggle_keyboard_arm()
        node.on_key_press(key('w'))
        assert writes == [b'[', b'w' * 8]
        assert node.click_held
        node.on_key_press(key('w'))
        pump(.5)
        assert writes == [b'[', b'w' * 8]  # No drift while held.
        node.on_key_press(key('f'))
        node.on_key_press(key('Return'))
        assert len(writes) == 2
        # Tk auto-repeat release/press must not end the drag.
        node.on_key_release(key('w'))
        node.on_key_press(key('w'))
        pump(.03)
        assert node.click_held and len(writes) == 2
        node.on_key_release(key('w'))
        pump(.03)
        assert writes[-2:] == [b']', b's' * 8]
        assert not node.click_held
        assert node.locked_current_coordinates == (-16, -19)
        # An opposite pair stops; releasing one starts a fresh centered drag.
        node.on_key_press(key('a'))
        node.on_key_press(key('d'))
        assert not node.click_held
        assert node.locked_current_coordinates == (-16, -19)
        node.on_key_release(key('a'))
        pump(.03)
        assert writes[-2:] == [b'[', b'd' * 8]
        assert node.click_held
        node.toggle_keyboard_arm()
        assert writes[-1] == b']' and not node.click_held
        # Re-arm after disarm away from center: center before pressing.
        node.toggle_keyboard_arm()
        node.on_key_press(key('w'))
        assert writes[-3:] == [b'a' * 8, b'[', b'w' * 8]
        node.toggle_keyboard_arm()
        # A failed movement releases the button and disarms.
        def fail_move(payload, **kwargs):
            if kwargs.get('action') == 'app_target_move' and b'w' in payload:
                return False
            return record(payload, **kwargs)
        node._write_command = fail_move
        node.toggle_keyboard_arm()
        node.on_key_press(key('w'))
        assert not node.keyboard_armed and not node.click_held
        assert writes[-1] == b']'
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
