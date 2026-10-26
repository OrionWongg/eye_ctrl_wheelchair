"""ROS/Tk integration with real DDS messages and dry-run serial output."""
import json
import time
import sys
from pathlib import Path

import rclpy
from std_msgs.msg import String
from whill_controller.controller_node import WhillControllerNode


def test_ros_eye_to_calibrated_drag_and_watchdog():
    calibration = Path(__file__).parents[1] / 'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args', '-p', 'hardware_enabled:=false',
                     '-p', 'whill_mode:=true', '-p', 'eye_control_enabled:=true',
                     '-p', 'unified_ui:=true',
                     '-p', f'calibration_file:={calibration}'])
    controller = None
    observer = None
    guide = None
    try:
        controller = WhillControllerNode()
        observer = rclpy.create_node('eye_ipad_test')
        pub = observer.create_publisher(String, '/eye_tracking/fusion_samples', 1)
        ui_pub = observer.create_publisher(String, '/whill/controller/ui', 1)
        sys.path.insert(0, str(Path(__file__).parents[4] / 'EYETLE/tools'))
        from fusion_guide import GuideWindow
        guide = GuideWindow(ipad_controls=True)
        panel = guide.ipad_panel
        panel.send = lambda action: ui_pub.publish(String(data=json.dumps(dict(
            action=action, stamp_ns=time.time_ns()))))
        observer.create_subscription(String, '/eye_control/feedback',
                                     lambda m: panel.update(json.loads(m.data)), 1)
        commands = []
        feedback = []
        observer.create_subscription(String, '/eye_control/feedback',
                                     lambda m: feedback.append(json.loads(m.data)), 1)
        observer.create_subscription(String, '/whill/controller/command',
                                     lambda m: commands.append(json.loads(m.data)), 100)

        def pump(duration):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                controller.root.update()
                guide.poll()
                rclpy.spin_once(observer, timeout_sec=0.)
                time.sleep(.005)

        pump(3.)
        assert controller.locked_current_coordinates == (-16, -19)
        assert not controller.calibrating
        assert feedback[-1]['blockers'] == ['IPAD_NOT_ARMED']
        assert feedback[-1]['reason'] == 'IPAD_NOT_ARMED'
        assert controller.root.state() == 'withdrawn'
        assert 'ipad_calibration_20261009.json' in panel.calibration.get()
        assert '左16 / 上19' in panel.calibration.get()
        ui_pub.publish(String(data=json.dumps(dict(action='arm', stamp_ns=1))))
        pump(.1)
        assert not controller.keyboard_armed
        panel.arm.invoke()
        pump(.1)
        assert controller.keyboard_armed
        seq = 0

        def sample(**changes):
            nonlocal seq
            seq += 1
            now = time.monotonic()
            data = dict(schema=1, session='dds-test', sequence=seq,
                        stamp_ns=time.time_ns(), source_t=now, horizontal=0.,
                        fresh=True, calibrated=True, open=True, motion_valid=True,
                        gesture_continuity=True, long_close=False, blink_event=None)
            data.update(changes)
            pub.publish(String(data=json.dumps(data)))
            pump(.05)
            # DDS delivery is asynchronous; wait for this sample rather than
            # starting the next blink before its preceding open event arrives.
            deadline = time.monotonic() + .15
            while controller.eye_drive.fusion.sequence < seq and time.monotonic() < deadline:
                pump(.005)
            assert controller.eye_drive.fusion.sequence == seq

        def blink(ident):
            start = time.monotonic()
            for _ in range(3):
                sample(open=False, motion_valid=False)
            end = time.monotonic()
            sample(blink_event=dict(id=ident, start=start, end=end,
                                    durations=[end-start]))

        for _ in range(5):
            sample()
        blink(1)
        assert not controller.click_held
        blink(2)
        assert controller.click_held, (controller.eye_drive.fusion.reason,
                                       controller.eye_drive.fusion.event_decision,
                                       controller.eye_drive.fusion.decision_details)
        pump(.12)
        assert feedback[-1]['target'] == 'wheelchair'
        assert feedback[-1]['state'] == 'ACTIVE'
        assert feedback[-1]['backend'] == 'ipad'
        assert feedback[-1]['blockers'] == []
        assert controller.locked_current_coordinates == (-16, -27)
        for _ in range(11):
            sample()
        sample(horizontal=.8)
        assert controller.locked_current_coordinates == (-8, -19)
        assert controller.eye_target == 'd'
        # Silence must release without any new publisher callback.
        pump(.35)
        assert not controller.click_held
        assert controller.eye_target is None
        assert controller.locked_current_coordinates == (-16, -19)
        assert any(c['action'] == 'eye_button_hold' for c in commands)
        assert any(c['action'] == 'eye_button_release' for c in commands)
        assert all(c['hardware_enabled'] is False for c in commands)
        assert any(c['action'] == 'app_target_move' and c['payload_ascii'] == 'w'*8
                   for c in commands)
        for _ in range(5):
            sample()
        assert not controller.click_held
        blink(3)
        assert controller.click_held
        old_arm_stamp = time.time_ns()
        panel.stop.invoke()
        pump(.1)
        assert not controller.click_held and not controller.keyboard_armed
        ui_pub.publish(String(data=json.dumps(dict(action='arm', stamp_ns=old_arm_stamp))))
        pump(.1)
        assert not controller.keyboard_armed
    finally:
        if guide is not None:
            guide.destroy()
        if controller is not None:
            controller.close()
            controller.destroy_node()
        if observer is not None:
            observer.destroy_node()
        rclpy.shutdown()


def test_continuous_eye_samples_are_not_starved_by_telemetry():
    """20 Hz frames aged 120 ms must survive the controller's own timers."""
    calibration = Path(__file__).parents[1] / 'config/ipad_calibration_20261009.json'
    rclpy.init(args=['--ros-args', '-p', 'hardware_enabled:=false',
                     '-p', 'whill_mode:=true', '-p', 'eye_control_enabled:=true',
                     '-p', 'unified_ui:=true',
                     '-p', f'calibration_file:={calibration}'])
    controller = None
    try:
        controller = WhillControllerNode()
        pub = controller.create_publisher(String, '/eye_tracking/fusion_samples', 1)
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            controller.root.update()
            time.sleep(.003)
        controller.toggle_keyboard_arm()
        assert controller.eye_blockers() == []
        sequence = 0
        reasons = set()
        next_sample = time.monotonic()
        deadline = next_sample + 4.
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_sample:
                sequence += 1
                next_sample = now + .05
                pub.publish(String(data=json.dumps(dict(
                    schema=1, session='continuous-input', sequence=sequence,
                    stamp_ns=time.time_ns()-120_000_000, source_t=now-.12,
                    horizontal=0., fresh=True, calibrated=True, open=True,
                    motion_valid=True, gesture_continuity=True,
                    long_close=False, blink_event=None))))
            controller.root.update()
            fusion = controller.eye_drive.fusion
            if fusion.sequence:
                reasons.add(fusion.reason)
            time.sleep(.003)
        assert 'FRAME_AGE_TIMEOUT' not in reasons
        assert 'SIGNAL_TIMEOUT' not in reasons
        assert reasons  # DDS actually delivered input; KEEP_LAST may skip frames.
        # Real silence still expires the last frame with the original limit.
        deadline = time.monotonic() + .35
        while time.monotonic() < deadline:
            controller.root.update()
            time.sleep(.003)
        assert controller.eye_drive.fusion.reason == 'FRAME_AGE_TIMEOUT'
    finally:
        if controller is not None:
            controller.close()
            controller.destroy_node()
        rclpy.shutdown()
