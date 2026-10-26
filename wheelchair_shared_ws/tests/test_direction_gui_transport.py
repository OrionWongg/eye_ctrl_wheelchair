"""Real GUI-topic → terminal keyboard → DirectionEvent, with no actuator nodes."""
import json
import os
import pty
import signal
import sys
import time

import pytest
import rclpy
from std_msgs.msg import String
from wheelchair_interfaces.msg import DirectionEvent, InputHealth


@pytest.mark.parametrize('hold_to_run', [False, True])
def test_gui_to_direction_event(hold_to_run):
    pid, terminal = pty.fork()
    if pid == 0:
        os.execl(sys.executable, sys.executable, '-c',
                 'from wheelchair_control.direction_input import main; main()',
                 *(['--hold-to-run'] if hold_to_run else []))
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('test_gui_keyboard_transport')
        pub = node.create_publisher(String, '/wheelchair/keyboard/gui_key', 10)
        events, health = [], []
        node.create_subscription(DirectionEvent, '/wheelchair/keyboard/direction_event', events.append, 10)
        node.create_subscription(InputHealth, '/wheelchair/keyboard/health', health.append, 10)
        def until(predicate):
            end = time.monotonic() + 5
            while time.monotonic() < end and not predicate():
                rclpy.spin_once(node, timeout_sec=.02)
            assert predicate()
        until(lambda: pub.get_subscription_count() > 0 and health
              and (hold_to_run or health[-1].source_valid))
        def send(key, age=0., action='press'):
            msg = String()
            msg.data = json.dumps(dict(key=key, action=action, stamp=node.get_clock().now().nanoseconds/1e9-age))
            pub.publish(msg)
        send('w', age=1.)
        end = time.monotonic()+.3
        while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.02)
        assert not events
        send('w')
        until(lambda: any(m.kind == 1 for m in events))
        start = next(m for m in events if m.kind == 1)
        assert start.source_session_id == health[-1].source_session_id
        if hold_to_run:
            until(lambda: health[-1].source_valid)
            for _ in range(12):
                send('w', action='hold')
                end=time.monotonic()+.05
                while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.005)
            assert health[-1].source_valid
            assert len({m.event_id for m in events if m.kind==1})==1
            send('w', action='release')
            until(lambda: any(m.kind == 5 for m in events) and not health[-1].source_valid)
            send('w', action='hold')
            end=time.monotonic()+.25
            while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.01)
            assert not health[-1].source_valid
            send('w')
            until(lambda: health[-1].source_valid)
            until(lambda: not health[-1].source_valid)  # GUI disappearance expires lease.
        else:
            send('s')
            until(lambda: any(m.kind == 5 for m in events))
    finally:
        os.kill(pid, signal.SIGINT)
        os.waitpid(pid, 0)
        os.close(terminal)
        if node:node.destroy_node()
        rclpy.shutdown()
