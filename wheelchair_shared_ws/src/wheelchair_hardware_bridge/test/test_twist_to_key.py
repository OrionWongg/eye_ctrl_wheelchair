import math

from geometry_msgs.msg import Twist

from wheelchair_control.planner import Config
from wheelchair_control.protocol import FORWARD, LEFT, RIGHT, STOP
from wheelchair_hardware_bridge.twist_to_key import to_ipad_twist


def accepted_by_ipad_controller(t):
    # Mirrors whill_controller.controller_node.on_navigation_command.
    return (t.linear.x >= 0 and not (abs(t.linear.x) > .05 and abs(t.angular.z) > .05)
            and all(math.isfinite(x) for x in (t.linear.x, t.angular.z)))


def test_every_executor_action_is_accepted_and_keeps_its_key():
    cfg = Config(forward_speed=.534, left_speed=.8, right_speed=.8)
    expected = {STOP: None, FORWARD: 'w', LEFT: 'a', RIGHT: 'd'}
    for action, key in expected.items():
        raw = Twist()
        raw.linear.x, raw.angular.z = cfg.velocity(action)
        out = to_ipad_twist(raw, .05)
        assert accepted_by_ipad_controller(out), (action, out)
        v, w = out.linear.x, out.angular.z
        got = 'w' if v > .05 else 'a' if w > .05 else 'd' if w < -.05 else None
        assert got == key


def test_turn_arc_is_rejected_without_adapter():
    raw = Twist()
    raw.linear.x, raw.angular.z = Config(left_speed=.8).velocity(LEFT)
    assert not accepted_by_ipad_controller(raw)
