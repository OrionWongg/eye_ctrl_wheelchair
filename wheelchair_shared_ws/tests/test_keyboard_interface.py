"""Real ROS keyboard adapter, isolated from desktop and physical hardware."""
import types
import pytest
import rclpy
from rclpy.signals import SignalHandlerOptions
from wheelchair_interfaces.msg import ControlStatus
from wheelchair_control.inputs import Keyboard
from wheelchair_control.protocol import STOP,FORWARD,LEFT

@pytest.fixture
def keyboard(monkeypatch):
    import wheelchair_control.inputs as inputs
    now=[100.]
    monkeypatch.setattr(inputs,'time',types.SimpleNamespace(monotonic=lambda:now[0]))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Keyboard()
    status=ControlStatus();status.session_id='safety';status.intent_session_id=node.session
    status.requires_rearm=True;status.ready_to_rearm=True;status.reason='STARTUP:READY_TO_REARM'
    def heartbeat(dt=.05):
        now[0]+=dt;status.sequence+=1;status.header.stamp=node.get_clock().now().to_msg();node.on_status(status)
    heartbeat()
    yield node,status,now,heartbeat
    node.destroy_node();rclpy.shutdown()

def test_direct_keys_no_r_and_no_five_second_limit(keyboard):
    node,status,now,hb=keyboard
    node.key('w');node.tick();assert node.armed and node.command==FORWARD
    status.requires_rearm=False;hb()
    for _ in range(160):
        hb();node.key('w');node.tick()
        assert node.armed and not node.locked and node.command==FORWARD
    node.key('a');assert node.command==LEFT

def test_obstacle_requires_explicit_recovery_even_held_w(keyboard):
    node,status,now,hb=keyboard
    node.key('w');status.requires_rearm=False;hb()
    status.requires_rearm=True;status.ready_to_rearm=False;status.reason='BLOCKED_FORWARD';hb()
    assert node.locked and not node.armed
    status.ready_to_rearm=True;hb()
    for _ in range(20):
        node.key('w');node.tick();hb()
        assert not node.armed and node.command==STOP
    node.key(' ');node.key('w');assert node.locked
    hb() # Fresh acknowledgement after the user STOP.
    node.key('r');assert not node.locked and not node.armed
    node.key('a');assert node.armed and node.command==LEFT

def test_user_stop_allows_next_direction_without_r(keyboard):
    node,status,now,hb=keyboard
    node.key('w');status.requires_rearm=False;hb()
    node.key(' ');assert not node.armed and not node.locked
    node.key('w');assert not node.armed # Need acknowledgement of disabled STOP first.
    status.requires_rearm=True;status.ready_to_rearm=False;status.reason='USER_EMERGENCY_STOP';hb()
    assert not node.locked
    status.ready_to_rearm=True;status.reason='USER_EMERGENCY_STOP:READY_TO_REARM';hb()
    node.key('w');assert node.armed and not node.locked

def test_startup_faults_do_not_require_r_but_motion_faults_do(keyboard):
    node,status,now,hb=keyboard
    status.ready_to_rearm=False;status.reason='ODOM_STALE';hb()
    node.key('w');assert not node.armed and not node.locked
    status.ready_to_rearm=True;hb();node.key('w')
    status.requires_rearm=False;hb();node.tick()
    now[0]+=.4;node.tick()
    assert node.locked and not node.armed
    hb();node.key('w');assert not node.armed

def test_missing_ack_still_stops(keyboard):
    node,status,now,hb=keyboard
    node.key('w');node.tick()
    for _ in range(11):hb();node.tick()
    assert not node.armed and node.locked and node.reason=='REARM_REJECTED'

def test_other_producer_ready_does_not_authorize_this_keyboard(keyboard):
    node,status,now,hb=keyboard
    status.intent_session_id='old-keyboard';hb();node.key('w')
    assert not node.armed
    status.intent_session_id=node.session;hb();node.key('w');assert node.armed
