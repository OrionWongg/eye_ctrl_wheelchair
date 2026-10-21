import json
import pytest
from eye_shared_control.confirm import ConfirmMachine, ConfirmConfig
from eye_shared_control.fusion import FusionInput, parse_sample
from eye_shared_control.confirm_node import direction_kind


class Feed:
    def __init__(self, h=0.):
        self.m = ConfirmMachine(ConfirmConfig(enter=.55, leave=.25))
        self.f = FusionInput(self.m)
        self.t = 10.
        self.seq = 0
        self.h = h

    def sample(self, event=None, **kw):
        self.t += .05
        self.seq += 1
        self.m.status(self.t, 'context', 1, True)
        d = dict(schema=1, session='a', sequence=self.seq, stamp_ns=int(self.t*1e9),
                 source_t=self.t, horizontal=self.h, fresh=True, calibrated=True,
                 open=True, motion_valid=True, gesture_continuity=True, long_close=False,
                 blink_event=event)
        d.update(kw)
        self.f.receive(json.dumps(d), self.t, self.t, True)
        return d

    def stable(self):
        for _ in range(8): self.sample()

    def blink(self, ident):
        start = self.t + .05
        for _ in range(3): self.sample(open=False, motion_valid=False)
        end = self.t + .05
        e = dict(id=ident, start=start, end=end, durations=[.15,.15])
        self.sample(e)
        return e



def arm(f):
    f.stable();f.blink(1)
    assert not f.f.active
    f.blink(2)
    assert f.f.active and direction_kind(f.m.pending)==1
    f.m.acknowledge(f.t, f.m.pending.event_id, True)


@pytest.mark.parametrize('h,kind', [(-.8,2),(.8,3)])
def test_double_blink_start_then_gaze_alone_turns(h,kind):
    f=Feed();arm(f)
    for _ in range(11): f.sample()
    f.h=h;f.sample()
    assert direction_kind(f.m.pending)==kind
    ident=f.m.pending.event_id
    f.m.acknowledge(f.t,ident,True)
    for _ in range(30): f.sample()
    assert f.m.pending is None  # holding side never repeats, even after pulse
    f.h=0.;f.sample();f.h=h;f.sample()
    assert direction_kind(f.m.pending)==kind and f.m.pending.event_id>ident


def test_center_does_not_interrupt_input_pulse_but_rearms_next_turn():
    f=Feed();arm(f)
    for _ in range(11): f.sample()
    f.h=-.8;f.sample()
    f.m.acknowledge(f.t,f.m.pending.event_id,True)
    f.h=0.;f.sample()
    assert f.f.turn==-1
    f.h=.8;f.sample()
    assert f.m.pending is None
    for _ in range(20):f.sample()
    assert direction_kind(f.m.pending)==3


@pytest.mark.parametrize('fault', [dict(long_close=True),dict(gesture_continuity=False),
    dict(motion_valid=False),dict(fresh=False),dict(calibrated=False),dict(stamp_ns=1)])
def test_pause_never_auto_resumes_and_one_new_blink_restores(fault):
    f=Feed();arm(f);f.sample(**fault)
    assert f.f.state=='PAUSED' and f.m.pending is None
    assert direction_kind(f.m.drain_events()[-1])==5
    f.stable();assert not f.f.active
    f.blink(3)
    assert f.f.active and direction_kind(f.m.pending)==1


def test_short_closure_stops_and_event_resumes():
    f=Feed();arm(f)
    start=f.t+.05
    for _ in range(3):f.sample(open=False,motion_valid=False)
    assert not f.f.active and not f.m.source_valid
    f.sample(dict(id=3,start=start,end=f.t+.05,durations=[.15,.15]))
    assert f.f.active


@pytest.mark.parametrize('resume', [False, True])
@pytest.mark.parametrize('h', [-.8, .8])
def test_post_blink_side_transient_is_discarded_before_normal_gaze(resume, h):
    f=Feed();arm(f)
    if resume:
        f.blink(3)
        assert direction_kind(f.m.pending)==1
        f.m.acknowledge(f.t,f.m.pending.event_id,True)
    f.h=h
    for _ in range(8):
        f.sample()
        assert f.m.pending is None
    f.h=0.
    for _ in range(4):f.sample()
    assert f.m.pending is None
    f.h=h;f.sample()
    assert direction_kind(f.m.pending)==(2 if h<0 else 3)


def test_post_blink_delay_requires_new_capture_not_just_timer_or_late_delivery():
    f=Feed();arm(f)
    f.h=.8
    for _ in range(8):f.sample()
    deadline=f.f.gaze_after
    f.m.status(deadline+.01,'context',1,True)
    f.f.tick(deadline+.01)
    assert f.m.pending is None
    f.t=deadline+.02
    f.sample(stamp_ns=int((deadline-.02)*1e9))
    assert f.m.pending is None
    f.sample()
    assert direction_kind(f.m.pending)==3


@pytest.mark.parametrize('fault', [dict(open=False,motion_valid=False),
    dict(gesture_continuity=False),dict(long_close=True)])
def test_post_blink_filter_does_not_delay_stop(fault):
    f=Feed();arm(f)
    f.sample(**fault)
    assert f.f.state=='PAUSED' and f.m.pending is None
    assert direction_kind(f.m.drain_events()[-1])==5


def test_duplicate_blink_cannot_arm_and_retired_session_cannot_refresh():
    f=Feed();f.stable();e=f.blink(1);d=f.sample(e)
    assert not f.f.active
    source=f.m.source_at
    assert not f.f.receive(json.dumps(d), f.t+.1, f.t+.1, True)[0]
    assert f.m.source_at==source
    f.sample(session='b',sequence=1)
    assert f.f.state=='DISARMED'
    assert not f.f.receive(json.dumps(d), f.t, f.t, True)[0]


def test_timeout_and_emergency_revoke_motion():
    f=Feed();arm(f);f.f.tick(f.t+.3)
    assert f.f.state=='PAUSED'
    f.f.pause('EMERGENCY_STOP',f.t+.3,disarm=True)
    assert f.f.state=='DISARMED'


def test_authorization_waits_briefly_for_navigation_health_recovery():
    f=Feed();arm(f)
    start=f.t+.05
    for _ in range(3):f.sample(open=False,motion_valid=False)
    f.m.can_accept=False
    # Bypass Feed's synthetic status refresh to reproduce manager health lag.
    t=f.t+.05
    d=dict(schema=1,session='a',sequence=f.seq+1,stamp_ns=int(t*1e9),source_t=t,
           horizontal=0.,open=True,motion_valid=True,fresh=True,calibrated=True,
           gesture_continuity=True,long_close=False,
           blink_event=dict(id=3,start=start,end=t,durations=[.15,.15]))
    f.f.receive(json.dumps(d),t,t,True)
    assert not f.f.active and f.f.authorization
    f.m.status(t+.05,'context',1,True)
    f.f.tick(t+.05)
    assert f.f.active


def test_old_blink_cannot_resume_after_fault():
    f=Feed();arm(f);f.sample(gesture_continuity=False)
    f.sample(dict(id=3,start=f.t-.18,end=f.t,durations=[.15,.15]))
    assert not f.f.active and f.f.event_decision=='REJECT_BEFORE_STOP'


@pytest.mark.parametrize('change', [dict(open=1),dict(horizontal=float('nan')),
    dict(sequence=True),dict(stamp_ns=1.2),dict(schema=True),dict(blink_event={})])
def test_strict_schema(change):
    f=Feed();d=f.sample();d.update(change)
    with pytest.raises((ValueError,KeyError,TypeError)): parse_sample(json.dumps(d))


def test_ros_fusion_subscription():
    import rclpy
    from eye_shared_control.node import EyeIntentNode
    rclpy.init(args=['--ros-args','-p','interaction_mode:=directional_intent','-p','eye_sample_mode:=fusion'])
    node=EyeIntentNode()
    try:
        assert '/eye_tracking/fusion_samples' in {s.topic_name for s in node.subscriptions}
        assert node.machine.cfg.enter==.55
        assert '/wheelchair/eye/direction_event' in {p.topic_name for p in node.publishers}
    finally: node.destroy_node();rclpy.shutdown()


def test_real_ros_json_transport_to_direction_event():
    import time
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from wheelchair_interfaces.msg import DirectionEvent, NavigationIntentStatus, IntentAck
    from eye_shared_control.node import EyeIntentNode
    rclpy.init(args=['--ros-args','-p','interaction_mode:=directional_intent',
                    '-p','eye_sample_mode:=fusion','-p','use_sim_time:=false'])
    node=EyeIntentNode();peer=Node('fusion_test_peer')
    pub=peer.create_publisher(String,'/eye_tracking/fusion_samples',1)
    status_pub=peer.create_publisher(NavigationIntentStatus,'/wheelchair/navigation_status',1)
    events=[]
    ack_pub=peer.create_publisher(IntentAck,'/wheelchair/intent_ack',10)
    def on_event(m):
        events.append(m)
        ack=IntentAck();ack.header.stamp=peer.get_clock().now().to_msg()
        ack.protocol_version=1;ack.source_session_id=m.source_session_id
        ack.manager_session_id='test_manager';ack.event_id=m.event_id;ack.status=1
        ack_pub.publish(ack)
    peer.create_subscription(DirectionEvent,'/wheelchair/eye/direction_event',on_event,10)
    seq=0;status_seq=0
    def pump(seconds, event=None, **kw):
        nonlocal seq,status_seq
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            now=time.monotonic();seq+=1;status_seq+=1
            status=NavigationIntentStatus();status.header.stamp=peer.get_clock().now().to_msg()
            status.manager_session_id='test_manager';status.sequence=status_seq;status.phase='IDLE';status.can_start=True
            status_pub.publish(status)
            d=dict(schema=1,session='ros_test',sequence=seq,stamp_ns=time.time_ns(),source_t=now,
                   horizontal=0.,fresh=True,calibrated=True,open=True,motion_valid=True,
                   gesture_continuity=True,long_close=False,blink_event=event)
            d.update(kw);pub.publish(String(data=json.dumps(d)))
            for _ in range(4):
                rclpy.spin_once(node,timeout_sec=.002);rclpy.spin_once(peer,timeout_sec=.002)
            time.sleep(.02)
    try:
        pump(.7)
        for ident in (1,2):
            start=time.monotonic();pump(.12,open=False,motion_valid=False)
            end=time.monotonic()
            pump(.07,dict(id=ident,start=start,end=end,durations=[end-start,end-start]))
        pump(.15,horizontal=-.8)
        assert not any(e.kind==2 for e in events)
        pump(.5)
        pump(.15,horizontal=-.8)
        assert any(e.kind==2 for e in events)
        assert len({e.event_id for e in events if e.kind==2})==1
        pump(.08,long_close=True)
        assert events[-1].kind==5
        before=len({e.event_id for e in events if e.kind==2})
        pump(.35)
        assert len({e.event_id for e in events if e.kind==2})==before
    finally:
        node.destroy_node();peer.destroy_node();rclpy.shutdown()




def test_processing_latency_does_not_turn_current_blink_into_pre_stop_event():
    f=Feed();arm(f)
    base=f.t+.05
    def receive(capture, closed=False, event=None):
        arrival=capture+.18  # fresh, but old frame expires before each next arrival
        f.seq+=1
        f.m.status(arrival,'context',1,True)
        d=dict(schema=1,session='a',sequence=f.seq,stamp_ns=int(capture*1e9),source_t=capture,
               horizontal=0.,open=not closed,motion_valid=not closed,fresh=True,
               calibrated=True,gesture_continuity=True,long_close=False,blink_event=event)
        f.f.receive(json.dumps(d),arrival,arrival,True)
    receive(base)
    receive(base+.1,True)
    assert not f.f.active  # motion protection is retained
    receive(base+.24,event=dict(id=3,start=base+.1,end=base+.24,durations=[.14,.14]))
    assert f.f.event_decision=='ACCEPTED'
    assert f.f.active


def test_short_closure_stops_without_claiming_sensor_is_broken():
    f=Feed();arm(f)
    f.sample(open=False,motion_valid=False)
    assert not f.f.active and not f.m.source_valid
    assert f.f.tracking_healthy(f.t)
    assert direction_kind(f.m.drain_events()[-1])==5
    assert not f.f.tracking_healthy(f.t+.3)
    f.sample(gesture_continuity=False)
    assert not f.f.tracking_healthy(f.t)


def test_arrival_silence_still_rejects_pre_timeout_blink():
    f=Feed();arm(f)
    f.f.tick(f.t+.41)
    assert f.f.reason=='SIGNAL_TIMEOUT'
    f.t+=.41
    f.sample(dict(id=3,start=f.t-.2,end=f.t,durations=[.2,.2]))
    assert not f.f.active and f.f.event_decision=='REJECT_BEFORE_STOP'


def test_old_blocked_status_cannot_erase_unacknowledged_resume():
    import time
    import rclpy
    from wheelchair_interfaces.msg import NavigationIntentStatus
    from eye_shared_control.node import EyeIntentNode
    rclpy.init(args=['--ros-args','-p','interaction_mode:=directional_intent',
                    '-p','eye_sample_mode:=fusion','-p','use_sim_time:=false'])
    node=EyeIntentNode()
    try:
        tr=node.confirm_transport;now=time.monotonic()
        tr.clock_valid=True;tr.manager_session='manager';tr.status_sequence=0
        tr.machine.status(now,'manager:old',1,True)
        tr.machine.source_at=now;tr.machine.source_valid=True
        tr.fusion_input.active=True;tr.fusion_input.emit(now,0)
        event=tr.machine.pending
        msg=NavigationIntentStatus();msg.header.stamp=node.get_clock().now().to_msg()
        msg.manager_session_id='manager';msg.sequence=1;msg.phase='BLOCKED'
        msg.reason='INPUT_HEALTH_TIMEOUT';msg.can_start=False
        tr.status(msg)
        assert tr.machine.pending is event and tr.fusion_input.active
        # A fault after ACK is still acted on; this is only an in-flight guard.
        tr.machine.acknowledge(now,event.event_id,True)
        msg.sequence=2;tr.status(msg)
        assert not tr.fusion_input.active
    finally:node.destroy_node();rclpy.shutdown()
