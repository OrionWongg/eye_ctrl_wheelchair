import pytest
from eye_shared_control.confirm_node import direction_kind
from test_confirm import Harness


@pytest.mark.parametrize('gaze,kind',[(-.8,2),(0.,1),(.8,3)])
def test_confirmed_gaze_maps_to_keyboard_direction_and_preserves_retries(gaze,kind):
    h=Harness();h.advance(.4,gaze);h.blink(gaze)
    assert not h.motions()
    h.blink(gaze)
    event=h.m.pending
    assert direction_kind(event)==kind
    h.advance(.2,gaze)
    assert {direction_kind(e) for e in h.motions().values()}=={kind}
    assert len(h.motions())==1


def test_long_close_and_invalid_are_direction_stop_without_old_motion():
    h=Harness();h.confirm(-.8)
    h.advance(.7,closed=True)
    assert h.m.pending is None
    assert direction_kind(h.events[-1])==5
    h.advance(.3,valid=False)
    h.advance(.5)
    assert h.m.pending is None


def test_directional_node_publishes_events_only_not_drive_or_legacy_intent():
    import rclpy
    from eye_shared_control.node import EyeIntentNode
    rclpy.init(args=['--ros-args','-p','interaction_mode:=directional_intent','-p','eye_sample_mode:=stamped'])
    node=EyeIntentNode()
    try:
        published={p.topic_name for p in node.publishers}
        subscribed={s.topic_name for s in node.subscriptions}
        assert '/wheelchair/eye/direction_event' in published
        assert '/wheelchair/eye/health' in published
        assert not published.intersection({'/wheelchair/intent','/wheelchair/action_safe','/wheelchair/sim/drive','/wheelchair/eye_intent','/wheelchair/eye/event'})
        assert '/eye_tracking/samples' in subscribed and '/eye_tracking/signals' not in subscribed
        assert '/wheelchair/navigation_status' in subscribed
    finally:node.destroy_node();rclpy.shutdown()
