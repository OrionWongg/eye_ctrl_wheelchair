import json
from wheelchair_control.direction_input import gui_key, KeyEvents
from wheelchair_control.direction_manager import START, STOP_EVENT


def test_gui_input_freshness_and_validation():
    for key in ('w', 'a', 'd', 'u', 'c', ' ', 's'):
        assert gui_key(json.dumps(dict(key=key, stamp=100.)), 100.1) == key
    for data in ('{}', 'null', '[]', 'broken',
                 '{"key":"w","stamp":"100"}',
                 '{"key":"x","stamp":100}'):
        assert gui_key(data, 100.1) is None
    for stamp in (99., 101., float('nan')):
        assert gui_key(json.dumps(dict(key='w', stamp=stamp)), 100.) is None


def test_gui_and_terminal_share_gesture_deduplication():
    keys = KeyEvents()
    assert keys.key('w', 1.) == START
    assert keys.key('w', 1.1) is None
    assert keys.key('w', 1.8) == START
    assert keys.key('s', 1.9) == STOP_EVENT
    assert keys.key('s', 2.) == STOP_EVENT


def test_hold_lease_release_timeout_and_late_heartbeat():
    from wheelchair_control.direction_input import HoldToRun
    lease = HoldToRun()
    assert not lease.active(1.)
    assert lease.update('w', 'press', 1.)
    # Long holds renew the same authorization, not new navigation events.
    for i in range(1, 50):
        assert lease.update('w', 'hold', 1.+i*.05)
    assert not lease.update('w', 'release', 3.46)
    assert not lease.update('w', 'hold', 3.47)  # Queued heartbeat cannot restart.
    assert lease.update('a', 'press', 4.)
    assert not lease.active(4.201)  # Lost GUI/process heartbeat stops.
    assert not lease.update('a', 'hold', 4.21)
    assert lease.update('d', 'press', 5.)
    assert not lease.update('s', 'press', 5.01)


def test_hold_multiple_keys_only_last_release_revokes():
    from wheelchair_control.direction_input import HoldToRun
    lease = HoldToRun()
    assert lease.update('w', 'press', 1.)
    assert lease.update('a', 'press', 1.01)
    assert lease.update('a', 'release', 1.02)
    assert not lease.update('w', 'release', 1.03)


def test_late_hold_cannot_renew_expired_key_before_timer_runs():
    from wheelchair_control.direction_input import HoldToRun
    lease = HoldToRun()
    assert lease.update('w', 'press', 1.)
    assert not lease.update('w', 'hold', 1.3)
