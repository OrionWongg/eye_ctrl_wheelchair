import json
from whill_controller.goal_lease import GoalLease


def send(lease, stamp, task='rviz:one', armed=True):
    lease.receive(json.dumps(dict(stamp=stamp, task_id=task)), stamp, stamp, armed)


def test_requires_local_arm_and_new_goal_after_rejection():
    lease=GoalLease()
    send(lease, 1., armed=False)
    send(lease, 1.1)
    assert not lease.valid(1.1)
    send(lease, 1.2, 'rviz:two')
    assert lease.valid(1.2)


def test_timeout_and_keyboard_revoke_cannot_be_renewed():
    lease=GoalLease();send(lease,1.)
    assert lease.valid(1.2)
    assert not lease.valid(1.26)
    send(lease,1.3)
    assert not lease.valid(1.3)
    send(lease,1.4,'rviz:two');lease.revoke()
    send(lease,1.5,'rviz:two')
    assert not lease.valid(1.5)


def test_stop_stale_and_replayed_authority():
    lease=GoalLease();send(lease,1.)
    send(lease,1.,'rviz:two')
    assert lease.task=='rviz:one'
    send(lease,1.1,'')
    assert not lease.valid(1.1)
    lease.receive(json.dumps(dict(stamp=2.,task_id='rviz:three')),3.,3.,True)
    assert not lease.valid(3.)
