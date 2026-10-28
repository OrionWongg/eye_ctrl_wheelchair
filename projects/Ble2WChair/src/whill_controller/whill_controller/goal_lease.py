"""Short-lived RViz authority; never supplies a drive command or local Arm."""
import json
import math


class GoalLease:
    def __init__(self):
        self.task = ''
        self.deadline = 0.
        self.last_stamp = -math.inf
        self.revoked = set()

    def revoke(self):
        if self.task:self.revoked.add(self.task)
        self.task = ''
        self.deadline = 0.

    def valid(self, now):
        if self.task and now >= self.deadline:self.revoke()
        return bool(self.task) and now < self.deadline

    def receive(self, payload, ros_now, now, armed):
        self.valid(now)
        try:
            data = json.loads(payload)
            stamp = float(data['stamp'])
            task = data['task_id']
            if not math.isfinite(stamp) or stamp <= self.last_stamp:
                return
            self.last_stamp = stamp
            age = ros_now-stamp
            if not isinstance(task, str) or len(task)>128 or not task.startswith('rviz:'):
                self.revoke();return
            if not armed or not -.05 <= age < .25:
                self.revoked.add(task);self.revoke();return
            if task in self.revoked:return
            self.task = task
            self.deadline = now + .25-max(0., age)
        except (ValueError, TypeError, KeyError):
            self.revoke()
