"""Shared offline/live eye interaction, without ROS or motion output."""
import math


class EyeInteraction:
    def __init__(self, input_loss_window=.2):
        if not math.isfinite(input_loss_window) or input_loss_window < 0:
            raise ValueError('input_loss_window must be finite and nonnegative')
        # Share the existing window: no additional tuning parameter.
        self.window = input_loss_window
        self.reset()

    def reset(self):
        self.state = 'DISARMED'
        self.reason = 'STARTUP'
        self.last_event = 0
        self.horizontal = 0.
        self.interruption_since = None
        self.candidate = self.candidate_since = None

    @staticmethod
    def direction(horizontal):
        return -1 if horizontal < -.55 else 1 if horizontal > .55 else 0

    @property
    def action(self):
        return {-1: 'LEFT', 0: 'FORWARD', 1: 'RIGHT'}[self.direction(self.horizontal)] if self.state == 'ACTIVE' else 'STOP'

    def snapshot(self):
        """The same result is displayed locally and consumed by live adapters."""
        return dict(state=self.state, reason=self.reason, action=self.action,
                    horizontal=self.horizontal, event_id=self.last_event)

    def start(self):
        """An explicit UI start uses the same subsequent eye observations."""
        if self.state == 'DISARMED':
            self.state, self.reason = 'PAUSED', 'START_BUTTON'

    def update(self, eyes, motion_valid, input_grace, left_event, horizontal, now):
        new_blink = left_event is not None and left_event['id'] > self.last_event
        if new_blink:
            self.last_event = left_event['id']
        if self.state == 'DISARMED':
            if not new_blink:
                return self.state
            self.state = 'PAUSED'
            self.reason = 'LEFT_BLINK_CONFIRMED'
        observed = eyes == ['open', 'open'] and motion_valid and not input_grace
        if not observed:
            self.candidate = self.candidate_since = None
            if self.interruption_since is None:
                self.interruption_since = now
            if self.state == 'ACTIVE' and now-self.interruption_since < self.window:
                self.reason = 'ACTION_FILTER_HOLD'
            else:
                self.state = 'PAUSED'
                self.reason = 'EYE_CLOSED' if 'closed' in eyes else 'INPUT_LOST'
            return self.state
        self.interruption_since = None
        if self.state != 'ACTIVE':
            self.state, self.reason = 'ACTIVE', 'BOTH_EYES_OPEN'
            self.horizontal = horizontal
            self.candidate = self.candidate_since = None
            return self.state
        target = self.direction(horizontal)
        if target == self.direction(self.horizontal):
            self.candidate = self.candidate_since = None
            self.reason = 'STEADY_GAZE'
        else:
            if target != self.candidate:
                self.candidate, self.candidate_since = target, now
            if now-self.candidate_since >= self.window:
                self.horizontal = horizontal
                self.candidate = self.candidate_since = None
                self.reason = 'STABLE_DIRECTION'
            else:
                self.reason = 'ACTION_FILTER_HOLD'
        return self.state
