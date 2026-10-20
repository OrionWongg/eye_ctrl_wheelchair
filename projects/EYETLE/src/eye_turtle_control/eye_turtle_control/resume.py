"""Resume only on a complete fresh short blink after a valid open sample."""
import math

class ResumeGesture:
    def __init__(self, blink_min=.06, blink_max=.4, max_gap=.25):
        if not all(math.isfinite(v) for v in (blink_min, blink_max, max_gap)) or not 0 < blink_min < blink_max or max_gap <= 0:
            raise ValueError('invalid blink resume thresholds')
        self.blink_min, self.blink_max, self.max_gap = blink_min, blink_max, max_gap
        self.reset()

    def reset(self):
        self.open_seen = False
        self.closed_at = self.last_at = None
        self.phase = 'OPEN_FIRST'
        self.progress = 0.

    def update(self, now, horizontal=0., closed=False, permitted=True):
        if not permitted or not math.isfinite(now) or not math.isfinite(horizontal):
            self.reset()
            return False
        if self.last_at is not None and not 0 <= now-self.last_at <= self.max_gap:
            self.reset()
        self.last_at = now
        if closed:
            if self.open_seen:
                if self.closed_at is None:
                    self.closed_at = now
                if now-self.closed_at > self.blink_max:
                    self.open_seen = False
                    self.closed_at = None
                    self.phase = 'OPEN_FIRST'
                else:
                    self.phase = 'REOPEN_TO_RESUME'
            return False
        duration = None if self.closed_at is None else now-self.closed_at
        self.closed_at = None
        self.open_seen = True
        self.phase = 'BLINK_TO_RESUME'
        return duration is not None and self.blink_min <= duration <= self.blink_max
