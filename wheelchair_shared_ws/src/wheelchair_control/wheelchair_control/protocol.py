"""No ROS dependencies. Every clock argument is explicit for deterministic tests."""
from dataclasses import dataclass
import math

STOP, FORWARD, LEFT, RIGHT = range(4)
NAMES = ('STOP','FORWARD','LEFT','RIGHT')

@dataclass
class Packet:
    session: str
    sequence: int
    stamp: float
    ttl: float
    command: int = STOP
    enabled: bool = False
    valid: bool = True
    reason: str = ''
    task_id: str = ''
    task_kind: int = 0
    reference_x: float = 0.
    reference_y: float = 0.
    reference_yaw: float = 0.
    target_yaw: float = 0.
    task_elapsed: float = 0.

class StreamGuard:
    def __init__(self,max_ttl):
        self.max_ttl=max_ttl
        self.session=None
        self.sequence=-1
        self.retired=set()
        self.deadline=-math.inf

    def accept(self,p,ros_now,now):
        if (not p.session or len(p.session)>128 or p.session in self.retired
            or p.sequence<0 or p.command not in range(4)
            or not all(math.isfinite(x) for x in (p.stamp,p.ttl,ros_now,now))
            or not 0<p.ttl<=self.max_ttl+1e-9):
            self.deadline=-math.inf
            return False,False,'INVALID_PACKET'
        age=ros_now-p.stamp
        if age < -.05 or age >= p.ttl:
            self.deadline=-math.inf
            return False,False,'STALE_SOURCE'
        changed=p.session!=self.session
        if not changed and p.sequence<=self.sequence:
            self.deadline=-math.inf
            return False,False,'REPLAY_OR_OUT_OF_ORDER'
        if changed:
            if self.session is not None: self.retired.add(self.session)
            self.session=p.session
        self.sequence=p.sequence
        self.deadline=now+min(p.ttl,p.ttl-max(0.,age))
        return True,changed,''

    def fresh(self,now): return now<self.deadline

class ClockGuard:
    """Monotonic timers detect a paused ROS clock and invalidate resumed motion."""
    def __init__(self,pause_timeout=.30):
        self.last_ros=None; self.progress_at=None; self.pause_timeout=pause_timeout
    def update(self,ros_now,now):
        if self.last_ros is None:
            self.last_ros=ros_now; self.progress_at=now
            return ''
        delta=ros_now-self.last_ros
        self.last_ros=ros_now
        if delta < -1e-6 or delta>1.0:
            self.progress_at=now
            return 'CLOCK_JUMP'
        if delta>1e-9: self.progress_at=now
        if now-self.progress_at>self.pause_timeout: return 'CLOCK_PAUSED'
        return ''
