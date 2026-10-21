"""Discrete action executor. Only fixed setpoints or STOP; never speed quantization."""
from .protocol import STOP
from .planner import toward

class Executor:
    def __init__(self,cfg,settle=.15,stop_timeout=2.):
        self.cfg=cfg;self.settle=settle;self.stop_timeout=stop_timeout
        self.desired=STOP;self.output=STOP;self.phase='STOPPED';self.reason='STARTUP'
        self.low_since=None;self.transition_at=0.;self.start_at=None;self.fault=False
        self.session_ready=False
        self.run_limit=None;self.running_at=None;self.limit_reached=False
    def new_session(self,now):
        self.request(STOP,now);self.session_ready=False;self.reason='ACTION_SESSION_CHANGED'
    def request(self,action,now,max_run=None):
        if action==STOP:
            self.run_limit=None;self.running_at=None;self.limit_reached=False
            self.session_ready=True
            if self.desired!=STOP or self.output!=STOP:
                self.transition_at=now
            self.desired=STOP;self.output=STOP;self.start_at=None
            self.phase='STOPPING';self.reason='STOP_REQUEST'
        elif not self.session_ready:
            self.output=STOP;self.reason='WAIT_INITIAL_STOP'
        elif action!=self.desired:
            self.run_limit=max_run;self.running_at=None;self.limit_reached=False
            self.desired=action;self.output=STOP;self.phase='STOPPING'
            self.transition_at=now;self.start_at=None;self.reason='DIRECTION_CHANGE'
        elif max_run is not None:
            # A renewal can tighten a bound, but never erase or extend it.
            self.run_limit=max_run if self.run_limit is None else min(self.run_limit,max_run)
        # Renewing the same action leaves transition clocks and output untouched.
    def tick(self,now,v,w,healthy,lease_fresh):
        low=healthy and abs(v)<.015 and abs(w)<.025
        if low:
            if self.low_since is None:self.low_since=now
        else:self.low_since=None
        stopped=self.low_since is not None and now-self.low_since>=self.settle
        if not healthy or not lease_fresh:
            self.desired=STOP;self.output=STOP;self.start_at=None
            self.phase='FAULT';self.fault=True;self.session_ready=False
            self.reason='ODOM_FAULT' if not healthy else 'ACTION_STALE'
            return STOP,stopped
        if self.fault:
            if self.desired==STOP and stopped and self.session_ready:
                self.fault=False;self.phase='STOPPED';self.reason='FAULT_CLEARED_STOP_ONLY'
            else:
                self.output=STOP;self.phase='FAULT';return STOP,stopped
        if self.desired==STOP:
            self.output=STOP;self.phase='STOPPED' if stopped else 'STOPPING'
            if not stopped and now-self.transition_at>self.stop_timeout:
                self.fault=True;self.phase='FAULT';self.reason='STOP_TIMEOUT';self.session_ready=False
            return STOP,stopped
        if self.limit_reached or (self.run_limit is not None and self.running_at is not None
                                 and now-self.running_at>=self.run_limit):
            if not self.limit_reached:self.transition_at=now
            self.limit_reached=True;self.output=STOP
            self.phase='STOPPED' if stopped else 'STOPPING';self.reason='RUN_LIMIT_REACHED'
            if not stopped and now-self.transition_at>self.stop_timeout:
                self.fault=True;self.phase='FAULT';self.reason='STOP_TIMEOUT';self.session_ready=False
            return STOP,stopped
        if self.phase=='RUNNING' and self.output==self.desired:
            return self.output,stopped
        if not stopped:
            self.output=STOP;self.phase='STOPPING'
            if now-self.transition_at>self.stop_timeout:
                self.fault=True;self.phase='FAULT';self.reason='STOP_TIMEOUT';self.desired=STOP;self.session_ready=False
            return STOP,False
        if self.start_at is None:self.start_at=now;self.phase='STARTING'
        if now-self.start_at<self.cfg.startup_delay:
            self.output=STOP;return STOP,True
        self.output=self.desired;self.phase='RUNNING';self.reason='FIXED_ACTION'
        if self.running_at is None:self.running_at=now
        return self.output,stopped
