"""Match completed per-eye blinks instead of timing simultaneous closure.

Unknown observations never establish closure. A short unknown gap may preserve
an already observed transition; sustained loss cancels its evidence.
"""
import math


class BlinkEvents:
    def __init__(self,count,minimum=.06,maximum=.4,skew=.18,unknown_gap=.12,max_gap=.25):
        if count not in (1,2):raise ValueError('Expected one or two eyes')
        self.count=count;self.minimum=minimum;self.maximum=maximum
        self.skew=skew;self.unknown_gap=unknown_gap;self.max_gap=max_gap
        self.serial=0;self.reset()

    def reset(self):
        self.last=None;self.open_seen=[False]*self.count;self.starts=[None]*self.count
        self.unknown_since=[None]*self.count;self.completed=[None]*self.count
        self.long_sent=False;self.status='OPEN_FIRST';self.continuity=False

    def update(self,states,now,permitted):
        event=None;long_close=False
        if (not permitted or len(states)!=self.count or not math.isfinite(now) or
            any(s not in ('open','closed','unknown') for s in states)):
            self.reset();return event,long_close
        gap=self.last is not None and not 0<now-self.last<=self.max_gap
        if gap:self.reset()
        self.last=now;self.continuity=not gap
        for i,s in enumerate(states):
            if s=='unknown':
                if self.unknown_since[i] is None:self.unknown_since[i]=now
                if now-self.unknown_since[i]>self.unknown_gap:
                    self.open_seen[i]=False;self.starts[i]=None;self.completed=[None]*self.count
                    self.continuity=False
                continue
            if self.unknown_since[i] is not None and now-self.unknown_since[i]>self.unknown_gap:
                self.open_seen[i]=False;self.starts[i]=None;self.completed=[None]*self.count
                self.continuity=False
            transition=self.unknown_since[i]
            self.unknown_since[i]=None
            if s=='closed':
                if self.open_seen[i] and self.starts[i] is None:
                    # Once closure is confirmed, include its brief ambiguous
                    # onset. Unknown alone still never creates an event.
                    self.starts[i]=transition if transition is not None else now
            else:
                if self.starts[i] is not None:
                    start=self.starts[i];duration=now-start
                    if self.minimum<=duration<=self.maximum:
                        self.completed[i]=(start,now)
                    else:self.completed=[None]*self.count
                self.starts[i]=None;self.open_seen[i]=True
        if all(s=='closed' for s in states) and all(t is not None for t in self.starts):
            if now-max(self.starts)>=.65 and not self.long_sent:
                long_close=True;self.long_sent=True;self.completed=[None]*self.count
        if all(s=='open' for s in states):self.long_sent=False
        # Bound pending eye pairs and never match two separate unilateral winks.
        for i,pair in enumerate(self.completed):
            if pair is not None and now-pair[1]>self.skew:self.completed[i]=None
        if all(p is not None for p in self.completed):
            starts=[p[0] for p in self.completed];ends=[p[1] for p in self.completed]
            matches=(max(starts)-min(starts)<=self.skew and max(ends)-min(ends)<=self.skew
                     and max(starts)<min(ends))
            if matches and self.continuity:
                self.serial+=1
                event=dict(id=self.serial,start=min(starts),end=max(ends),
                           durations=[b-a for a,b in self.completed])
            self.completed=[None]*self.count
        self.status=('BLINK_MATCHED' if event else 'LONG_CLOSE' if long_close else
                     'TRACKING_LOST' if not self.continuity else
                     'CLOSING' if any(t is not None for t in self.starts) else 'WATCHING')
        return event,long_close
