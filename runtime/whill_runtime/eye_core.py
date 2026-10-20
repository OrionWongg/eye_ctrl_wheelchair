"""Calibration and binocular blink matching reconstructed from the project notes."""
from collections import deque
import json
import os
from pathlib import Path
import tempfile

import numpy as np


class Blinks:
    def __init__(self, eyes=2):
        self.previous = ['unknown'] * eyes
        self.started = [None] * eyes
        self.completed = [None] * eyes
        self.unknown = [None] * eyes
        self.last = None
        self.ident = 0
        self.long_sent = False

    def update(self, states, now, fresh=True):
        if not fresh or (self.last is not None and not 0 < now-self.last <= .25):
            self.previous = ['unknown'] * len(states)
            self.started = [None] * len(states)
            self.completed = [None] * len(states)
            self.unknown = [None] * len(states)
        self.last = now
        long_close = False
        for i, state in enumerate(states):
            if state == 'unknown':
                if self.unknown[i] is None:
                    self.unknown[i] = now
                if now-self.unknown[i] > .12:
                    self.started[i] = self.completed[i] = None
                    self.previous[i] = 'unknown'
                continue
            unknown_at = self.unknown[i]
            if unknown_at is not None and now-unknown_at > .12:
                self.started[i] = self.completed[i] = None
                self.previous[i] = 'unknown'
            self.unknown[i] = None
            if state == 'closed' and self.previous[i] == 'open':
                self.started[i] = unknown_at if unknown_at is not None else now
            if state == 'open' and self.started[i] is not None:
                start = self.started[i]
                self.completed[i] = (start, now) if .06 <= now-start <= .4 else None
                self.started[i] = None
            self.previous[i] = state
        if all(s == 'closed' for s in states) and all(t is not None for t in self.started):
            if now-max(self.started) >= .65 and not self.long_sent:
                self.long_sent = True
                self.completed = [None]*len(states)
                long_close = True
        else:
            self.long_sent = False
        event = None
        if all(c is not None for c in self.completed):
            starts, ends = zip(*self.completed)
            if max(starts)-min(starts) <= .18 and max(ends)-min(ends) <= .18 and max(starts) < min(ends) and now-max(ends) <= .18:
                self.ident += 1
                event = dict(id=self.ident, start=min(starts), end=max(ends),
                             durations=[end-start for start,end in self.completed])
            self.completed = [None]*len(states)
        return event, long_close


class Calibration:
    # Storage order LEFT/CENTER/RIGHT, collection order CENTER/LEFT/RIGHT/CENTER.
    ORDER = (1, 0, 2, 1)
    LABELS = ('LOOK CENTER', 'LOOK LEFT', 'LOOK RIGHT', 'CONFIRM CENTER')

    def __init__(self, path, binding, reset=False):
        self.path, self.binding = Path(path), binding
        self.points = np.full((3, 2, 2), np.nan)
        self.selected = np.zeros((2, 2), bool)
        self.ready = False
        self.step = 0
        self.loaded = False
        self.samples = deque()
        self.stage_at = None
        self.message = ''
        if not reset:
            try:
                data = json.loads(self.path.read_text())
                if data['schema'] == 'whill-eye-fusion-v1' and data['binding'] == binding:
                    points = np.array(data['points'], dtype=float)
                    selected = np.array(data['selected'], dtype=bool)
                    if points.shape == (3,2,2) and selected.shape == (2,2) and selected.any(axis=1).all() and np.isfinite(points[:,selected]).all():
                        self.points, self.selected = points, selected
                        self.loaded, self.step = True, 3
            except (OSError, ValueError, KeyError, TypeError):
                pass

    @property
    def label(self):
        return 'READY - DOUBLE BLINK TO START' if self.ready else self.LABELS[self.step]

    def feed(self, detections, now, fresh):
        if self.ready:
            return
        if self.stage_at is None:
            self.stage_at = now
        valid = fresh and all(d['state'] == 'open' for d in detections)
        if not valid:
            self.samples.clear()
            return
        if now-self.stage_at < 1.:
            return
        values = np.array([d['features'] for d in detections], dtype=float)
        self.samples.append((now, values))
        if len(self.samples) < 15 or now-self.samples[0][0] < 1.5:
            return
        stack = np.array([s[1] for s in self.samples])
        finite = np.isfinite(stack).all(axis=0)
        spread = np.full((2,2), np.inf)
        median = np.full((2,2), np.nan)
        if finite.any():
            spread[finite] = np.percentile(stack[:,finite],90,axis=0)-np.percentile(stack[:,finite],10,axis=0)
            median[finite] = np.median(stack[:,finite],axis=0)
        stable = finite & (spread <= .025)
        if not stable.any(axis=1).all():
            self.samples.popleft()
            self.message = 'HOLD EYES STEADY; CHECK CAMERA POSITION'
            return
        median[~stable] = np.nan
        if self.step < 3:
            self.points[self.ORDER[self.step]] = median
            self.step += 1
        else:
            left, center, right = self.points
            selected = np.isfinite(self.points).all(axis=0) & ((left-center)*(right-center) < 0) & (np.minimum(abs(left-center),abs(right-center)) > .008)
            selected &= stable & (abs(median-center) <= .06)
            if not selected.any(axis=1).all():
                self.message = 'LEFT/CENTER/RIGHT NOT SEPARABLE - R TO RESTART'
                self.samples.clear()
                return
            self.points[1], self.selected = np.where(selected, median, center), selected
            try:
                self.save()
            except OSError as exc:
                self.message = f'SAVE FAILED: {exc}'
                return
            self.ready = True
        self.samples.clear()
        self.stage_at = now

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Nonselected values are JSON null, never nonstandard NaN.
        points = [[[float(v) if np.isfinite(v) else None for v in methods] for methods in eyes] for eyes in self.points]
        payload = dict(schema='whill-eye-fusion-v1', binding=self.binding, points=points, selected=self.selected.tolist())
        fd, name = tempfile.mkstemp(dir=self.path.parent, prefix='.calibration-')
        try:
            with os.fdopen(fd,'w') as out:
                json.dump(payload,out,allow_nan=False)
                out.flush()
                os.fsync(out.fileno())
            os.replace(name,self.path)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    def direction(self, detections):
        if not self.ready:
            return 0., False
        eyes = []
        for i,d in enumerate(detections):
            mapped, weights = [], []
            for j,f in enumerate(d['features']):
                if not self.selected[i,j] or f is None:
                    continue
                left,center,right = self.points[:,i,j]
                delta = f-center
                gain = right-center if delta*(right-center) >= 0 else -(left-center)
                mapped.append(delta/gain)
                weights.append(d['quality'][j])
            if not mapped or max(mapped)-min(mapped) > .5:
                return 0., False
            eyes.append(float(np.clip(np.average(mapped,weights=weights),-1,1)))
        if abs(eyes[0]-eyes[1]) > .65:
            return 0., False
        return float(np.mean(eyes)), True
