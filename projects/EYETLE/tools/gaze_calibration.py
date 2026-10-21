"""Post-keypress gaze sampling and atomic, per-camera calibration profiles."""
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

DIRECTIONS = ('left', 'center', 'right')
KEYS = {'left': 'A / 1', 'center': 'S / 2', 'right': 'D / 3'}


def valid_points(points, complete=False):
    if not isinstance(points, dict) or any(k not in DIRECTIONS for k in points):
        return False
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) or not 0 <= v <= 1 for v in points.values()):
        return False
    if complete:
        if set(points) != set(DIRECTIONS):
            return False
        left, center, right = (points[k] for k in DIRECTIONS)
        return ((left-center)*(right-center) < 0 and
                min(abs(left-center), abs(right-center)) > .015)
    return True


def camera_identity(device):
    resolved = Path(device).resolve()
    for candidate in sorted(Path('/dev/v4l/by-path').glob('*-video-index0')):
        if candidate.resolve() == resolved:
            return str(candidate)
    return str(resolved)


def model_signature(paths):
    return [hashlib.sha256(Path(p).read_bytes()).hexdigest() if Path(p).is_file()
            else None for p in paths]


class CalibrationStore:
    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.warning = ''

    def read(self):
        empty = {'schema': 1, 'profiles': {}, 'drafts': {}}
        try:
            data = json.loads(self.path.read_text())
            if (not isinstance(data, dict) or data.get('schema') != 1 or
                    not isinstance(data.get('profiles'), dict) or
                    not isinstance(data.get('drafts'), dict)):
                raise ValueError('Unsupported calibration schema')
            return data
        except FileNotFoundError:
            return empty
        except (OSError, ValueError) as exc:
            self.warning = f'Cannot load calibration {self.path}: {exc}'
            return empty

    @staticmethod
    def key(binding):
        return hashlib.sha256(json.dumps(binding, sort_keys=True).encode()).hexdigest()

    def load(self, bindings):
        data = self.read()
        values, loaded = [], []
        for binding in bindings:
            key = self.key(binding)
            profile = data['profiles'].get(key, {})
            draft = data['drafts'].get(key, {})
            if (isinstance(profile, dict) and profile.get('binding') == binding and
                    valid_points(profile.get('values'), complete=True)):
                values.append(dict(profile['values'])); loaded.append(True)
            elif (isinstance(draft, dict) and draft.get('binding') == binding and
                  valid_points(draft.get('values')) and len(draft['values']) < 3):
                values.append(dict(draft['values'])); loaded.append(False)
            else:
                values.append({}); loaded.append(False)
        return values, loaded

    def save(self, bindings, values, quality, complete):
        if complete and not all(valid_points(v, complete=True) for v in values):
            raise ValueError('Invalid calibration cannot replace saved profiles')
        data = self.read()
        now = datetime.now(timezone.utc).isoformat()
        for binding, points, stats in zip(bindings, values, quality):
            key = self.key(binding)
            previous = data['profiles'].get(key, {})
            prior_quality = previous.get('quality', {}) if isinstance(previous, dict) else {}
            record = dict(binding=binding, values=points, quality=stats or prior_quality, saved_at=now)
            if complete:
                data['profiles'][key] = record
                data['drafts'].pop(key, None)
            else:
                data['drafts'][key] = record
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                                             prefix=self.path.name+'.', delete=False) as out:
                temporary = out.name
                json.dump(data, out, indent=2, allow_nan=False)
                out.write('\n'); out.flush(); os.fsync(out.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)


class GazeCalibration:
    def __init__(self, store, bindings, count=60, settle=.4, max_spread=.025):
        if count < 10 or settle < 0 or max_spread <= 0:
            raise ValueError('Invalid sampling settings')
        self.store, self.bindings = store, bindings
        self.count, self.settle, self.max_spread = count, settle, max_spread
        self.values, loaded = store.load(bindings)
        self.loaded = sum(loaded)
        self.quality = [{} for _ in bindings]
        self.active = None
        self.pending_save = False
        self.status = ('Saved calibration loaded | C: recalibrate' if self.ready
                       else self.prompt())

    @property
    def ready(self):
        return (self.active is None and not self.pending_save and
                all(valid_points(v, complete=True) for v in self.values))

    @property
    def expected(self):
        return next((d for d in DIRECTIONS if any(d not in v for v in self.values)), None)

    def prompt(self):
        name = self.expected
        return f'Look {name.upper()}, press {KEYS[name]} to START' if name else 'Calibration complete'

    def restart(self):
        # Preserve the last complete disk profile until its replacement succeeds.
        self.values = [{} for _ in self.bindings]
        self.quality = [{} for _ in self.bindings]
        self.active = None
        self.pending_save = False
        self.status = self.prompt()

    def begin(self, name, now):
        if self.pending_save:
            self.status = 'Save failed; press ENTER to retry'
            return False
        if self.active is not None:
            return False  # Key repeat cannot restart or contaminate a capture.
        if self.ready and name == 'left':
            self.restart()
        if name != self.expected:
            self.status = self.prompt()
            return False
        self.active = name
        self.targets = [i for i,v in enumerate(self.values) if name not in v]
        self.buffers = {i: deque(maxlen=self.count) for i in self.targets}
        self.start_at = now+self.settle
        self.status = f'{name.upper()}: hold gaze; capture starts in {self.settle:.1f}s'
        return True

    def save_pending(self):
        complete = all(valid_points(v, complete=True) for v in self.values)
        try:
            self.store.save(self.bindings, self.values, self.quality, complete)
        except (OSError, ValueError) as exc:
            self.pending_save = True
            self.status = 'SAVE FAILED; ENTER: retry | C: restart'
            print(f'Calibration save failed: {exc}', flush=True)
            return False
        self.pending_save = False
        self.status = ('Calibration SAVED; auto-load next start | C: recalibrate'
                       if complete else 'Point saved. '+self.prompt())
        return True

    def observe(self, positions, states, now, frame_valid=True):
        if self.active is None or now < self.start_at:
            return
        if (not frame_valid or any(states[i] != 'open' or positions[i] is None or
                                  not math.isfinite(positions[i]) or
                                  not 0 <= positions[i] <= 1 for i in self.targets)):
            self.status = f'{self.active.upper()}: {len(self.buffers[self.targets[0]])}/{self.count} - waiting for OPEN eyes'
            return
        for i in self.targets:
            self.buffers[i].append(positions[i])
        n = len(self.buffers[self.targets[0]])
        self.status = f'{self.active.upper()}: capturing {n}/{self.count} - hold gaze'
        if n < self.count:
            return
        spreads = {i: float(np.percentile(self.buffers[i],90)-np.percentile(self.buffers[i],10))
                   for i in self.targets}
        if any(s > self.max_spread for s in spreads.values()):
            self.status = f'{self.active.upper()}: gaze moving; collecting a stable window'
            return
        name = self.active
        for i in self.targets:
            self.values[i][name] = float(np.median(self.buffers[i]))
            self.quality[i][name] = dict(samples=n, spread=spreads[i])
        self.active = None
        # A good right endpoint must straddle center relative to the saved left.
        if any(len(v) == 3 and not valid_points(v, complete=True) for v in self.values):
            for i in self.targets:
                self.values[i].pop(name, None)
                self.quality[i].pop(name, None)
            self.status = 'Range/order invalid; C: restart or '+self.prompt()
            return
        self.pending_save = True
        self.save_pending()
