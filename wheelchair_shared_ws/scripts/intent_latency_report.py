#!/usr/bin/env python3
"""Summarize recorded stages, never infer missing acquisition or physical motion."""
import argparse
import json
import math
from collections import defaultdict

PAIRS = {
    'source_to_preview': ('source_sample', 'preview', 'sample_sequence'),
    'reception_to_preview': ('received', 'preview', 'sample_sequence'),
    'stable_to_confirmation': ('stable', 'confirmation', 'candidate_id'),
    'confirmation_to_ack': ('confirmation', 'ack', 'event_id'),
    'ack_to_motion': ('ack', 'motion_started', 'event_id'),
    'stop_transition_wait': ('stop_wait_started', 'stop_wait_ended', 'event_id'),
    'stop_gesture_to_recognized': ('stop_gesture', 'stop_recognized', 'event_id'),
    'stop_recognized_to_published': ('stop_recognized', 'stop_published', 'event_id'),
    'stop_published_to_stopped': ('stop_published', 'stopped', 'event_id'),
    'stop_gesture_to_stopped': ('stop_gesture', 'stopped', 'event_id'),
}


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    return ordered[low] + (ordered[math.ceil(index)] - ordered[low]) * (index - low)


def summarize(records):
    metrics = {}
    for name, (begin, end, identity) in PAIRS.items():
        starts, ends = {}, {}
        for row in records:
            value = row.get(identity)
            stamp = row.get('monotonic_s')
            if value in (None, '') or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                continue
            # Acquisition is measurable only if mapped to this same monotonic clock.
            if name == 'source_to_preview' and row.get('reception_timed', False):
                continue
            key = (row.get('clock_domain', 'local'), row.get('source_session_id', ''), value)
            dest = starts if row.get('stage') == begin else ends if row.get('stage') == end else None
            if dest is not None:
                dest[key] = min(stamp, dest.get(key, stamp))
        samples = [(ends[key] - stamp) * 1000 for key, stamp in starts.items()
                   if key in ends and ends[key] >= stamp]
        metrics[name] = dict(count=len(samples),
                             p50_ms=percentile(samples, .5) if samples else None,
                             p95_ms=percentile(samples, .95) if samples else None,
                             max_ms=max(samples) if samples else None,
                             unmatched_starts=sum(key not in ends for key in starts),
                             invalid_order=sum(key in ends and ends[key] < stamp for key, stamp in starts.items()))
    return dict(metrics=metrics, notes=[
        'Missing stages are unmeasured (null), never zero latency.',
        'Legacy eye input has reception timing only; reception_to_preview is not camera end-to-end.',
        'Use one host monotonic clock domain, or explicitly synchronize timestamps before merging.',
        'ACK is authorization; motion_started and stopped require measured odometry.',
        'Percentiles use linear interpolation; maxima are observed, not guaranteed bounds.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', nargs='+')
    options = parser.parse_args()
    records = []
    for path in options.files:
        with open(path) as handle:
            for number, line in enumerate(handle, 1):
                if line.strip():
                    try:
                        row = json.loads(line)
                        if not isinstance(row, dict):
                            raise ValueError('expected object')
                        records.append(row)
                    except (ValueError, TypeError) as error:
                        parser.error(f'{path}:{number}: {error}')
    print(json.dumps(summarize(records), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
