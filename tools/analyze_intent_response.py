#!/usr/bin/env python3
"""Report intended/target/measured directions separately; never label a blocked intent as a tracking pass."""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


def vector(row, names):
    try:
        return tuple(float(row[n]) for n in names)
    except (ValueError, KeyError):
        return (0.0, 0.0, 0.0)


def angle(a, b):
    if sum(x*x for x in a) < .0025 or sum(x*x for x in b) < .0025:
        return None
    cross = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    return math.degrees(math.atan2(math.sqrt(sum(x*x for x in cross)), sum(x*y for x, y in zip(a, b))))


def summarize(rows, start, end):
    selected = [r for r in rows if start <= float(r['t']) < end]
    moving = [(float(r['t']), angle(vector(r, ('qx', 'qy', 'qz')), vector(r, ('vx', 'vy', 'vz')))) for r in selected]
    errors = [e for _, e in moving if e is not None]
    tail = [e for t, e in moving if t >= end-2 and e is not None]
    first = next((t-start for t, e in moving if e is not None and e < 10), None)
    events = []
    stamps = set()
    references = []
    target_errors = []
    for r in selected:
        stamp = r.get('diagnostic_stamp', '')
        if not stamp or stamp in stamps:
            continue
        stamps.add(stamp)
        q = vector(r, ('input_x', 'input_y', 'input_z'))
        # Exclude stale diagnostic rows across changes or after operator release.
        input_alignment = angle(q, vector(r, ('qx', 'qy', 'qz')))
        if input_alignment is None or input_alignment > .1:
            continue
        error = angle(q, vector(r, ('reference_intent_x', 'reference_intent_y', 'reference_intent_z')))
        if error is not None:
            references.append(error)
        target_error = angle(q, vector(r, ('target_x', 'target_y', 'target_z')))
        if target_error is not None:
            target_errors.append(target_error)
        if r.get('command_changed') == '1':
            events.append({'time': float(r['t']), 'reason': r['reset_reason'], 'continued': r['continued']})
    return {'start_s': start, 'end_s': end, 'first_measured_error_below_10deg_s': first,
            'moving_samples': len(errors), 'median_measured_error_deg': median(errors) if errors else None,
            'last_2s_median_measured_error_deg': median(tail) if tail else None,
            'max_reference_intent_error_deg': max(references) if references else None,
            'median_target_intent_error_deg': median(target_errors) if target_errors else None,
            'command_change_events': events}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    args = parser.parse_args()
    rows = list(csv.DictReader(args.trace.open()))
    phases = [('initial', 1, 3), ('slow_sweep', 3, 7.8), ('hold_20deg', 7.8, 10),
              ('left', 10, 28), ('reverse', 28, 60), ('return_forward', 60, 80)]
    print(json.dumps({'trace': str(args.trace), 'phases': {name: summarize(rows, a, b) for name, a, b in phases}}, indent=2))
