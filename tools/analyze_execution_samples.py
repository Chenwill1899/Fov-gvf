#!/usr/bin/env python3
"""Audit received/applied Twist correspondence from instrumented Isaac traces."""
import argparse
import csv
import json
import math
from pathlib import Path


def audit(path):
    rows = list(csv.DictReader(path.open()))
    counts = dict(frames=len(rows), active_fresh_frames=0, subscriber_disagreements=0,
                  callback_contract_errors=0, inactive_nonzero_targets=0,
                  callback_stale_nonzero_targets=0)
    max_disagreement = 0.0
    sources = set()
    for row in rows:
        source = row['twist_source']
        sources.add(source)
        active = any(abs(float(row[k])) > 1e-6 for k in ('qx', 'qy', 'qz'))
        received = tuple(float(row['received_body_' + k]) for k in 'xyz')
        applied = tuple(float(row['applied_body_' + k]) for k in 'xyz')
        receipt_seq = int(row['received_twist_sequence'])
        applied_seq = int(row['applied_twist_sequence'])
        age = float(row['twist_age_s'])
        fresh = receipt_seq > 0 and math.isfinite(age) and 0 <= age <= .10 + 1e-9
        nonzero = any(abs(x) > 1e-12 for x in applied)
        if not active and nonzero:
            counts['inactive_nonzero_targets'] += 1
        if active and fresh:
            counts['active_fresh_frames'] += 1
            difference = math.sqrt(sum((a - b)**2 for a, b in zip(received, applied)))
            max_disagreement = max(max_disagreement, difference)
            counts['subscriber_disagreements'] += difference > 1e-9
        if source == 'callback':
            if active and fresh:
                bad = applied_seq != receipt_seq or applied != received
            else:
                bad = nonzero or applied_seq != 0
                counts['callback_stale_nonzero_targets'] += not fresh and nonzero
            counts['callback_contract_errors'] += bad
    return dict(path=str(path), sources=sorted(sources), **counts,
                max_subscriber_disagreement_mps=max_disagreement)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--pattern', default='*.csv')
    parser.add_argument('--output', default='execution_samples_audit.json')
    args = parser.parse_args()
    results = [audit(path) for path in sorted(args.directory.glob(args.pattern))]
    if not results:
        raise SystemExit('no instrumented CSV traces')
    report = dict(runs=results,
                  passed=all(not (row['callback_contract_errors'] or row['inactive_nonzero_targets'])
                             for row in results),
                  interpretation='OmniGraph and Python receive independently. Disagreement is not an '
                                 'end-to-end latency measurement. Callback checks bind the applied '
                                 'vector to its own received sequence; physical braking follows later.')
    (args.directory / args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
