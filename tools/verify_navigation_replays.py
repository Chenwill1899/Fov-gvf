#!/usr/bin/env python3
"""Re-run saved controller decisions; timing watchdogs are deliberately excluded."""
import argparse
import json
from pathlib import Path
import subprocess
import math

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('directory', type=Path)
p.add_argument('--executable', type=Path, default=Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/lib/pc_gvf/paper_replay'))
a = p.parse_args()
results = []
for file in sorted(a.directory.glob('frame_*.bin')):
    expected = json.loads(file.with_suffix('.json').read_text())
    process = subprocess.run([str(a.executable), str(file)], text=True, capture_output=True, timeout=30)
    if process.returncode:
        results.append({'file': str(file), 'pass': False, 'error': process.stderr})
        continue
    actual = json.loads(process.stdout)
    exact = all(actual[k] == expected[k] for k in ('status', 'reason', 'accepted', 'free_directions', 'refined'))
    numeric = all(math.isclose(actual[k], expected[k], rel_tol=1e-10, abs_tol=1e-10)
                  for k in ('best_prefix', 'required_prefix'))
    numeric &= all(math.isclose(x, y, rel_tol=1e-10, abs_tol=1e-10)
                   for x, y in zip(actual['command'], expected['command']))
    for key in ('command_changed', 'reset_reason', 'continued'):
        if key in expected:
            exact &= actual.get(key) == expected[key]
    if 'intent_change_angle' in expected:
        numeric &= math.isclose(actual.get('intent_change_angle', float('nan')), expected['intent_change_angle'], rel_tol=1e-10, abs_tol=1e-10)
    results.append({'file': str(file), 'pass': exact and numeric, 'expected': expected, 'actual': actual})
print(json.dumps({'count': len(results), 'pass': bool(results) and all(r['pass'] for r in results),
                  'frames': results}, ensure_ascii=False, indent=2))
raise SystemExit(0 if results and all(r['pass'] for r in results) else 1)
