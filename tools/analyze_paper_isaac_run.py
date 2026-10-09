#!/usr/bin/env python3
"""Summarize a bounded forward-intent Isaac run; never treat launch success as navigation success."""
import argparse
import json
import math
import re
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('log', type=Path)
parser.add_argument('--minimum-forward-progress', type=float, default=8.)
args = parser.parse_args()
text = args.log.read_text()
nav = re.findall(r'\[NAV\] t=([\d.]+)s position=\(([-\d.,]+)\).*?collision_blocks=(\d+)', text)
final = re.search(r'\[MANUAL RESULT\] TIMEOUT position=\(([-\d.,]+)\)', text)
if not nav or final is None:
    raise SystemExit('Missing initial/final position evidence')
start = [float(x) for x in nav[0][1].split(',')]
end = [float(x) for x in final[1].split(',')]
blocks = [int(x) for x in re.findall(r'collision_blocks=(\d+)', text)]
metrics = {
    'log': args.log.name,
    'start': start, 'end': end,
    'displacement_m': math.dist(start, end),
    'forward_progress_m': end[0]-start[0],
    'minimum_forward_progress_m': args.minimum_forward_progress,
    'external_collision_blocks': max(blocks, default=-1),
    'compute_deadline_status_transitions': text.count(']: COMPUTE_DEADLINE\n'),
    'clean_controller_exit': bool(re.search(r'\[depth_angular_controller-\d+\]: process has finished cleanly', text)),
}
performance = re.search(r'\[PERF CTRL FINAL\].*?command_fps\(ros/wall\)=([\d.]+)/([\d.]+).*?callback_to_publish_ms\(mean/p95/max\)=([\d.]+)/([\d.]+)/([\d.]+)', text)
if performance:
    metrics.update(command_ros_hz=float(performance[1]), command_wall_hz=float(performance[2]),
                   callback_p95_ms=float(performance[4]), callback_max_ms=float(performance[5]))
# A progress check and the paper's 20 ms update requirement are separate claims.
cycle = re.search(r'\[PERF CTRL FINAL\].*?cycle_ms\(mean/p95/max\)=([\d.]+)/([\d.]+)/([\d.]+)', text)
if cycle:
    metrics['command_interval_max_ms'] = float(cycle[3])
metrics['every_control_update_within_20ms'] = (performance is not None and cycle is not None and
                                             float(performance[5]) <= 20. and float(cycle[3]) <= 20.)
metrics['forward_navigation_pass'] = (metrics['forward_progress_m'] >= args.minimum_forward_progress and
                                      metrics['external_collision_blocks'] == 0 and metrics['clean_controller_exit'])
print(json.dumps(metrics, indent=2))
raise SystemExit(0 if metrics['forward_navigation_pass'] else 1)
