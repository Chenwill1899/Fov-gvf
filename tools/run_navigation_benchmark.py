#!/usr/bin/env python3
"""Serial trial driver; invokes the project's supported main run entrypoint."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('label')
parser.add_argument('algorithms', nargs='+', choices=['ego1p5', 'ego1p3', 'ego1p2', 'ego1p1', 'ego1p0', 'user_ego'])
parser.add_argument('--repeats', type=int, default=1)
parser.add_argument('--repeat-start', type=int, default=1, help='first unique trial index for interleaved profile groups')
parser.add_argument('--abort-stall', type=float, default=0.,
    help='exploratory trials only: abort after this many stationary seconds; 0 keeps full protocol')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = root / 'performance/navigation_benchmark_20260929'
for repeat in range(args.repeat_start, args.repeat_start+args.repeats):
    ordered=args.algorithms if repeat%2 else list(reversed(args.algorithms))
    for algorithm in ordered:
        run_id = f'{args.label}_{algorithm}_{repeat}'
        result = output / f'{run_id}.json'
        log = output / f'{run_id}.log'
        if result.exists() or log.exists():
            raise SystemExit(f'refusing to overwrite {run_id}')
        environment = dict(os.environ, ISAAC_MANUAL_INPUT_MODE='goal',
            ISAAC_HEADLESS='1', FOV_GVF_RVIZ='false',
            FOV_GVF_SCENE_MODE='cloud',
            FOV_GVF_BENCHMARK_ALGORITHM=algorithm,
            ISAAC_GOAL_PROTOCOL=str(output/'protocol.json'),
            ISAAC_BENCHMARK_RESULT=str(result),
            ISAAC_ACCEPTANCE_TRACE=str(output/f'{run_id}.csv'),
            FOV_GVF_RUN_ID=run_id,
            FOV_GVF_PERFORMANCE_LOG=str(output/'PERFORMANCE.md'),
            FOV_GVF_REPLAY_DIR=str(output/f'{run_id}_replays'),
            ISAAC_MANUAL_TIMEOUT='0', ISAAC_EXPLORATION_STALL_S=str(args.abort_stall), ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1')
        start = time.time()
        print(f'START {run_id}', flush=True)
        with log.open('w') as stream:
            completed = subprocess.run(['bash', str(root/'scripts/run_isaac_fov_gvf_navigation.sh')],
                cwd=root, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        if result.exists():
            data = json.loads(result.read_text())
        else:
            data = dict(status='RUNTIME_ERROR', algorithm=algorithm, run_id=run_id)
        data.update(process_exit_code=completed.returncode, process_wall_s=time.time()-start)
        result.write_text(json.dumps(data, indent=2))
        print(f'END {run_id} {data["status"]} {data.get("simulation_elapsed_s")}', flush=True)
