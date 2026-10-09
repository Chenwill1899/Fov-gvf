#!/usr/bin/env python3
"""Run the six C++ production-core ablations, preserving every stdout/stderr."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('output',type=Path)
p.add_argument('--build',action='store_true')
p.add_argument('--checks', nargs='+', help='Run selected production checks for one integration stage')
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
a.output.mkdir(parents=True,exist_ok=False)
if a.build:
    with (a.output/'build.log').open('w') as stream:
        subprocess.run(['bash',str(root/'scripts/build_isaac_ros_workspace.sh')],cwd=root,stdout=stream,stderr=subprocess.STDOUT,check=True)
build=Path('/tmp/fov_gvf_ego1p5_isaac_build/pc_gvf')
checks=['depth_uncertainty_check','dynamic_obstacles_check','dynamic_closed_loop_check','incremental_field_check','spherical_memory_check','spherical_ingest_check','operator_intent_check','shared_obstacles_check','six_axis_replay_check']
if a.checks:
    unknown=set(a.checks)-set(checks)
    if unknown:p.error(f'unknown checks: {sorted(unknown)}')
    checks=a.checks
results=[]
for check in checks:
    exe=build/check
    start=time.monotonic()
    run=subprocess.run([str(exe)],text=True,capture_output=True,timeout=120)
    (a.output/(check+'.csv')).write_text(run.stdout)
    (a.output/(check+'.stderr')).write_text(run.stderr)
    results.append(dict(check=check,returncode=run.returncode,wall_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
    (a.output/'summary.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
raise SystemExit(0 if all(r['returncode']==0 for r in results) else 1)
