#!/usr/bin/env python3
"""Repeat identical world-intent / independent-yaw traces, with unchanged P2 binary.

Both algorithms use this project's main shell and identical simulator/scene.
FOV_GVF_INSTALL chooses the algorithm's own installed launch and executable.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('label')
p.add_argument('--case', choices=['sweep','spin','recovery'], default='sweep')
p.add_argument('--repeats',type=int,default=1)
p.add_argument('--algorithms',nargs='+',default=['ego1p2','ego1p5'],choices=['ego1p2','ego1p3','ego1p5'])
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
out=root/'performance/omni_20261006';out.mkdir(exist_ok=True)
fixture=root/'src/pc_gvf/test/fixtures/paper'/('isaac_omni_'+a.case+'_trace.json')
duration={'sweep':40,'spin':64,'recovery':72}[a.case]
if a.case=='recovery':fixture=root/'src/pc_gvf/test/fixtures/paper/isaac_stall_repair_trace.json'
for repeat in range(1,a.repeats+1):
    algorithms=a.algorithms if repeat%2 else list(reversed(a.algorithms))
    for algorithm in algorithms:
        name=f'{a.label}_{a.case}_{algorithm}_{repeat}'
        env=dict(os.environ,PYTHONNOUSERSITE='1',ISAAC_MANUAL_INPUT_MODE='trace',ISAAC_HEADLESS='1',
            FOV_GVF_RVIZ='false',FOV_GVF_SCENE_MODE='cloud',ISAAC_INTENT_TRACE=str(fixture),
            ISAAC_ACCEPTANCE_TRACE=str(out/(name+'.csv')),ISAAC_MANUAL_TIMEOUT=str(duration),
            FOV_GVF_RUN_ID=name,FOV_GVF_REPLAY_DIR=str(out/(name+'_replays')),
            FOV_GVF_PERFORMANCE_LOG=str(out/(name+'_performance.md')),ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1',
            FOV_GVF_INSTALL=f'/tmp/fov_gvf_{algorithm}_isaac_install')
        prefix=Path(env['FOV_GVF_INSTALL']);binary=prefix/'pc_gvf/lib/pc_gvf/depth_angular_controller'
        launch=prefix/'pc_gvf/share/pc_gvf/launch/isaac_cloud_navigation.launch.py'
        sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
        metadata={'algorithm':algorithm,'case':a.case,'duration_s':duration,'trace_sha256':sha(fixture),
            'controller_sha256':sha(binary),'launch_sha256':sha(launch),
            'shared_simulator_sha256':sha(root/'scripts/isaac/run_fov_gvf_navigation.py'),
            'scene_sha256':sha(root/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd')}
        start=time.monotonic();print('START',name,flush=True)
        with (out/(name+'.log')).open('x') as log:
            result=subprocess.run(['bash','scripts/run_isaac_fov_gvf_navigation.sh'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
        metadata.update(returncode=result.returncode,process_wall_s=time.monotonic()-start)
        (out/(name+'_process.json')).write_text(json.dumps(metadata,indent=2)+'\n')
        if result.returncode:raise SystemExit(f'{name} failed: {result.returncode}')
        audit=subprocess.run(['/usr/bin/python3','tools/analyze_envelope_trace.py',str(out/(name+'.csv'))],cwd=root,env=env,text=True,capture_output=True,check=True)
        (out/(name+'_audit.json')).write_text(audit.stdout)
        data=json.loads(audit.stdout)
        print('END',name,json.dumps({k:data[k] for k in ('stopped_with_input_s','longest_stopped_with_input_s','distance_travelled_m','swept_sphere_overlap_segments','external_collision_blocks')}),flush=True)
