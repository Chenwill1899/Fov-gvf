#!/usr/bin/env python3
"""Paired same-binary teleoperation trials through the supported Isaac entrypoint."""
import argparse, hashlib, json, os, subprocess, time
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('label');p.add_argument('--case',choices=['jitter','spin','sweep','recovery'],default='jitter')
p.add_argument('--compare',choices=['operator','spherical','defaults'],default='operator')
p.add_argument('--repeats',type=int,default=2)
a=p.parse_args();root=Path(__file__).resolve().parents[1]
out=root/'performance/six_axis_20261007/manual';out.mkdir(exist_ok=True)
fixture=root/'src/pc_gvf/test/fixtures/paper'/dict(jitter='isaac_operator_jitter_trace.json',spin='isaac_omni_spin_trace.json',sweep='isaac_omni_sweep_trace.json',recovery='isaac_stall_repair_trace.json')[a.case]
duration=dict(jitter=34,spin=64,sweep=40,recovery=72)[a.case]
exe=Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/lib/pc_gvf/depth_angular_controller')
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
frozen=sha(exe)
for repeat in range(1,a.repeats+1):
 for enabled in ((False,True) if repeat%2 else (True,False)):
  assert sha(exe)==frozen,'binary changed during frozen group'
  name=f'{a.label}_{a.case}_{"on" if enabled else "off"}_{repeat}'
  flags=dict(FOV_GVF_DEPTH_UNCERTAINTY='0',FOV_GVF_DYNAMIC_OBSTACLES='0',FOV_GVF_INCREMENTAL_FIELD='1',FOV_GVF_SPHERICAL_MEMORY='0',FOV_GVF_OPERATOR_ASSISTANCE='1',FOV_GVF_SHARED_OBSTACLES='0')
  if a.compare=='operator':flags['FOV_GVF_OPERATOR_ASSISTANCE']=str(int(enabled))
  elif a.compare=='spherical':flags['FOV_GVF_SPHERICAL_MEMORY']=str(int(enabled))
  else:
   flags['FOV_GVF_OPERATOR_ASSISTANCE']=flags['FOV_GVF_INCREMENTAL_FIELD']=str(int(enabled))
  env=dict(os.environ,**flags,PYTHONNOUSERSITE='1',ISAAC_MANUAL_INPUT_MODE='trace',ISAAC_HEADLESS='1',FOV_GVF_RVIZ='false',FOV_GVF_SCENE_MODE='cloud',ISAAC_INTENT_TRACE=str(fixture),ISAAC_ACCEPTANCE_TRACE=str(out/(name+'.csv')),ISAAC_MANUAL_TIMEOUT=str(duration),FOV_GVF_RUN_ID=name,FOV_GVF_REPLAY_DIR=str(out/(name+'_replays')),FOV_GVF_PERFORMANCE_LOG=str(out/(name+'_performance.md')),ROS_DOMAIN_ID='42',FOV_GVF_WAIT_FOR_LOCK='1')
  meta=dict(algorithm='p5_'+a.compare+('_on' if enabled else '_off'),case=a.case,duration_s=duration,flags=flags,trace_sha256=sha(fixture),controller_sha256=frozen,launch_sha256=sha(root/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'),shared_simulator_sha256=sha(root/'scripts/isaac/run_fov_gvf_navigation.py'),scene_sha256=sha(root/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'))
  begin=time.monotonic();print('START',name,flush=True)
  with (out/(name+'.log')).open('x') as log:r=subprocess.run(['bash',str(root/'scripts/run_isaac_fov_gvf_navigation.sh')],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
  meta.update(returncode=r.returncode,process_wall_s=time.monotonic()-begin);(out/(name+'_process.json')).write_text(json.dumps(meta,indent=2))
  if r.returncode:raise SystemExit(f'{name} failed: {r.returncode}')
  audit=subprocess.run(['/usr/bin/python3',str(root/'tools/analyze_envelope_trace.py'),str(out/(name+'.csv'))],env=env,text=True,capture_output=True,check=True)
  (out/(name+'_audit.json')).write_text(audit.stdout);print('END',name,flush=True)
