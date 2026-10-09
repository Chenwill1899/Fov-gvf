#!/usr/bin/env python3
"""Same-binary profile ablation, interleaved and audited after each trial."""
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('label');p.add_argument('--repeats',type=int,default=2)
p.add_argument('--profiles',nargs='+',choices=['baseline','default','spherical','uncertainty'],default=['baseline','default','spherical'])
a=p.parse_args();root=Path(__file__).resolve().parents[1];out=root/'performance/six_axis_20261007/final';out.mkdir(exist_ok=True)
manifest=out/(a.label+'_goals.json')
if manifest.exists():p.error('refusing to overwrite manifest')
exe=Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/lib/pc_gvf/depth_angular_controller')
launch=root/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
frozen=sha(exe);launch_sha=sha(launch);runs=[]
for repeat in range(1,a.repeats+1):
 for profile in (a.profiles if repeat%2 else list(reversed(a.profiles))):
  assert sha(exe)==frozen and sha(launch)==launch_sha,'frozen runtime changed'
  flags=dict(FOV_GVF_DEPTH_UNCERTAINTY=str(int(profile=='uncertainty')),FOV_GVF_DYNAMIC_OBSTACLES='0',FOV_GVF_INCREMENTAL_FIELD=str(int(profile!='baseline')),FOV_GVF_SPHERICAL_MEMORY=str(int(profile=='spherical')),FOV_GVF_OPERATOR_ASSISTANCE=str(int(profile!='baseline')),FOV_GVF_SHARED_OBSTACLES='0')
  env=dict(os.environ,**flags,PYTHONNOUSERSITE='1');label=a.label+'_'+profile
  begin=time.monotonic();run=subprocess.run(['/usr/bin/python3',str(root/'tools/run_navigation_benchmark.py'),label,'ego1p5','--repeat-start',str(repeat)],cwd=root,env=env)
  result=root/f'performance/navigation_benchmark_20260929/{label}_ego1p5_{repeat}.json'
  runs.append(dict(profile=profile,repeat=repeat,flags=flags,returncode=run.returncode,wall_s=time.monotonic()-begin,result=json.loads(result.read_text()) if result.exists() else None))
  manifest.write_text(json.dumps(dict(controller_sha256=frozen,launch_sha256=launch_sha,runs=runs),indent=2))
  if result.exists():
   with (out/(label+'_analysis.stdout')).open('w') as stream:subprocess.run(['/usr/bin/python3',str(root/'tools/analyze_navigation_benchmark.py'),label],cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True)
  if run.returncode:raise SystemExit(run.returncode)
