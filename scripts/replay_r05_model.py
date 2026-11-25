#!/usr/bin/python3
"""Replay the latest audited P3 ARRIVED flight with the original R05 view and ARL model."""
import argparse,fcntl,json,os,subprocess
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path);p.add_argument('--preview',action='store_true');p.add_argument('--validate-depth',action='store_true');a=p.parse_args()
if os.geteuid()==0:p.error('render as the ordinary user; no sudo/FIFO is needed for replay')
run=a.run.resolve() if a.run else None
if run is None:
 for folder in sorted((ROOT/'docs/performance/r05_model_sync_20261117').glob('run_2*'),reverse=True):
  try:
   if json.loads((folder/'execution.json').read_text()).get('audit_passed') and json.loads((folder/'simulation/result.json').read_text())['status']=='ARRIVED':run=folder;break
  except (FileNotFoundError,KeyError):pass
if run is None or not run.is_relative_to(ROOT):p.error('requires an audited ARRIVED run inside P3')
lock=open('/tmp/fov_gvf_user_navigation.lock','a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
out=ROOT/'docs/performance/r05_visual_match_20261118'/('preview_' if a.preview else 'replay_')
out=out.with_name(out.name+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1')
for key in ['LD_PRELOAD','LD_AUDIT','PYTHONPATH','PYTHONHOME']:env.pop(key,None)
cmd=[str(ROOT.parent/'isaacsim/python.sh'),str(ROOT/'src/r05_model/replay.py'),'--run',str(run),'--output',str(out)]
if a.preview:cmd.append('--preview')
if a.validate_depth:cmd.append('--validate-depth')
log=out.with_suffix('.console.log')
print('OUTPUT_DIR='+str(out),flush=True)
with log.open('w') as stream:r=subprocess.run(cmd,env=env,stdout=stream,stderr=subprocess.STDOUT)
if r.returncode or not (out/'video.json').is_file() or (out/'failure.json').exists():raise RuntimeError('replay failed; see '+str(log))
print('OUTPUT_DIR='+str(out),flush=True)
