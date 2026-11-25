"""Run the same-frame FIFO5 comparison after a completed R05 shadow flight."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

out=Path(__file__).resolve().parent
root=out.parents[2]
if os.getuid()==0:sys.exit('launch as the ordinary project user')
guide=out.parent/'auto_turn_20261119'
load=lambda p: json.loads(p.read_text())
measured=load(root/load(guide/'latest_build.json')['manifest'])
for filename in ('controller.cpp', 'image_depth_clearance.hpp'):
    archived_hash = measured['source_identity']['apps/auto_cruise/' + filename]
    assert hashlib.sha256((root/'src/auto_cruise'/filename).read_bytes()).hexdigest() == archived_hash
valid=[]
for r in sorted((out/'parity').glob('run_*')):
    e=load(r/'execution.json')
    if e.get('audit_passed') and e['scope']['scenario']=='r05' and load(r/'simulation/result.json')['status']=='ARRIVED':
        valid.append(r)
if not valid:sys.exit('complete the R05 per-ball validation first')
sim=valid[-1]/'simulation'
exe=out/'paired_queries'
cmd=['/usr/bin/sudo','/usr/bin/prlimit','--rtprio=5:5','--rttime=5000000:6000000','--','/usr/bin/setpriv',
     f'--reuid={os.getuid()}',f'--regid={os.getgid()}','--init-groups','--bounding-set=-all',
     '--inh-caps=-all','--ambient-caps=-all','--no-new-privs','--',str(exe),str(sim/'simulation.sensors.json'),str(sim/'refinement_frames')]
with open('/tmp/fov_gvf_user_navigation.lock','rb') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    with (out/'paired_query_timings.tmp').open('w') as output:
        subprocess.run(cmd,stdout=output,check=True)
    (out/'paired_query_timings.tmp').replace(out/'paired_query_timings.csv')
evidence={'capture':str(sim.relative_to(root)), 'paired_binary_sha256':hashlib.sha256(exe.read_bytes()).hexdigest(),
          'measured_flight_binary':measured['binary'], 'compared_geometry_sources':{
             f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (out/'before/controller.cpp',root/'src/auto_cruise/image_depth_clearance.hpp',out/'paired_queries.cpp')}}
(out/'paired_identity.json').write_text(json.dumps(evidence,indent=2)+'\n')
