#!/usr/bin/env python3
"""Paired replay of actual P5 states; original watchdog timing is not simulated."""
import argparse, hashlib, json, subprocess, time
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('output',type=Path)
p.add_argument('directories',nargs='+',type=Path)
p.add_argument('--baseline',required=True,type=Path)
p.add_argument('--current',type=Path,default=Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/lib/pc_gvf/paper_replay'))
p.add_argument('--repeats',type=int,default=3)
a=p.parse_args()
if a.output.exists():p.error('refusing to overwrite output')
files=sorted({f for d in a.directories for f in d.glob('frame_*.bin')})
if not files:p.error('no frames')
rows=[];legacy_equal=True
for file in files:
    original=subprocess.run([str(a.baseline),str(file)],text=True,capture_output=True,timeout=30)
    old=json.loads(original.stdout) if original.returncode==0 else None
    for repeat in range(a.repeats):
        for enabled in ((False,True) if repeat%2==0 else (True,False)):
            args=[str(a.current),str(file)]+(['--incremental-field'] if enabled else [])
            begin=time.perf_counter();run=subprocess.run(args,text=True,capture_output=True,timeout=30)
            elapsed=1000*(time.perf_counter()-begin)
            result=json.loads(run.stdout) if run.returncode==0 else None
            if not enabled:legacy_equal &= result==old and old is not None
            rows.append(dict(file=str(file),repeat=repeat,incremental=enabled,ms=elapsed,returncode=run.returncode,result=result,stderr=run.stderr,baseline_result=old))
    a.output.write_text(json.dumps(dict(complete=False,rows=rows),indent=2))
summary={}
for enabled in (False,True):
    rs=[r for r in rows if r['incremental']==enabled]
    summary[str(enabled)]=dict(mean_ms=sum(r['ms'] for r in rs)/len(rs),max_ms=max(r['ms'] for r in rs),accepted=sum(bool(r['result'] and r['result']['accepted']) for r in rs),runs=len(rs))
a.output.write_text(json.dumps(dict(complete=True,frames=len(files),legacy_equal=legacy_equal,summary=summary,scope='same stored P5 core state; subprocess launch included; no frontend or watchdog',baseline_sha256=hashlib.sha256(a.baseline.read_bytes()).hexdigest(),current_sha256=hashlib.sha256(a.current.read_bytes()).hexdigest(),rows=rows),indent=2))
print(json.dumps(dict(frames=len(files),legacy_equal=legacy_equal,summary=summary),indent=2))
raise SystemExit(0 if legacy_equal and all(r['returncode']==0 for r in rows) else 1)
