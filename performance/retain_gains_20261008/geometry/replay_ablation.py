#!/usr/bin/env python3
"""Frozen four-variant interleaved replay; no timing-based early stopping."""
from pathlib import Path
import hashlib,json,os,resource,statistics,subprocess,time
stage=Path(__file__).resolve().parent
root=stage.parents[2]
plan_path=stage/'plan.json';plan=json.loads(plan_path.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
variants=['baseline','v2','v4','combined']
executables={v:stage/v/'paper_replay' for v in variants}
freeze=json.loads((stage/'frozen_manifest.json').read_text())
assert sha(plan_path)==freeze['plan_sha256']
for v in variants:
    assert sha(executables[v])==freeze['binary_sha256'][v]
    assert sha(stage/v/'paper_guidance.cpp')==plan['source_sha256'][v]
for f in plan['fixtures']:assert sha(root/f['file'])==f['sha256']
path=stage/'replay_ablation.json'
if path.exists():raise RuntimeError('refusing to overwrite prior measurements')
manifest=dict(complete=False,started_unix=time.time(),plan_sha256=sha(plan_path),sha256=freeze['binary_sha256'],source_sha256=plan['source_sha256'],executables={v:str(p) for v,p in executables.items()},frames=plan['fixtures'],orders=plan['orders'],environment={'OMP_WAIT_POLICY':'PASSIVE'},rows=[])
def save():path.write_text(json.dumps(manifest,indent=2)+'\n')
save();env=dict(os.environ,OMP_WAIT_POLICY='PASSIVE')
for repeat,order in enumerate(plan['orders']):
    for frame in plan['fixtures']:
        case={}
        for position,variant in enumerate(order):
            before=resource.getrusage(resource.RUSAGE_CHILDREN);start=time.perf_counter()
            r=subprocess.run([str(executables[variant]),str(root/frame['file'])],capture_output=True,text=True,env=env)
            wall=(time.perf_counter()-start)*1000;after=resource.getrusage(resource.RUSAGE_CHILDREN)
            row=dict(repeat=repeat,variant=variant,group=frame['group'],file=frame['file'],within_frame_position=position,wall_ms=wall,cpu_ms=1000*((after.ru_utime+after.ru_stime)-(before.ru_utime+before.ru_stime)),returncode=r.returncode,stdout=r.stdout,stderr=r.stderr)
            case[variant]=row;manifest['rows'].append(row)
        same=True
        for variant,row in case.items():
            row['exact_equal_to_baseline']=row['returncode']==case['baseline']['returncode']==0 and row['stdout']==case['baseline']['stdout']
            same=same and row['exact_equal_to_baseline']
        save()
        if not same:raise RuntimeError('output/returncode mismatch; complete case retained')
    print('batch',repeat+1,'37 four-variant cases exact',flush=True)
for v in variants:
    assert sha(executables[v])==freeze['binary_sha256'][v]
    assert sha(stage/v/'paper_guidance.cpp')==plan['source_sha256'][v]
for f in plan['fixtures']:assert sha(root/f['file'])==f['sha256']
assert sha(plan_path)==freeze['plan_sha256']
def percentile(values,q):
    a=sorted(values);u=(len(a)-1)*q;i=int(u);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(u-i)
def summarize(rows):
    return {metric:{'mean':statistics.mean(r[metric] for r in rows),'p50':percentile([r[metric] for r in rows],.5),'p95':percentile([r[metric] for r in rows],.95),'max':max(r[metric] for r in rows)} for metric in ['wall_ms','cpu_ms']}
groups=['all','stall16','inherited21']
summary={g:{v:summarize([r for r in manifest['rows'] if r['variant']==v and (g=='all' or r['group']==g)]) for v in variants} for g in groups}
batch={str(n):{g:{v:summarize([r for r in manifest['rows'] if r['repeat']==n and r['variant']==v and (g=='all' or r['group']==g)]) for v in variants} for g in groups} for n in range(len(plan['orders']))}
manifest.update(complete=True,ended_unix=time.time(),exact_four_variant_cases=len(plan['orders'])*len(plan['fixtures']),exact_comparisons_to_baseline=3*len(plan['orders'])*len(plan['fixtures']),summary=summary,batch_summary=batch)
save();(stage/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(stage/'batch_summary.json').write_text(json.dumps(batch,indent=2)+'\n')
print('complete; summaries saved',flush=True)
