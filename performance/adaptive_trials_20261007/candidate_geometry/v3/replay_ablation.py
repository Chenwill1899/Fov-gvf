#!/usr/bin/env python3
"""Read-only same-snapshot ablation of frozen and candidate production cores."""
from pathlib import Path
import argparse,hashlib,json,os,resource,subprocess,time,statistics
parser=argparse.ArgumentParser();parser.add_argument("--label",choices=["exploration","confirmation"],default="exploration");args=parser.parse_args()
root=Path(__file__).resolve().parents[4]
stage=Path(__file__).resolve().parent
executables={'baseline':stage.parent.parent/'baseline/paper_replay','candidate':stage/'paper_replay'}
frames=[]
for group,source in [('stall16','stall_replay.json'),('inherited21','inherited_replay.json')]:
    for row in json.loads((root/'performance/six_axis_refinement_20261007'/source).read_text())['rows']:
        if row['file'] not in [f['file'] for f in frames]:frames.append({'group':group,'file':row['file']})
assert len(frames)==37
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
frozen={k:sha(p) for k,p in executables.items()}
frame_sha256={f['file']:sha(root/f['file']) for f in frames}
manifest=dict(label=args.label, started_unix=time.time(), frame_sha256=frame_sha256, executables={k:str(p) for k,p in executables.items()},sha256=frozen,frames=frames,environment={'OMP_WAIT_POLICY':'PASSIVE'},complete=False,rows=[])
path=stage/('replay_ablation.json' if args.label=='exploration' else 'confirmation_ablation.json')
if path.exists():raise RuntimeError('refusing overwrite')
path.write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,OMP_WAIT_POLICY='PASSIVE')
for repeat in range(5):
    for frame in frames:
        pair={}
        for variant in (['baseline','candidate'] if (repeat+(args.label=='confirmation'))%2==0 else ['candidate','baseline']):
            before=resource.getrusage(resource.RUSAGE_CHILDREN);start=time.perf_counter()
            r=subprocess.run([str(executables[variant]),str(root/frame['file'])],capture_output=True,text=True,env=env)
            wall=(time.perf_counter()-start)*1000;after=resource.getrusage(resource.RUSAGE_CHILDREN)
            row=dict(repeat=repeat,variant=variant,**frame,wall_ms=wall,cpu_ms=1000*((after.ru_utime+after.ru_stime)-(before.ru_utime+before.ru_stime)),returncode=r.returncode,stdout=r.stdout,stderr=r.stderr)
            manifest['rows'].append(row);pair[variant]=row
        same=(pair['baseline']['returncode']==pair['candidate']['returncode']==0 and pair['baseline']['stdout']==pair['candidate']['stdout'])
        manifest['rows'][-1]['pair_exact_output_equal']=same
        path.write_text(json.dumps(manifest,indent=2)+'\n')
        if not same:raise RuntimeError('decision/output mismatch preserved')
    print('batch',repeat+1,'37 pairs equal',flush=True)
assert all(sha(p)==frozen[k] for k,p in executables.items())
assert all(sha(root/f)==digest for f,digest in frame_sha256.items())
def percentile(vals,q):
    a=sorted(vals);u=(len(a)-1)*q;i=int(u);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(u-i)
summary={}
for group in ['all','stall16','inherited21']:
    summary[group]={}
    for variant in executables:
        rows=[r for r in manifest['rows'] if r['variant']==variant and (group=='all' or r['group']==group)]
        summary[group][variant]={metric:{'mean':statistics.mean(r[metric] for r in rows),'p50':percentile([r[metric] for r in rows],.5),'p95':percentile([r[metric] for r in rows],.95),'max':max(r[metric] for r in rows)} for metric in ['wall_ms','cpu_ms']}
    summary[group]['p95_wall_reduction_fraction']=1-summary[group]['candidate']['wall_ms']['p95']/summary[group]['baseline']['wall_ms']['p95']
batch_summary={}
for repeat in range(5):
    batch_summary[str(repeat)]={}
    for group in ['all','stall16','inherited21']:
        group_stats={}
        for variant in executables:
            rows=[r for r in manifest['rows'] if r['repeat']==repeat and r['variant']==variant and (group=='all' or r['group']==group)]
            group_stats[variant]={metric:{'mean':statistics.mean(r[metric] for r in rows),'p95':percentile([r[metric] for r in rows],.95),'max':max(r[metric] for r in rows)} for metric in ['wall_ms','cpu_ms']}
        group_stats['p95_wall_reduction_fraction']=1-group_stats['candidate']['wall_ms']['p95']/group_stats['baseline']['wall_ms']['p95']
        batch_summary[str(repeat)][group]=group_stats
manifest.update(complete=True,ended_unix=time.time(),exact_pairs=185,summary=summary,batch_summary=batch_summary)
path.write_text(json.dumps(manifest,indent=2)+'\n')
(stage/('summary.json' if args.label=='exploration' else 'confirmation_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
(stage/('batch_summary.json' if args.label=='exploration' else 'confirmation_batch_summary.json')).write_text(json.dumps(batch_summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
