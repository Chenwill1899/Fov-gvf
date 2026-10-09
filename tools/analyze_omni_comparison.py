#!/usr/bin/env python3
"""Aggregate all matching trials, retaining every failure and safety audit."""
import argparse,csv,json,re
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--directory',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[1];out=a.directory or root/'performance/omni_20261006'
results=[]
for meta in sorted(out.glob(a.label+'_*_process.json')):
    m=json.loads(meta.read_text());name=meta.name.removesuffix('_process.json')
    rows=list(csv.DictReader((out/(name+'.csv')).open()))
    arr=lambda keys:np.array([[float(r[k]) for k in keys] for r in rows])
    t=arr(['t'])[:,0];dt=np.diff(t);v=arr(['vx','vy','vz']);q=arr(['qx','qy','qz'])
    speed=np.linalg.norm(v,axis=1);qs=np.linalg.norm(q,axis=1);active=qs>.05
    moving=active&(speed>.05)
    cosine=np.sum(v[moving]*q[moving],axis=1)/speed[moving]/qs[moving]
    angles=np.degrees(np.arccos(np.clip(cosine,-1,1)))
    error=np.linalg.norm(v-q,axis=1)
    acc=np.diff(v,axis=0)/dt[:,None]
    jerk=np.diff(acc,axis=0)/((dt[1:]+dt[:-1])/2)[:,None]
    perf=(out/(name+'_performance.md')).read_text()
    compute=re.findall(r'Avoidance compute mean / P50 / P95 / max: ([\d.]+) / ([\d.]+) / ([\d.]+) / ([\d.]+)',perf)
    log=(out/(name+'.log')).read_text()
    audit=json.loads((out/(name+'_audit.json')).read_text())
    r=dict(name=name,algorithm=m['algorithm'],case=m['case'],returncode=m['returncode'],
        moving_angle_mean_deg=float(np.mean(angles)),moving_angle_p95_deg=float(np.percentile(angles,95)),
        active_vector_error_rms_mps=float(np.sqrt(np.mean(error[active]**2))),
        active_projected_progress_m=float(np.sum(np.sum(v[:-1]*q[:-1],axis=1)/np.maximum(qs[:-1],1e-10)*dt*active[:-1])),
        jerk_rms_mps3=float(np.sqrt(np.mean(np.sum(jerk*jerk,axis=1)))),
        camera_switches=log.count('3D guidance switched to view'),
        compute_deadlines=log.count('COMPUTE_DEADLINE'),
        compute_ms=dict(zip(['mean','p50','p95','max'],map(float,compute[-1]))) if compute else None,
        safety={k:audit[k] for k in ('swept_sphere_overlap_segments','external_collision_blocks','minimum_swept_sphere_clearance_m')},
        stop_s=audit['stopped_with_input_s'],longest_stop_s=audit['longest_stopped_with_input_s'])
    results.append(r)
groups={}
for r in results:groups.setdefault(r['case']+'/'+r['algorithm'],[]).append(r)
summary={}
keys=['moving_angle_mean_deg','moving_angle_p95_deg','active_vector_error_rms_mps','active_projected_progress_m','jerk_rms_mps3','stop_s','longest_stop_s','camera_switches','compute_deadlines']
for group,rs in groups.items():
    summary[group]=dict(runs=len(rs),means={k:float(np.mean([r[k] for r in rs])) for k in keys},
        safe_runs=sum(r['returncode']==0 and r['safety']['swept_sphere_overlap_segments']==0 and r['safety']['external_collision_blocks']==0 for r in rs),
        compute_mean_ms=float(np.mean([r['compute_ms']['mean'] for r in rs if r['compute_ms']])) if any(r['compute_ms'] for r in rs) else None)
report=dict(label=a.label,runs=results,summary=summary,
    interpretation='Tracking error includes necessary avoidance. Zero collisions alone is insufficient; compare progress and stalls alongside error. This audit covers only these scripted static Cloud scenarios.')
(out/(a.label+'_analysis.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
