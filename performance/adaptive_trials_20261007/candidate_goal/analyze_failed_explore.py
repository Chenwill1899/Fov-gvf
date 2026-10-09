#!/usr/bin/env python3
"""Lightweight descriptive diagnosis of completed B pairs 1/2/3; no simulator."""
import csv, json, math, re
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[3]
folder=root/'performance/navigation_benchmark_20260929'
logs=root/'performance/adaptive_trials_20261007/closed_loop/p5_goal_polish_explore_20261007'
protocol=json.loads((folder/'protocol.json').read_text())
start=np.array(protocol['start']);goal=np.array(protocol['goal']);axis=(goal-start)/np.linalg.norm(goal-start)
output={'scope':'All three matched pairs, including after3 timeout; descriptive diagnosis, no new experiment.', 'runs':[]}
for variant in ('before','after'):
 for rep in (1,2,3):
  stem=f'p5_goal_polish_explore_20261007_{variant}_ego1p5_{rep}'
  rows=list(csv.DictReader((folder/(stem+'.csv')).open()))
  def vec(keys):return np.array([[float(row[k]) if row[k] else float("nan") for k in keys] for row in rows])
  t=vec(['t'])[:,0];dt=np.diff(t);p=vec(['x','y','z']);v=vec(['vx','vy','vz']);q=vec(['qx','qy','qz']);target=vec(['target_x','target_y','target_z'])
  speed=np.linalg.norm(v,axis=1);active=np.linalg.norm(q,axis=1)>.05;tn=np.linalg.norm(target,axis=1);target_unit=target/np.maximum(tn[:,None],1e-15)
  jumps=np.degrees(np.arccos(np.clip(np.sum(target_unit[:-1]*target_unit[1:],axis=1),-1,1)))
  valid=active[:-1]&active[1:]&(tn[:-1]>.5)&(tn[1:]>.5)
  stamps=vec(['diagnostic_stamp'])[:,0];fresh=np.diff(stamps)>0
  fast=fresh&(np.diff(stamps)<=.05)&valid
  accel=np.diff(v,axis=0)/dt[:,None];jerk=np.linalg.norm(np.diff(accel,axis=0)/dt[1:,None],axis=1)
  deadlines=np.array([row['status']=='COMPUTE_DEADLINE' for row in rows]);near=np.zeros(len(jerk),dtype=bool)
  for when in t[deadlines]:near|=np.abs(t[2:]-when)<=.25
  age=vec(['twist_age_s'])[:,0];seq=vec(['applied_twist_sequence'])[:,0];projection=(p-start)@axis
  perf=(logs/f'p5_goal_polish_explore_20261007_goal_{variant}_{rep}_performance.md').read_text()
  log=(logs/f'p5_goal_polish_explore_20261007_goal_{variant}_{rep}.log').read_text()
  refine=[tuple(map(int,m)) for m in re.findall(r'GOAL_REFINEMENT checks=(\d+) improvements=(\d+)',log)]
  metrics=dict(run=stem,active_s=float(dt[active[:-1]].sum()),slow_below_1p5_s=float(dt[active[:-1]&(speed[:-1]<1.5)].sum()),active_mean_speed=float(np.sum(speed[:-1]*dt*active[:-1])/np.sum(dt*active[:-1])),target_tv_deg=float(jumps[valid].sum()),target_jumps_gt2=int(np.sum(valid&(jumps>2))),fast_target_jumps_gt2=int(np.sum(fast&(jumps>2))),fast_target_jumps_gt5=int(np.sum(fast&(jumps>5))),deadline_csv_rows=int(deadlines.sum()),deadline_unique_diagnostics=len(set(stamps[deadlines])),deadline_occupancy_s=float(dt[deadlines[:-1]].sum()),active_zero_applied_seq_s=float(dt[active[:-1]&(seq[:-1]==0)].sum()),active_stale_sample_s=float(dt[active[:-1]&(age[:-1]>.10)].sum()),jerk_rms=float(np.sqrt(np.mean(jerk**2))),jerk_energy_fraction_within_250ms_deadline=float(np.sum(jerk[near]**2)/np.sum(jerk**2)),refinement_throttled_log_samples=len(refine),refinement_throttled_log_positive_samples=sum(b>0 for a,b in refine),performance_lines=[line for line in perf.splitlines() if 'Avoidance compute mean' in line or 'Control callback to publish mean' in line])
  metrics['bins']=[]
  for lo in range(0,140,20):
   mask=active[:-1]&(projection[:-1]>=lo)&(projection[:-1]<lo+20)
   metrics['bins'].append(dict(projected_progress_begin_m=lo,time_s=float(dt[mask].sum()),slow_s=float(dt[mask&(speed[:-1]<1.5)].sum()),deadline_s=float(dt[mask&deadlines[:-1]].sum()),target_jumps_gt2=int(np.sum(mask&valid&(jumps>2)))))
  idx=np.flatnonzero(fast&(jumps>5))
  metrics['first_fast_jumps_gt5']=[dict(t=float(t[i+1]),angle_deg=float(jumps[i]),intent_change_deg=float(np.degrees(np.arccos(np.clip(q[i]@q[i+1]/max(np.linalg.norm(q[i])*np.linalg.norm(q[i+1]),1e-15),-1,1)))),position=p[i+1].tolist(),status=rows[i+1]['status']) for i in idx[:12]]
  output['runs'].append(metrics)
path=Path(__file__).with_name('failed_explore_diagnosis.json')
with path.open('x') as f:json.dump(output,f,indent=2)
for run in output['runs']:
 print(json.dumps({k:v for k,v in run.items() if k not in ('first_fast_jumps_gt5','performance_lines')},ensure_ascii=False))
