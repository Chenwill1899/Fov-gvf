#!/usr/bin/env python3
"""Check all predeclared trials; report every metric and failed gate."""
import argparse,json,csv,hashlib
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('label');a=p.parse_args()
root=Path(__file__).resolve().parent
m=json.loads((root/(a.label+'_analysis.json')).read_text())
navigation=root.parent/'navigation_benchmark_20260929'
f=json.loads((navigation/('omni_'+a.label+'_analysis.json')).read_text())
checks={};stats={}
keys=['moving_angle_mean_deg','active_vector_error_rms_mps','active_projected_progress_m','stop_s','longest_stop_s','jerk_rms_mps3']
for case,n in [('sweep',3),('spin',3),('recovery',1)]:
 for algorithm in ['ego1p2','ego1p3']:
  runs=[r for r in m['runs'] if r['case']==case and r['algorithm']==algorithm]
  group=case+'/'+algorithm
  checks[group+'/count']=len(runs)==n
  checks[group+'/safe']=all(r['returncode']==0 and r['safety']['swept_sphere_overlap_segments']==0 and r['safety']['external_collision_blocks']==0 for r in runs)
  stats[group]={k:{'mean':float(np.mean([r[k] for r in runs])), 'std':float(np.std([r[k] for r in runs],ddof=1)) if len(runs)>1 else None,'min':min(r[k] for r in runs),'max':max(r[k] for r in runs)} for k in keys}
  checks[group+'/speed_contract']=True
  for r in runs:
   rows=list(csv.DictReader((root/(r['name']+'.csv')).open()))
   v=np.array([[float(row[k]) for k in ['vx','vy','vz']] for row in rows]);t=np.array([float(row['t']) for row in rows]);acc=np.linalg.norm(np.diff(v,axis=0)/np.diff(t)[:,None],axis=1)
   bounds={'speed':float(np.linalg.norm(v,axis=1).max()),'vertical':float(np.abs(v[:,2]).max()),'accel':float(acc.max())}
   stats[r['name']+'/bounds']=bounds
   checks[group+'/speed_contract'] &= bounds['speed']<=2.+1e-6 and bounds['vertical']<=1.+1e-6 and bounds['accel']<=1.2+1e-5
for case in ['sweep','spin']:
 old=stats[case+'/ego1p2'];new=stats[case+'/ego1p3']
 for key in ['moving_angle_mean_deg','active_vector_error_rms_mps','stop_s']:
  checks[case+'/'+key+'_nonregression']=new[key]['mean']<=old[key]['mean']+1e-9
 checks[case+'/progress_nonregression']=new['active_projected_progress_m']['mean']>=old['active_projected_progress_m']['mean']-1e-9
for alg in ['ego1p2','ego1p3']:
 checks['goal/'+alg+'/count_and_safe']=f['groups'][alg]['runs']==5 and f['groups'][alg]['safe_arrivals']==5
 trials=[t for t in f['trials'] if t['algorithm']==alg]
 checks['goal/'+alg+'/speed_contract']=all(t.get('maximum_speed_mps',float('inf'))<=2.+1e-6 and t.get('maximum_vertical_speed_mps',float('inf'))<=1.+1e-6 and t.get('acceleration_max_mps2',float('inf'))<=1.2+1e-5 for t in trials)
old=f['groups']['ego1p2'];new=f['groups']['ego1p3']
checks['goal/time_nonregression']=new['mean_arrival_sim_s']<=old['mean_arrival_sim_s']
checks['goal/stop_nonregression']=new['mean_stopped_s']<=old['mean_stopped_s']+1e-9
report={'label':a.label,'checks':checks,'all_predeclared_checks_pass':all(checks.values()),'statistics':stats,'navigation':f['groups'],'limits':'Means of this frozen finite static Cloud test suite; not a proof of universal superiority. Jerk and runtime are reported even when they regress.'}
(root/(a.label+'_acceptance.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'passed':all(checks.values()),'failed':[k for k,v in checks.items() if not v]},indent=2))
