#!/usr/bin/env python3
"""Run fixed c36 routes sequentially; retain each attempt and full timing samples."""
from pathlib import Path
import os,json,csv,hashlib,subprocess,time,statistics,datetime,signal,sys,re,shutil
R=Path('/home/ubuntu5090/dhm/EGO2P6');T=R/'performance/c36_10x5_20261019_2032';TAG='c36_10x5_20261019_2032'
def stamp():return datetime.datetime.now().astimezone().isoformat()
def write(p,v):
 q=p.with_suffix(p.suffix+'.tmp');q.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n');q.replace(p)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert Path(__file__).resolve().parent==T
routes=json.loads((R/'performance/sync_candidate24_20261019_1240/source_5090_comparison/routes.json').read_text())['routes']
assert [r['id'] for r in routes]==[f'r{x:02d}' for x in range(1,11)]
identity={}
for folder in ['src','scripts','tools','profiles','deployment/rtx5090_c36','runtime/rtx5090_c36/install']:
 for p in (R/folder).rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc','.pyo']:identity[str(p.relative_to(R))]=sha(p)
for n in ['scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd','scenes/ego_swarm_cloud/occupancy.bin']:identity[n]=sha(R/n)
if (T/'frozen_identity.json').exists():assert json.loads((T/'frozen_identity.json').read_text())['files']==identity,'Runtime/source changed since batch start'
else:write(T/'frozen_identity.json',dict(created=stamp(),files=identity,algorithm='c36',numerical_changes=0))
if not (T/'schedule.json').exists():write(T/'schedule.json',[dict(route=r['id'],repeat=n,kind='base') for n in range(1,6) for r in routes])
records=json.loads((T/'records.json').read_text()) if (T/'records.json').exists() else []
fields=['route','repeat','kind','attempt','label','status','has_trajectory','safe_arrival','accepted','sim_s','wall_s','path_m','actual_route_average_mps','actual_peak_mps','actual_final_distance_m','avoid_mean_ms','avoid_p95_ms','avoid_max_ms','guidance_samples','deadline_samples','collision_blocks','swept_overlap_segments','minimum_swept_clearance_m','limits_violations','sensor_hash','frozen_unchanged','exclusion','audit_error','result']
def save():
 write(T/'records.json',records)
 q=T/'results.csv.tmp'
 with q.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(records)
 q.replace(T/'results.csv')
def state(status,**extra):write(T/'state.json',dict(status=status,updated=stamp(),attempts=len(records),flights=sum(x['has_trajectory'] for x in records),safe_arrivals=sum(x['safe_arrival'] for x in records),completed_slots=len({(x['route'],x['repeat'],x['kind']) for x in records if x['has_trajectory']}),planned_slots=len(json.loads((T/'schedule.json').read_text())),**extra))
def check_frozen():
 for n,h in identity.items():assert sha(R/n)==h,('modified',n)
 assert shutil.disk_usage(R).free>8*1024**3,'Less than 8GiB free on server'
def collect(route,repeat,kind,attempt,label,folder,rc,wall):
 result=folder/'original_v10_a16_r01.json';audit=result.with_suffix('.audit.json');traj=result.with_suffix('.csv')
 a=json.loads(audit.read_text()) if audit.exists() else {};d=json.loads(result.read_text()) if result.exists() else {}
 rows=list(csv.DictReader(traj.open())) if traj.exists() else [];ss=list(folder.glob('PERFORMANCE.md.*.samples.csv'));samples=list(csv.DictReader(ss[0].open())) if len(ss)==1 else []
 values=[float(x['avoid_ms']) for x in samples];ordered=sorted(values)
 rec={k:a.get(k) for k in fields if k in a}
 rec.update(route=route,repeat=repeat,kind=kind,attempt=attempt,label=label,status=a.get('status',d.get('status','STARTUP_ERROR')),has_trajectory=len(rows)>=3,safe_arrival=bool(a.get('safe_arrival')),accepted=bool(a.get('accepted')),wall_s=wall,sim_s=a.get('sim_s',d.get('sim_s')),avoid_mean_ms=statistics.mean(values) if values else None,avoid_p95_ms=ordered[round(.95*(len(ordered)-1))] if values else None,avoid_max_ms=max(values) if values else None,guidance_samples=len(values),deadline_samples=sum(x['status']=='COMPUTE_DEADLINE' for x in samples),limits_violations=json.dumps(a.get('limits_violations',{}),sort_keys=True),sensor_hash=a.get('sensor_profile_sha256'),frozen_unchanged=a.get('additional_checks',{}).get('frozen_execution_matches',False),exclusion='',result=str(result.relative_to(R)),process_exit_code=rc,audit_error=a.get('audit_error'),audit=a)
 if rows:
  end=float(rows[-1]['t']);window=[x for x in rows if float(x['t'])>=end-60]
  xyz=[[float(x[k]) for k in ['x','y','z']] for x in window]
  rec['final_window']=dict(duration_s=float(window[-1]['t'])-float(window[0]['t']),z_min=min(x[2] for x in xyz),z_max=max(x[2] for x in xyz),position_end=xyz[-1],net_displacement_m=sum((xyz[-1][j]-xyz[0][j])**2 for j in range(3))**.5,span_xyz=[max(p[j] for p in xyz)-min(p[j] for p in xyz) for j in range(3)])
  rec['boundary_review_candidate']=not rec['safe_arrival'] and len(window)>3 and (rec['final_window']['z_min']>18.5 or rec['final_window']['z_max']<1.3)
 return rec
state('starting')
for slot in json.loads((T/'schedule.json').read_text()):
 route,repeat,kind=slot['route'],slot['repeat'],slot['kind']
 past=[x for x in records if (x['route'],x['repeat'],x['kind'])==(route,repeat,kind)]
 if any(x['has_trajectory'] for x in past):continue
 for attempt in range(len(past)+1,4):
  check_frozen();label=f'{TAG}_{route}_n{repeat:02d}_{kind}_a{attempt:02d}';folder=R/'performance/p6_speed_20261019/rtx5090_c36'/label
  assert not folder.exists(),folder
  state('running',current=dict(route=route,repeat=repeat,kind=kind,attempt=attempt,label=label))
  env=dict(os.environ,EGO2P6_GOAL_ROUTE=route,EGO2P6_GOAL_HEADLESS='1',EGO2P6_GOAL_RVIZ='false',EGO5090_MANUAL_POLICY='1',EGO5090_ABORT_STALL_S='60');env.pop('EGO2P6_CONTROLLER_CPU_AFFINITY',None);env.pop('BASH_ENV',None)
  print('START',stamp(),route,repeat,kind,attempt,flush=True);beg=time.monotonic()
  with (T/f'{label}.console.log').open('x') as log:
   p=subprocess.Popen(['bash',str(R/'deployment/rtx5090_c36/run_goal.sh'),label],cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   try:rc=p.wait(timeout=1200)
   except subprocess.TimeoutExpired:
    os.killpg(p.pid,signal.SIGINT)
    try:rc=p.wait(timeout=30)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);rc=p.wait(timeout=30)
    raise RuntimeError('external wall watchdog; stopped queue for inspection')
  check_frozen();rec=collect(route,repeat,kind,attempt,label,folder,rc,time.monotonic()-beg);records.append(rec);save()
  print('END',stamp(),route,repeat,rec['status'],'safe=',rec['safe_arrival'],'sim=',rec.get('sim_s'),'avoid_mean=',rec['avoid_mean_ms'],flush=True)
  if rec['has_trajectory']:break
  if attempt>=3:raise RuntimeError('3 startup failures for one slot; inspect infrastructure')
state('base_schedule_complete')
write(T/'route_summary.json',{r['id']:dict(flights=sum(x['has_trajectory'] and x['route']==r['id'] for x in records),safe_arrivals=sum(x['safe_arrival'] and x['route']==r['id'] for x in records)) for r in routes})
print('SCHEDULE_COMPLETE',stamp(),len(records),flush=True)
