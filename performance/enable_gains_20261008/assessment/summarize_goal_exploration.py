#!/usr/bin/env python3
"""Snapshot the six-profile goal exploration; no inference or new acceptance rule.

Reads existing manifest/process/navigation/performance/audit files only. Every
recorded failure is retained. An incomplete batch is not assumed to be running
or successful. One-run differences are observations, not reproducible effects.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

FEATURES={
 'uncertainty':('FOV_GVF_DEPTH_UNCERTAINTY','depth_uncertainty_enabled'),
 'dynamic':('FOV_GVF_DYNAMIC_OBSTACLES','paper_dynamic_obstacles'),
 'incremental':('FOV_GVF_INCREMENTAL_FIELD','paper_incremental_field'),
 'memory':('FOV_GVF_SPHERICAL_MEMORY','paper_spherical_memory'),
 'operator':('FOV_GVF_OPERATOR_ASSISTANCE','operator_assistance'),
 'shared':('FOV_GVF_SHARED_OBSTACLES','paper_shared_obstacles'),
}
METRICS=['arrival_s','simulation_duration_s','stop_s','longest_stop_s','path_length_m',
         'path_efficiency','jerk_rms_mps3','jerk_p95_mps3','callback_p95_ms','callback_max_ms']
HIGHER_BETTER={'path_efficiency'}
INCIDENTAL_ENV={'FOV_GVF_RUN_ID','FOV_GVF_PERFORMANCE_LOG','FOV_GVF_REPLAY_DIR',
 'FOV_GVF_RUNTIME_MANIFEST','ISAAC_BENCHMARK_RESULT','ISAAC_ACCEPTANCE_TRACE','FOV_GVF_CONTROLLER_EXECUTABLE'}
INCIDENTAL_PARAMS={'performance_log_path','performance_run_id','paper_replay_directory'}


def sha(path):
 return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fingerprint(value):
 return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def load(path):
 return json.loads(Path(path).read_text())


def finite(value):
 return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def artifact(path):
 path=Path(path)
 return {'path':str(path.resolve()),'sha256':sha(path)} if path.is_file() else {'path':str(path.resolve()),'missing':True}


def expected_flags(profile):
 if profile not in ('baseline','shared','memory','dynamic','uncertainty','combined'):
  raise ValueError('unknown profile '+str(profile))
 on={'uncertainty','dynamic','memory','shared'} if profile=='combined' else set() if profile=='baseline' else {profile}
 return {env:str(int(name in on)) for name,(env,param) in FEATURES.items()}


def goal_metrics(row):
 arrived=row.get('status')=='ARRIVED';elapsed=row.get('simulation_elapsed_s')
 metrics={'arrival_s':elapsed if arrived else None,'simulation_duration_s':elapsed,
  'stop_s':row.get('stopped_with_input_s'),'longest_stop_s':row.get('longest_stop_s'),
  'path_length_m':row.get('distance_travelled_m'),'jerk_rms_mps3':row.get('jerk_rms_mps3'),
  'jerk_p95_mps3':row.get('jerk_p95_mps3'),'path_efficiency':None,
  'wall_elapsed_s':row.get('wall_elapsed_s')}
 start=row.get('protocol',{}).get('start');goal=row.get('protocol',{}).get('goal');length=metrics['path_length_m']
 if arrived and isinstance(start,list) and isinstance(goal,list) and len(start)==len(goal)==3 and all(finite(v) for v in start+goal) and finite(length) and length>0:
  metrics['path_efficiency']=math.dist(start,goal)/length
 return metrics


def performance_metrics(path):
 text=Path(path).read_text();result={}
 for phrase,prefix in [('Control callback to publish','callback'),('Avoidance compute','core_compute')]:
  matches=re.findall(re.escape(phrase)+r' mean / P50 / P95 / max: ([0-9.eE+-]+) / ([0-9.eE+-]+) / ([0-9.eE+-]+) / ([0-9.eE+-]+)',text)
  if matches:
   result.update({prefix+'_'+key+'_ms':float(v) for key,v in zip(('mean','p50','p95','max'),matches[-1])})
 return result


def difference(before,after,name):
 if not finite(before) or not finite(after):return dict(before=before,after=after,change=None,relative_change=None,observation='MISSING_OR_NOT_APPLICABLE')
 delta=after-before
 favorable=delta>0 if name in HIGHER_BETTER else delta<0
 return dict(before=before,after=after,change=delta,relative_change=delta/abs(before) if before else None,
  observation='unchanged' if delta==0 else 'favorable_single_run' if favorable else 'adverse_single_run',
  interpretation='Descriptive observation only; no significance, reproducibility or retention inference.')


def summarize(manifest_path,project):
 manifest_path=Path(manifest_path).resolve();project=Path(project).resolve()
 manifest=load(manifest_path)
 if manifest.get('case')!='goal':raise ValueError('this collector is for goal exploration only')
 errors=[];pending=[];records=[];inputs=[artifact(manifest_path)];file_checks={}
 for path,expected in manifest['file_sha256'].items():
  p=Path(path);actual=sha(p) if p.is_file() else None
  file_checks[path]={'expected':expected,'actual':actual,'matches':actual==expected}
 if not all(r['matches'] for r in file_checks.values()):errors.append('current runtime file differs from driver freeze; see file_checks')
 selected=list(manifest.get('runs',[]));known={Path(m['log']).with_name(Path(m['log']).stem+'_process.json') for m in selected}
 for process in manifest_path.parent.glob('*_process.json'):
  if process not in known:
   selected.append(load(process));pending.append('unlisted process retained (manifest may be mid-write): '+str(process))
 ids=set()
 for meta in selected:
  env=meta.get('experiment_environment',{});rid=env.get('FOV_GVF_RUN_ID');profile=meta.get('profile',meta.get('variant'))
  if not rid or rid in ids:raise ValueError('missing/duplicate run ID')
  ids.add(rid);log=Path(meta['log']);stem=log.with_suffix('');process=log.with_name(log.stem+'_process.json')
  result_path=Path(meta.get('result_path',env.get('ISAAC_BENCHMARK_RESULT','/missing_result')))
  runtime_path=result_path.with_suffix('.parameters.json')
  analysis_path=result_path.parent/(result_path.stem.rsplit('_ego1p5_',1)[0]+'_analysis.json')
  geometry_path=log.with_name(log.stem+'_audit.json');motion_path=log.with_name(log.stem+'_motion_limits.json')
  artifacts={name:artifact(path) for name,path in [('process',process),('result',result_path),('analysis',analysis_path),('runtime',runtime_path),('geometry_audit',geometry_path),('motion_audit',motion_path),('csv',meta['csv']),('performance',meta['performance']),('log',log)]}
  issues=[]
  for name,entry in artifacts.items():
   if entry.get('missing'):issues.append('missing '+name)
  result=load(result_path) if result_path.is_file() else meta.get('result',{})
  rows=load(analysis_path).get('trials',[]) if analysis_path.is_file() else []
  trials=[row for row in rows if row.get('run_id')==rid]
  row=trials[0] if len(trials)==1 else result
  if len(trials)!=1:issues.append('individual navigation analysis missing/duplicate')
  runtime=load(runtime_path) if runtime_path.is_file() else {}
  geometric=load(geometry_path) if geometry_path.is_file() else {}
  motion=load(motion_path) if motion_path.is_file() else {}
  flags=expected_flags(profile);params=runtime.get('effective',{})
  expected_binary=manifest['binary_sha256'].get(profile);binary=Path(meta['controller_executable'])
  binary_values=[expected_binary,meta.get('controller_sha256'),runtime.get('executable_sha256'),sha(binary) if binary.is_file() else None]
  expected_cloud=manifest['file_sha256'].get(str(project/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'))
  common=runtime.get('common_source_sha256',{})
  geometry_zero=(geometric.get('swept_sphere_overlap_segments')==0 and geometric.get('external_collision_blocks')==0)
  gates={
   'goal_arrived':result.get('status')=='ARRIVED' if 'status' in result else None,
   'process_exit_zero':meta.get('returncode')==0,
   'driver_hard_checks_pass':meta.get('hard_checks_pass') is True,
   'driver_geometry_audit_pass':meta.get('audit_geometry_safe') is True and meta.get('audit_returncode')==0,
   'independent_geometry_zero_sweep_external':geometry_zero if geometric else None,
   'analysis_zero_sweep_external':row.get('swept_overlap_segments')==0 and row.get('collision_blocks')==0 if len(trials)==1 else None,
   'geometry_scene_occupancy_match':geometric.get('scene_sha256')==manifest['file_sha256'].get(str(project/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd')) and geometric.get('occupancy_sha256')==manifest['file_sha256'].get(str(project/'scenes/ego_swarm_cloud/occupancy.bin')) if geometric else None,
   'motion_bounds_pass':meta.get('motion_limits_pass') is True and motion.get('status')=='PASS' and motion.get('sha256')==artifacts['csv'].get('sha256') and bool(motion.get('metrics')) and all(m.get('violation_count')==0 and m.get('nonfinite_derived_count')==0 for m in motion['metrics'].values()) if motion else None,
   'runtime_binary_match':bool(expected_binary) and len(set(binary_values))==1,
   'actual_default_goal_route':env.get('ISAAC_MANUAL_INPUT_MODE')=='goal' and 'FOV_GVF_CONTROLLER_EXECUTABLE' not in env and Path(runtime.get('executable','/missing')).resolve()==binary.resolve() and binary.name=='depth_angular_controller_goal',
   'launch_and_runtime_manifest_match':runtime.get('launch_sha256')==meta.get('launch_sha256')==expected_cloud and meta.get('runtime_manifest_sha256')==artifacts['runtime'].get('sha256') if runtime else None,
   'common_runtime_sources_match':bool(common) and all(manifest['file_sha256'].get(str(project/rel))==digest for rel,digest in common.items()),
   'actual_feature_flags_match_profile':meta.get('flags')==flags and all(env.get(k)==v for k,v in flags.items()) and all(params.get(param) is (flags[envkey]=='1') for envkey,param in FEATURES.values()),
   'actual_fixed_goal_parameter':params.get('use_fixed_goal') is True,
   'callback_execution_source':env.get('ISAAC_TWIST_SAMPLE_SOURCE')=='callback',
   'navigation_audit_pass':meta.get('navigation_audit_verified') is True and meta.get('navigation_audit_returncode')==0,
   'analysis_matches_result_status':row.get('status')==result.get('status') if len(trials)==1 else None,
   'manifest_process_same':load(process)==meta if process.is_file() else None,
  }
  metrics=goal_metrics(row)
  if Path(meta['performance']).is_file():metrics.update(performance_metrics(meta['performance']))
  for metric in METRICS:
   if metric not in metrics:metrics[metric]=None
   if metrics[metric] is not None and not finite(metrics[metric]):issues.append('nonfinite '+metric);metrics[metric]=None
  text=log.read_text(errors='replace') if log.is_file() else ''
  lifecycle={'nonzero_child_exits':[int(v) for v in re.findall(r'(?:exit code|exit_code)[ :=]+(-?\d+)',text,re.I) if int(v)!=0],
   'compute_deadline_log_occurrences':text.count('COMPUTE_DEADLINE'),'rosout_invalid_context_occurrences':text.count("Failed to publish log message to rosout: publisher's context is invalid")}
  excluded_env=INCIDENTAL_ENV|{envkey for envkey,param in FEATURES.values()}
  excluded_params=INCIDENTAL_PARAMS|{param for envkey,param in FEATURES.values()}
  conditions={'environment':{k:v for k,v in env.items() if k not in excluded_env},'effective_parameters':{k:v for k,v in params.items() if k not in excluded_params},'protocol':result.get('protocol'),'runtime_files':manifest['file_sha256']}
  records.append(dict(id=rid,profile=profile,repeat=meta['repeat'],status=result.get('status','UNKNOWN'),
   expected_flags=flags,metrics=metrics,hard_gates=gates,failed_hard_gates=[k for k,v in gates.items() if v is False],
   unknown_hard_gates=[k for k,v in gates.items() if v is None],issues=issues,lifecycle=lifecycle,
   condition_sha256_excluding_feature_choices=fingerprint(conditions),profile_sha256=fingerprint({'binary':expected_binary,'flags':flags}),
   artifacts=artifacts,driver_process_wall_s=meta.get('process_wall_s'),driver_verification_errors=meta.get('verification_errors',[]),
   motion_row_count=motion.get('row_count'),motion_metrics=motion.get('metrics'),raw_navigation_analysis=row))
  pending.extend(rid+': '+issue for issue in issues)
 order=[(i+1,profile) for i,profiles in enumerate(manifest['order_by_repeat']) for profile in profiles]
 recorded=[(r['repeat'],r['profile']) for r in records]
 inventory_complete=order==recorded
 if not inventory_complete:pending.append('planned profile inventory/order not yet fully recorded')
 profiles={};comparisons={}
 for profile in manifest['variants']:
  runs=[r for r in records if r['profile']==profile]
  profiles[profile]={'n':len(runs),'expected_n':sum(p==profile for i,p in order),'runs':runs,
   'scope':'n=1 is exploration only; no statistical significance or reproducibility inferred.' if len(runs)<=1 else 'Raw repeated observations; this script does not perform inference.'}
  if profile=='baseline':continue
  comparisons[profile]=[]
  for run in runs:
   baseline=[r for r in records if r['profile']=='baseline' and r['repeat']==run['repeat']]
   if len(baseline)!=1:comparisons[profile].append({'repeat':run['repeat'],'status':'BASELINE_NOT_AVAILABLE'});continue
   before=baseline[0]
   comparisons[profile].append({'repeat':run['repeat'],'baseline_id':before['id'],'candidate_id':run['id'],
    'baseline_arrived':before['status']=='ARRIVED','candidate_arrived':run['status']=='ARRIVED',
    'same_nonfeature_conditions':before['condition_sha256_excluding_feature_choices']==run['condition_sha256_excluding_feature_choices'],
    'metrics':{metric:difference(before['metrics'].get(metric),run['metrics'].get(metric),metric) for metric in METRICS},
    'scope':'One matched exploratory observation; no acceptance threshold or causal claim.'})
   if before['status']!='ARRIVED' or run['status']!='ARRIVED':
    comparisons[profile][-1]['metrics']['path_length_m']['observation']='PARTIAL_ROUTE_LENGTH_NOT_COMPARABLE'
 failures=[{'id':r['id'],'profile':r['profile'],'failed':r['failed_hard_gates']} for r in records if r['failed_hard_gates']]
 return dict(schema_version=1,scope='goal_profile_exploration_only',manifest_complete=manifest.get('complete') is True,
  inventory_complete=inventory_complete,complete=manifest.get('complete') is True and inventory_complete,
  state='EXPLORATION_WITH_RECORDED_HARD_FAILURE' if failures else 'COMPLETE_EXPLORATION' if manifest.get('complete') is True and inventory_complete else 'PARTIAL_EXPLORATION',
  recorded_runs=len(records),planned_runs=len(order),profiles=profiles,baseline_comparisons=comparisons,
  failed_hard_gates=failures,integrity_errors=errors,pending=pending,run_id_uniqueness_checked=True,
  unique_nonfeature_condition_count=len({r['condition_sha256_excluding_feature_choices'] for r in records}),
  file_checks=file_checks,evidence=inputs,tool_sha256=sha(__file__),
  limitations=['No significance test, confidence interval, minimum gain threshold or retention decision is computed.',
   'A recorded TIMEOUT remains a goal-arrival hard-gate failure even when the driver treats it as expected completion and finishes the batch.',
   'Single-run timing, jerk and tail differences remain observations; prior mechanism evidence and activation_plan govern the root decision.',
   'A partial manifest does not distinguish a currently running batch from an early stop. Missing profiles are not successes.',
   'Runtime identities are audited while current files still match the frozen driver. Save this result before any deliberate source/deployment change.',
   'All completed failure records are preserved. Process wall time may include launch/lock wait and is not control-loop latency.',
   'Selected component safety tests and shared asynchronous/peer checks are separate evidence; this script neither runs nor substitutes for them.'])


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--project',type=Path,default=Path(__file__).resolve().parents[3]);args=p.parse_args()
 result=summarize(args.manifest,args.project);args.output.parent.mkdir(parents=True,exist_ok=True)
 with args.output.open('x') as out:json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
 print(json.dumps({k:result[k] for k in ('state','complete','recorded_runs','planned_runs','failed_hard_gates','integrity_errors','pending')},indent=2))
 return 0


if __name__=='__main__':main()
