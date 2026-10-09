"""Read all task-labelled trials; keep stages, failures and limitations distinct."""
from pathlib import Path
import hashlib,json,math,ast,subprocess
root=Path(__file__).resolve().parents[2];lab=Path(__file__).resolve().parent
nav=root/'performance/navigation_benchmark_20260929';manual=lab/'manual'
goal_labels=['p5_six_baseline_20261007','p5_six_uncertainty_20261007','p5_six_uncertainty_incremental_20261007']+['p5_six_final_20261007_'+x for x in ('baseline','default','spherical')]+['p5_six_recovery_v2_'+x for x in ('baseline','default')]
man_labels=['p5_operator_final','p5_spherical_final','p5_default_spin','p5_operator_v2']
goals={name:json.loads((nav/(name+'_analysis.json')).read_text()) for name in goal_labels}
mans={name:json.loads((manual/(name+'_analysis.json')).read_text()) for name in man_labels}
gt=[t for d in goals.values() for t in d['trials']];mt=[t for d in mans.values() for t in d['runs']]
assert len(gt)==18 and len(mt)==12
assert len({x['run_id'] for x in gt})==18 and len({x['name'] for x in mt})==12
protected=json.loads((lab/'baseline/other_projects_sha256.json').read_text());integrity={}
for project,files in protected.items():
 changed=[f for f,h in files.items() if not (root.parent/project/f).exists() or hashlib.sha256((root.parent/project/f).read_bytes()).hexdigest()!=h]
 integrity[project]=dict(files=len(files),changed=changed)
current={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for d in ('src','scripts','tools') for p in (root/d).rglob('*') if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache'))}
before=json.loads((lab/'baseline/source_sha256.json').read_text());changes=[dict(path=p,before=before.get(p),after=current.get(p)) for p in sorted(set(current)|set(before)) if p in current and current[p]!=before.get(p)]
(lab/'source_changes.json').write_text(json.dumps(changes,indent=2))
result=dict(status='six_optional_mechanisms_implemented_and_ablated__global_superiority_not_demonstrated',defaults={k:False for k in ('depth_uncertainty','dynamic_obstacles','incremental_field','spherical_memory','operator_assistance','shared_obstacles')},goal_trials=len(gt),manual_trials=len(mt),total_isaac_trials=len(gt)+len(mt),goal_safe_arrivals=sum(bool(t.get('safe_arrival')) for t in gt),manual_safe_runs=sum(t['returncode']==0 and t['safety']['swept_sphere_overlap_segments']==0 and t['safety']['external_collision_blocks']==0 for t in mt),swept_overlaps=sum(t.get('swept_overlap_segments',0) for t in gt)+sum(t['safety']['swept_sphere_overlap_segments'] for t in mt),external_protection=sum(t.get('collision_blocks',0) for t in gt)+sum(t['safety']['external_collision_blocks'] for t in mt),goal_groups={k:v['groups']['ego1p5'] for k,v in goals.items()},manual_groups={k:v['summary'] for k,v in mans.items()},integrity=integrity,source_changes=len(changes),unvalidated=['dynamic Isaac scene','real multi-robot flight','sensor error calibration','human subjective study','general navigation superiority','frontend recovery shortcut positive runtime timing (not triggered in v2 trials)'],validation=dict(core_tests='accepted_optional/ctest.log',ablations='accepted_optional/ablations/summary.json',platform_tests='final/platform_tests_ros_env.log',combination_replays=64,ros_shared_positive='frontend_followup/ros_control.json',ros_all_features='frontend_followup/ros_all_features.json',fresh_p5_stall_replay='stage3_incremental/p5_stall_replay.json',inherited_replay='accepted_optional/inherited_replay.json',default_parameters='accepted_optional/default_parameters.json',final_snapshot='accepted_optional/source_sha256.json'))
assert '100% tests passed, 0 tests failed out of 31' in (lab/'accepted_optional/ctest.log').read_text()
assert '15 passed' in (lab/'final/platform_tests_ros_env.log').read_text()
checks=json.loads((lab/'accepted_optional/ablations/summary.json').read_text())
assert len(checks)==9 and all(x['returncode']==0 for x in checks)
assert json.loads((lab/'accepted_optional/inherited_replay.json').read_text())['legacy_equal']
params=json.loads((lab/'accepted_optional/default_parameters.json').read_text())
assert not params['differences_from_measured_baseline_native'] and not any(params['new_flags'].values())
assert all(not x['changed'] for x in integrity.values())
result['validation'].update(core_tests_passed=31,platform_tests_passed=15,mechanism_checks_passed=9,flags_off_matches_original_p5=True,default_matches_measured_baseline=True)
(lab/'completion_summary.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:result[k] for k in ('status','goal_trials','manual_trials','total_isaac_trials','goal_safe_arrivals','manual_safe_runs','swept_overlaps','external_protection','integrity','source_changes')},indent=2))
