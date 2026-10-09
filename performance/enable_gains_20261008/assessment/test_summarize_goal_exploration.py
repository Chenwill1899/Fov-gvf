import json
from pathlib import Path
import pytest
from summarize_goal_exploration import expected_flags,goal_metrics,difference,performance_metrics,summarize


def test_combined_only_enables_requested_four():
 flags=expected_flags('combined')
 assert sum(v=='1' for v in flags.values())==4
 assert flags['FOV_GVF_INCREMENTAL_FIELD']==flags['FOV_GVF_OPERATOR_ASSISTANCE']=='0'


def test_single_profile_is_independent():
 flags=expected_flags('memory')
 assert sum(v=='1' for v in flags.values())==1 and flags['FOV_GVF_SPHERICAL_MEMORY']=='1'


def test_timeout_is_not_successful_arrival_or_efficient_partial_path():
 row={'status':'TIMEOUT','simulation_elapsed_s':240.,'distance_travelled_m':.5,'protocol':{'start':[0,0,0],'goal':[10,0,0]}}
 m=goal_metrics(row)
 assert m['arrival_s'] is None and m['simulation_duration_s']==240 and m['path_efficiency'] is None


def test_zero_baseline_keeps_absolute_without_fake_relative_change():
 r=difference(0.,.2,'stop_s')
 assert r['change']==.2 and r['relative_change'] is None and r['observation']=='adverse_single_run'


def test_small_positive_change_is_observation_without_reproducibility_claim():
 r=difference(1.,1.-1e-12,'jerk_rms_mps3')
 assert r['observation']=='favorable_single_run'
 assert 'reproducibility' in r['interpretation']


def test_efficiency_higher_is_favorable():
 assert difference(.7,.8,'path_efficiency')['observation']=='favorable_single_run'


def test_missing_metric_is_not_zero():
 assert difference(1.,None,'callback_max_ms')['observation']=='MISSING_OR_NOT_APPLICABLE'


def test_complete_callback_not_core_compute(tmp_path):
 p=tmp_path/'perf.md';p.write_text('Avoidance compute mean / P50 / P95 / max: 1 / 2 / 3 / 4 ms\nControl callback to publish mean / P50 / P95 / max: 5 / 6 / 7 / 88 ms')
 d=performance_metrics(p)
 assert d['callback_p95_ms']==7 and d['callback_max_ms']==88 and d['core_compute_p95_ms']==3


def empty_manifest(path,complete=False):
 m={'case':'goal','variants':{'baseline':'/missing','memory':'/missing'},'binary_sha256':{},'file_sha256':{},'order_by_repeat':[['baseline','memory']],'runs':[],'complete':complete}
 path.write_text(json.dumps(m));return m


def test_no_recorded_runs_is_partial_not_success(tmp_path):
 p=tmp_path/'manifest.json';empty_manifest(p)
 d=summarize(p,tmp_path)
 assert not d['complete'] and d['state']=='PARTIAL_EXPLORATION' and d['recorded_runs']==0
 assert d['profiles']['memory']['n']==0


def test_complete_bit_cannot_hide_missing_profile(tmp_path):
 p=tmp_path/'manifest.json';empty_manifest(p,True)
 d=summarize(p,tmp_path)
 assert d['manifest_complete'] and not d['complete'] and not d['inventory_complete']


def test_driver_hard_pass_does_not_hide_goal_timeout(tmp_path):
 p=tmp_path/'manifest.json';m=empty_manifest(p)
 result=tmp_path/'batch_baseline_ego1p5_1.json';result.write_text(json.dumps({'status':'TIMEOUT','simulation_elapsed_s':240.}))
 m['binary_sha256']={'baseline':'expected'}
 meta={'variant':'baseline','profile':'baseline','repeat':1,'hard_checks_pass':True,'returncode':0,
       'experiment_environment':{'FOV_GVF_RUN_ID':'batch_baseline_ego1p5_1','ISAAC_BENCHMARK_RESULT':str(result)},
       'result_path':str(result),'log':str(tmp_path/'run.log'),'csv':str(tmp_path/'run.csv'),
       'performance':str(tmp_path/'perf.md'),'controller_executable':str(tmp_path/'missing')}
 m['runs']=[meta];p.write_text(json.dumps(m))
 d=summarize(p,tmp_path)
 assert d['state']=='EXPLORATION_WITH_RECORDED_HARD_FAILURE'
 run=d['profiles']['baseline']['runs'][0]
 assert run['hard_gates']['driver_hard_checks_pass'] is True and 'goal_arrived' in run['failed_hard_gates']
 assert run['status']=='TIMEOUT' and run['metrics']['simulation_duration_s']==240.
