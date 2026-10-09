import copy
import csv
import json
import math
from pathlib import Path
import pytest
from common import paired, direction_class, summary, write_new, sha, load
from collect_geometry import valid_output
from collect_tasks import verify_runtime, compare_runs, conditions_payload, contracts_check, FEATURES
from task_metrics import normalize, csv_release, performance, lifecycle, launch_time

HERE=Path(__file__).resolve().parent
POLICY=load(HERE/'policy.json')


def test_tiny_gain_has_no_magnitude_floor():
    r=paired([1.,1.,1.],[1.-1e-12]*3)
    assert r['classification']=='reproducible_gain'
    assert r['relative_improvement']<1e-10


def test_tiny_repeatable_harm_is_preserved():
    assert paired([1.]*3,[1.+1e-12]*3)['classification']=='reproducible_regression'


def test_mixed_noise_is_not_a_regression():
    assert paired([10.]*3,[10.01,9.99,10.001])['classification']=='unresolved_variation'


def test_zero_baseline_has_absolute_change_without_fake_relative_gain():
    r=paired([0.]*3,[.1]*3)
    assert r['relative_improvement'] is None
    assert r['classification']=='reproducible_regression'


def test_higher_better_progress_direction():
    assert paired([1.,2.,3.],[1.1,2.1,3.1],True)['classification']=='reproducible_gain'


def test_two_repeats_cannot_pass():
    assert paired([2.,2.],[1.,1.])['classification']=='insufficient_repeats'


def test_exact_bootstrap_is_lightweight_for_three():
    assert 'exact empirical' in paired([2.]*3,[1.]*3)['bootstrap_method']


def test_unchanged_is_not_gain():
    assert paired([1.,2.,3.],[1.,2.,3.])['classification']=='unchanged'


@pytest.mark.parametrize('bad',[[1.,float('nan')],[1.,float('inf')],[True,2.],[]])
def test_invalid_summary_rejected(bad):
    with pytest.raises(ValueError):summary(bad)


def test_mismatched_pairs_rejected():
    with pytest.raises(ValueError):paired([1.],[1.,2.])


def test_bootstrap_interval_alone_cannot_hide_mixed_directions():
    assert direction_class([.01,.1],5,3,8)=='unresolved_variation'


def test_output_write_never_overwrites_failure(tmp_path):
    p=tmp_path/'failed.json';write_new(p,{'failed':True})
    with pytest.raises(FileExistsError):write_new(p,{'failed':False})
    assert load(p)=={'failed':True}


@pytest.fixture
def output():
    return dict(status='TRACKING',reason='READY',accepted=True,command=[0.,0.,0.],required_prefix=0.,best_prefix=1.,free_directions=3,refined=False,command_changed=False,intent_change_angle=0.,reset_reason='',continued=False)


def test_finite_replay_output_accepted(output):
    assert valid_output(json.dumps(output))==output


@pytest.mark.parametrize('field,value',[('command',[float('nan'),0.,0.]),('best_prefix',float('inf')),('accepted',1),('free_directions',True),('command',[0.,0.])])
def test_invalid_replay_output_not_equivalence(output,field,value):
    output[field]=value
    with pytest.raises(ValueError):valid_output(json.dumps(output))


def test_extra_replay_field_not_silently_ignored(output):
    output['new_safety_status']='REJECT'
    with pytest.raises(ValueError):valid_output(json.dumps(output))


def test_failed_goal_cannot_get_favorable_path_efficiency():
    row=dict(status='TIMEOUT',simulation_elapsed_s=240.,distance_travelled_m=.5,protocol={'start':[0.,0.,0.],'goal':[10.,0.,0.]})
    metrics,safety=normalize(row,'goal')
    assert metrics['path_efficiency'] is None and not safety['arrived']


def test_arrived_goal_path_efficiency():
    row=dict(status='ARRIVED',distance_travelled_m=20.,protocol={'start':[0.,0.,0.],'goal':[10.,0.,0.]})
    assert normalize(row,'goal')[0]['path_efficiency']==.5


def make_csv(path, rows):
    with path.open('w') as f:
        out=csv.writer(f);out.writerow(['t','qx','qy','qz','vx','vy','vz','applied_target_x','applied_target_y','applied_target_z'])
        out.writerows(rows)


def test_release_right_censor_not_zero(tmp_path):
    p=tmp_path/'trace.csv';make_csv(p,[[0,1,0,0,1,0,0,1,0,0],[.1,0,0,0,0,0,0,0,0,0]])
    d=csv_release(p)
    assert not d['complete'] and d['release_target_s'] is None


def test_release_needs_settled_values_after_rebound(tmp_path):
    p=tmp_path/'trace.csv';make_csv(p,[[0,1,0,0,1,0,0,1,0,0],[.1,0,0,0,0,0,0,0,0,0],[.2,0,0,0,.1,0,0,.1,0,0],[.3,0,0,0,0,0,0,0,0,0],[.4,0,0,0,0,0,0,0,0,0]])
    d=csv_release(p)
    assert d['complete'] and d['release_target_s']==pytest.approx(.2)


def test_no_release_is_unknown(tmp_path):
    p=tmp_path/'trace.csv';make_csv(p,[[0,1,0,0,1,0,0,1,0,0],[.1,1,0,0,1,0,0,1,0,0]])
    assert not csv_release(p)['complete']


def test_release_nonmonotonic_time_rejected(tmp_path):
    p=tmp_path/'trace.csv';make_csv(p,[[0,1,0,0,1,0,0,1,0,0],[0,0,0,0,0,0,0,0,0,0]])
    with pytest.raises(ValueError):csv_release(p)


def test_full_callback_not_core_and_max_retained(tmp_path):
    p=tmp_path/'perf.md';p.write_text('Avoidance compute mean / P50 / P95 / max: 1 / 1 / 2 / 3 ms\nControl callback to publish mean / P50 / P95 / max: 5 / 6 / 7 / 80 ms')
    d=performance(p)
    assert d['callback_p95_ms']==7 and d['core_compute_p95_ms']==2 and d['callback_max_ms']==80


def test_shutdown_minus11_and_deadline_retained(tmp_path):
    p=tmp_path/'run.log';p.write_text('[shutdown] process has died exit code -11\nCOMPUTE_DEADLINE\nexit code 0')
    d=lifecycle(p)
    assert d['nonzero_child_exits']==[-11] and d['compute_deadlines']==1


def test_actual_launch_order_uses_after_lock_log(tmp_path):
    p=tmp_path/'run.log';p.write_text('START 2000\n[INFO] [launch]: All log files can be found below /tmp/foo/2026-10-08-10-00-01-123456-starry-1234\n')
    assert launch_time(p)=='2026-10-08T10:00:01.123456'


@pytest.fixture
def runtime_fixture(tmp_path):
    project=tmp_path/'project';launch=project/'src/pc_gvf/launch';launch.mkdir(parents=True)
    install=tmp_path/'install';dest=install/'pc_gvf/share/pc_gvf/launch';dest.mkdir(parents=True)
    files={}
    for name in ('isaac_cloud_navigation.launch.py','navigation_benchmark.launch.py'):
        for directory in (launch,dest):
            p=directory/name;p.write_text(name);files[str(p)]=sha(p)
    helper=project/'helper.py';helper.write_text('helper');files[str(helper)]=sha(helper)
    binary=tmp_path/'controller';binary.write_text('binary')
    result=tmp_path/'result.json';runtime_path=result.with_suffix('.parameters.json')
    runtime=dict(executable=str(binary),executable_sha256=sha(binary),launch_sha256=files[str(launch/'isaac_cloud_navigation.launch.py')],effective={'safe_radius':.58},common_source_sha256={'helper.py':sha(helper)})
    runtime_path.write_text(json.dumps(runtime))
    flags={k:'0' for k in FEATURES};env={**flags,'FOV_GVF_INSTALL':str(install),'ISAAC_TWIST_SAMPLE_SOURCE':'callback'}
    meta=dict(variant='baseline',case='goal',controller_executable=str(binary),controller_sha256=sha(binary),launch_sha256=runtime['launch_sha256'],result_path=str(result),experiment_environment=env,flags=flags,runtime_binary_verified=True,runtime_launch_verified=True,runtime_common_sources_verified=True)
    manifest=dict(binary_sha256={'baseline':sha(binary)},file_sha256=files)
    freeze=dict(binary_sha256={'depth_angular_controller':sha(binary)},archive_matches=True)
    return project,meta,manifest,freeze,runtime_path


def verify_fixture(f):
    project,meta,manifest,freeze,_=f
    return verify_runtime(meta,manifest,freeze,project,{})[1]


def test_runtime_without_removed_bc_params_is_valid(runtime_fixture):
    assert verify_fixture(runtime_fixture)['verified']


def test_runtime_actual_binary_change_detected(runtime_fixture):
    Path(runtime_fixture[1]['controller_executable']).write_text('other')
    assert not verify_fixture(runtime_fixture)['verified']


def test_runtime_launch_change_detected(runtime_fixture):
    p=next(p for p in runtime_fixture[2]['file_sha256'] if '/install/' in p)
    Path(p).write_text('changed')
    assert not verify_fixture(runtime_fixture)['verified']


def test_runtime_missing_actual_params_rejected(runtime_fixture):
    p=runtime_fixture[-1];r=load(p);r['effective']={};p.write_text(json.dumps(r))
    assert not verify_fixture(runtime_fixture)['verified']


def test_runtime_six_flags_not_assumed(runtime_fixture):
    runtime_fixture[1]['flags'].pop(next(iter(FEATURES)))
    assert not verify_fixture(runtime_fixture)['verified']


def test_runtime_callback_binding_required(runtime_fixture):
    runtime_fixture[1]['experiment_environment']['ISAAC_TWIST_SAMPLE_SOURCE']='poll'
    assert not verify_fixture(runtime_fixture)['verified']


def test_runtime_archive_mismatch_rejected(runtime_fixture):
    runtime_fixture[3]['archive_matches']=False
    assert not verify_fixture(runtime_fixture)['verified']


def test_runtime_driver_failed_verification_not_ignored(runtime_fixture):
    runtime_fixture[1]['runtime_binary_verified']=False
    assert not verify_fixture(runtime_fixture)['verified']


def make_runs():
    rows={}
    for case in ('goal','jitter'):
        for v in ('baseline','v4'):
            for i in range(1,4):
                rid=f'{case}_{v}_{i}';rows[rid]=dict(id=rid,variant=v,case=case,repeat=i,conditions_sha256=case,metrics={'arrival_s':10.+(.1 if v=='v4' else 0.),'angle_mean_deg':2.})
    return rows


def test_unmatched_failed_repeat_never_dropped():
    rows=make_runs();del rows['goal_v4_3']
    result,pending=compare_runs(rows,['baseline','v4'],POLICY)
    assert pending and result['baseline_to_v4']['goal']['metrics']['arrival_s']['classification']=='MISSING_OR_CENSORED'


def test_scenario_harm_not_offset_by_other_metrics():
    result,_=compare_runs(make_runs(),['baseline','v4'],POLICY)
    assert result['baseline_to_v4']['goal']['reproducible_regressions']==['arrival_s']
    assert result['baseline_to_v4']['jitter']['status']=='NO_REPRODUCIBLE_REGRESSION_OBSERVED'


def test_missing_task_metric_is_visible():
    result,_=compare_runs(make_runs(),['baseline','v4'],POLICY)
    assert 'release_brake_s' in result['baseline_to_v4']['jitter']['unresolved_metrics']


def test_condition_mismatch_prevents_automatic_status():
    rows=make_runs();rows['jitter_v4_1']['conditions_sha256']='other'
    result,pending=compare_runs(rows,['baseline','v4'],POLICY)
    assert 'paired conditions differ' in result['baseline_to_v4']['jitter']['issues'] and pending


def test_four_variant_comparisons_preserve_incremental_ablations():
    rows=make_runs()
    for case in ('goal','jitter'):
        for v in ('v2','combined'):
            for i in range(1,4):
                rows[f'{case}_{v}_{i}']={**rows[f'{case}_baseline_{i}'],'variant':v,'id':f'{case}_{v}_{i}'}
    result,_=compare_runs(rows,['baseline','v2','v4','combined'],POLICY)
    assert set(result)=={'baseline_to_v2','baseline_to_v4','v2_to_combined','v4_to_combined','baseline_to_combined'}


def test_missing_contracts_are_unknown_not_pass():
    result=contracts_check(None,'v4',{}, {},Path('.'))
    assert result['status']=='UNKNOWN'


def test_missing_metrics_keep_global_collection_pending():
    _,pending=compare_runs(make_runs(),['baseline','v4'],POLICY)
    assert any('missing/censored callback_p95_ms' in x for x in pending)


def test_missing_goal_outcome_is_unknown_not_claimed_timeout():
    assert normalize({},'goal')[1]['arrived'] is None
