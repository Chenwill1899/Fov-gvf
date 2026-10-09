"""Boundary/regression tests for acceptance decisions, not simulator tests."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import pytest

spec = importlib.util.spec_from_file_location("adaptive_assess", Path(__file__).with_name("assess.py"))
a = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = a
spec.loader.exec_module(a)


@pytest.fixture
def experiment(tmp_path):
    evidence = tmp_path/"attestation.txt"
    evidence.write_text("synthetic test evidence only")
    before, after, pairs = {}, {}, []
    contracts = {sha: {k: dict(cases=1, unsafe=0, passed=True, evidence=[evidence.name]) for k in a.CONTRACTS} for sha in ("a"*64, "b"*64)}
    for phase, n in (("explore",3),("confirm",5)):
        for i in range(n):
            bid, aid = f"{phase}_before_{i}", f"{phase}_after_{i}"
            order = ["before","after"] if i%2 == 0 else ["after","before"]
            pairs.append(dict(phase=phase,before=bid,after=aid,order=order))
            for side, rid, out, fingerprint in [("before",bid,before,"a"*64),("after",aid,after,"b"*64)]:
                row = dict(run_id=rid,status="ARRIVED",simulation_elapsed_s=70 if side=="before" else 68,
                    collision_blocks=0,swept_overlap_segments=0,minimum_swept_clearance_m=.2,process_exit_code=0,
                    distance_travelled_m=10,protocol=dict(start=[0,0,0],goal=[9,0,0]),
                    stopped_with_input_s=.25,longest_stop_s=.25,jerk_rms_mps3=2)
                run = a.normalize(row,tmp_path)
                run["metrics"]["compute_p95_ms"] = 8
                run["lifecycle"] = dict(nonzero_child_exits=[],compute_deadlines=0,rosout_invalid_context=0)
                run["metadata"] = dict(variant_sha256=fingerprint,conditions_sha256="c"*64,
                    freeze_verified=True,sequence=(len(pairs)-1)*2+1+order.index(side))
                out[rid]=run
    plan=dict(policy_sha256=a.POLICY_SHA256,registration_evidence=[evidence.name],
        strata={"goal:fixed_goal":dict(primary="arrival_s")},pairs=pairs)
    supplement=dict(contracts=contracts)
    return before,after,plan,supplement,tmp_path


def test_retention_requires_consistent_practical_independent_gain(experiment):
    r=a.evaluate(*experiment)
    assert r["decision"]=="CONFIRM_RETAIN"
    assert r["strata"]["goal:fixed_goal"]["confirm"]["metrics"]["arrival_s"]["primary_one_sided_sign_p"]==1/32


def test_three_pairs_are_exploratory_only(experiment):
    b,c,p,s,d=experiment
    b={k:v for k,v in b.items() if k.startswith("explore")}
    c={k:v for k,v in c.items() if k.startswith("explore")}
    assert a.evaluate(b,c,p,s,d,stage="explore")["decision"]=="EXPLORE_PROMISING"
    assert a.evaluate(b,c,p,s,d)["decision"]=="INSUFFICIENT"


def test_single_pair_never_passes(experiment):
    b,c,p,s,d=experiment
    b={k:v for k,v in b.items() if k=="explore_before_0"}
    c={k:v for k,v in c.items() if k=="explore_after_0"}
    p["pairs"]=p["pairs"][:1]
    assert a.evaluate(b,c,p,s,d,stage="explore")["decision"]=="INSUFFICIENT"


@pytest.mark.parametrize("field,value",[("sweep",1),("external",1),("clearance",-.001),("exit_code",-11),("arrived",False)])
def test_any_safety_failure_rejects_not_averages(experiment,field,value):
    b,c,p,s,d=experiment
    c["confirm_after_0"]["safety"][field]=value
    assert a.evaluate(*experiment)["decision"]=="REJECT"


def test_missing_unknown_and_stale_not_zero(experiment):
    b,c,p,s,d=experiment
    s["contracts"]={}
    r=a.evaluate(*experiment)
    assert r["decision"]=="INSUFFICIENT"
    assert any("unknown_rejection" in x for x in r["insufficient"])


def test_unsafe_negative_control_rejects(experiment):
    b,c,p,s,d=experiment
    s["contracts"]["b"*64]["expired_depth_rejection"]["unsafe"]=1
    assert a.evaluate(*experiment)["decision"]=="REJECT"


@pytest.mark.parametrize("value",[None,float("nan"),float("inf")])
def test_missing_nonfinite_metric_not_accepted(experiment,value):
    b,c,p,s,d=experiment
    c["confirm_after_0"]["metrics"]["compute_p95_ms"]=value
    assert a.evaluate(*experiment)["decision"]=="INSUFFICIENT"


def test_small_noise_is_allowed_inside_prespecified_margin(experiment):
    b,c,p,s,d=experiment
    for i in range(5):c[f"confirm_after_{i}"]["metrics"]["jerk_rms_mps3"]=2+(-.01 if i%2 else .01)
    assert a.evaluate(*experiment)["decision"]=="CONFIRM_RETAIN"


def test_guard_regression_cannot_be_compensated_by_time_gain(experiment):
    b,c,p,s,d=experiment
    for run in c.values():run["metrics"]["jerk_rms_mps3"]=2.2
    assert a.evaluate(*experiment)["decision"]=="REJECT"


def test_uncertain_noninferiority_is_not_pass(experiment):
    b,c,p,s,d=experiment
    for i in range(5):c[f"confirm_after_{i}"]["metrics"]["compute_p95_ms"]=8+[-2,2,-2,2,0][i]
    r=a.evaluate(*experiment)
    assert r["decision"]=="INSUFFICIENT"
    assert any("noninferiority uncertain" in x for x in r["insufficient"])


def test_timing_std_reported_not_gated_when_baseline_constant(experiment):
    b,c,p,s,d=experiment
    for i in range(5):c[f"confirm_after_{i}"]["metrics"]["arrival_s"]=68+[-.8,.8,-.8,.8,0][i]
    assert a.evaluate(*experiment)["decision"]=="CONFIRM_RETAIN"


def test_worst_arrival_cannot_hide_in_mean(experiment):
    b,c,p,s,d=experiment
    for i in range(5):c[f"confirm_after_{i}"]["metrics"]["arrival_s"]=[64,64,64,64,73][i]
    assert a.evaluate(*experiment)["decision"]=="REJECT"


def test_duplicate_pair_not_more_samples(experiment):
    b,c,p,s,d=experiment
    p["pairs"].append(copy.deepcopy(p["pairs"][-1]))
    assert a.evaluate(*experiment)["decision"]=="REJECT"


def test_failure_outside_pairs_is_retained(experiment):
    b,c,p,s,d=experiment
    run=copy.deepcopy(c["explore_after_0"])
    run["safety"]["arrived"]=False
    c["unpaired_failed_candidate"]=run
    r=a.evaluate(*experiment)
    assert r["decision"]=="REJECT"
    assert "unpaired_failed_candidate" in r["unpaired_run_ids"]


def test_grouped_not_alternated_execution_does_not_pass(experiment):
    b,c,p,s,d=experiment
    for pair in p["pairs"]:pair["order"]=["before","after"]
    assert a.evaluate(*experiment)["decision"]!="CONFIRM_RETAIN"


def test_freeze_changed_between_phases_rejects(experiment):
    b,c,p,s,d=experiment
    c["confirm_after_0"]["metadata"]["variant_sha256"]="d"*64
    assert a.evaluate(*experiment)["decision"]=="REJECT"


def test_same_trial_not_before_and_after(experiment):
    b,c,p,s,d=experiment
    c[next(iter(b))]=copy.deepcopy(next(iter(b.values())))
    with pytest.raises(ValueError,match="both variants"):a.evaluate(*experiment)


def test_zero_baseline_percent_is_undefined():
    result=a.compare_metric("stop_s",[0,0,0],[0,0,0])
    assert result["mean_change_percent"] is None
    assert result["tolerance"]==.05
    assert not result["practical_improvement"]


def test_ties_do_not_inflate_significance():
    assert a.sign_p([1,1,1,1,0])==6/32
    assert a.sign_p([1,1,1,1,1])==1/32


def test_core_compute_is_not_full_callback(tmp_path):
    perf=tmp_path/"perf.md"
    perf.write_text("Avoidance compute mean / P50 / P95 / max: 1 / 2 / 3 / 4 ms\nControl callback to publish mean / P50 / P95 / max: 5 / 6 / 7 / 8 ms")
    assert a.read_perf(perf)==dict(core_compute_p95_ms=3,compute_p95_ms=7)


def write_trace(path,values):
    path.write_text("t,qx,qy,qz,vx,vy,vz,applied_target_x,applied_target_y,applied_target_z\n"+"\n".join(",".join(map(str,[t,q,0,0,v,0,0,u,0,0])) for t,q,v,u in values))


def test_release_settling_and_command_vs_plant_are_separate(tmp_path):
    path=tmp_path/"trace.csv"
    write_trace(path,[(0,1,1,1),(.1,0,.9,0),(.2,0,.1,0),(.3,0,0,0),(.4,0,0,0)])
    r=a.csv_release(path)
    assert r["complete"]
    assert r["target_p95_s"]==0
    assert r["brake_p95_s"]==pytest.approx(.2)


def test_release_rebound_and_censoring_not_silent_success(tmp_path):
    path=tmp_path/"trace.csv"
    write_trace(path,[(0,1,1,1),(.1,0,0,0),(.2,0,.1,1),(.3,1,.1,1)])
    r=a.csv_release(path)
    assert not r["complete"] and r["brake_p95_s"] is None
    assert r["events"][0]["target_s"] is None


def test_no_release_is_missing_not_zero(tmp_path):
    path=tmp_path/"trace.csv"
    write_trace(path,[(0,1,1,1),(.1,1,1,1)])
    r=a.csv_release(path)
    assert not r["complete"] and r["event_count"]==0


def test_duplicate_csv_time_fails(tmp_path):
    path=tmp_path/"trace.csv"
    write_trace(path,[(0,1,1,1),(0,0,0,0)])
    with pytest.raises(ValueError,match="strictly"):a.csv_release(path)


def test_shutdown_crash_does_not_disappear(tmp_path):
    path=tmp_path/"run.log"
    path.write_text("[BENCHMARK RESULT] ARRIVED\nSimulation App Shutting Down\nprocess died [pid 2, exit code -11]\nCOMPUTE_DEADLINE")
    assert a.read_log(path)["nonzero_child_exits"]==[-11]
    assert a.read_log(path)["compute_deadlines"]==1


def test_failure_without_metrics_loaded(tmp_path):
    path=tmp_path/"analysis.json"
    path.write_text(json.dumps(dict(trials=[dict(run_id="failed",status="TIMEOUT")])) )
    runs,_=a.load_analysis([path])
    assert "failed" in runs and not runs["failed"]["safety"]["arrived"]


def test_aggregate_only_input_rejected(tmp_path):
    path=tmp_path/"analysis.json"
    path.write_text('{"groups":{"mean":0}}')
    with pytest.raises(ValueError,match="aggregate"):a.load_analysis([path])


def test_guardrails_only_does_not_claim_improvement(experiment):
    b,c,p,s,d=experiment
    p["scope"]="guardrails_only"
    p["strata"]["goal:fixed_goal"]={}
    for run in c.values():run["metrics"]["arrival_s"]=70
    assert a.evaluate(*experiment)["decision"]=="GUARDRAILS_CONFIRMED"


def test_repeated_execution_sequence_rejects(experiment):
    b,c,p,s,d=experiment
    c["confirm_after_1"]["metadata"]["sequence"]=c["confirm_after_0"]["metadata"]["sequence"]
    assert a.evaluate(*experiment)["decision"]=="REJECT"


def test_negative_time_is_invalid():
    with pytest.raises(ValueError,match="negative"):a.compare_metric("arrival_s",[1,1,1],[-1,-1,-1])


def test_nonfinite_evidence_saved_as_valid_json():
    value=a.json_safe({"input":float("nan")})
    assert json.dumps(value,allow_nan=False)=='{"input": {"invalid_numeric": "nan"}}'
