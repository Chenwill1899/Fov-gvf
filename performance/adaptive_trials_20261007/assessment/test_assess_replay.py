"""Independent replay-policy boundary tests, no C++ binaries or simulator."""
import importlib.util
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).parent))
import assess_replay as a


@pytest.fixture
def replay(tmp_path):
    evidence=tmp_path/"registration.txt";evidence.write_text("synthetic test only")
    before,after,pairs={},{},[]
    for phase,n in [("explore",3),("confirm",5)]:
        for i in range(n):
            bid,aid=f"{phase}_before_{i}",f"{phase}_after_{i}"
            order=["before","after"] if i%2==0 else ["after","before"]
            pairs.append(dict(phase=phase,before=bid,after=aid,order=order))
            for side,rid,out in [("before",bid,before),("after",aid,after)]:
                out[rid]=dict(run_id=rid,sequence=2*(len(pairs)-1)+1+order.index(side),p95_ms=10 if side=="before" else 8,max_ms=20,
                    decisions_sha256="a"*64,decision_count=37,fixtures_sha256="b"*64,variant_sha256=("c" if side=="before" else "d")*64,numeric_checks_passed=True)
    plan=dict(policy_sha256=a.POLICY_SHA256,registration_evidence=[evidence.name],pairs=pairs)
    return before,after,plan,"confirm",tmp_path


def test_valid_replay_is_only_mechanism_verdict(replay):
    r=a.evaluate(*replay)
    assert r["decision"]=="MECHANISM_CONFIRM_RETAIN"
    assert "never certifies navigation" in r["interpretation"]


def test_decision_difference_rejects_despite_speed(replay):
    b,c,*_=replay;c["confirm_after_0"]["decisions_sha256"]="f"*64
    assert a.evaluate(*replay)["decision"]=="REJECT"


def test_numeric_failure_rejects(replay):
    b,c,*_=replay;c["confirm_after_0"]["numeric_checks_passed"]=False
    assert a.evaluate(*replay)["decision"]=="REJECT"


def test_missing_or_nonfinite_replay_timing_not_pass(replay):
    b,c,*_=replay;c["confirm_after_0"]["p95_ms"]=float("nan")
    assert a.evaluate(*replay)["decision"]=="INSUFFICIENT"


def test_different_fixture_set_not_same_experiment(replay):
    b,c,*_=replay;c["confirm_after_0"]["fixtures_sha256"]="f"*64
    assert a.evaluate(*replay)["decision"]=="REJECT"


def test_three_pairs_not_confirmatory(replay):
    b,c,p,_,d=replay
    b={k:v for k,v in b.items() if k.startswith("explore")};c={k:v for k,v in c.items() if k.startswith("explore")}
    assert a.evaluate(b,c,p,"explore",d)["decision"]=="EXPLORE_PROMISING"
    assert a.evaluate(b,c,p,"confirm",d)["decision"]=="INSUFFICIENT"


def test_small_gain_not_practical(replay):
    b,c,*_=replay
    for r in c.values():r["p95_ms"]=9.5
    assert a.evaluate(*replay)["decision"]=="NO_PRACTICAL_IMPROVEMENT"


def test_one_inconsistent_pair_prevents_n5_confirmation(replay):
    b,c,*_=replay;c["confirm_after_0"]["p95_ms"]=10.01
    assert a.evaluate(*replay)["decision"]=="INSUFFICIENT"


def test_unpaired_failure_kept(replay):
    b,c,*_=replay;c["failed_extra"]={**c["explore_after_0"],"run_id":"failed_extra","numeric_checks_passed":False}
    r=a.evaluate(*replay)
    assert r["decision"]=="REJECT" and "failed_extra" in r["unpaired_run_ids"]
