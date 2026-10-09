#!/usr/bin/env python3
"""Paired geometric replay timing assessment; this does not certify navigation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
from assess import digest, finite, sign_p, std, t95, json_safe

POLICY = dict(version="adaptive_replay_v1", exploration_pairs=3, confirmation_pairs=5,
    primary="p95_ms", minimum_relative_improvement=.10, sign_test_alpha=.05,
    decisions="exact same SHA256 and count on same fixture set", numeric_checks="required",
    max_timing="reported, not a navigation latency claim")
POLICY_SHA256 = hashlib.sha256(json.dumps(POLICY,sort_keys=True).encode()).hexdigest()


def load(paths):
    runs, sources = {}, []
    for path in paths:
        path = Path(path).resolve()
        content = json.loads(path.read_text())
        rows = content.get("runs")
        if not isinstance(rows,list):
            raise ValueError("expected runs array; each row is an independent whole-fixture timing batch")
        sources.append(dict(path=str(path),sha256=digest(path)))
        for row in rows:
            rid=row.get("run_id")
            if not rid or rid in runs:
                raise ValueError("missing or repeated replay run_id")
            runs[rid]=row
    return runs,sources


def evaluate(before,after,plan,stage="confirm",base=Path(".")):
    failed,missing=[],[]
    if set(before)&set(after):
        raise ValueError("replay batch cannot be both before and after")
    rows={**before,**after}
    if plan.get("policy_sha256")!=POLICY_SHA256:
        missing.append("unregistered/mismatched replay policy")
    paths=plan.get("registration_evidence",[])
    registration=[dict(path=str((base/p).resolve()),sha256=digest(base/p)) for p in paths]
    if not registration:missing.append("no externally recorded preregistration evidence")
    for rid,r in rows.items():
        for name in ("p95_ms","max_ms"):
            if not finite(r.get(name)) or r[name]<=0:missing.append(rid+": invalid "+name)
        if finite(r.get("p95_ms")) and finite(r.get("max_ms")) and r["p95_ms"]>r["max_ms"]:
            failed.append(rid+": P95 exceeds max")
        if r.get("numeric_checks_passed") is not True:
            failed.append(rid+": numeric validity/equivalence check did not pass")
        if not isinstance(r.get("decision_count"),int) or isinstance(r["decision_count"],bool) or r["decision_count"]<=0:
            missing.append(rid+": empty/invalid decision count")
        for name in ("variant_sha256","fixtures_sha256","decisions_sha256"):
            import re
            if not re.fullmatch(r"[0-9a-f]{64}",str(r.get(name,""))):missing.append(rid+": missing "+name)
    for side,runs in (("before",before),("after",after)):
        if len({r.get("variant_sha256") for r in runs.values()})!=1:
            failed.append(side+": freeze changed across batches")
    if len({r.get("fixtures_sha256") for r in rows.values()})!=1:
        failed.append("fixture set changed across batches")
    used=set();groups={"explore":[],"confirm":[]}
    for pair in plan.get("pairs",[]):
        phase=pair.get("phase")
        if phase not in groups:
            failed.append("invalid registered phase");continue
        if stage=="explore" and phase=="confirm":continue
        b,a=pair.get("before"),pair.get("after")
        if b not in before or a not in after:
            missing.append(f"missing batch {b}/{a}");continue
        if b in used or a in used:
            failed.append("reused timing batch");continue
        used.update((b,a));rb,ra=before[b],after[a]
        for key in ("decisions_sha256","decision_count","fixtures_sha256"):
            if rb.get(key)!=ra.get(key):failed.append(f"{b}/{a}: {key} differs")
        order=pair.get("order")
        if order not in (["before","after"],["after","before"]):
            missing.append("missing registered order")
        elif not all(isinstance(r.get("sequence"),int) and not isinstance(r["sequence"],bool) for r in (rb,ra)):
            missing.append("actual execution sequence missing")
        elif (rb["sequence"]<ra["sequence"])!=(order[0]=="before"):
            failed.append("actual order differs from registration")
        groups[phase].append(pair)
    unpaired=sorted(set(rows)-used)
    if unpaired:missing.append("unpaired batches retained: "+",".join(unpaired))
    seq=[r.get("sequence") for r in rows.values()]
    if all(isinstance(x,int) for x in seq) and len(set(seq))!=len(seq):failed.append("execution sequence reused")
    comparisons={};promising=False;confirmed=False
    for phase,pairs in groups.items():
        if stage=="explore" and phase=="confirm":continue
        needed=3 if phase=="explore" else 5
        if len(pairs)<needed:missing.append(f"{phase}: fewer than {needed} independent timing pairs")
        orders=[p.get("order") for p in pairs]
        if any(x==y for x,y in zip(orders,orders[1:])):missing.append(phase+": AB/BA did not alternate")
        blocks=[[rows[p[s]].get("sequence") for s in ("before","after")] for p in pairs]
        if all(isinstance(x,int) for block in blocks for x in block) and any(max(a)>=min(b) for a,b in zip(blocks,blocks[1:])):
            failed.append(phase+": paired blocks not in registered order")
        b=[before[p["before"]].get("p95_ms") for p in pairs]
        a=[after[p["after"]].get("p95_ms") for p in pairs]
        if not b or any(not finite(x) or x<=0 for x in b+a):continue
        improvements=[x-y for x,y in zip(b,a)]
        ratio=[(x-y)/x for x,y in zip(b,a)]
        mean=statistics.mean(improvements);sd=std(improvements)
        half=t95(len(b)-1)*sd/math.sqrt(len(b)) if sd is not None else None
        practical=statistics.mean(a)<=.9*statistics.mean(b)
        p=sign_p(improvements)
        comparisons[phase]=dict(pairs=len(b),before_p95_mean_ms=statistics.mean(b),after_p95_mean_ms=statistics.mean(a),
            before_p95_std_ms=std(b),after_p95_std_ms=std(a),
            paired_improvement_ms=improvements,paired_relative_improvement=ratio,
            aggregate_relative_improvement=mean/statistics.mean(b),one_sided_sign_p=p,
            central90_t_improvement_interval=[mean-half,mean+half] if half is not None else [None,None],
            practical_improvement=practical,before_batch_max_ms=[before[q["before"]].get("max_ms") for q in pairs],
            after_batch_max_ms=[after[q["after"]].get("max_ms") for q in pairs])
        if statistics.mean(a)>statistics.mean(b):failed.append(phase+": P95 mean regressed")
        if phase=="explore":promising=practical and sum(x>0 for x in improvements)>=math.ceil(2*len(b)/3)
        else:confirmed=practical and p<=.05
    se=[rows[p[s]].get("sequence") for p in groups["explore"] for s in ("before","after")]
    sc=[rows[p[s]].get("sequence") for p in groups["confirm"] for s in ("before","after")]
    if se and sc and all(isinstance(x,int) for x in se+sc) and max(se)>=min(sc):failed.append("confirmation ran before exploration completed")
    if failed:decision="REJECT"
    elif missing:decision="INSUFFICIENT"
    elif not promising:decision="NO_PRACTICAL_IMPROVEMENT"
    elif stage=="explore":decision="EXPLORE_PROMISING"
    elif not confirmed:decision="INSUFFICIENT";missing.append("primary improvement not independently confirmed")
    else:decision="MECHANISM_CONFIRM_RETAIN"
    return dict(decision=decision,policy=POLICY,policy_sha256=POLICY_SHA256,stage=stage,
        failures=failed,insufficient=missing,comparisons=comparisons,all_runs=rows,
        unpaired_run_ids=unpaired,registration_evidence=registration,
        interpretation="Each sample is a timing batch over the same fixtures, not a trajectory or independent geometric scene. Digest/numeric checks are supplied by the producer and must cover safety decisions including negative cases. Retaining this mechanism also requires separate goal/manual safety and noninferiority; this verdict alone never certifies navigation.")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--before",nargs="+",type=Path,required=True)
    p.add_argument("--after",nargs="+",type=Path,required=True)
    p.add_argument("--plan",type=Path,required=True)
    p.add_argument("--stage",choices=("explore","confirm"),default="confirm")
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error("output exists; preserve prior results and select a new filename")
    try:
        before,bp=load(a.before);after,ap=load(a.after)
        report=evaluate(before,after,json.loads(a.plan.read_text()),a.stage,a.plan.resolve().parent)
        report["input_files"]=bp+ap+[dict(path=str(a.plan.resolve()),sha256=digest(a.plan))]
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open("x") as f:json.dump(json_safe(report),f,indent=2,allow_nan=False);f.write("\n")
        print(json.dumps(dict(decision=report["decision"],output=str(a.output))))
        return 0 if report["decision"] in ("MECHANISM_CONFIRM_RETAIN","EXPLORE_PROMISING") else 2
    except (ValueError,KeyError,TypeError,OSError) as exc:p.error(str(exc))

if __name__=="__main__":raise SystemExit(main())
