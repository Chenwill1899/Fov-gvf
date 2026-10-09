#!/usr/bin/env python3
"""Audit the four-variant raw replay and classify the published paired intervals.

Does not execute a binary or recompute a benchmark. The interval producer and all
its frozen inputs are hashed; raw order, equality and all summary values are
independently recomputed here. Existing evidence is never overwritten.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
from common import load, sha, evidence, write_new, finite, summary, direction_class


FIELDS={"status", "reason", "accepted", "command", "required_prefix", "best_prefix",
        "free_directions", "refined", "command_changed", "intent_change_angle", "reset_reason", "continued"}


def valid_output(text):
    d=json.loads(text)
    if set(d)!=FIELDS:raise ValueError("replay output field inventory changed")
    for k in ("status","reason","reset_reason"):
        if not isinstance(d[k],str):raise ValueError("invalid discrete field: "+k)
    for k in ("accepted","refined","command_changed","continued"):
        if type(d[k]) is not bool:raise ValueError("invalid boolean field: "+k)
    if type(d["free_directions"]) is not int or d["free_directions"]<0:
        raise ValueError("invalid direction count")
    if not isinstance(d["command"],list) or len(d["command"])!=3:
        raise ValueError("invalid command")
    if not all(finite(x) for x in d["command"]+[d[k] for k in ("required_prefix","best_prefix","intent_change_angle")]):
        raise ValueError("nonfinite continuous replay output")
    return d


def close(a,b):
    return finite(a) and finite(b) and math.isclose(a,b,rel_tol=1e-11,abs_tol=1e-10)


def collect(directory, project, policy):
    directory=Path(directory).resolve();project=Path(project).resolve()
    raw_path=directory/"replay_ablation.json"
    raw=load(raw_path);plan=load(directory/"plan.json");freeze=load(directory/"frozen_manifest.json")
    paired=load(directory/"paired_analysis.json")
    errors=[];notes=[];gp=policy["geometry"]
    def check(ok,message):
        if not ok:errors.append(message)
    check(raw.get("complete") is True,"raw experiment incomplete")
    check(len(plan["orders"])==gp["batches"],"planned batch count differs")
    check(len(plan["fixtures"])==gp["fixtures"],"planned fixture count differs")
    check(plan["required_incremental_comparisons"]==gp["required_comparisons"],"required comparisons differ")
    check(raw["orders"]==plan["orders"],"raw execution orders differ")
    check(raw["frames"]==plan["fixtures"],"raw fixture inventory differs")
    check(raw["plan_sha256"]==freeze["plan_sha256"]==paired["plan_sha256"]==sha(directory/"plan.json"),"plan hash differs")
    check(paired["raw_sha256"]==sha(raw_path),"paired analysis raw hash differs")
    check(raw["sha256"]==freeze["binary_sha256"],"binary hash inventories differ")
    check(raw["source_sha256"]==freeze["source_sha256"]==plan["source_sha256"],"source hash inventories differ")
    for file,expected in freeze["files_sha256"].items():
        p=directory/file;check(p.is_file() and sha(p)==expected,"frozen file differs: "+file)
    for variant,expected in raw["sha256"].items():
        p=Path(raw["executables"][variant]);check(p.is_file() and sha(p)==expected,"executable differs: "+variant)
    for f in plan["fixtures"]:
        p=project/f["file"];check(p.is_file() and sha(p)==f["sha256"],"fixture differs: "+f["file"])
    variants=gp["variants"];groups=gp["groups"];metrics=gp["metrics"];stats=gp["statistics"]
    expected=[(i,f["file"],v,pos,f["group"]) for i,order in enumerate(plan["orders"])
              for f in plan["fixtures"] for pos,v in enumerate(order)]
    actual=[(r["repeat"],r["file"],r["variant"],r["within_frame_position"],r["group"]) for r in raw["rows"]]
    check(actual==expected,"raw row inventory, pairing or execution order differs")
    baseline={(r["repeat"],r["file"]):r for r in raw["rows"] if r["variant"]=="baseline"}
    for r in raw["rows"]:
        try:valid_output(r["stdout"])
        except (ValueError,KeyError,TypeError) as exc:errors.append(str(exc))
        check(r["returncode"]==0,"nonzero replay exit")
        ref=baseline.get((r["repeat"],r["file"]))
        check(ref is not None and r["stdout"]==ref["stdout"] and r.get("exact_equal_to_baseline") is True,"exact output mismatch")
        check(all(finite(r[m]) and r[m]>=0 for m in metrics),"invalid replay timing")
    batch={};pooled={}
    for group in groups:
        pooled[group]={}
        for v in variants:
            selected=[r for r in raw["rows"] if r["variant"]==v and (group=="all" or r["group"]==group)]
            pooled[group][v]={m:summary([r[m] for r in selected]) for m in metrics}
            for m in metrics:
                for s in stats:
                    check(close(pooled[group][v][m][s],raw["summary"][group][v][m][s]),"pooled summary differs")
    for i in range(len(plan["orders"])):
        batch[str(i)]={}
        for g in groups:
            batch[str(i)][g]={}
            for v in variants:
                selected=[r for r in raw["rows"] if r["repeat"]==i and r["variant"]==v and (g=="all" or r["group"]==g)]
                batch[str(i)][g][v]={m:summary([r[m] for r in selected]) for m in metrics}
                for m in metrics:
                    for s in stats:
                        check(close(batch[str(i)][g][v][m][s],raw["batch_summary"][str(i)][g][v][m][s]),"batch summary differs")
    differential={}
    for variant in variants[1:]:
        path=directory/variant/"geometry_differential.csv"
        inherited=False
        if not path.is_file():
            origin=directory.parent
            prior=load(origin/"frozen_manifest.json")
            check(prior["source_sha256"].get(variant)==raw["source_sha256"].get(variant),"inherited arithmetic check geometry source differs: "+variant)
            path=origin/variant/"geometry_differential.csv"
            check(sha(path)==prior["files_sha256"].get(variant+"/geometry_differential.csv"),"inherited differential evidence hash differs: "+variant)
            inherited=True
        rows=list(csv.DictReader(path.open()))
        check(len(rows)==1,"differential result row count differs")
        d={k:int(v) for k,v in rows[0].items()};differential[variant]=dict(counts=d,evidence=evidence(path),inherited_same_geometry_source=inherited)
        check(d.get("queries",0)>0 and d.get("mismatches") == 0,"arithmetic differential failed: "+variant)
        for k,v in d.items():
            if "mismatch" in k:check(v==0,"differential mismatch: "+variant+"/"+k)
    comparisons={}
    for b,a in gp["required_comparisons"]+gp["descriptive_comparisons"]:
        key=b+"_to_"+a;values={};harm=[];uncertain_adverse=[];benefit=[]
        for g in groups:
            values[g]={}
            for m in metrics:
                values[g][m]={}
                for s in stats:
                    p=paired["comparisons"][key][g][m][s]
                    before=[batch[str(i)][g][b][m][s] for i in range(len(plan["orders"]))]
                    after=[batch[str(i)][g][a][m][s] for i in range(len(plan["orders"]))]
                    check(all(close(x,y) for x,y in zip(before,p["before"])) and len(before)==len(p["before"]),"paired before vector differs")
                    check(all(close(x,y) for x,y in zip(after,p["after"])) and len(after)==len(p["after"]),"paired after vector differs")
                    good=sum(y<x for x,y in zip(before,after));bad=sum(y>x for x,y in zip(before,after))
                    check((good,bad)==(p["improved_batches"],p["worsened_batches"]),"paired direction counts differ")
                    bs=summary(before);ass=summary(after);effect=1-ass["mean"]/bs["mean"]
                    check(close(effect,p["relative_reduction"]),"paired relative effect differs")
                    ci=p["bootstrap_95pct_ci"]
                    check(len(ci)==2 and all(finite(x) for x in ci) and ci[0]<=ci[1],"invalid interval")
                    classification=direction_class(ci,good,bad,len(before))
                    values[g][m][s]={**p,"before_summary":bs,"after_summary":ass,"classification":classification}
                    label=g+"/"+m+"/"+s
                    if classification=="reproducible_regression":harm.append(label)
                    if classification=="unresolved_variation" and effect<0:uncertain_adverse.append(label)
                    if classification=="reproducible_gain" and g=="all" and m=="wall_ms" and s in ("mean","p95"):benefit.append(label)
        pooled_change={g:{m:{s:1-pooled[g][a][m][s]/pooled[g][b][m][s] for s in stats} for m in metrics} for g in groups}
        comparisons[key]=dict(before=b,after=a,metrics=values,pooled_relative_reduction=pooled_change,
            reproducible_benefit=benefit,reproducible_regression=harm,unresolved_adverse_estimates=uncertain_adverse,
            geometry_status="REGRESSION" if harm else ("ELIGIBLE_FOR_TASK_CHECK" if benefit else "UNRESOLVED_BENEFIT"))
    required=[comparisons[b+"_to_"+a]["geometry_status"] for b,a in gp["required_comparisons"]]
    return dict(scope="geometry runtime only; actual Isaac and build-specific safety contracts pending",
        decision="INVALID_EVIDENCE" if errors else ("FOUR_VARIANTS_ELIGIBLE_FOR_TASK_CHECK" if all(x=="ELIGIBLE_FOR_TASK_CHECK" for x in required) else "REVIEW_COMPONENTS"),
        errors=list(dict.fromkeys(errors)),row_count=len(raw["rows"]),complete_four_variant_cases=len(expected)//4,
        exact_nonbaseline_comparisons=sum(r["variant"]!="baseline" for r in raw["rows"]),
        independent_batches=len(plan["orders"]),paired_frame_count=gp["fixtures"],
        comparisons=comparisons,pooled_summary=pooled,batch_summary=batch,differential=differential,
        evidence=[evidence(directory/p) for p in ("plan.json","frozen_manifest.json","replay_ablation.json","paired_analysis.json","analyze_ablation.py")],
        policy=policy,ci_scope="Reviewed frozen geometry/analyze_ablation.py generated the supplied 20000 paired batch bootstrap intervals; this collector independently recomputes raw summaries, pairing, effect and signs. Intervals are descriptive and unadjusted.")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory",type=Path,required=True)
    parser.add_argument("--project",type=Path,default=Path(__file__).resolve().parents[3])
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args();policy_path=Path(__file__).with_name("policy.json")
    result=collect(args.directory,args.project,load(policy_path));result["policy_evidence"]=evidence(policy_path)
    write_new(args.output,result)
    print(json.dumps({k:result[k] for k in ("decision","errors","row_count","complete_four_variant_cases","exact_nonbaseline_comparisons")}))
    return 1 if result["errors"] else 0


if __name__=="__main__":raise SystemExit(main())
