#!/usr/bin/env python3
"""Adapt raw same-snapshot replay batches without changing timing/policy/data."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import assess
import assess_replay

DISCRETE = ("status","reason","accepted","free_directions","refined","command_changed","reset_reason","continued")
CONTINUOUS = ("command","required_prefix","best_prefix","intent_change_angle")


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()
def load(path):return json.loads(Path(path).read_text())


def parse_output(text):
    out=json.loads(text)
    if set(out)!=set(DISCRETE+CONTINUOUS):raise ValueError("replay output schema differs")
    if any(not isinstance(out[k],str) for k in ("status","reason","reset_reason")):raise ValueError("invalid replay status")
    if any(type(out[k]) is not bool for k in ("accepted","refined","command_changed","continued")):raise ValueError("invalid replay boolean")
    if type(out["free_directions"]) is not int or out["free_directions"]<0:raise ValueError("invalid free direction count")
    command=out["command"]
    if not isinstance(command,list) or len(command)!=3 or any(not assess.finite(x) for x in command):raise ValueError("invalid/nonfinite command")
    if any(not assess.finite(out[k]) for k in CONTINUOUS[1:]):raise ValueError("invalid/nonfinite replay scalar")
    return out


def stats(values):
    if not values or any(not assess.finite(v) or v<0 for v in values):return None
    return dict(n=len(values),mean=statistics.mean(values),p50=assess.quantile(values,.5),p95=assess.quantile(values,.95),max=max(values))


def adapt_phase(path,candidate_id,phase,project,baseline_source,candidate_source,sequence_offset=0,expected_frames=37):
    path=Path(path).resolve();data=load(path)
    manifest=data.get("manifest",data)
    rows=data.get("rows",manifest.get("rows",[]));frames=manifest.get("frames",[])
    problems=[]
    if manifest.get("complete") is not True:problems.append("raw phase incomplete; never treated as completed timing batches")
    filenames=[f["file"] for f in frames]
    if len(filenames)!=expected_frames or len(set(filenames))!=expected_frames:problems.append("fixture inventory count/uniqueness mismatch")
    hashes=manifest.get("frame_sha256",{})
    if set(hashes)!=set(filenames):problems.append("fixture hash inventory missing/different")
    for file in filenames:
        actual=project/file
        if not actual.is_file() or hashes.get(file)!=sha(actual):problems.append("fixture bytes differ/missing: "+file)
    binaries=manifest.get("sha256",{})
    for side in ("baseline","candidate"):
        executable=Path(manifest.get("executables",{}).get(side,"/missing"))
        if not executable.is_file() or sha(executable)!=binaries.get(side):problems.append("frozen executable differs: "+side)
    source_hashes={"baseline_source_manifest":sha(baseline_source) if baseline_source.is_file() else None,
        "candidate_source":sha(candidate_source) if candidate_source.is_file() else None}
    if any(v is None for v in source_hashes.values()):problems.append("source provenance missing")
    frame_fingerprint=canonical({"ordered_frames":frames,"sha256":hashes})
    by_key={}
    for index,row in enumerate(rows):
        key=(row.get("repeat"),row.get("variant"),row.get("file"))
        if key in by_key:problems.append("duplicate replay execution: "+str(key))
        else:by_key[key]=(index,row)
        if row.get("variant") not in ("baseline","candidate") or row.get("file") not in filenames:
            problems.append("unexpected replay execution: "+str(key))
        if type(row.get("repeat")) is not int or row["repeat"]<0:problems.append("invalid repeat index")
    repeats=sorted({r.get("repeat") for r in rows if type(r.get("repeat")) is int and r["repeat"]>=0})
    output={"before":[],"after":[]};pairs=[];batch_audits=[]
    for repeat in repeats:
        expected_order=["baseline","candidate"] if (repeat+(phase=="confirm"))%2==0 else ["candidate","baseline"]
        pair_errors=[];exact_pairs=0
        for file in filenames:
            left=by_key.get((repeat,"baseline",file));right=by_key.get((repeat,"candidate",file))
            if left is None or right is None:pair_errors.append("missing paired fixture: "+file);continue
            li,l=left;ri,r=right
            if abs(li-ri)!=1 or (li<ri)!=(expected_order[0]=="baseline"):
                pair_errors.append("within-fixture execution order mismatch: "+file)
            same=l.get("returncode")==r.get("returncode")==0 and l.get("stdout")==r.get("stdout")
            if not same:pair_errors.append("return code or exact stdout mismatch: "+file)
            else:exact_pairs+=1
            second=rows[max(li,ri)]
            if second.get("pair_exact_output_equal") is not True:pair_errors.append("raw exact-pair attestation missing/false: "+file)
        ids={side:f"{candidate_id}_{phase}_{side}_{repeat+1}" for side in ("before","after")}
        pairs.append(dict(phase=phase,before=ids["before"],after=ids["after"],
            order=["before" if s=="baseline" else "after" for s in expected_order]))
        for source_side,side in (("baseline","before"),("candidate","after")):
            indexed=[by_key[(repeat,source_side,file)] for file in filenames if (repeat,source_side,file) in by_key]
            batch_rows=[r for _,r in indexed];local_errors=list(problems)+list(pair_errors)
            if len(batch_rows)!=expected_frames:local_errors.append("batch does not contain complete fixed fixture set")
            decoded=[]
            for row in batch_rows:
                try:
                    decoded.append(dict(file=row["file"],discrete={k:v for k,v in parse_output(row.get("stdout","")).items() if k in DISCRETE}))
                except (ValueError,TypeError,KeyError) as exc:local_errors.append(row["file"]+": "+str(exc))
                if row.get("returncode")!=0:local_errors.append(row["file"]+": nonzero replay exit")
            wall=stats([r.get("wall_ms") for r in batch_rows]);cpu=stats([r.get("cpu_ms") for r in batch_rows])
            if wall is None:local_errors.append("invalid wall timings")
            seqs=[i+1+sequence_offset for i,_ in indexed]
            payload={"binary_sha256":binaries.get(source_side),"baseline_source_manifest_sha256":source_hashes["baseline_source_manifest"],
                "candidate_source_sha256":source_hashes["candidate_source"] if side=="after" else None,
                "environment":manifest.get("environment",{})}
            record=dict(run_id=ids[side],sequence=min(seqs) if seqs else None,
                p95_ms=wall["p95"] if wall else None,max_ms=wall["max"] if wall else None,
                decisions_sha256=canonical(decoded),decision_count=len(decoded),fixtures_sha256=frame_fingerprint,
                variant_sha256=canonical(payload),numeric_checks_passed=not local_errors,
                variant_payload=payload,phase=phase,batch=repeat+1,frame_count=len(batch_rows),wall_stats=wall,cpu_stats=cpu,
                first_execution_index=min(seqs) if seqs else None,last_execution_index=max(seqs) if seqs else None,
                errors=local_errors,exact_output_pairs_in_batch=exact_pairs)
            output[side].append(record)
        batch_audits.append(dict(repeat=repeat,exact_pairs=exact_pairs,expected_pairs=expected_frames,errors=pair_errors))
    pooled={}
    for group in ["all"]+list(dict.fromkeys(f["group"] for f in frames)):
        pooled[group]={}
        for variant in ("baseline","candidate"):
            selected=[r for r in rows if r.get("variant")==variant and (group=="all" or r.get("group")==group)]
            pooled[group][variant]={k:stats([r.get(k) for r in selected]) for k in ("wall_ms","cpu_ms")}
        old=pooled[group]["baseline"]["wall_ms"];new=pooled[group]["candidate"]["wall_ms"]
        pooled[group]["pooled_wall_p95_reduction_fraction"]=(1-new["p95"]/old["p95"]) if old and new and old["p95"]>0 else None
    audit=dict(path=str(path),sha256=sha(path),phase=phase,complete=manifest.get("complete"),started_unix=manifest.get("started_unix"),ended_unix=manifest.get("ended_unix"),
        source_files={str(baseline_source):source_hashes["baseline_source_manifest"],str(candidate_source):source_hashes["candidate_source"]},
        problems=problems,rows=len(rows),batches=batch_audits,pooled=pooled,
        sequence_definition="First actual execution row index of a 37-frame timing batch; variants are interleaved per frame, not sequential whole-variant blocks.")
    return output,pairs,audit,len(rows)


def collect(exploration,confirmation,candidate_id,project,baseline_source,candidate_source,plan=None,expected_frames=37):
    combined={"before":[],"after":[]};pairs=[];audits=[];offset=0
    for phase,path in (("explore",exploration),("confirm",confirmation)):
        if path is None:continue
        rows,phase_pairs,audit,count=adapt_phase(path,candidate_id,phase,project,baseline_source,candidate_source,offset,expected_frames)
        for side in combined:combined[side].extend(rows[side])
        pairs.extend(phase_pairs);audits.append(audit);offset+=count
    if confirmation is not None:
        first,last=audits
        if not all(assess.finite(x) for x in (first["ended_unix"],last["started_unix"])) or first["ended_unix"]>=last["started_unix"]:
            for rows in combined.values():
                for row in rows:row["numeric_checks_passed"]=False;row["errors"].append("confirmation not demonstrably after exploration ended")
    retrospective=plan is None
    if plan is None:
        plan=dict(candidate_id=candidate_id,policy_sha256=assess_replay.POLICY_SHA256,registration_evidence=[],pairs=pairs,
            retrospective_mapping=True,note="IDs mapped after measurement from actual producer order. This is not preregistration and cannot support a retention verdict.")
    return combined,plan,dict(phases=audits,retrospective_mapping=retrospective,
        scope="Per-batch P95 uses exactly the fixed fixture population. Pooled summaries are descriptive; their frame counts are not independent repetitions. The assessment uses the unchanged replay policy.")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--explore",type=Path,required=True);p.add_argument("--confirm",type=Path)
    p.add_argument("--candidate-id",required=True);p.add_argument("--plan",type=Path)
    p.add_argument("--project",type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument("--baseline-source",type=Path);p.add_argument("--candidate-source",type=Path)
    p.add_argument("--output-dir",type=Path,required=True)
    args=p.parse_args()
    if args.output_dir.exists():p.error("output exists; preserve it and choose a new directory")
    try:
        project=args.project.resolve()
        baseline=(args.baseline_source or project/'performance/adaptive_trials_20261007/baseline/source_sha256.json').resolve()
        source=(args.candidate_source or args.explore.parent/'paper_guidance.cpp').resolve()
        loaded_plan=load(args.plan) if args.plan else None
        rows,plan,audit=collect(args.explore,args.confirm,args.candidate_id,project,baseline,source,loaded_plan)
        args.output_dir.mkdir(parents=True)
        def write(name,value):
            path=args.output_dir/name
            with path.open('x') as f:json.dump(assess.json_safe(value),f,indent=2,allow_nan=False);f.write('\n')
            return path
        write('before_batches.json',{'runs':rows['before']});write('after_batches.json',{'runs':rows['after']})
        write('mapping_plan.json',plan);write('collection_audit.json',audit)
        result=assess_replay.evaluate({r['run_id']:r for r in rows['before']},{r['run_id']:r for r in rows['after']},plan,
            'confirm' if args.confirm else 'explore',args.plan.resolve().parent if args.plan else args.output_dir.resolve())
        result['adapter_evidence']=dict(collector_sha256=sha(Path(__file__)),assessment_sha256=sha(Path(assess_replay.__file__)),
            collection_audit_sha256=sha(args.output_dir/'collection_audit.json'),retrospective_mapping=audit['retrospective_mapping'])
        write('assessment.json',result)
        print(json.dumps(dict(decision=result['decision'],output_dir=str(args.output_dir),before_batches=len(rows['before']),after_batches=len(rows['after']))))
        return 0 if result['decision'] in ('MECHANISM_CONFIRM_RETAIN','EXPLORE_PROMISING') else 2
    except (OSError,ValueError,KeyError,TypeError) as exc:p.error(str(exc))

if __name__=='__main__':raise SystemExit(main())
