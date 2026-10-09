#!/usr/bin/env python3
"""Read retained-gain Isaac artifacts; never run simulation or apply old thresholds."""
from __future__ import annotations
import argparse
import hashlib
import json
import re
import tarfile
import xml.etree.ElementTree as ET
from pathlib import Path
from common import load, sha, evidence, canonical_sha, write_new, finite, paired
from task_metrics import normalize, csv_release, performance, lifecycle, launch_time

INCIDENTAL_ENV={"FOV_GVF_CONTROLLER_EXECUTABLE","FOV_GVF_RUN_ID","FOV_GVF_RUNTIME_MANIFEST",
    "FOV_GVF_REPLAY_DIR","FOV_GVF_PERFORMANCE_LOG","ISAAC_ACCEPTANCE_TRACE","ISAAC_BENCHMARK_RESULT"}
INCIDENTAL_PARAMS={"performance_log_path","performance_run_id","paper_replay_directory"}
FEATURES={"FOV_GVF_DEPTH_UNCERTAINTY","FOV_GVF_DYNAMIC_OBSTACLES","FOV_GVF_INCREMENTAL_FIELD",
    "FOV_GVF_SPHERICAL_MEMORY","FOV_GVF_OPERATOR_ASSISTANCE","FOV_GVF_SHARED_OBSTACLES"}


def frozen_sources(directory):
    directory=Path(directory).resolve();sources=load(directory/"source_sha256.json")
    actual={};contents={}
    with tarfile.open(directory/"sources.tar.gz") as archive:
        for member in archive.getmembers():
            if not member.isfile():continue
            key=member.name.removeprefix("./")
            if key in actual:raise ValueError("duplicate archive source: "+key)
            data=archive.extractfile(member).read();actual[key]=hashlib.sha256(data).hexdigest()
            contents[key]=data
    mismatch=sorted(k for k in sources.keys()|actual.keys() if sources.get(k)!=actual.get(k))
    binary=load(directory/"binary_sha256.json")
    return dict(directory=str(directory),source_manifest_sha256=sha(directory/"source_sha256.json"),
        source_archive_sha256=sha(directory/"sources.tar.gz"),source_sha256=sources,
        binary_sha256=binary,archive_matches=not mismatch,archive_mismatch=mismatch),contents


def conditions_payload(meta,manifest,runtime):
    payload=dict(case=meta["case"],flags=meta.get("flags"),duration_s=meta.get("duration_s"),
        trace_sha256=meta.get("trace_sha256"),scene_sha256=meta.get("scene_sha256"),
        environment={k:v for k,v in meta["experiment_environment"].items() if k not in INCIDENTAL_ENV},
        runtime_files=manifest["file_sha256"])
    if meta["case"]=="goal":
        payload["protocol"]=meta.get("result",{}).get("protocol")
        payload["effective_parameters"]={k:v for k,v in runtime.get("effective",{}).items() if k not in INCIDENTAL_PARAMS}
        payload["runtime_environment"]=runtime.get("runtime_environment",{})
    return payload


def verify_runtime(meta,manifest,freeze,project,file_hashes):
    errors=[];env=meta["experiment_environment"]
    rtpath=(Path(meta["result_path"]).with_suffix(".parameters.json") if meta.get("result_path")
            else Path(env.get("FOV_GVF_RUNTIME_MANIFEST","/missing_runtime")))
    runtime=load(rtpath) if rtpath.is_file() else {}
    if not runtime:errors.append("actual runtime manifest missing")
    binary=Path(meta["controller_executable"]);expected=manifest["binary_sha256"].get(meta["variant"])
    hashes=dict(manifest=expected,process=meta.get("controller_sha256"),runtime=runtime.get("executable_sha256"),
        frozen=freeze["binary_sha256"].get("depth_angular_controller"),actual=sha(binary) if binary.is_file() else None)
    if not expected or len(set(hashes.values()))!=1:errors.append("actual binary SHA mismatch")
    if not runtime.get("executable") or Path(runtime["executable"]).resolve()!=binary.resolve():errors.append("runtime executable path mismatch")
    cloud=project/"src/pc_gvf/launch/isaac_cloud_navigation.launch.py"
    files=manifest["file_sha256"];cloud_sha=files.get(str(cloud))
    if not cloud_sha or runtime.get("launch_sha256")!=cloud_sha or meta.get("launch_sha256")!=cloud_sha:
        errors.append("actual cloud launch SHA mismatch")
    for path,expected_sha in files.items():
        if path not in file_hashes:
            p=Path(path);file_hashes[path]=sha(p) if p.is_file() else None
        if file_hashes[path]!=expected_sha:errors.append("frozen runtime file changed: "+path)
    install=Path(env.get("FOV_GVF_INSTALL","/missing_install"))/"pc_gvf/share/pc_gvf/launch"
    for name in ("isaac_cloud_navigation.launch.py","navigation_benchmark.launch.py"):
        src=project/"src/pc_gvf/launch"/name;dest=install/name
        if files.get(str(dest)) is None or files.get(str(dest))!=files.get(str(src)):
            errors.append("installed launch absent/differs from source: "+name)
    common=runtime.get("common_source_sha256",{})
    for rel,digest in common.items():
        if files.get(str(project/rel))!=digest:errors.append("actual common source differs: "+rel)
    if meta["case"]=="goal" and (not runtime.get("effective") or not common):
        errors.append("goal runtime effective/common-source evidence missing")
    if env.get("ISAAC_TWIST_SAMPLE_SOURCE")!="callback":errors.append("execution source is not callback")
    if set(meta.get("flags",{}))!=FEATURES or any(meta["flags"].get(k)!="0" or env.get(k)!="0" for k in FEATURES):
        errors.append("six-feature flags missing or enabled")
    for key in ("runtime_binary_verified","runtime_launch_verified","runtime_common_sources_verified"):
        if meta.get(key) is not True:errors.append("driver check not true: "+key)
    if not freeze["archive_matches"]:errors.append("frozen source archive mismatch")
    return runtime,dict(verified=not errors,errors=errors,binary=hashes,
        runtime_file=evidence(rtpath) if rtpath.is_file() else None,
        manual_parameter_scope="Manual has no actual ROS parameter dump; frozen launch+environment only. Common-source boolean with empty map is not direct runtime-source observation." if meta["case"]!="goal" else None,
        installed_launch_scope="Source and installed launch SHA captured before driver execution and audited after; cloud runtime directly records SHA, goal wrapper __file__ is not independently recorded.")


def contracts_check(registry,variant,freeze,contents,base):
    entry=registry.get("variants",{}).get(variant) if registry else None
    if not entry:return dict(status="UNKNOWN",issues=["no build-bound reviewed negative-case inventory"],checks={})
    issues=[]
    if entry.get("source_manifest_sha256")!=freeze["source_manifest_sha256"]:issues.append("contract source manifest differs")
    if entry.get("controller_sha256")!=freeze["binary_sha256"].get("depth_angular_controller"):issues.append("contract controller differs")
    cap=entry.get("capability_summary",{})
    cap_path=base/cap.get("path","missing")
    if not cap_path.is_file() or sha(cap_path)!=cap.get("sha256"):
        issues.append("build-bound capability summary missing/differs")
    else:
        data=load(cap_path);variant_data=data.get("variants",{}).get(variant,{})
        if variant_data.get("sha256")!=freeze["binary_sha256"].get("libpc_gvf_depth_angular_core.a") or variant_data.get("source_manifest_sha256")!=freeze["source_manifest_sha256"]:
            issues.append("capability frozen library/source association differs")
        for test in ("paper_guidance_check","observed_space_check"):
            rows=[r for r in data.get("runs",[]) if r.get("variant")==variant and r.get("test")==test]
            if len(rows)!=1 or rows[0].get("test_returncode")!=0 or rows[0].get("link_returncode")!=0:
                issues.append("required variant capability test not passed: "+test)
            elif not Path(rows[0]["executable"]).is_file() or sha(rows[0]["executable"])!=rows[0]["executable_sha256"]:
                issues.append("capability check executable differs: "+test)
    for name in ("unknown_rejection","expired_depth_rejection","stale_command_rejection"):
        check=entry.get("checks",{}).get(name,{})
        if type(check.get("cases")) is not int or check["cases"]<=0 or check.get("unsafe")!=0 or check.get("passed") is not True:
            issues.append("negative-case outcome missing/failed: "+name)
        if not check.get("source_assertions") or not check.get("test_passes"):issues.append("assertion/pass evidence missing: "+name)
        for source in check.get("source_assertions",[]):
            data=contents.get(source["file"],b"")
            if not data or hashlib.sha256(data).hexdigest()!=source["sha256"]:issues.append("contract source hash mismatch: "+source["file"])
            if not source.get("contains") or any(token.encode() not in data for token in source.get("contains",[])):
                issues.append("specific contract assertion not present: "+source["file"])
        for ev in check.get("test_passes",[]):
            path=base/ev["path"]
            if not path.is_file() or sha(path)!=ev["sha256"]:issues.append("contract test log hash mismatch: "+str(path));continue
            if ev.get("junit_case"):
                cases=[n for n in ET.parse(path).iter("testcase") if n.get("name")==ev["junit_case"]]
                if len(cases)!=1 or any(n.tag in ("failure","error","skipped") for n in cases[0]):
                    issues.append("specific junit test not passed: "+ev["junit_case"])
            elif not ev.get("pattern") or not re.search(ev["pattern"],path.read_text(errors="replace")):
                issues.append("specific test pass not present: "+str(path))
    return dict(status="VERIFIED_SELECTED_CASES" if not issues else "UNKNOWN_OR_FAILED",issues=issues,checks=entry.get("checks",{}),
        scope="A human-reviewed selected regression inventory; passing these fixtures does not prove all real unknown/expiry/stale cases safe.")


def discover_analysis(path,manifest):
    if manifest["case"]!="goal":return [path.parent/(path.parent.name+"_analysis.json")]
    return list(dict.fromkeys(Path(m["result_path"]).parent/(Path(m["result_path"]).stem.rsplit("_ego1p5_",1)[0]+"_analysis.json")
        for m in manifest["runs"] if m.get("result_path")))


def compare_runs(runs,variants,policy):
    pairs=policy["geometry"]["required_comparisons"] if len(variants)==4 else [["baseline",next(v for v in variants if v!="baseline")]]
    if len(variants)==4:pairs=pairs+[["baseline","combined"]]
    result={};cases=sorted({r["case"] for r in runs.values()});pending=[]
    for b,a in pairs:
        contrast={}
        for case in cases:
            before={r["repeat"]:r for r in runs.values() if r["variant"]==b and r["case"]==case}
            after={r["repeat"]:r for r in runs.values() if r["variant"]==a and r["case"]==case}
            # Never silently drop a failure, missing repeat or missing metric.
            repeats=sorted(set(before)|set(after));metrics={};issues=[]
            if set(before)!=set(after):issues.append("unmatched repeats")
            if len(repeats)<policy["isaac"]["minimum_repeats"]:issues.append("fewer than three pairs")
            good_pairs=[(before[i],after[i]) for i in repeats if i in before and i in after]
            if any(x["conditions_sha256"]!=y["conditions_sha256"] for x,y in good_pairs):issues.append("paired conditions differ")
            wanted=policy["isaac"]["metrics_goal" if case=="goal" else "metrics_manual"]
            for metric in wanted:
                bv=[r["metrics"].get(metric) for r,_ in good_pairs];av=[r["metrics"].get(metric) for _,r in good_pairs]
                if len(good_pairs)!=len(repeats) or not bv or not all(finite(v) for v in bv+av):
                    metrics[metric]=dict(classification="MISSING_OR_CENSORED",before=bv,after=av);continue
                metrics[metric]=paired(bv,av,metric in ("path_efficiency","progress_m"))
            harm=[k for k,v in metrics.items() if v["classification"]=="reproducible_regression"]
            unresolved=[k for k,v in metrics.items() if v["classification"] in ("unresolved_variation","MISSING_OR_CENSORED","insufficient_repeats")]
            contrast[case]=dict(repeats=repeats,metrics=metrics,issues=issues,reproducible_regressions=harm,
                unresolved_metrics=unresolved,status="REPRODUCIBLE_REGRESSION" if harm else ("INCOMPLETE" if issues else "NO_REPRODUCIBLE_REGRESSION_OBSERVED"))
            pending.extend(b+"_to_"+a+"/"+case+": "+x for x in issues)
            pending.extend(b+"_to_"+a+"/"+case+": missing/censored "+k for k,v in metrics.items() if v["classification"]=="MISSING_OR_CENSORED")
        result[b+"_to_"+a]=contrast
    return result,pending


def collect(manifest_paths,freeze_dirs,analysis_paths,registry_path,project,policy):
    project=Path(project).resolve();freezes={};contents={}
    for variant,directory in freeze_dirs.items():freezes[variant],contents[variant]=frozen_sources(directory)
    registry=load(registry_path) if registry_path else None
    contracts={v:contracts_check(registry,v,f,contents[v],Path(registry_path).parent if registry_path else Path('.')) for v,f in freezes.items()}
    issues=[];hard_failures=[];pending=[];inputs=[];selected=[];auto=[];manifests=[];seen=set();file_hashes={}
    for p in manifest_paths:
        p=Path(p).resolve();m=load(p);manifests.append((p,m));inputs.append(evidence(p));auto+=discover_analysis(p,m)
        if m.get("complete") is not True:pending.append("manifest incomplete: "+str(p))
        if set(m["variants"])!=set(freezes):issues.append("manifest/freeze variant inventory differs: "+str(p))
        expected=[(i+1,v) for i,order in enumerate(m["order_by_repeat"]) for v in order]
        actual=[(r["repeat"],r["variant"]) for r in m["runs"]]
        if actual!=expected:pending.append("run inventory/order incomplete or differs: "+str(p))
        known=set()
        for meta in m["runs"]:
            process=Path(meta["log"]).with_name(Path(meta["log"]).stem+"_process.json");known.add(process)
            if not process.is_file() or load(process)!=meta:issues.append("manifest/process mismatch: "+str(process))
            else:inputs.append(evidence(process))
            selected.append((p,m,meta))
        for process in p.parent.glob("*_process.json"):
            if process not in known:
                selected.append((p,m,load(process)));issues.append("orphan process retained: "+str(process));inputs.append(evidence(process))
    indexed={}
    for path in dict.fromkeys(Path(p).resolve() for p in (analysis_paths or auto)):
        if not path.is_file():pending.append("analysis missing: "+str(path));continue
        inputs.append(evidence(path));data=load(path);rows=data.get("trials",data.get("runs",[]))
        for row in rows:
            rid=row.get("run_id",row.get("name"))
            if not rid or rid in indexed:raise ValueError("missing/duplicate analysis run ID: "+str(rid))
            indexed[rid]=row
    runs={}
    for path,m,meta in selected:
        rid=meta["experiment_environment"]["FOV_GVF_RUN_ID"]
        if rid in seen:raise ValueError("duplicate run selected: "+rid)
        seen.add(rid);v=meta["variant"];case=meta["case"]
        if v not in freezes:raise ValueError("no freeze for variant: "+v)
        runtime,audit=verify_runtime(meta,m,freezes[v],project,file_hashes)
        issues.extend(rid+": "+x for x in audit["errors"])
        row=indexed.get(rid)
        if row is None:pending.append("analysis missing for run: "+rid);row=meta.get("result",{})
        metrics,safety=normalize(row,case)
        if meta["returncode"]!=safety["exit_code"]:issues.append("analysis/process exit mismatch: "+rid)
        if meta["returncode"]!=0:hard_failures.append(rid+": process returned "+str(meta["returncode"]))
        if safety["sweep"]!=0 or safety["external"]!=0:
            (pending if safety["sweep"] is None or safety["external"] is None else hard_failures).append(rid+": swept/external safety evidence not zero")
        if case=="goal" and safety["arrived"] is not True:
            (pending if safety["arrived"] is None else hard_failures).append(rid+": goal arrival not established" if safety["arrived"] is None else rid+": goal not ARRIVED")
        if meta.get("audit_safe") is not True or meta.get("audit_returncode")!=0:hard_failures.append(rid+": driver safety audit not passed")
        artifacts={};log_result={};release={};actual_time=None
        for key in ("csv","performance","log"):
            p=Path(meta[key])
            if not p.is_file():pending.append(rid+": missing "+key);continue
            artifacts[key]=evidence(p)
            if key=="performance":metrics.update(performance(p))
            if key=="log":
                log_result=lifecycle(p)
                try:actual_time=launch_time(p)
                except ValueError as exc:issues.append(rid+": "+str(exc))
            if key=="csv" and case!="goal":
                try:release=csv_release(p);metrics.update({k:release[k] for k in ("release_target_s","release_brake_s")})
                except (ValueError,KeyError) as exc:pending.append(rid+": "+str(exc))
        conditions=conditions_payload(meta,m,runtime)
        vp=dict(source_manifest_sha256=freezes[v]["source_manifest_sha256"],source_archive_sha256=freezes[v]["source_archive_sha256"],controller_sha256=meta["controller_sha256"],flags=meta["flags"])
        runs[rid]=dict(id=rid,variant=v,case=case,repeat=meta["repeat"],variant_sha256=canonical_sha(vp),
            conditions_sha256=canonical_sha(conditions),variant_payload=vp,conditions_payload=conditions,
            actual_launch_local_time=actual_time,runtime_audit=audit,metrics=metrics,safety=safety,
            lifecycle=log_result,release=release,artifacts=artifacts,raw_analysis=row,process_metadata=meta)
    if set(indexed)-seen:issues.append("unselected analysis rows remain: "+", ".join(sorted(set(indexed)-seen)))
    chronological=sorted(runs.values(),key=lambda r:r["actual_launch_local_time"] or "")
    if len({r["actual_launch_local_time"] for r in chronological})!=len(chronological):issues.append("missing/duplicate actual execution timestamp")
    sequence=[]
    for i,r in enumerate(chronological,1):r["sequence"]=i;sequence.append(dict(id=r["id"],variant=r["variant"],case=r["case"],repeat=r["repeat"],sequence=i,actual_launch_local_time=r["actual_launch_local_time"]))
    for path,m in manifests:
        ids=[r["experiment_environment"]["FOV_GVF_RUN_ID"] for r in m["runs"]]
        observed=[r["id"] for r in chronological if r["id"] in ids]
        if observed!=ids:issues.append("post-lock execution order differs from manifest: "+str(path))
    comparisons,cpending=compare_runs(runs,list(freezes),policy);pending+=cpending
    cases={r["case"] for r in runs.values()}
    pending.extend("required case absent: "+case for case in policy["isaac"]["cases"] if case not in cases)
    pending.extend("negative contract evidence not verified: "+v for v,c in contracts.items() if c["status"]!="VERIFIED_SELECTED_CASES")
    regressions=[comp+"/"+case+"/"+metric for comp,cs in comparisons.items() for case,d in cs.items() for metric in d["reproducible_regressions"]]
    decision="HARD_GATE_FAILURE" if hard_failures else ("INVALID_EVIDENCE" if issues else ("REPRODUCIBLE_TASK_REGRESSION" if regressions else ("PENDING_EVIDENCE" if pending else "NO_REPRODUCIBLE_TASK_REGRESSION_OBSERVED")))
    lifecycle_summary={}
    for variant in freezes:
        lifecycle_summary[variant]={}
        for case in cases:
            selected=[r for r in runs.values() if r["variant"]==variant and r["case"]==case]
            lifecycle_summary[variant][case]=dict(
                compute_deadlines_per_run=[r["lifecycle"].get("compute_deadlines") for r in selected],
                nonzero_child_exits_per_run=[r["lifecycle"].get("nonzero_child_exits") for r in selected],
                rosout_invalid_context_per_run=[r["lifecycle"].get("rosout_invalid_context") for r in selected],
                scope="Raw log event counts include shutdown; not unique control deadlines inferred from text repetition.")
    return dict(decision=decision,runs=runs,run_count=len(runs),actual_sequence=sequence,comparisons=comparisons,lifecycle_summary=lifecycle_summary,
        hard_failures=hard_failures,integrity_issues=list(dict.fromkeys(issues)),pending=list(dict.fromkeys(pending)),
        reproducible_task_regressions=regressions,contracts=contracts,freezes=freezes,evidence=inputs,
        policy=policy,scope="Task capability guardrails; geometry runtime gains assessed separately. Three pairs and unadjusted descriptive intervals do not prove population noninferiority. Every run and negative/missing metric is retained; lifecycle shutdown errors are reported separately from swept safety.")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest",nargs="+",type=Path,required=True)
    p.add_argument("--freeze",nargs="+",required=True,help="baseline=DIR v2=DIR v4=DIR combined=DIR")
    p.add_argument("--analysis",nargs="+",type=Path)
    p.add_argument("--contracts",type=Path)
    p.add_argument("--project",type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args();freezes={}
    for item in args.freeze:
        k,sep,v=item.partition("=")
        if not sep or k in freezes:p.error("invalid/duplicate --freeze")
        freezes[k]=Path(v)
    if "baseline" not in freezes or len(freezes) not in (2,4):p.error("baseline plus one candidate or all four freezes required")
    policy_path=Path(__file__).with_name("policy.json")
    result=collect(args.manifest,freezes,args.analysis,args.contracts,args.project,load(policy_path))
    result["policy_evidence"]=evidence(policy_path);write_new(args.output,result)
    print(json.dumps({k:result[k] for k in ("decision","run_count","hard_failures","integrity_issues","pending","reproducible_task_regressions")}))
    return 0 if result["decision"]=="NO_REPRODUCIBLE_TASK_REGRESSION_OBSERVED" else 2


if __name__=="__main__":raise SystemExit(main())
