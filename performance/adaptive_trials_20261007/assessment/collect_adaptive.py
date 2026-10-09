#!/usr/bin/env python3
"""Collect run_adaptive_ablation evidence without running a simulator or tests.

Only creates new output files; source manifests, original logs and analyses remain
unchanged. See ADAPTER.md for verification scope and contract provenance.
"""
from __future__ import annotations
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import tarfile
import xml.etree.ElementTree as ET
import assess

OPTIONS = ("FOV_GVF_TRIAL_GOAL_POLISH", "FOV_GVF_TRIAL_MANUAL_CONTINUOUS")
INCIDENTAL_ENV = {
    "FOV_GVF_CONTROLLER_EXECUTABLE", "FOV_GVF_RUN_ID", "FOV_GVF_RUNTIME_MANIFEST",
    "FOV_GVF_REPLAY_DIR", "FOV_GVF_PERFORMANCE_LOG", "ISAAC_ACCEPTANCE_TRACE",
    "ISAAC_BENCHMARK_RESULT",
}
INCIDENTAL_PARAMS = {"paper_goal_refinement_steps", "paper_manual_continuous_refinement",
    "performance_log_path", "performance_run_id", "paper_replay_directory"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def evidence(path):
    p=Path(path).resolve()
    return dict(path=str(p),sha256=sha(p))


def frozen_sources(directory):
    directory=Path(directory).resolve()
    manifest_path=directory/"source_sha256.json"
    expected=load(manifest_path)
    actual={}
    with tarfile.open(directory/"sources.tar.gz") as archive:
        for member in archive.getmembers():
            if member.isfile():
                key=member.name.removeprefix("./")
                if key in actual:raise ValueError("duplicate frozen archive source: "+key)
                actual[key]=hashlib.sha256(archive.extractfile(member).read()).hexdigest()
    mismatch=sorted(k for k in expected.keys()|actual.keys() if expected.get(k)!=actual.get(k))
    return dict(directory=str(directory),manifest_sha256=sha(manifest_path),
        archive_sha256=sha(directory/"sources.tar.gz"),source_sha256=expected,
        binary_sha256=load(directory/"binary_sha256.json"),archive_matches=not mismatch,
        archive_mismatch=mismatch,files=len(actual))


def run_id(meta):
    rid=meta.get("experiment_environment",{}).get("FOV_GVF_RUN_ID")
    if not rid:raise ValueError("run metadata has no FOV_GVF_RUN_ID")
    return rid


def launch_time(log):
    # ROS launch creates its logging directory only after the shell acquires the
    # shared Isaac lock. PERF Started / driver START precede the lock and cannot
    # order concurrent drivers correctly.
    matches=re.findall(r"All log files can be found below [^\n]*/(\d{4}-\d\d-\d\d-\d\d-\d\d-\d\d-\d+)-",Path(log).read_text(errors="replace"))
    if len(matches)!=1:raise ValueError("need exactly one actual ROS launch timestamp: "+str(log))
    return datetime.strptime(matches[0],"%Y-%m-%d-%H-%M-%S-%f").isoformat(timespec="microseconds")


def conditions_payload(meta,manifest,runtime):
    env=meta.get("experiment_environment",{})
    condition_env={k:v for k,v in env.items() if k not in INCIDENTAL_ENV and k not in OPTIONS}
    # Run IDs/output paths are incidental. Input paths intentionally remain,
    # together with their content hashes in the runtime file manifest.
    result=dict(case=meta["case"],environment=condition_env,flags=meta.get("flags",{}),
        duration_s=meta.get("duration_s"),trace_sha256=meta.get("trace_sha256"),
        scene_sha256=meta.get("scene_sha256"),runtime_file_sha256=manifest["file_sha256"])
    if meta["case"]=="goal":
        result["goal_protocol"]=meta.get("result",{}).get("protocol")
        result["effective_parameters"]={k:v for k,v in runtime.get("effective",{}).items() if k not in INCIDENTAL_PARAMS}
        result["runtime_environment"]=runtime.get("runtime_environment",{})
    return result


def variant_payload(meta,freeze):
    env=meta.get("experiment_environment",{})
    return dict(source_manifest_sha256=freeze["manifest_sha256"],
        source_archive_sha256=freeze["archive_sha256"],
        controller_sha256=meta.get("controller_sha256"),flags=meta.get("flags",{}),
        experimental_options={key:env.get(key,"0") for key in OPTIONS})


def verify_runtime(meta,manifest,freeze,project):
    errors=[];checks={}
    env=meta.get("experiment_environment",{})
    runtime_path=(Path(meta["result_path"]).with_suffix(".parameters.json") if meta["case"]=="goal"
        else Path(env.get("FOV_GVF_RUNTIME_MANIFEST","/missing_runtime")))
    runtime=load(runtime_path) if runtime_path.is_file() else {}
    if not runtime:errors.append("actual runtime manifest missing")
    expected_binary=manifest["binary_sha256"].get(meta["variant"])
    frozen_binary=freeze["binary_sha256"].get("depth_angular_controller")
    binary=Path(meta.get("controller_executable","/missing_binary"))
    checks["binary_sha256"]=dict(expected=expected_binary,freeze=frozen_binary,
        metadata=meta.get("controller_sha256"),runtime=runtime.get("executable_sha256"),
        actual=sha(binary) if binary.is_file() else None)
    if len(set(checks["binary_sha256"].values()))!=1 or not expected_binary:
        errors.append("binary SHA differs between freeze/manifest/process/runtime/file")
    if not runtime.get("executable") or Path(runtime["executable"]).resolve()!=binary.resolve():
        errors.append("actual executable path differs")
    cloud=project/"src/pc_gvf/launch/isaac_cloud_navigation.launch.py"
    benchmark=project/"src/pc_gvf/launch/navigation_benchmark.launch.py"
    selected=benchmark if meta["case"]=="goal" else cloud
    install=Path(env.get("FOV_GVF_INSTALL","/missing_install"))/"pc_gvf/share/pc_gvf/launch"/selected.name
    files=manifest.get("file_sha256",{})
    cloud_expected=files.get(str(cloud))
    checks["cloud_launch"]=dict(expected=cloud_expected,process=meta.get("launch_sha256"),runtime=runtime.get("launch_sha256"))
    if not cloud_expected or len(set(checks["cloud_launch"].values()))!=1:
        errors.append("actual cloud launch SHA differs/missing")
    checks["selected_installed_launch"]=dict(path=str(install),resolved=str(install.resolve()),
        expected=files.get(str(selected)),actual=sha(install) if install.is_file() else None,
        scope="post-run installed launch hash; goal runtime directly records cloud parameter source, not wrapper __file__")
    if checks["selected_installed_launch"]["actual"]!=checks["selected_installed_launch"]["expected"] or not checks["selected_installed_launch"]["actual"]:
        errors.append("selected installed launch differs/missing")
    for path,expected in files.items():
        p=Path(path)
        if not p.is_file() or sha(p)!=expected:errors.append("driver-frozen runtime file differs/missing: "+path)
    for rel,actual in runtime.get("common_source_sha256",{}).items():
        if files.get(str(project/rel))!=actual:errors.append("actual shared runtime source differs: "+rel)
    if meta["case"]=="goal":
        effective=runtime.get("effective",{})
        wanted={"paper_goal_refinement_steps":int(env.get(OPTIONS[0],"0")),
            "paper_manual_continuous_refinement":env.get(OPTIONS[1],"0")=="1"}
        checks["trial_effective_parameters"]={k:dict(expected=v,actual=effective.get(k)) for k,v in wanted.items()}
        if any(effective.get(k)!=v for k,v in wanted.items()):errors.append("actual goal trial parameter differs")
    else:
        checks["trial_effective_parameters"]={"scope":"manual runtime has no parameter dump; inferred from frozen launch + captured environment, not independently observed ROS values"}
    if not freeze["archive_matches"]:errors.append("frozen archive does not match source manifest")
    if meta.get("runtime_binary_verified") is not True:errors.append("driver binary verification did not pass")
    return runtime,dict(verified=not errors,errors=errors,checks=checks,
        runtime_file=evidence(runtime_path) if runtime_path.is_file() else None)


def reviewed_contracts(registry_path,freeze):
    """Validate the reviewed inventory; never derive case counts from 'N passed'."""
    if registry_path is None:return {},["no reviewed contract inventory supplied"]
    path=Path(registry_path).resolve();inventory=load(path);base=path.parent
    errors=[]
    if inventory.get("source_manifest_sha256")!=freeze["manifest_sha256"]:errors.append("contract inventory source freeze differs")
    if inventory.get("controller_sha256")!=freeze["binary_sha256"].get("depth_angular_controller"):errors.append("contract inventory controller freeze differs")
    for rel,expected in inventory.get("source_sha256",{}).items():
        if freeze["source_sha256"].get(rel)!=expected:errors.append("contract source fingerprint differs: "+rel)
        copy_path=base/"contract_evidence"/Path(rel).name
        if not copy_path.is_file() or sha(copy_path)!=expected:errors.append("contract frozen source copy differs: "+rel)
    ctest=base/"contract_evidence/ctest_candidates_original.log"
    ctest_text=ctest.read_text() if ctest.is_file() else ""
    for check in inventory.get("checks",{}).values():
        for test in check.get("test_names",[]):
            if not re.search(r"Test\s+#\d+:\s+"+re.escape(test)+r"\s+\.+\s+Passed",ctest_text):errors.append("test pass not found: "+test)
        for raw in check.get("evidence",[]):
            if not (base/raw).is_file():errors.append("contract evidence missing: "+raw)
    xml=base/"contract_evidence/manual_control_math_39_passed.xunit.xml"
    if xml.is_file():
        target=inventory.get("checks",{}).get("stale_command_rejection",{}).get("python_case")
        cases=[n for n in ET.parse(xml).iter("testcase") if n.get("name")==target]
        if len(cases)!=1 or list(cases[0]):errors.append("specific stale command pytest case not passed")
    else:errors.append("specific stale command XML missing")
    checks={}
    for name,check in inventory.get("checks",{}).items():
        check=dict(check)
        check["evidence"]=[str((base/raw).resolve()) for raw in check.get("evidence",[])]+[str(path),str(Path(freeze["directory"])/"source_sha256.json")]
        if errors:check.update(cases=0,unsafe=None,passed=False,verification_errors=errors)
        checks[name]=check
    return checks,errors


def discover_analysis(manifest_path,manifest):
    paths=[]
    if manifest.get("case")=="goal":
        for meta in manifest["runs"]:
            if meta.get("result_path"):
                result=Path(meta["result_path"])
                label=result.stem.rsplit("_ego1p5_",1)[0]
                paths.append(result.parent/(label+"_analysis.json"))
    else:
        paths.append(manifest_path.parent/(manifest_path.parent.name+"_analysis.json"))
    return list(dict.fromkeys(paths))


def collect(manifests,freeze_paths,analysis_paths=None,registry=None,project=None,allow_incomplete=False):
    project=Path(project or Path(__file__).resolve().parents[3]).resolve()
    freezes={side:frozen_sources(path) for side,path in freeze_paths.items()}
    contracts_by_side={};contract_issues={}
    for side,freeze in freezes.items():contracts_by_side[side],contract_issues[side]=reviewed_contracts(registry,freeze)
    selected=[];issues=[];evidence_files=[];discovered=[];extra_process=[]
    for file in manifests:
        file=Path(file).resolve();manifest=load(file);evidence_files.append(evidence(file))
        if manifest.get("complete") is not True:
            issues.append("manifest incomplete: "+str(file))
            if not allow_incomplete:raise ValueError(issues[-1]+"; wait or explicitly use --allow-incomplete")
        discovered.extend(discover_analysis(file,manifest))
        known_logs=set()
        for meta in manifest.get("runs",[]):
            log=Path(meta["log"]);known_logs.add(log.stem)
            process=log.with_name(log.stem+"_process.json")
            if not process.is_file() or load(process)!=meta:issues.append("process/manifest mismatch: "+str(process))
            else:evidence_files.append(evidence(process))
            selected.append((file,manifest,meta))
        for process in file.parent.glob("*_process.json"):
            if process.name.removesuffix("_process.json") not in known_logs:
                extra_process.append(str(process));meta=load(process)
                selected.append((file,manifest,meta));issues.append("orphan process record retained: "+str(process))
    paths=list(dict.fromkeys(Path(p).resolve() for p in (analysis_paths or discovered)))
    indexed={}
    for path in paths:
        if not path.is_file():issues.append("analysis missing: "+str(path));continue
        data=load(path);rows=data.get("trials",data.get("runs"))
        if not isinstance(rows,list):raise ValueError("analysis has no individual rows: "+str(path))
        evidence_files.append(evidence(path))
        for row in rows:
            rid=row.get("run_id",row.get("name"))
            if not rid:raise ValueError("analysis row lacks ID")
            if rid in indexed:raise ValueError("duplicate analysis row: "+rid)
            indexed[rid]=row
    times={};records=[];seen=set()
    for file,manifest,meta in selected:
        rid=run_id(meta)
        if rid in seen:raise ValueError("duplicate run across supplied manifests: "+rid)
        seen.add(rid)
        try:times[rid]=launch_time(meta["log"])
        except (ValueError,OSError) as exc:issues.append(str(exc))
        records.append((rid,file,manifest,meta))
    if len(set(times.values()))!=len(times):issues.append("actual launch timestamps are not unique")
    sequence={rid:i+1 for i,(rid,_) in enumerate(sorted(times.items(),key=lambda item:item[1]))}
    supplement=dict(runs={},contracts={});outputs={"before":[],"after":[]};run_audits={}
    for rid,file,manifest,meta in records:
        side=meta["variant"];freeze=freezes[side]
        runtime,audit=verify_runtime(meta,manifest,freeze,project)
        row=indexed.get(rid)
        if row is None:
            issues.append("completed process lacks individual analysis: "+rid)
            row=({**meta.get("result",{}),"run_id":rid,"algorithm":"ego1p5"} if meta["case"]=="goal" else
                {"name":rid,"algorithm":meta.get("algorithm"),"case":meta["case"],"returncode":meta.get("returncode")})
        expected_exit=meta.get("returncode")
        actual_exit=row.get("process_exit_code",row.get("returncode"))
        if actual_exit!=expected_exit:audit["errors"].append("analysis/process exit status differs");audit["verified"]=False
        if meta.get("audit_safe") is not True:audit["errors"].append("driver safe audit not true");audit["verified"]=False
        vp=variant_payload(meta,freeze);cp=conditions_payload(meta,manifest,runtime)
        variant=canonical_sha(vp);conditions=canonical_sha(cp)
        supplement["contracts"][variant]=contracts_by_side[side]
        supplement["runs"][rid]=dict(metadata=dict(sequence=sequence.get(rid),variant_sha256=variant,
            conditions_sha256=conditions,freeze_verified=audit["verified"] and not issues,
            actual_launch_local_time=times.get(rid),sequence_source="post-lock ROS launch log directory timestamp"),
            csv=str(Path(meta["csv"]).resolve()),performance=str(Path(meta["performance"]).resolve()),log=str(Path(meta["log"]).resolve()))
        for key in ("csv","performance","log"):
            path=Path(supplement["runs"][rid][key])
            if not path.is_file():issues.append(rid+": missing "+key)
        outputs[side].append(row)
        run_audits[rid]=dict(manifest=str(file),freeze=audit,variant_payload=vp,conditions_payload=cp)
    # Global inventory incompleteness must not be hidden by valid last rows.
    if issues:
        for run in supplement["runs"].values():run["metadata"]["freeze_verified"]=False
    supplement["collection_audit"]=dict(issues=issues,contract_issues=contract_issues,
        run_audits=run_audits,input_files=evidence_files,freezes=freezes,
        orphan_process_files=extra_process,unused_analysis_ids=sorted(set(indexed)-seen),
        actual_sequence=[dict(run_id=rid,sequence=sequence[rid],launch_time=times[rid]) for rid in sequence],
        scope="Launch/env/component checks only. Manual actual parameter dump is unavailable; goal wrapper installed-file hash is a post-run audit. Source and binary archives are associated frozen artifacts; no rebuild was performed.")
    if set(indexed)-seen:
        issues.append("analysis contains runs absent from provided manifests; keep full manifest coverage")
        for run in supplement["runs"].values():run["metadata"]["freeze_verified"]=False
    return supplement,outputs


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest",nargs="+",type=Path,required=True)
    p.add_argument("--freeze",type=Path,required=True)
    p.add_argument("--before-freeze",type=Path)
    p.add_argument("--analysis",nargs="+",type=Path)
    p.add_argument("--contracts",type=Path,default=Path(__file__).with_name("contracts_bc_reviewed.json"))
    p.add_argument("--project",type=Path)
    p.add_argument("--allow-incomplete",action="store_true")
    p.add_argument("--output-dir",type=Path,required=True)
    p.add_argument("--plan",type=Path)
    p.add_argument("--stage",choices=("explore","confirm"),default="explore")
    args=p.parse_args()
    if args.output_dir.exists():p.error("output directory exists; use a new path to preserve prior evidence")
    try:
        supplement,outputs=collect(args.manifest,{"before":args.before_freeze or args.freeze,"after":args.freeze},
            args.analysis,args.contracts,args.project,args.allow_incomplete)
        args.output_dir.mkdir(parents=True)
        def write(name,value):
            target=args.output_dir/name
            with target.open("x") as f:json.dump(assess.json_safe(value),f,indent=2,allow_nan=False);f.write("\n")
            return target
        sp=write("supplement.json",supplement)
        # All rows are copied exactly; profile detection in assess uses row keys.
        bp=write("before_analysis.json",{"runs":outputs["before"]})
        ap=write("after_analysis.json",{"runs":outputs["after"]})
        decision="COLLECTED"
        if args.plan:
            before,bprov=assess.load_analysis([bp]);after,aprov=assess.load_analysis([ap])
            assess.augment(before,supplement,args.output_dir.resolve());assess.augment(after,supplement,args.output_dir.resolve())
            result=assess.evaluate(before,after,load(args.plan),supplement,args.output_dir.resolve(),args.stage)
            result["input_files"]=bprov+aprov+[evidence(sp),evidence(args.plan)]
            result["raw_supplement_evidence"]={rid:{"metadata":run["metadata"],"files":run["supplements"],"release":run.get("release"),"lifecycle":run.get("lifecycle")} for rid,run in {**before,**after}.items()}
            write("assessment.json",result);decision=result["decision"]
        print(json.dumps(dict(decision=decision,output_dir=str(args.output_dir),runs=sum(map(len,outputs.values())),collection_issues=supplement["collection_audit"]["issues"])))
        return 2 if supplement["collection_audit"]["issues"] or decision in ("REJECT","INSUFFICIENT","NO_PRACTICAL_IMPROVEMENT") else 0
    except (OSError,ValueError,KeyError,TypeError,ET.ParseError) as exc:p.error(str(exc))

if __name__=="__main__":raise SystemExit(main())
