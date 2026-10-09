#!/usr/bin/env python3
"""Read-only paired navigation assessment; no simulator or production imports.

CLI never overwrites evidence. See README.md for registration/attestation limits.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Metric:
    unit: str
    direction: str
    tolerance_abs: float
    tolerance_rel: float
    improve_abs: float
    improve_rel: float


POLICY_VERSION = "adaptive_assessment_v1"
# A positive oriented delta always means worse. Absolute floors avoid divisions
# by zero and allow 60 Hz quantization/noise; relative limits scale with the task.
METRICS = {
    "arrival_s": Metric("s", "lower", .20, .01, .30, .01),
    "path_efficiency": Metric("ratio", "higher", .01, 0., .005, 0.),
    "stop_s": Metric("s", "lower", .05, .10, .10, .10),
    "longest_stop_s": Metric("s", "lower", .05, .10, .05, .10),
    "jerk_rms_mps3": Metric("m/s^3", "lower", .02, .05, .05, .10),
    "compute_p95_ms": Metric("ms", "lower", .20, .10, .20, .10),
    "angle_mean_deg": Metric("degree", "lower", .25, .03, .50, .05),
    "angle_p95_deg": Metric("degree", "lower", 1., .05, 1., .05),
    "progress_m": Metric("m", "higher", .10, .01, .20, .01),
    "vector_error_rms_mps": Metric("m/s", "lower", .005, .03, .01, .05),
    "release_target_p95_s": Metric("s", "lower", 1./60., .05, 1./60., .10),
    "release_brake_p95_s": Metric("s", "lower", .05, .05, .05, .10),
}
REQUIRED = {
    "goal": ("arrival_s", "path_efficiency", "stop_s", "longest_stop_s", "jerk_rms_mps3", "compute_p95_ms"),
    "manual": ("angle_mean_deg", "angle_p95_deg", "progress_m", "vector_error_rms_mps", "stop_s", "longest_stop_s", "jerk_rms_mps3", "compute_p95_ms", "release_target_p95_s", "release_brake_p95_s"),
}
CONTRACTS = ("unknown_rejection", "expired_depth_rejection", "stale_command_rejection")
POLICY = dict(version=POLICY_VERSION, metrics={k: asdict(v) for k, v in METRICS.items()},
    required=REQUIRED, exploration_pairs=3, confirmation_pairs=5,
    primary_sign_test_alpha=.05, goal_worst_run_absolute_margin_s=1.0,
    goal_worst_run_relative_margin=.03, release_target_max_s=.05,
    contracts=CONTRACTS)
POLICY_SHA256 = hashlib.sha256(json.dumps(POLICY, sort_keys=True).encode()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quantile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    i = (len(ordered) - 1) * probability
    lo = int(i)
    return ordered[lo] + (i - lo) * (ordered[min(lo + 1, len(ordered) - 1)] - ordered[lo])


def std(values):
    return statistics.stdev(values) if len(values) >= 2 else None


def t95(df):
    # One-sided 95% Student critical values, equivalently central 90% CI.
    # df>=30 uses df30, deliberately conservative rather than normal asymptotics.
    values = [6.313752, 2.919986, 2.353364, 2.131847, 2.015048, 1.943180,
              1.894579, 1.859548, 1.833113, 1.812461, 1.795885, 1.782288,
              1.770933, 1.761310, 1.753050, 1.745884, 1.739607, 1.734064,
              1.729133, 1.724718, 1.720743, 1.717144, 1.713872, 1.710882,
              1.708141, 1.705618, 1.703288, 1.701131, 1.699127, 1.697261]
    return values[min(df, 30) - 1]


def sign_p(improvements):
    # Ties remain non-wins: never silently drop an inconvenient pair.
    n, wins = len(improvements), sum(x > 0 for x in improvements)
    return sum(math.comb(n, k) for k in range(wins, n + 1)) / (2 ** n) if n else None


def compare_metric(name, before, after):
    spec = METRICS[name]
    if len(before) != len(after) or not before:
        raise ValueError("metrics require nonempty equal-sized paired samples")
    if any(not finite(x) for x in before + after):
        raise ValueError("nonfinite metric")
    if name != "progress_m" and any(x < 0 for x in before + after):
        raise ValueError("negative metric outside signed progress")
    sign = 1 if spec.direction == "lower" else -1
    delta = [(a - b) * sign for b, a in zip(before, after)]
    bmean, amean, mean = map(statistics.mean, (before, after, delta))
    tolerance = max(spec.tolerance_abs, abs(bmean) * spec.tolerance_rel)
    threshold = max(spec.improve_abs, abs(bmean) * spec.improve_rel)
    sd = std(delta)
    half = t95(len(delta) - 1) * sd / math.sqrt(len(delta)) if sd is not None else None
    upper = mean + half if half is not None else None
    lower = mean - half if half is not None else None
    state = "REGRESSION" if mean > tolerance + 1e-12 else (
        "NONINFERIOR" if upper is not None and upper <= tolerance + 1e-12 else "UNCERTAIN")
    return dict(unit=spec.unit, direction=spec.direction, n=len(delta),
        before_mean=bmean, after_mean=amean, before_std=std(before), after_std=std(after),
        before_min=min(before), before_max=max(before), after_min=min(after), after_max=max(after),
        oriented_paired_changes=delta, mean_oriented_change=mean, std_paired_change=sd,
        mean_change_percent=(amean-bmean)/abs(bmean)*100 if bmean else None,
        paired_change_central90_t_interval=[lower, upper],
        tolerance=tolerance, improvement_threshold=threshold, noninferiority=state,
        practical_improvement=-mean >= threshold-1e-12,
        improvement_pairs=sum(x < 0 for x in delta), tie_pairs=sum(x == 0 for x in delta),
        primary_one_sided_sign_p=sign_p([-x for x in delta]))


def load_analysis(paths, pattern=None):
    result = {}
    provenance = []
    expression = re.compile(pattern) if pattern else None
    for path in paths:
        path = Path(path).resolve()
        data = json.loads(path.read_text())
        rows = data.get("trials", data.get("runs"))
        if not isinstance(rows, list):
            raise ValueError(f"{path}: expected trials or runs array, aggregate means are insufficient")
        provenance.append(dict(path=str(path), sha256=digest(path)))
        for row in rows:
            rid = row.get("run_id", row.get("name"))
            if not isinstance(rid, str) or not rid:
                raise ValueError(f"{path}: row missing run id; cannot discard failed rows")
            if expression and not expression.search(rid):
                continue
            if rid in result:
                raise ValueError(f"duplicate run id: {rid}")
            result[rid] = normalize(row, path.parent)
    if not result:
        raise ValueError("no individual runs selected")
    return result, provenance


def normalize(row, directory):
    goal = "status" in row or "simulation_elapsed_s" in row
    profile = "goal" if goal else "manual"
    metrics = dict()
    mappings = ({"arrival_s":"simulation_elapsed_s", "stop_s":"stopped_with_input_s"} if goal else
        {"angle_mean_deg":"moving_angle_mean_deg", "angle_p95_deg":"moving_angle_p95_deg",
         "progress_m":"active_projected_progress_m", "vector_error_rms_mps":"active_vector_error_rms_mps", "stop_s":"stop_s"})
    mappings.update({"jerk_rms_mps3":"jerk_rms_mps3", "longest_stop_s":"longest_stop_s"})
    for dest, source in mappings.items():
        if source in row:
            metrics[dest] = row[source]
    if goal:
        start, target = row.get("protocol", {}).get("start"), row.get("protocol", {}).get("goal")
        length = row.get("distance_travelled_m")
        if isinstance(start, list) and isinstance(target, list) and len(start) == len(target) == 3 and all(finite(x) for x in start+target) and finite(length) and length > 0:
            # Fixed start-to-goal distance / actual path. Same arrival tolerance
            # is required by conditions_sha256; can exceed 1 for early stopping.
            metrics["path_efficiency"] = math.dist(start, target) / length
    core = row.get("compute_ms")
    if isinstance(core, dict):
        metrics["core_compute_p95_ms"] = core.get("p95")
    safety = row.get("safety", {})
    return dict(id=row.get("run_id", row.get("name")), profile=profile,
        scenario=row.get("case", "fixed_goal"), metrics=metrics,
        safety=dict(arrived=row.get("status") == "ARRIVED" if goal else None,
            sweep=row.get("swept_overlap_segments", safety.get("swept_sphere_overlap_segments")),
            external=row.get("collision_blocks", safety.get("external_collision_blocks")),
            clearance=row.get("minimum_swept_clearance_m", safety.get("minimum_swept_sphere_clearance_m")),
            exit_code=row.get("process_exit_code", row.get("returncode"))),
        raw=row, directory=str(directory), metadata={}, supplements=[])


def csv_release(path):
    with Path(path).open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    columns = ["t", "qx", "qy", "qz", "vx", "vy", "vz", "applied_target_x", "applied_target_y", "applied_target_z"]
    if len(rows) < 2 or any(k not in rows[0] for k in columns):
        raise ValueError("release CSV requires t, q, v and applied_target columns")
    values = [[float(row[k]) for k in columns] for row in rows]
    if any(not math.isfinite(x) for row in values for x in row):
        raise ValueError("nonfinite release CSV")
    times = [r[0] for r in values]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("release CSV times must increase strictly")
    active = [math.sqrt(sum(x*x for x in r[1:4])) > .05 for r in values]
    speeds = [math.sqrt(sum(x*x for x in r[4:7])) for r in values]
    targets = [math.sqrt(sum(x*x for x in r[7:10])) for r in values]
    events = []
    for i in range(1, len(rows)):
        if active[i] or not active[i-1]:
            continue
        stop = next((j for j in range(i+1, len(rows)) if active[j]), len(rows))
        # Require settled values for the remainder of the release interval,
        # rather than accepting a transient zero followed by rebound.
        def settle(series, limit):
            bad = [j for j in range(i, stop) if series[j] > limit]
            j = bad[-1]+1 if bad else i
            if j >= stop or stop-j < 2:
                return None  # Right-censored: one final sample is not settling.
            return times[j]-times[i]
        events.append(dict(t=times[i], target_s=settle(targets, 1e-6), brake_s=settle(speeds, .05),
            observed_interval_s=times[stop-1]-times[i], initial_speed_mps=speeds[i]))
    targets_ok = [e["target_s"] for e in events if e["target_s"] is not None]
    brakes_ok = [e["brake_s"] for e in events if e["brake_s"] is not None]
    complete = bool(events) and len(targets_ok) == len(brakes_ok) == len(events)
    return dict(events=events, event_count=len(events), complete=complete,
        target_p95_s=quantile(targets_ok, .95) if complete else None,
        brake_p95_s=quantile(brakes_ok, .95) if complete else None,
        target_max_s=max(targets_ok) if complete else None)


def read_perf(path):
    content = Path(path).read_text()
    result = {}
    for phrase, key in [("Control callback to publish", "compute_p95_ms"), ("Avoidance compute", "core_compute_p95_ms")]:
        matches = re.findall(re.escape(phrase) + r" mean / P50 / P95 / max: ([0-9.eE+-]+) / ([0-9.eE+-]+) / ([0-9.eE+-]+) / ([0-9.eE+-]+)", content)
        if matches:
            result[key] = float(matches[-1][2])
    return result


def read_log(path):
    content = Path(path).read_text(errors="replace")
    return dict(nonzero_child_exits=[int(x) for x in re.findall(r"(?:exit code|exit_code)[ :=]+(-?\d+)", content, re.I) if int(x) != 0],
        compute_deadlines=content.count("COMPUTE_DEADLINE"),
        rosout_invalid_context=content.count("Failed to publish log message to rosout: publisher's context is invalid"))


def augment(runs, supplement, base):
    entries = supplement.get("runs", {})
    for rid, run in runs.items():
        info = entries.get(rid, {})
        run["metadata"] = info.get("metadata", {})
        for key, parser in [("csv", csv_release), ("performance", read_perf), ("log", read_log)]:
            raw = info.get(key)
            if raw is None:
                sibling = Path(run["directory"])/(rid + {"csv":".csv", "performance":"_performance.md", "log":".log"}[key])
                path = sibling if sibling.exists() else None
            else:
                path = (base / raw).resolve()
            if path is None:
                continue
            run["supplements"].append(dict(kind=key, path=str(path), sha256=digest(path)))
            if key == "csv" and run["profile"] != "manual":
                continue
            value = parser(path)
            if key == "performance":
                run["metrics"].update(value)
            elif key == "csv":
                run["release"] = value
                if value["complete"]:
                    run["metrics"].update(release_target_p95_s=value["target_p95_s"], release_brake_p95_s=value["brake_p95_s"])
            else:
                run["lifecycle"] = value


def evidence_paths(paths, base):
    if not isinstance(paths, list) or not paths:
        return []
    return [dict(path=str((base/p).resolve()), sha256=digest(base/p)) for p in paths]


def contracts_for(run, supplement, base):
    fingerprint = run["metadata"].get("variant_sha256")
    suite = supplement.get("contracts", {}).get(fingerprint, {}) if fingerprint else {}
    failures, missing, evidence = [], [], []
    for name in CONTRACTS:
        check = suite.get(name, {})
        if finite(check.get("unsafe")) and check["unsafe"] != 0:
            failures.append(name+": unsafe responses")
        valid = check.get("passed") is True and isinstance(check.get("cases"), int) and not isinstance(check["cases"], bool) and check["cases"] > 0 and check.get("unsafe") == 0
        paths = evidence_paths(check.get("evidence"), base)
        if not valid or not paths:
            missing.append(name+": no complete same-freeze evidence")
        evidence.extend(paths)
    return failures, missing, evidence


def safety_check(run, supplement, base):
    failed, missing = [], []
    safety = run["safety"]
    if run["profile"] == "goal" and not safety["arrived"]:
        failed.append("goal did not ARRIVE")
    for field in ("sweep", "external", "exit_code"):
        value = safety[field]
        if not finite(value):
            missing.append(field+" audit missing/nonfinite")
        elif value != 0:
            failed.append(field+" is nonzero")
    if not finite(safety["clearance"]):
        missing.append("swept clearance audit missing/nonfinite")
    elif safety["clearance"] < 0:
        failed.append("negative swept clearance")
    if "lifecycle" not in run:
        missing.append("actual process log not audited")
    elif run["lifecycle"]["nonzero_child_exits"]:
        failed.append("nonzero child process exit, including shutdown")
    if run["profile"] == "manual":
        release = run.get("release", {})
        if not release.get("complete"):
            missing.append("no fully observed release episodes")
        elif release["target_max_s"] > POLICY["release_target_max_s"]+1e-9:
            failed.append("release target-zero exceeds 0.05 s")
    cf, cm, evidence = contracts_for(run, supplement, base)
    failed += cf
    missing += cm
    metadata = run["metadata"]
    for field in ("variant_sha256", "conditions_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(metadata.get(field, ""))):
            missing.append(field+" not SHA256")
    if metadata.get("freeze_verified") is not True:
        missing.append("freeze not independently verified")
    return dict(failed=failed, missing=missing, contract_evidence=evidence)


def descriptive(before, after):
    result = {}
    keys = sorted({(r["profile"], r["scenario"]) for r in list(before.values())+list(after.values())})
    for profile, scenario in keys:
        groups = [[r for r in runs.values() if (r["profile"], r["scenario"]) == (profile, scenario)] for runs in (before, after)]
        metrics = {}
        for name in REQUIRED[profile]:
            vals = [[r["metrics"].get(name) for r in group] for group in groups]
            valid = [[v for v in vs if finite(v)] for vs in vals]
            metrics[name] = dict(before_n=len(valid[0]), after_n=len(valid[1]),
                before_mean=statistics.mean(valid[0]) if valid[0] else None,
                after_mean=statistics.mean(valid[1]) if valid[1] else None,
                before_std=std(valid[0]), after_std=std(valid[1]),
                incomplete=any(len(v)!=len(g) for v,g in zip(valid,groups)))
        result[profile+":"+scenario] = dict(before_runs=len(groups[0]), after_runs=len(groups[1]), metrics=metrics)
    return result


def evaluate(before, after, plan, supplement, base=Path("."), stage="confirm"):
    if set(before) & set(after):
        raise ValueError("same run appears in both variants")
    allruns = {**before, **after}
    audit = {rid: safety_check(run, supplement, base) for rid, run in allruns.items()}
    failed = [rid+": "+e for rid,v in audit.items() for e in v["failed"]]
    missing = [rid+": "+e for rid,v in audit.items() for e in v["missing"]]
    report = dict(stage=stage, policy=POLICY, policy_sha256=POLICY_SHA256,
        descriptive=descriptive(before, after), safety_audit=audit, strata={},
        all_run_ids=list(allruns), failures=failed, insufficient=missing,
        interpretation="Safety/metadata contracts are caller-supplied attestations with hashed evidence, not proofs extracted from arbitrary documents. Only registered scenarios are covered; no general statistical guarantee or real-flight claim.")
    if plan is None:
        report["decision"] = "REJECT" if failed else "DESCRIPTIVE_ONLY"
        missing.append("no prespecified plan; no retention decision")
        return report
    guardrails_only = plan.get("scope") == "guardrails_only"
    report["scope"] = plan.get("scope", "candidate")
    if plan.get("scope", "candidate") not in ("candidate", "guardrails_only"):
        failed.append("invalid assessment scope")
    if plan.get("policy_sha256") != POLICY_SHA256:
        missing.append("plan policy_sha256 missing or differs from this policy")
    if not plan.get("registration_evidence"):
        missing.append("no externally recorded preregistration evidence")
    else:
        report["registration_evidence"] = evidence_paths(plan["registration_evidence"], base)
    pairs = plan.get("pairs", [])
    used, strata = set(), {}
    for pair in pairs:
        if stage == "explore" and pair.get("phase") == "confirm":
            continue
        b, a = pair.get("before"), pair.get("after")
        if b not in before or a not in after:
            missing.append(f"planned pair missing: {b}/{a}")
            continue
        if b in used or a in used:
            failed.append(f"run reused across pairs/phases: {b}/{a}")
            continue
        used.update((b,a))
        rb, ra = before[b], after[a]
        if rb["profile"] != ra["profile"] or rb["scenario"] != ra["scenario"]:
            failed.append(f"different paired scenarios: {b}/{a}")
            continue
        phase = pair.get("phase")
        if phase not in ("explore", "confirm"):
            failed.append(f"invalid phase: {phase}")
            continue
        key = rb["profile"]+":"+rb["scenario"]
        strata.setdefault(key, {"explore":[], "confirm":[]})[phase].append(pair)
        mb, ma = rb["metadata"], ra["metadata"]
        if mb.get("conditions_sha256") != ma.get("conditions_sha256"):
            failed.append(f"paired conditions differ: {b}/{a}")
        order = pair.get("order")
        if order not in (["before","after"], ["after","before"]):
            missing.append(f"missing registered pair order: {b}/{a}")
        else:
            seqb, seqa = mb.get("sequence"), ma.get("sequence")
            if not all(isinstance(x, int) and not isinstance(x, bool) for x in (seqb,seqa)) or seqb == seqa:
                missing.append(f"missing actual execution sequence: {b}/{a}")
            elif (seqb < seqa) != (order[0] == "before"):
                failed.append(f"execution differs from planned order: {b}/{a}")
    unused = sorted(set(allruns)-used)
    if unused:
        missing.append("unpaired runs retained; candidate cannot be selected while excluding them: "+",".join(unused))
    report["unpaired_run_ids"] = unused
    registered = plan.get("strata", {})
    if set(registered) != set(strata):
        missing.append("registered strata differ from observed paired strata")
    sequences = [r["metadata"].get("sequence") for r in allruns.values()]
    known_sequences = [x for x in sequences if isinstance(x, int) and not isinstance(x, bool)]
    if len(known_sequences) != len(set(known_sequences)):
        failed.append("actual sequence number reused by distinct runs")
    primary_count = sum(bool(v.get("primary")) for v in registered.values())
    if guardrails_only and primary_count != 0:
        missing.append("guardrails_only scope must not declare a primary benefit")
    elif not guardrails_only and primary_count != 1:
        missing.append("exactly one prespecified primary stratum/metric is required")
    primary_confirmed = False
    primary_promising = False
    confirmations_complete = True
    for key, phases in strata.items():
        profile = key.split(":",1)[0]
        rspec = registered.get(key, {})
        primary = rspec.get("primary")
        if primary is not None and primary not in REQUIRED[profile]:
            failed.append(key+": invalid primary metric")
        record = report["strata"][key] = {}
        fingerprints = {side:{allruns[p[side]]["metadata"].get("variant_sha256") for phase in phases.values() for p in phase} for side in ("before","after")}
        if any(len(v)!=1 for v in fingerprints.values()):
            failed.append(key+": candidate/baseline freeze changed between phases; restart registration")
        conditions = {allruns[p["before"]]["metadata"].get("conditions_sha256") for phase in phases.values() for p in phase}
        if len(conditions) != 1:
            failed.append(key+": conditions changed between repetitions")
        for phase, selected in phases.items():
            if stage == "explore" and phase == "confirm":
                continue
            threshold = 3 if phase == "explore" else 5
            out = record[phase] = dict(pairs=len(selected), metrics={})
            if len(selected) < threshold:
                if phase == "explore": missing.append(key+": fewer than 3 exploration pairs")
                else: confirmations_complete = False
            # Alternate AB/BA in the registered list and check actual ordering.
            orders = [p.get("order") for p in selected]
            if any(a == b for a,b in zip(orders,orders[1:])):
                missing.append(key+":"+phase+": pair order did not alternate AB/BA")
            blocks = [[allruns[p[side]]["metadata"].get("sequence") for side in ("before","after")] for p in selected]
            if all(isinstance(x,int) for block in blocks for x in block):
                if any(max(left) >= min(right) for left,right in zip(blocks,blocks[1:])):
                    failed.append(key+":"+phase+": paired blocks were not executed in registered order")
            for name in REQUIRED[profile]:
                values = [[allruns[p[side]]["metrics"].get(name) for p in selected] for side in ("before","after")]
                if not selected:
                    continue
                if any(not finite(v) or (name != "progress_m" and v < 0) for vs in values for v in vs):
                    missing.append(key+":"+phase+": missing/nonfinite/invalid metric "+name)
                    continue
                result = compare_metric(name, *values)
                out["metrics"][name] = result
                if result["noninferiority"] == "REGRESSION":
                    failed.append(key+":"+phase+": practical regression "+name)
                elif phase == "confirm" and len(selected) >= 5 and result["noninferiority"] != "NONINFERIOR":
                    missing.append(key+": noninferiority uncertain for "+name)
                if name == primary and len(selected) >= threshold and result["practical_improvement"]:
                    if phase == "explore" and result["improvement_pairs"] >= math.ceil(2*len(selected)/3):
                        primary_promising = True
                    if phase == "confirm" and result["primary_one_sided_sign_p"] <= .05:
                        primary_confirmed = True
            if profile == "goal" and len(selected) >= 2 and "arrival_s" in out["metrics"]:
                arr = out["metrics"]["arrival_s"]
                # Run-to-run std is descriptive, not a gate near zero variance.
                limit = max(1.0, .03 * arr["before_max"])
                out["arrival_worst_run_tolerance_s"] = limit
                if arr["after_max"]-arr["before_max"] > limit+1e-12:
                    failed.append(key+":"+phase+": worst arrival-time regression")
        # Confirmation must start after ALL exploratory pairs, with no reused IDs.
        seqe = [allruns[p[s]]["metadata"].get("sequence") for p in phases["explore"] for s in ("before","after")]
        seqc = [allruns[p[s]]["metadata"].get("sequence") for p in phases["confirm"] for s in ("before","after")]
        if seqe and seqc and all(isinstance(x,int) for x in seqe+seqc) and min(seqc)<=max(seqe):
            failed.append(key+": confirmation not independent subsequent runs")
    if failed:
        report["decision"] = "REJECT"
    elif missing:
        report["decision"] = "INSUFFICIENT"
    elif guardrails_only:
        if stage == "explore":
            report["decision"] = "GUARDRAILS_EXPLORE_OK"
        elif confirmations_complete:
            report["decision"] = "GUARDRAILS_CONFIRMED"
        else:
            report["decision"] = "INSUFFICIENT"
            missing.append("fewer than 5 independent confirmation pairs per registered stratum")
    elif not primary_promising:
        report["decision"] = "NO_PRACTICAL_IMPROVEMENT"
    elif stage == "explore":
        report["decision"] = "EXPLORE_PROMISING"
    elif not confirmations_complete:
        report["decision"] = "INSUFFICIENT"
        missing.append("fewer than 5 independent confirmation pairs per registered stratum")
    elif not primary_confirmed:
        report["decision"] = "INSUFFICIENT"
        missing.append("primary not confirmed by one-sided sign test at 0.05")
    else:
        report["decision"] = "CONFIRM_RETAIN"
    return report


def json_safe(value):
    """Retain invalid numeric evidence in standards-compliant JSON, not as NaN."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_numeric": repr(value)}
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", nargs="+", type=Path, required=True)
    parser.add_argument("--after", nargs="+", type=Path, required=True)
    parser.add_argument("--before-filter")
    parser.add_argument("--after-filter")
    parser.add_argument("--stage", choices=("explore", "confirm"), default="confirm")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--supplement", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists: choose a new path; prior results must be preserved")
    try:
        before, bp = load_analysis(args.before, args.before_filter)
        after, ap = load_analysis(args.after, args.after_filter)
        supplement = json.loads(args.supplement.read_text()) if args.supplement else {}
        # Evidence and registration paths share supplement's directory (or plan's).
        base = (args.supplement or args.plan or Path.cwd()).resolve()
        base = base.parent if (args.supplement or args.plan) else base
        plan = json.loads(args.plan.read_text()) if args.plan else None
        augment(before, supplement, base)
        augment(after, supplement, base)
        result = evaluate(before, after, plan, supplement, base, args.stage)
        result["input_files"] = bp+ap+[{"path":str(p.resolve()), "sha256":digest(p)} for p in (args.plan,args.supplement) if p]
        result["raw_supplement_evidence"] = {rid:{"metadata":run["metadata"], "files":run["supplements"], "release":run.get("release"), "lifecycle":run.get("lifecycle")} for rid,run in {**before,**after}.items()}
        result["selection_filters"] = dict(before=args.before_filter,after=args.after_filter)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as output:
            json.dump(json_safe(result), output, indent=2, allow_nan=False)
            output.write("\n")
        print(json.dumps(dict(decision=result["decision"], output=str(args.output), failures=len(result["failures"]), insufficient=len(result["insufficient"]))))
        return 0 if result["decision"] in ("CONFIRM_RETAIN","EXPLORE_PROMISING","DESCRIPTIVE_ONLY","GUARDRAILS_CONFIRMED","GUARDRAILS_EXPLORE_OK") else 2
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
