#!/usr/bin/env python3
"""Independent all-sample motion hard gates; never launch or modify a simulation."""
from __future__ import annotations
import argparse
import ast
import csv
import hashlib
import io
import json
import math
from pathlib import Path

TOLERANCE = 1e-6
LIMITS = {"actual_speed_mps": 2.0, "actual_vertical_speed_mps": 1.0,
          "actual_acceleration_mps2": 1.2, "applied_target_speed_mps": 2.0,
          "applied_target_vertical_speed_mps": 1.0}
COLUMNS = ("t", "x", "y", "z", "vx", "vy", "vz",
           "applied_target_x", "applied_target_y", "applied_target_z")
SOURCE_SUFFIXES = {"protocol": "performance/navigation_benchmark_20260929/protocol.json",
                   "simulator": "scripts/isaac/run_fov_gvf_navigation.py",
                   "motion_helper": "scripts/isaac/manual_control_math.py"}
DEFAULTS = {"ISAAC_KEYBOARD_SPEED": 2.0, "ISAAC_KEYBOARD_VERTICAL_SPEED": 1.0,
            "ISAAC_PLANT_ACCEL": 1.2, "ISAAC_PLANT_TAU": .22}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def new_metric(limit):
    return dict(limit=limit, floating_tolerance=TOLERANCE, allowed_maximum=limit+TOLERANCE,
                finite_values_checked=0, raw_max=None, raw_max_at_s=None,
                violation_count=0, first_violation=None, nonfinite_derived_count=0)


def observe(metric, value, t, row, interval_start=None):
    if not math.isfinite(value):
        metric["nonfinite_derived_count"] += 1
        metric["violation_count"] += 1
    else:
        metric["finite_values_checked"] += 1
        if metric["raw_max"] is None or value > metric["raw_max"]:
            metric["raw_max"], metric["raw_max_at_s"] = value, t
        if value <= metric["allowed_maximum"]:
            return
        metric["violation_count"] += 1
    if metric["first_violation"] is None:
        metric["first_violation"] = dict(row=row, t_s=t,
            value=value if math.isfinite(value) else str(value), interval_start_s=interval_start)


def audit_csv(path):
    path = Path(path)
    out = dict(path=str(path.resolve()), sha256=None, status="FAIL", row_count=0,
               integrity_errors=[], invalid_sample_count=0, invalid_examples=[],
               start_s=None, end_s=None, metrics={k: new_metric(v) for k, v in LIMITS.items()})
    try:
        data = path.read_bytes(); out["sha256"] = digest(data)
        reader = csv.DictReader(io.StringIO(data.decode("utf-8-sig")))
        headers = reader.fieldnames or []
        if len(headers) != len(set(headers)):
            out["integrity_errors"].append("duplicate CSV columns")
        missing = [k for k in COLUMNS if k not in headers]
        if missing:
            out["integrity_errors"].append("missing required columns: " + ", ".join(missing))
        if out["integrity_errors"]:
            return out
        previous = None
        previous_time = None
        for index, row in enumerate(reader, 1):
            out["row_count"] += 1
            try:
                if None in row or any(row[k] is None or row[k] == "" for k in COLUMNS):
                    raise ValueError("missing or extra fields")
                values = [float(row[k]) for k in COLUMNS]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("nonfinite required motion field")
            except (TypeError, ValueError, OverflowError) as error:
                out["invalid_sample_count"] += 1
                if len(out["invalid_examples"]) < 20:
                    out["invalid_examples"].append(dict(row=index, message=str(error)))
                previous = None  # Never bridge an invalid sample to dilute an acceleration.
                continue
            t = values[0]
            if out["start_s"] is None:
                out["start_s"] = t
            out["end_s"] = t
            monotonic = previous_time is None or t > previous_time
            if not monotonic:
                out["integrity_errors"].append(f"non-increasing time at row {index}")
            previous_time = t
            v, target = values[4:7], values[7:10]
            observe(out["metrics"]["actual_speed_mps"], math.hypot(*v), t, index)
            observe(out["metrics"]["actual_vertical_speed_mps"], abs(v[2]), t, index)
            observe(out["metrics"]["applied_target_speed_mps"], math.hypot(*target), t, index)
            observe(out["metrics"]["applied_target_vertical_speed_mps"], abs(target[2]), t, index)
            if previous is not None and monotonic:
                old_t, old_v = previous
                dt = t - old_t
                if not math.isfinite(dt):
                    out["integrity_errors"].append(f"nonfinite time interval at row {index}")
                else:
                    acceleration = math.hypot(*((b-a)/dt for a, b in zip(old_v, v)))
                    observe(out["metrics"]["actual_acceleration_mps2"], acceleration, t, index, old_t)
            previous = (t, v)
    except (OSError, UnicodeError, csv.Error, OverflowError) as error:
        out["integrity_errors"].append(type(error).__name__ + ": " + str(error))
    if out["row_count"] < 2:
        out["integrity_errors"].append("need at least two samples")
    if out["invalid_sample_count"]:
        out["integrity_errors"].append("one or more required motion samples invalid")
    if not out["integrity_errors"] and not any(m["violation_count"] for m in out["metrics"].values()):
        out["status"] = "PASS"
    return out


def provenance(manifest):
    """Bind physical constants to driver-frozen source files; no imports/evaluation."""
    errors, sources, contents = [], {}, {}
    frozen = manifest.get("file_sha256", {})
    if not isinstance(frozen, dict) or not all(isinstance(p, str) for p in frozen):
        frozen = {}
        errors.append("invalid frozen source hash mapping")
    for name, suffix in SOURCE_SUFFIXES.items():
        paths = [p for p in frozen if p.endswith("/" + suffix)]
        if len(paths) != 1:
            errors.append("need exactly one frozen " + name); continue
        path = Path(paths[0])
        try:
            data = path.read_bytes(); actual = digest(data)
            sources[name] = dict(path=str(path), expected_sha256=frozen[str(path)], actual_sha256=actual,
                                 matches=actual == frozen[str(path)], project_relative_path=suffix)
            if actual != frozen[str(path)]:
                errors.append("frozen source changed: " + name)
            contents[name] = data
        except OSError as error:
            errors.append("missing source " + name + ": " + str(error))
    protocol = {}
    if "protocol" in contents:
        try:
            parsed = json.loads(contents["protocol"])
            if not isinstance(parsed, dict):
                raise ValueError("protocol must be an object")
            for key, expected in {"max_speed_mps": 2., "max_vertical_speed_mps": 1.,
                                  "plant_accel_mps2": 1.2, "plant_tau_s": .22}.items():
                value = parsed.get(key)
                protocol[key] = value if number(value) else repr(value)
                if not number(value) or value != expected:
                    errors.append("protocol constant differs: " + key)
        except (ValueError, TypeError) as error:
            errors.append("invalid protocol: " + str(error))
    defaults = {}
    if "simulator" in contents:
        try:
            tree = ast.parse(contents["simulator"])
            for node in ast.walk(tree):
                if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                        and node.func.id == "scalar_from_env" and len(node.args) >= 2
                        and isinstance(node.args[0], ast.Constant) and isinstance(node.args[1], ast.Constant)):
                    key, value = node.args[0].value, node.args[1].value
                    if key in DEFAULTS:
                        if key in defaults:
                            errors.append("duplicate simulator default: " + key)
                        defaults[key] = value if number(value) else repr(value)
            for key, expected in DEFAULTS.items():
                if not number(defaults.get(key)) or defaults[key] != expected:
                    errors.append("simulator default differs: " + key)
        except (SyntaxError, ValueError) as error:
            errors.append("invalid simulator source: " + str(error))
    return dict(status="FAIL" if errors else "PASS", errors=errors, sources=sources,
                protocol=protocol, simulator_defaults=defaults,
                scope="Hashes bind files to the frozen driver manifest. Manual runtime does not directly attest simulator/common-source execution; frozen files plus driver identity checks are the available evidence.")


def run_identity(meta, proof, manifest):
    errors = []
    env = meta.get("experiment_environment", {})
    for key, expected in DEFAULTS.items():
        if key in env:
            try:
                actual = float(env[key])
                if not math.isfinite(actual) or actual != expected:
                    errors.append("physics override differs: " + key)
            except (ValueError, TypeError):
                errors.append("invalid physics override: " + key)
    for key in ("runtime_binary_verified", "runtime_launch_verified", "runtime_common_sources_verified"):
        if meta.get(key) is not True:
            errors.append("driver identity evidence not verified: " + key)
    expected_sha = manifest.get("binary_sha256", {}).get(meta.get("variant"))
    if not expected_sha or meta.get("controller_sha256") != expected_sha:
        errors.append("run controller differs from frozen manifest variant")
    expected_simulator = proof.get("sources", {}).get("simulator", {}).get("expected_sha256")
    if not expected_simulator or meta.get("shared_simulator_sha256") != expected_simulator:
        errors.append("run simulator SHA differs from frozen source")
    runtime_path = (Path(meta["result_path"]).with_suffix(".parameters.json") if meta.get("result_path")
                    else Path(env.get("FOV_GVF_RUNTIME_MANIFEST", "/missing_runtime")))
    runtime_sha = None
    try:
        data = runtime_path.read_bytes(); runtime_sha = digest(data); runtime = json.loads(data)
        if not isinstance(runtime, dict):
            raise ValueError("runtime manifest must be an object")
        if not meta.get("controller_executable") or runtime.get("executable") != meta["controller_executable"]:
            errors.append("runtime/process controller path differs or is missing")
        if (runtime.get("executable_sha256") != meta.get("controller_sha256")
                or not meta.get("controller_sha256")):
            errors.append("runtime/process controller SHA differs or is missing")
        if runtime.get("launch_sha256") != meta.get("launch_sha256") or not meta.get("launch_sha256"):
            errors.append("runtime/process launch SHA differs or is missing")
        if meta.get("case") == "goal":
            if env.get("ISAAC_GOAL_PROTOCOL") != proof.get("sources", {}).get("protocol", {}).get("path"):
                errors.append("actual goal protocol path differs")
            common = runtime.get("common_source_sha256", {})
            for source in proof.get("sources", {}).values():
                if common.get(source["project_relative_path"]) != source["expected_sha256"]:
                    errors.append("actual goal common-source SHA differs: " + source["project_relative_path"])
        elif not runtime.get("executable"):
            errors.append("manual runtime executable missing")
    except (OSError, ValueError, TypeError) as error:
        errors.append("missing/invalid runtime manifest: " + str(error))
    return dict(status="FAIL" if errors else "PASS", errors=errors,
                runtime_manifest=str(runtime_path), runtime_manifest_sha256=runtime_sha)


def audit_manifests(paths):
    result = dict(schema_version=1, status="FAIL", thresholds=LIMITS, floating_tolerance=TOLERANCE,
        scope="All rows and adjacent valid intervals in every supplied manifest CSV, including start/release/stop. No active-only mask, no averaging across runs, no bound on target slew or jerk. Required finite fields are t, position, actual velocity and applied_target; optional diagnostic fields are excluded.",
        manifests=[], runs=[], errors=[], expected_run_count=0, audited_run_count=0)
    seen_manifests, seen_runs, seen_csv = set(), set(), set()
    for item in paths:
        path = Path(item).resolve()
        if path in seen_manifests:
            result["errors"].append("duplicate manifest: " + str(path)); continue
        seen_manifests.add(path)
        try:
            data = path.read_bytes(); manifest = json.loads(data)
            entry = dict(path=str(path), sha256=digest(data), errors=[], provenance=provenance(manifest))
            result["manifests"].append(entry)
            if manifest.get("complete") is not True:
                entry["errors"].append("manifest is incomplete")
            expected = [(i+1, variant) for i, order in enumerate(manifest.get("order_by_repeat", [])) for variant in order]
            actual = [(row.get("repeat"), row.get("variant")) for row in manifest.get("runs", [])]
            result["expected_run_count"] += len(expected)
            if not expected or actual != expected:
                entry["errors"].append("planned run inventory/order missing or differs")
            for meta in manifest.get("runs", []):
                rid = meta.get("experiment_environment", {}).get("FOV_GVF_RUN_ID")
                csv_path = Path(meta.get("csv", "/missing_csv")).resolve()
                problems = []
                if not rid or rid in seen_runs:
                    problems.append("missing/duplicate run ID")
                if csv_path in seen_csv:
                    problems.append("duplicate CSV shared by runs")
                seen_runs.add(rid); seen_csv.add(csv_path)
                identity = run_identity(meta, entry["provenance"], manifest)
                motion = audit_csv(csv_path)
                status = ("PASS" if not problems and identity["status"] == "PASS" and motion["status"] == "PASS"
                          and entry["provenance"]["status"] == "PASS" and not entry["errors"] else "FAIL")
                result["runs"].append(dict(id=rid, variant=meta.get("variant"), case=meta.get("case"), repeat=meta.get("repeat"),
                    manifest=str(path), status=status, errors=problems, identity=identity, motion=motion))
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
            result["errors"].append("invalid manifest " + str(path) + ": " + str(error))
    result["audited_run_count"] = len(result["runs"])
    result["failed_run_count"] = sum(r["status"] != "PASS" for r in result["runs"])
    if (result["runs"] and result["expected_run_count"] == result["audited_run_count"] and not result["errors"]
            and not result["failed_run_count"] and all(not m["errors"] and m["provenance"]["status"] == "PASS" for m in result["manifests"])):
        result["status"] = "PASS"
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, nargs="+", action="extend", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("refusing to overwrite existing output")
    result = audit_manifests(args.manifest)
    result["tool"] = dict(path=str(Path(__file__).resolve()), sha256=digest(Path(__file__).read_bytes()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({k: result[k] for k in ("status", "expected_run_count", "audited_run_count", "failed_run_count")}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
