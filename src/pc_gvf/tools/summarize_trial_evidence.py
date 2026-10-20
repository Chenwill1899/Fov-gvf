#!/usr/bin/env python3
"""Summarize only this P1 campaign and its two explicitly named P1 baselines.

Reads evidence without changing it. Missing values remain null/blank. A formal
trial has an explicitly recorded zero exploration-stall threshold; repetitions
are counted from actual trial records, never protocol.final_repeats.
"""
import argparse
import collections
import csv
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


HERE = Path(__file__).resolve().parents[3] / "docs/performance/p6_speed_20261019"
BASELINE = HERE / "baseline"
VOLATILE_SETTINGS = {"performance_log_path", "performance_run_id",
                     "paper_replay_directory", "paper_replay_dir"}
ENVIRONMENT_LOCATIONS = {"FOV_GVF_INSTALL", "FOV_GVF_CONTROLLER_EXECUTABLE",
                        "FOV_GVF_RUN_ID", "FOV_GVF_PERFORMANCE_LOG", "FOV_GVF_REPLAY_DIR",
                        "ISAAC_BENCHMARK_RESULT", "ISAAC_GOAL_PROTOCOL", "ISAAC_ACCEPTANCE_TRACE",
                        "ISAAC_EXPLORATION_STALL_S"}
SUFFIXES = (".audit.json", ".parameters.json", ".manifest.json", ".protocol.json")
NUMBER = r"([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def evidence_path(stem, suffix):
    # A trial name may contain a decimal speed (v2.5): with_suffix() would
    # incorrectly discard the remainder of that stem.
    return Path(str(stem) + suffix)


def load(path, errors):
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text())
        if not isinstance(value, dict):
            raise ValueError("expected JSON object")
        return value
    except (OSError, ValueError) as error:
        errors.append({"path": str(path), "error": str(error)})
        return {}


def first(*values):
    return next((value for value in values if value is not None), None)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def value_or_none(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: value_or_none(item) for key, item in value.items()}
    if isinstance(value, list):
        return [value_or_none(item) for item in value]
    return value


def parse_performance(result, parameters, stem):
    """Prefer the precise per-run Markdown section; use its own log as fallback."""
    effective = parameters.get("effective", {})
    run_id = first(result.get("run_id"), effective.get("performance_run_id"))
    performance_path = Path(effective.get("performance_log_path", stem.parent / "PERFORMANCE.md"))
    section = ""
    if run_id and performance_path.is_file():
        raw = performance_path.read_text(errors="replace")
        pattern = r"(?m)^### Run " + re.escape(run_id) + r"\s*$([\s\S]*?)(?=^### Run |\Z)"
        matches = list(re.finditer(pattern, raw))
        if matches:
            section = matches[-1].group(1)
    output = {}
    patterns = {
        "isaac_sim_fps": (r"Runtime FPS \(simulation time / wall time\): " + NUMBER + r" / " + NUMBER, 1),
        "isaac_wall_fps": (r"Runtime FPS \(simulation time / wall time\): " + NUMBER + r" / " + NUMBER, 2),
        "controller_sim_fps": (r"(?m)^- Command FPS \(ROS simulation time / active wall time\): " + NUMBER + r" / " + NUMBER, 1),
        "controller_wall_fps": (r"(?m)^- Command FPS \(ROS simulation time / active wall time\): " + NUMBER + r" / " + NUMBER, 2),
        "guidance_wall_fps": (r"Guidance FPS \(active wall time\): " + NUMBER, 1),
        "bridge_wall_fps": (r"Published control FPS \(ROS simulation time / active wall time\): " + NUMBER + r" / " + NUMBER, 2),
        "frame_interval_mean_ms": (r"Isaac frame interval mean / P50 / P95 / max: " + NUMBER + r" / " + NUMBER + r" / " + NUMBER + r" / " + NUMBER, 1),
        "frame_interval_p95_ms": (r"Isaac frame interval mean / P50 / P95 / max: " + NUMBER + r" / " + NUMBER + r" / " + NUMBER + r" / " + NUMBER, 3),
        "frame_interval_max_ms": (r"Isaac frame interval mean / P50 / P95 / max: " + NUMBER + r" / " + NUMBER + r" / " + NUMBER + r" / " + NUMBER, 4),
    }
    for key, (pattern, group) in patterns.items():
        match = re.search(pattern, section)
        if match:
            output[key] = float(match.group(group))
    log = evidence_path(stem, ".log")
    if log.is_file():
        lines = [line for line in log.read_text(errors="replace").splitlines()
                 if line.startswith("[PERF ISAAC FINAL]") and
                 (not run_id or "run=" + run_id + " " in line)]
        if lines:
            line = lines[-1]
            match = re.search(r"fps\(sim/wall\)=" + NUMBER + "/" + NUMBER, line)
            if match:
                output.setdefault("isaac_sim_fps", float(match.group(1)))
                output.setdefault("isaac_wall_fps", float(match.group(2)))
            match = re.search(r"frame_interval_ms\(mean/p50/p95/max\)=" +
                              "/".join([NUMBER] * 4), line)
            if match:
                for key, index in (("frame_interval_mean_ms", 1),
                                   ("frame_interval_p95_ms", 3),
                                   ("frame_interval_max_ms", 4)):
                    output.setdefault(key, float(match.group(index)))
    return output


def identify_map(stem, audit, protocol):
    if stem.name.startswith("original_"):
        return "original"
    if stem.name.startswith("long_"):
        return "long"
    scene = audit.get("scene", "")
    if "speed_cloud_long" in scene:
        return "long"
    if "ego_swarm_cloud" in scene:
        return "original"
    goal = protocol.get("goal", [])
    if goal == [64.0, 11.2, 2.8]:
        return "original"
    if goal == [384.0, 11.2, 2.8]:
        return "long"
    return None


def has_navigation_trace(path):
    """Require two actual recorded samples, not merely an empty/header file."""
    if not path.is_file():
        return False
    try:
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            first_row, second_row = next(reader, None), next(reader, None)
            return bool(first_row and second_row and
                        math.isfinite(float(first_row["t"])) and
                        math.isfinite(float(second_row["t"])) and
                        float(second_row["t"]) > float(first_row["t"]))
    except (OSError, KeyError, TypeError, ValueError):
        return False


def read_trial(stem, errors):
    paths = {name: evidence_path(stem, suffix) for name, suffix in
             (("result", ".json"), ("audit", ".audit.json"),
              ("parameters", ".parameters.json"), ("manifest", ".manifest.json"),
              ("protocol", ".protocol.json"), ("sensors", ".sensors.json"))}
    data = {name: load(path, errors) for name, path in paths.items()}
    result, audit, parameters, manifest = (data[key] for key in
                                          ("result", "audit", "parameters", "manifest"))
    protocol = first(result.get("protocol"), data["protocol"]) or {}
    # Version can select audit semantics, so retain it. Only the planned repeat
    # count is excluded; arrival/timeout/warmup/deadband are all part of identity.
    protocol_semantics = {key: value for key, value in protocol.items() if key != "final_repeats"}
    protocol_sha = digest(protocol_semantics) if protocol else None
    effective = parameters.get("effective", {})
    stable = {key: value for key, value in effective.items() if key not in VOLATILE_SETTINGS}
    settings_sha = digest(stable) if effective else None
    executable_sha = first(parameters.get("executable_sha256"), manifest.get("actual_executable_sha256"))
    # Script/bridge changes alter execution semantics even when controller bytes
    # are unchanged. Do not merge those runs merely because their C++ SHA matches.
    execution_sources = {key: value for key, value in parameters.get("common_source_sha256", {}).items()
                         if key != "goal_protocol"}
    execution_sources_sha = digest(execution_sources) if execution_sources else None
    # Effective settings capture launch parameters, including newly added ones.
    # Runtime environment also matters for sample selection and timing. Exclude
    # evidence locations and the exploration cutoff, which defines phase rather
    # than controller configuration; keep every other recorded environment key.
    environment_identity = {
        "effective_runtime": parameters.get("runtime_environment", {}),
        "recorded_requested": {key: value for key, value in manifest.get("requested_environment", {}).items()
                               if key not in ENVIRONMENT_LOCATIONS},
    }
    environment_sha = digest(environment_identity) if any(environment_identity.values()) else None
    sensors = data["sensors"]
    offset_required = (protocol.get("version") == "ego2p1_sim4_v6" or
                       "depth_sensor_camera_offset_m" in protocol)
    sensor_required = (protocol.get("version") == "ego2p1_sim4_v5" or
                       "depth_sensor_hfov_deg" in protocol or offset_required)
    sensor_profile_sha = None
    sensor_integrity = False if sensor_required else None
    sensor_file_sha = None
    if sensors:
        try:
            fields = ('name', 'prim_path', 'depth_topic', 'camera_info_topic', 'frame_id',
                'width', 'height', 'focal_length', 'horizontal_aperture', 'vertical_aperture',
                'hfov_deg', 'vfov_deg', 'fx', 'fy', 'cx', 'cy', 'projection', 'lens_distortion_model',
                'horizontal_aperture_offset', 'vertical_aperture_offset')
            if sensors['version'] == 'ego2p1_runtime_sensors_v2':
                fields += ('mount_path', 'mount_translation_robot_m',
                    'camera_position_robot_m', 'robot_from_optical_rotation', 'camera_local_matrix4x4')
            profile = dict(version=sensors['version'], render_width=sensors['render_width'],
                render_height=sensors['render_height'], cameras=[{key: camera[key] for key in fields}
                    for camera in sorted(sensors['cameras'], key=lambda item: item['prim_path'])])
            sensor_profile_sha = digest(profile)
            sensor_file_sha = hashlib.sha256(paths['sensors'].read_bytes()).hexdigest()
            frozen_sensor = stem.parent / manifest.get('sensors_blob', 'missing_sensor_blob')
            required_checks = ['sensor_record_valid', 'sensor_request_matches', 'sensor_frozen_matches']
            if offset_required:
                required_checks += ['camera_offset_request_matches', 'camera_extrinsics_valid',
                                    'camera_offset_parameter_matches']
            sensor_integrity = bool((not offset_required or sensors['version'] == 'ego2p1_runtime_sensors_v2') and
                sensor_file_sha == manifest.get('sensors_file_sha256') and
                sensor_profile_sha == manifest.get('sensor_profile_sha256') and
                frozen_sensor.is_file() and hashlib.sha256(frozen_sensor.read_bytes()).hexdigest() == sensor_file_sha and
                audit.get('sensor_profile_sha256') == sensor_profile_sha and
                all(audit.get('additional_checks', {}).get(key) is True for key in required_checks))
        except (OSError, KeyError, TypeError, ValueError) as error:
            errors.append({'path': str(paths['sensors']), 'error': str(error)})
            sensor_integrity = False
    settings_match = (settings_sha == manifest["controller_settings_sha256"]
                      if manifest.get("controller_settings_sha256") and settings_sha else None)
    actual_matches = (executable_sha == manifest["actual_executable_sha256"]
                      if manifest.get("actual_executable_sha256") and executable_sha else None)
    expected_matches = (executable_sha == manifest["expected_executable_sha256"]
                        if manifest.get("expected_executable_sha256") and executable_sha else None)
    integrity_ok = bool(executable_sha and settings_sha and
                        (sensor_integrity if sensor_required or sensors else True) and protocol_sha and
                        settings_match is not False and actual_matches is not False and
                        expected_matches is not False and
                        not manifest.get("changed_files_during_run") and
                        (not manifest or (manifest.get("complete") is True and
                                          manifest.get("execution_matches_frozen") is True)))
    abort = first(result.get("exploration_stall_s"), audit.get("exploration_stall_s"))
    if abort is None:
        recorded = manifest.get("requested_environment", {}).get("ISAAC_EXPLORATION_STALL_S")
        try:
            abort = float(recorded) if recorded is not None else None
        except (TypeError, ValueError):
            pass
    stage = "formal" if abort == 0 else "exploration" if finite(abort) and abort > 0 else "unknown"
    status = first(result.get("status"), audit.get("status"))
    terminal = bool(result and status and status not in ("RUNNING", "PENDING", "STARTING"))
    # A manifest/parameter-only record is shown as pending, never as a failed or
    # successful completed flight; an actual startup error is a retained failure.
    if not status:
        status = "PENDING_OR_INCOMPLETE"
    flew = has_navigation_trace(evidence_path(stem, ".csv"))
    if not terminal:
        classification = "pending_or_incomplete"
    elif not flew:
        classification = "startup_failure_no_navigation_trace"
    elif audit.get("accepted") is True:
        classification = "safe_arrival_accepted"
    elif audit.get("safe_arrival") is True:
        classification = "safe_arrival_cap_not_attained"
    elif status == "ARRIVED" and audit.get("safe_arrival") is False:
        classification = "unsafe_arrival"
    elif status == "ARRIVED":
        classification = "arrival_without_complete_audit"
    elif status == "ABORTED_EXPLORATION_STALL":
        classification = "exploration_stall"
    elif status == "TIMEOUT":
        classification = "completed_timeout"
    elif status == "COLLISION_GUARD":
        classification = "collision_guard"
    elif status == "RUNTIME_ERROR":
        classification = "runtime_error_with_navigation_trace"
    else:
        classification = "terminated_not_accepted"
    reported_status = ("UNSAFE_ARRIVAL" if classification == "unsafe_arrival" else
                       "ARRIVED_UNAUDITED" if classification == "arrival_without_complete_audit" else status)
    limits = audit.get("limits_violations", {})
    limit_values = [limits.get(key) for key in ("speed", "vertical", "acceleration", "applied_command")]
    total_limits = sum(limit_values) if all(finite(value) for value in limit_values) else None
    performance = audit.get("performance", {})
    row = {
        "trial": stem.name, "batch": stem.parent.name, "map": identify_map(stem, audit, protocol),
        "phase": stage, "status": status, "terminal": terminal,
        "has_navigation_trace": flew, "classification": classification, "reported_status": reported_status,
        "safe_arrival": audit.get("safe_arrival"), "cap_attained": audit.get("cap_attained"),
        "accepted": audit.get("accepted"), "audit_present": bool(audit),
        "identity_complete_and_consistent": integrity_ok,
        "speed_mps": first(protocol.get("max_speed_mps"), audit.get("requested_speed_mps")),
        "accel_mps2": first(protocol.get("plant_accel_mps2"), audit.get("requested_accel_mps2")),
        "tau_s": first(protocol.get("plant_tau_s"), audit.get("plant_tau_s")),
        "actual_speed_limit_mps": effective.get("max_speed"),
        "actual_accel_limit_mps2": effective.get("command_accel_limit"),
        "actual_brake_limit_mps2": effective.get("brake_accel_limit"),
        "actual_tau_s": effective.get("velocity_response_time"),
        "vertical_speed_mps": first(protocol.get("max_vertical_speed_mps"), effective.get("max_vertical_speed")),
        "horizon_s": first(protocol.get("planning_horizon_s"), effective.get("planning_horizon")),
        "pipeline_delay_s": first(protocol.get("pipeline_delay_s"), effective.get("paper_pipeline_delay")),
        "restart_clearance_m": first(protocol.get("restart_clearance_m"), effective.get("paper_restart_clearance")),
        "direction_rate_radps": effective.get("max_direction_rate"),
        "max_depth_m": first(protocol.get("max_depth_m"), effective.get("max_depth")),
        "smooth_speed": effective.get("paper_smooth_speed"),
        "emergency_certificate": effective.get("paper_emergency_braking_certificate"),
        "history_interval_s": effective.get("paper_history_sample_interval"),
        "history_max_observations": effective.get("paper_history_max_observations"),
        "clearance_reward": effective.get("safe_goal_clearance_reward"),
        "free_space_floor_enabled": effective.get("paper_free_space_floor_enabled"),
        "free_space_floor_z_m": effective.get("paper_free_space_floor_z"),
        "exploration_stall_s": abort,
        "sim_s": first(audit.get("sim_s"), result.get("simulation_elapsed_s")),
        "flight_wall_s": first(audit.get("wall_s"), result.get("wall_elapsed_s")),
        "process_wall_s": result.get("process_wall_s"),
        "path_m": audit.get("path_m"),
        "route_average_mps": first(audit.get("actual_route_average_mps"), audit.get("route_average_mps")),
        "peak_mps": audit.get("actual_peak_mps"), "p95_mps": audit.get("actual_p95_mps"),
        "time_at_95pct_cap_s": audit.get("time_at_95pct_cap_s"),
        "longest_at_95pct_cap_s": audit.get("longest_at_95pct_cap_s"),
        "time_ge_3p8_s": audit.get("time_ge_3p8_s"), "longest_ge_3p8_s": audit.get("longest_ge_3p8_s"),
        "minimum_swept_clearance_m": audit.get("minimum_swept_clearance_m"),
        "swept_overlap_segments": audit.get("swept_overlap_segments"),
        "collision_blocks": first(audit.get("collision_blocks"), result.get("collision_blocks")),
        "limit_violations_total": total_limits,
        "limit_speed_count": limits.get("speed"), "limit_vertical_count": limits.get("vertical"),
        "limit_acceleration_count": limits.get("acceleration"), "limit_applied_command_count": limits.get("applied_command"),
        "stopped_s": audit.get("stopped_s"), "longest_stop_s": audit.get("longest_stop_s"),
        "acceleration_max_mps2": audit.get("acceleration_max_mps2"), "jerk_rms_mps3": audit.get("jerk_rms_mps3"),
        "callback_mean_ms": performance.get("callback_mean_ms"),
        "callback_p95_ms": performance.get("callback_p95_ms"), "callback_max_ms": performance.get("callback_max_ms"),
        "avoid_mean_ms": performance.get("avoid_mean_ms"), "avoid_p95_ms": performance.get("avoid_p95_ms"),
        "avoid_max_ms": performance.get("avoid_max_ms"),
        "final_distance_m": first(audit.get("actual_final_distance_m"), result.get("final_distance_m")),
        "executable_sha256": executable_sha, "controller_settings_sha256": settings_sha,
        "protocol_semantics_sha256": protocol_sha,
        "manifest_settings_match": settings_match,
        "execution_sources_sha256": execution_sources_sha,
        "runtime_environment_sha256": environment_sha,
        "runtime_environment_identity": environment_identity,
        "sensor_profile_sha256": sensor_profile_sha,
        "sensors_file_sha256": sensor_file_sha,
        "sensor_evidence_required": sensor_required,
        "sensor_identity_consistent": sensor_integrity,
        "requested_depth_hfov_deg": protocol.get("depth_sensor_hfov_deg"),
        "requested_depth_hfov_field_present": "depth_sensor_hfov_deg" in protocol,
        "requested_depth_hfov_env": manifest.get("requested_environment", {}).get("ISAAC_DEPTH_HFOV_DEG"),
        "actual_depth_hfov_deg": [camera.get("hfov_deg") for camera in sensors["cameras"]] if sensors.get("cameras") else None,
        "actual_depth_vfov_deg": [camera.get("vfov_deg") for camera in sensors["cameras"]] if sensors.get("cameras") else None,
        "sensor_evidence_source": sensors.get("evidence_source"),
        "sensor_record_version": sensors.get("version"),
        "camera_extrinsics_required": offset_required,
        "requested_camera_offset_m": protocol.get("depth_sensor_camera_offset_m"),
        "requested_camera_offset_field_present": "depth_sensor_camera_offset_m" in protocol,
        "requested_camera_offset_env": manifest.get("requested_environment", {}).get("ISAAC_DEPTH_CAMERA_OFFSET_M"),
        "effective_camera_offset_m": effective.get("camera_offset"),
        "actual_camera_positions_robot_m": {c.get('prim_path'): c.get('camera_position_robot_m') for c in sensors['cameras']} if sensors.get('cameras') and sensors.get('version') == 'ego2p1_runtime_sensors_v2' else None,
        "actual_robot_from_optical_rotations": {c.get('prim_path'): c.get('robot_from_optical_rotation') for c in sensors['cameras']} if sensors.get('cameras') and sensors.get('version') == 'ego2p1_runtime_sensors_v2' else None,
        "camera_extrinsics_valid": audit.get('additional_checks', {}).get('camera_extrinsics_valid'),
        "source_tree_sha256": manifest.get("source_tree_sha256"),
        "scene_sha256": audit.get("scene_sha256"), "occupancy_sha256": audit.get("occupancy_sha256"),
        "manifest_complete": manifest.get("complete"),
        "execution_matches_frozen": manifest.get("execution_matches_frozen"),
        "changed_files_during_run": manifest.get("changed_files_during_run"),
        "rejection_reasons": audit.get("rejection_reasons"), "audit_error": audit.get("audit_error"),
        "status_seconds": audit.get("status_seconds"),
        **{name + "_path": str(path) if path.is_file() else None for name, path in paths.items()},
        "trace_path": str(evidence_path(stem, ".csv")) if evidence_path(stem, ".csv").is_file() else None,
        "log_path": str(evidence_path(stem, ".log")) if evidence_path(stem, ".log").is_file() else None,
    }
    for key in ("isaac_sim_fps", "isaac_wall_fps", "controller_sim_fps", "controller_wall_fps",
                "guidance_wall_fps", "bridge_wall_fps", "frame_interval_mean_ms",
                "frame_interval_p95_ms", "frame_interval_max_ms"):
        row[key] = None
    row.update(parse_performance(result, parameters, stem))
    identity = {key: row[key] for key in ("executable_sha256", "controller_settings_sha256",
                "protocol_semantics_sha256", "execution_sources_sha256", "runtime_environment_sha256", "sensor_profile_sha256", "map", "speed_mps", "accel_mps2", "tau_s")}
    if not executable_sha or not settings_sha or not protocol_sha:
        identity["missing_identity_unique_trial"] = str(stem)
    row["group_id"] = digest(identity)
    # A separate cross-map family may differ only in goal coordinates and map
    # evidence paths. It never replaces the stricter per-map repetition group.
    cross_map_stable = {key: value for key, value in stable.items()
                        if key not in {"goal_x", "goal_y", "goal_z", "paper_seed_scene", "paper_seed_occupancy"}}
    cross_map_environment = {
        "effective_runtime": environment_identity["effective_runtime"],
        "recorded_requested": {key: value for key, value in environment_identity["recorded_requested"].items()
                               if key not in {"FOV_GVF_SCENE", "FOV_GVF_ESDF_OCCUPANCY", "FOV_GVF_ACTUAL_SCENE"}},
    }
    row["cross_map_settings_sha256"] = digest(cross_map_stable) if stable else None
    family = {"executable_sha256": executable_sha, "settings": cross_map_stable,
              "execution_sources_sha256": execution_sources_sha, "environment": cross_map_environment,
              "sensor_profile_sha256": sensor_profile_sha,
              "protocol": {key: value for key, value in protocol_semantics.items() if key not in {"goal", "timeout_s"}}}
    if not executable_sha or not stable:
        family["missing_identity_unique_trial"] = str(stem)
    row["cross_map_family_id"] = digest(family)
    return value_or_none(row)


def aggregate(rows):
    grouped = collections.defaultdict(list)
    for row in rows:
        grouped[row["group_id"]].append(row)
    groups = []
    for group_id, trials in sorted(grouped.items()):
        first_row = trials[0]
        formal = [row for row in trials if row["phase"] == "formal"]
        formal_finished = [row for row in formal if row["terminal"]]
        formal_accepted = [row for row in formal_finished if row["accepted"] is True]
        geometry_values = {key: sorted({row[key] for row in trials if row[key]})
                           for key in ("scene_sha256", "occupancy_sha256")}
        geometry_consistent = all(len(values) <= 1 for values in geometry_values.values())
        geometry_complete = all(row["scene_sha256"] and row["occupancy_sha256"]
                                for row in formal_finished)
        formal_pass = bool(len(formal_finished) >= 3 and len(formal) == len(formal_finished)
                           and len(formal_accepted) == len(formal_finished)
                           and geometry_consistent and geometry_complete
                           and all(row["has_navigation_trace"] for row in formal_finished)
                           and all(row["identity_complete_and_consistent"] for row in formal_finished))
        successes = [row for row in trials if row["safe_arrival"] is True and finite(row["sim_s"])]
        accepted = [row for row in trials if row["accepted"] is True]
        formal_times = [row["sim_s"] for row in formal_accepted if finite(row["sim_s"])]
        groups.append({
            "group_id": group_id,
            **{key: first_row[key] for key in ("map", "speed_mps", "accel_mps2", "tau_s",
              "executable_sha256", "controller_settings_sha256", "protocol_semantics_sha256", "execution_sources_sha256", "runtime_environment_sha256", "sensor_profile_sha256")},
            "source_tree_sha256_values": sorted({row["source_tree_sha256"] for row in trials if row["source_tree_sha256"]}),
            "geometry_sha256_values": geometry_values, "geometry_consistent": geometry_consistent,
            "formal_geometry_identity_complete": bool(geometry_complete),
            "total_records": len(trials), "terminal_records": sum(row["terminal"] for row in trials),
            "actual_flights": sum(row["has_navigation_trace"] for row in trials),
            "startup_failures_no_navigation_trace": sum(row["classification"] == "startup_failure_no_navigation_trace" for row in trials),
            "exploration_records": sum(row["phase"] == "exploration" for row in trials),
            "formal_records": len(formal), "formal_finished": len(formal_finished),
            "formal_actual_flights": sum(row["has_navigation_trace"] for row in formal_finished),
            "formal_safe_arrivals": sum(row["safe_arrival"] is True for row in formal_finished),
            "formal_accepted": len(formal_accepted), "accepted_all_phases": len(accepted),
            "repeated_pass_this_campaign": formal_pass,
            "safe_arrival_best_sim_s": min((row["sim_s"] for row in successes), default=None),
            "safe_arrival_mean_sim_s": statistics.mean(row["sim_s"] for row in successes) if successes else None,
            "formal_accepted_best_sim_s": min(formal_times, default=None),
            "formal_accepted_mean_sim_s": statistics.mean(formal_times) if formal_times else None,
            "formal_accepted_stdev_sim_s": statistics.stdev(formal_times) if len(formal_times) > 1 else None,
            "statuses": dict(collections.Counter(row["status"] for row in trials)),
            "reported_statuses": dict(collections.Counter(row["reported_status"] for row in trials)),
            "classifications": dict(collections.Counter(row["classification"] for row in trials)),
            "trials": [row["result_path"] or row["protocol_path"] for row in trials],
        })
    return groups


def brief(row):
    return {key: row[key] for key in ("trial", "batch", "map", "phase", "speed_mps", "accel_mps2",
            "tau_s", "sim_s", "flight_wall_s", "cap_attained", "accepted", "peak_mps",
            "group_id", "result_path", "audit_path", "sensor_profile_sha256",
            "requested_depth_hfov_deg", "actual_depth_hfov_deg", "actual_depth_vfov_deg",
            "free_space_floor_enabled", "free_space_floor_z_m")}


def aggregate_screening(rows):
    """Repeated early-stop screening never counts as formal confirmation."""
    grouped = collections.defaultdict(list)
    for row in rows:
        if row['phase'] == 'exploration':
            grouped[(row['group_id'], row['exploration_stall_s'])].append(row)
    groups = []
    for (group_id, threshold), trials in sorted(grouped.items()):
        base = trials[0]
        finished = [r for r in trials if r['terminal']]
        accepted = [r for r in finished if r['accepted'] is True]
        geometry = {k: {r[k] for r in finished if r[k]} for k in ('scene_sha256','occupancy_sha256')}
        identity_ok = bool(finished and all(r['identity_complete_and_consistent'] for r in finished)
                           and all(r['scene_sha256'] and r['occupancy_sha256'] for r in finished)
                           and all(len(v)==1 for v in geometry.values()))
        times = [r['sim_s'] for r in accepted if finite(r['sim_s'])]
        groups.append(dict(
            screening_group_id=digest(dict(group_id=group_id,exploration_stall_s=threshold)),
            group_id=group_id, cross_map_family_id=base['cross_map_family_id'],
            map=base['map'], speed_mps=base['speed_mps'], accel_mps2=base['accel_mps2'], tau_s=base['tau_s'],
            exploration_stall_s=threshold, records=len(trials), finished=len(finished),
            actual_flights=sum(r['has_navigation_trace'] for r in finished), accepted=len(accepted),
            identity_and_geometry_consistent=identity_ok,
            repeated_screen_pass=bool(len(finished)>=3 and len(finished)==len(trials)
                and len(accepted)==len(finished) and identity_ok and all(r['has_navigation_trace'] for r in finished)),
            accepted_best_sim_s=min(times,default=None), accepted_mean_sim_s=statistics.mean(times) if times else None,
            accepted_stdev_sim_s=statistics.stdev(times) if len(times)>1 else None,
            batches=sorted({r['batch'] for r in trials}),
            reported_statuses=dict(collections.Counter(r['reported_status'] for r in trials)),
            trials=[r['result_path'] or r['protocol_path'] for r in trials]))
    return groups


def build_summary(rows, errors):
    groups = aggregate(rows)
    screening = aggregate_screening(rows)
    maps = sorted({row["map"] for row in rows if row["map"]})
    fastest, fastest_accepted, highest, fastest_repeated = {}, {}, {}, {}
    for map_name in maps:
        safe = [row for row in rows if row["map"] == map_name and row["safe_arrival"] is True and finite(row["sim_s"])]
        accepted = [row for row in safe if row["accepted"] is True]
        repeated = [group for group in groups if group["map"] == map_name and group["repeated_pass_this_campaign"]]
        fastest[map_name] = brief(min(safe, key=lambda row: row["sim_s"])) if safe else None
        fastest_accepted[map_name] = brief(min(accepted, key=lambda row: row["sim_s"])) if accepted else None
        highest[map_name] = max(repeated, key=lambda group: group["speed_mps"]) if repeated else None
        fastest_repeated[map_name] = min(repeated, key=lambda group: group["formal_accepted_mean_sim_s"]) if repeated else None
    return {
        "schema": "ego2p1_campaign_summary_v1",
        "generated_at": datetime.now(timezone(timedelta(hours=8))).isoformat(),
        "scope": "Only p1_speed_20261015 plus sim4_20261015/p1_baseline_v4; inherited P0 trials excluded.",
        "claim_boundary": "本轮已测范围；重复通过不等于全局最高速度、绝对最短时间或任意场景可靠。",
        "phase_rule": "Recorded exploration_stall_s > 0 = exploration; exactly 0 = formal; missing = unknown.",
        "repeat_rule": "Same executable/settings/full protocol except final_repeats/execution sources/recorded runtime environment/actual sensor profile/map/speed/accel/tau identity; consistent geometry; >=3 actual completed formal records, all accepted and identity consistent; pending formal records prevent passing.",
        "screening_rule": "Exploration remains exploration. Repeated screening additionally requires the same positive stall threshold, >=3 actual completed records all accepted and identity/geometry consistent; it never substitutes for formal zero-cutoff validation.",
        "stable_settings_excluded_keys": sorted(VOLATILE_SETTINGS),
        "protocol_identity_excluded_keys": ["final_repeats"],
        "cross_map_protocol_additional_excluded_keys": ["goal", "timeout_s"],
        "record_count": len(rows), "terminal_count": sum(row["terminal"] for row in rows),
        "actual_flight_count": sum(row["has_navigation_trace"] for row in rows),
        "completed_flight_count": sum(row["terminal"] and row["has_navigation_trace"] for row in rows),
        "startup_failure_count": sum(row["classification"] == "startup_failure_no_navigation_trace" for row in rows),
        "pending_count": sum(not row["terminal"] for row in rows),
        "safe_arrivals": sum(row["safe_arrival"] is True for row in rows),
        "accepted": sum(row["accepted"] is True for row in rows),
        "unsafe_arrival_count": sum(row["classification"] == "unsafe_arrival" for row in rows),
        "phase_counts": dict(collections.Counter(row["phase"] for row in rows)),
        "status_counts": dict(collections.Counter(row["status"] for row in rows)),
        "reported_status_counts": dict(collections.Counter(row["reported_status"] for row in rows)),
        "classification_counts": dict(collections.Counter(row["classification"] for row in rows)),
        "flight_status_counts": dict(collections.Counter(row["status"] for row in rows if row["has_navigation_trace"])),
        "fastest_single_safe_by_map": fastest,
        "fastest_single_accepted_by_map": fastest_accepted,
        "highest_repeated_speed_by_map": highest,
        "fastest_repeated_mean_time_by_map": fastest_repeated,
        "highest_repeated_screening_speed_by_map": {name:max((g for g in screening if g['map']==name and g['repeated_screen_pass']),key=lambda g:g['speed_mps'],default=None) for name in maps},
        "read_errors": errors, "groups": groups, "screening_groups": screening, "trials": rows,
    }


def cross_map_successes(summary):
    families = collections.defaultdict(list)
    repeated = {group["group_id"] for group in summary["groups"] if group["repeated_pass_this_campaign"]}
    for row in summary["trials"]:
        if row["accepted"] is True and row["identity_complete_and_consistent"] and finite(row["sim_s"]):
            families[row["cross_map_family_id"]].append(row)
    results = []
    for family_id, rows in families.items():
        by_map = {name: [row for row in rows if row["map"] == name] for name in ("original", "long")}
        if not all(by_map.values()):
            continue
        best = {name: min(trials, key=lambda row: row["sim_s"]) for name, trials in by_map.items()}
        screening_thresholds = [{g['exploration_stall_s'] for g in summary.get('screening_groups',[])
                                if g['cross_map_family_id']==family_id and g['map']==name and g['repeated_screen_pass']}
                               for name in ('original','long')]
        common_thresholds = sorted(screening_thresholds[0] & screening_thresholds[1])
        results.append({"family_id": family_id, "speed_mps": rows[0]["speed_mps"],
                        "accel_mps2": rows[0]["accel_mps2"], "tau_s": rows[0]["tau_s"],
                        "best_by_map": {name: brief(row) for name, row in best.items()},
                        "both_maps_repeated_screen_pass": bool(common_thresholds),
                        "screening_stall_thresholds_both_maps_passed": common_thresholds,
                        "both_maps_repeated_pass": all(any(row["group_id"] in repeated for row in trials)
                                                       for trials in by_map.values())})
    return sorted(results, key=lambda entry: sum(row["sim_s"] for row in entry["best_by_map"].values()))


def fmt(value):
    if value is None:
        return "—"
    if type(value) is bool:
        return "是" if value else "否"
    if isinstance(value, float):
        return f"{value:.3f}".rstrip("0").rstrip(".")
    return str(value).replace("|", "\\|").replace("\n", " ")


def table(headers, entries):
    lines = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    lines += ["| " + " | ".join(fmt(value) for value in row) + " |" for row in entries]
    return "\n".join(lines)


def markdown(summary):
    rows = summary["trials"]
    groups = summary["groups"]
    lines = ["# EGO2P1 本轮速度试验结果", "", f"生成：{summary['generated_at']}。",
        "", "只纳入本研究目录和明确命名的 P1 基线，继承的 P0 结果不计入。探索为记录的卡停阈值 >0，正式为阈值 =0；缺失值显示 —，不补成 0。",
        "", f"现有 {summary['record_count']} 条启动记录，{summary['terminal_count']} 条已终止；其中已终止实际飞行 {summary['completed_flight_count']} 条、无导航轨迹的启动失败 {summary['startup_failure_count']} 条，另有进行中/未完整记录 {summary['pending_count']} 条。安全到达 {summary['safe_arrivals']} 条，安全到达且达到请求档位 {summary['accepted']} 条，抵达但独立安全审计拒绝（UNSAFE_ARRIVAL）{summary['unsafe_arrival_count']} 条。混合版本合计不代表某一配置成功率；启动失败不冒充飞行卡停。",
        "", "ARRIVED 只是终点条件成立，不等于安全通过；UNSAFE_ARRIVAL、TIMEOUT 与探索 STALL 分开，原始状态仍保留在 CSV/JSON 的 status 字段。",
        "", "同组按实际二进制 SHA、排除输出路径/运行名后的全量有效设置、执行脚本身份、已记录运行环境、地图与请求速度/a/τ区分，并检查地图内容一致。仅同组至少 3 条实际正式记录全部结束、全部 accepted 且身份一致，才标“本轮重复通过”；探索不进入正式分母，计划重复数不计数。",
        "", "## 最快单次与重复确认", ""]
    for map_name, fastest in summary["fastest_single_safe_by_map"].items():
        highest = summary["highest_repeated_speed_by_map"][map_name]
        if fastest:
            lines.append(f"- {map_name} 最快单次安全到达：{fmt(fastest['sim_s'])} s，请求 {fmt(fastest['speed_mps'])} m/s，达速 {fmt(fastest['cap_attained'])}，{fastest['batch']}/{fastest['trial']}。")
        else:
            lines.append(f"- {map_name}：当前没有安全到达成绩。")
        lines.append(f"- {map_name} 本轮最高重复通过速度：{fmt(highest['speed_mps'])} m/s，正式 {highest['formal_accepted']}/{highest['formal_finished']}。" if highest else f"- {map_name}：尚无至少 3 次全通过的正式组，不能给出本轮重复通过最高速度。")
    lines += ["", "下述速度和时间只覆盖实际扫描范围，不是全局上限或绝对最优。仿真时间已扣预热；飞行墙钟与含启动/退出的进程墙钟分别报告。", "", "## 重复筛查（保留正卡停阈值，不等于正式确认）", "",
        table(["筛查组", "地图/速度", "卡停阈值s", "accepted/已结束", "已有记录", "重复筛查全通过", "成功均时/最佳s", "批次"],
              [[g['screening_group_id'][:12],f"{g['map']}/{fmt(g['speed_mps'])}",g['exploration_stall_s'],f"{g['accepted']}/{g['finished']}",g['records'],g['repeated_screen_pass'],
                f"{fmt(g['accepted_mean_sim_s'])}/{fmt(g['accepted_best_sim_s'])}",','.join(g['batches'])]
               for g in summary.get('screening_groups',[]) if g['records']>=3 or any(n.startswith('final_native_') for n in g['batches'])]),
        "", "筛查按严格配置组再按卡停阈值分组；3次筛查全通过也不计入以下正式重复通过。单次筛查及全部失败仍在逐轮表中。", "", "## 同版同配置分组（正式确认另计）", "",
        table(["组", "地图", "v/a/τ", "探索/正式已完成", "正式accepted", "重复通过", "正式成功均时/最佳s", "状态"],
              [[g["group_id"][:12], g["map"], "/".join(fmt(g[k]) for k in ("speed_mps", "accel_mps2", "tau_s")),
                f"{g['exploration_records']}/{g['formal_finished']}", f"{g['formal_accepted']}/{g['formal_finished']}",
                g["repeated_pass_this_campaign"], f"{fmt(g['formal_accepted_mean_sim_s'])}/{fmt(g['formal_accepted_best_sim_s'])}",
                json.dumps(g["reported_statuses"], ensure_ascii=False)] for g in groups]),
        "", "## 逐轮导航与安全", "",
        table(["批次/试次", "阶段", "类别/状态/accepted", "v/a/τ", "仿真/飞行墙钟s", "峰值/P95m/s", "95%累计/最长s", "净距m", "扫/护/限", "停车/最长s", "证据"],
              [[r["batch"] + "/" + r["trial"], r["phase"], r["classification"] + "/" + r["reported_status"] + "/" + fmt(r["accepted"]),
                "/".join(fmt(r[k]) for k in ("speed_mps", "accel_mps2", "tau_s")),
                f"{fmt(r['sim_s'])}/{fmt(r['flight_wall_s'])}", f"{fmt(r['peak_mps'])}/{fmt(r['p95_mps'])}",
                f"{fmt(r['time_at_95pct_cap_s'])}/{fmt(r['longest_at_95pct_cap_s'])}", r["minimum_swept_clearance_m"],
                "/".join(fmt(r[k]) for k in ("swept_overlap_segments", "collision_blocks", "limit_violations_total")),
                f"{fmt(r['stopped_s'])}/{fmt(r['longest_stop_s'])}",
                f"[审计](<{r['audit_path']}>)" if r["audit_path"] else "—"] for r in rows]),
        "", "## 逐轮计算与运行性能", "",
        table(["批次/试次", "callback均/P95/峰值ms", "避障均/P95/峰值ms", "Isaac墙钟FPS", "控制墙钟FPS", "帧间隔P95/峰值ms", "进程墙钟s", "路长/均速", "jerk RMS"],
              [[r["batch"] + "/" + r["trial"],
                "/".join(fmt(r[k]) for k in ("callback_mean_ms", "callback_p95_ms", "callback_max_ms")),
                "/".join(fmt(r[k]) for k in ("avoid_mean_ms", "avoid_p95_ms", "avoid_max_ms")),
                r["isaac_wall_fps"], r["controller_wall_fps"], f"{fmt(r['frame_interval_p95_ms'])}/{fmt(r['frame_interval_max_ms'])}",
                r["process_wall_s"], f"{fmt(r['path_m'])}/{fmt(r['route_average_mps'])}", r["jerk_rms_mps3"]] for r in rows]),
        "", "仿真固定 60 Hz 采样不代表实际控制严格 60 Hz。正式组的均时只计算 accepted 轨迹；失败完整保留在同组状态、逐轮表和 CSV/JSON。", ""]
    if summary["read_errors"]:
        lines += ["## 读取异常", "", "```json", json.dumps(summary["read_errors"], ensure_ascii=False, indent=2), "```", ""]
    return "\n".join(lines)


def report_draft(summary):
    """Compact current evidence, with historical scores clearly separated."""
    rows = summary["trials"]
    formal = [g for g in summary["groups"] if g["formal_records"]]
    repeated = [g for g in formal if g["repeated_pass_this_campaign"]]
    floor_rows = [r for r in rows if r["free_space_floor_enabled"] is True]
    sensor_rows = [r for r in rows if r["sensor_evidence_required"]]
    common = cross_map_successes(summary)
    highest_common = max((f['speed_mps'] for f in common), default=None)
    floor_common = [f for f in common if all(r['free_space_floor_enabled'] is True for r in f['best_by_map'].values())]
    tested = sorted({r['speed_mps'] for r in rows if r['terminal'] and r['has_navigation_trace'] and finite(r['speed_mps'])})
    independent = load(HERE / 'independent_check.json', [])
    lines = ["# EGO2P1 避障参数优化报告草稿（进行中）", "",
        f"证据快照：{summary['generated_at']}。仅报告已落盘的实际结果，不预填后续确认。",
        "", f"已结束实际仿真 {summary['completed_flight_count']} 次，另有无导航轨迹的启动失败 {summary['startup_failure_count']} 次、进行中或证据未完整 {summary['pending_count']} 次。安全且达速 {summary['accepted']} 次；到达但安全审计拒绝 {summary['unsafe_arrival_count']} 次。跨源码、参数与地图合计只说明工作量，不是一个配置的成功率。",
        "", f"当前严格正式重复通过组 {len(repeated)} 个。同地图、同二进制、完整有效参数、执行源码、运行环境、完整运动/到达/审计协议一致，且实际至少 3 次正式全部通过，才报告本轮重复通过。协议只排除计划重复数 final_repeats；版本、到达半径/速度/保持时间、预热、死区、超时及初始点均保留。",
        "", "当前源码默认参数尚未完成双图正式重复验收，不能作为已验证默认交付。已执行速度档位：" + "、".join(fmt(v) for v in tested) + (" m/s；2 m/s仍未测。" if 2 not in tested else " m/s。"),
        "", "原 a64/τ=.03 正式组与后续 a16/τ=.10 阶段分别保留。最新研究在自由空间认证中加入 z=0 半空间相交，旨在让认证地面边界与独立审计一致；新版本必须重测，不能继承旧 moderate 探索组的成功。",
        "", f"记录中显式启用自由空间地面约束的试次 {len(floor_rows)} 条，其中 accepted={sum(r['accepted'] is True for r in floor_rows)}。字段缺失代表未记录，不能补成关闭或开启。",
        "", "原生相机的 floor_confirm_v4 原图正式试次在240 s超时，虽没有扫掠重叠或保护介入，仍未完成任务。随后探索运行时相机视场覆盖：只修改 USD SessionLayer，不改场景文件；新v5协议与实际相机读回单独冻结，不能与旧v4混组。",
        "", "## 已观测速度和时间", "",
        "以下是本轮各历史版本中的最快安全且达速单次，不代表最新修复版，也不是全局上限。两图的各自最快值不能自动拼成一个共同配置。", ""]
    entries = []
    for name in ("original", "long"):
        best = summary["fastest_single_accepted_by_map"].get(name)
        high = summary["highest_repeated_speed_by_map"].get(name)
        entries.append([name, best["speed_mps"] if best else None,
                        best["sim_s"] if best else None, best["flight_wall_s"] if best else None,
                        best["batch"] if best else "无",
                        f"{fmt(high['speed_mps'])} m/s ({high['formal_accepted']}/{high['formal_finished']})" if high else "尚未建立"])
    lines += [table(["地图", "最快单次请求m/s", "仿真s", "飞行墙钟s", "历史批次", "最高正式重复速度"], entries), "",
        f"本轮历史阶段中，同一配置在双图各一次安全且达速的最高速度为 {fmt(highest_common)} m/s。它不是三次重复结论，也不自动适用于最新默认或地面约束版本。", "",
        "a16/τ=.10、延迟0、垂速1的旧 moderate 阶段：", "",
        table(["速度m/s", "原图", "扩图"], [[v] + ["；".join(f"{r['reported_status']} {fmt(r['sim_s'])} s" for r in rows if r["batch"] == f"moderate_v{v}_a16" and r["map"] == name) or "未记录" for name in ("original", "long")] for v in (4,6,8,10,12)]),
        "", "8 m/s、垂速4曾缩短原图单次时间，但扩图卡停；它不能替代垂速1的双图共同候选。速度高不保证任务更快。未测档位不使用 P0 旧结果填充。",
        "", "## 收敛配置的重复筛查与正式验证", "",
        "final_native 批次固定当前生产版本、90°/.22 m相机、a16/τ.10、垂速1、floor启用且z=0、H1。4 m/s计划双图各3次，2/6/8 m/s计划双图各1次；均保留15 s低速提前终止，因此属于探索筛查。以下只填实际记录，未完成计划不补样本；名称final不代表通过。", "",
        table(["图/速度", "卡停阈值s", "实际accepted/已结束", "已有记录", "三次筛查全通过", "成功均时/最佳s", "状态"],
              [[f"{g['map']}/{fmt(g['speed_mps'])}",g['exploration_stall_s'],f"{g['accepted']}/{g['finished']}",g['records'],g['repeated_screen_pass'],
                f"{fmt(g['accepted_mean_sim_s'])}/{fmt(g['accepted_best_sim_s'])}",json.dumps(g['reported_statuses'],ensure_ascii=False)]
               for g in summary.get('screening_groups',[]) if any(n.startswith('final_native_') for n in g['batches'])]),
        "", "正式完整时限验证另计：必须卡停阈值0，原图240 s/扩图600 s期限，同组至少3次实际全部安全达速。重复筛查不能替代这一口径。", 
        "", "## 地面约束阶段的可用单次结果", "",
        f"显式开启地面约束的记录中，共有 {sum(r['accepted'] is True for r in floor_rows)} 条安全且达速，双图同配置均通过的候选 {len(floor_common)} 个。以下各图最快属于不同配置，不能拼接成双图成功：", "",
        table(["地图", "最快批次", "v/a/τ", "仿真/墙钟s", "净距m", "扫/护/限"],
              [[name, best['batch'], '/'.join(fmt(best[k]) for k in ('speed_mps','accel_mps2','tau_s')),
                f"{fmt(best['sim_s'])}/{fmt(best['flight_wall_s'])}", best['minimum_swept_clearance_m'],
                '/'.join(fmt(best[k]) for k in ('swept_overlap_segments','collision_blocks','limit_violations_total'))]
               for name in ('original','long')
               for best in [min((r for r in floor_rows if r['map']==name and r['accepted'] is True and finite(r['sim_s'])), key=lambda r:r['sim_s'], default=None)] if best]),
        "", "最新低垂速 lowvz01_v4 结果：", "",
        table(["图", "结果", "用时s", "实际峰值m/s", "95%档累计/最长s", "净距m"],
              [[r['map'],r['reported_status'],r['sim_s'],r['peak_mps'],f"{fmt(r['time_at_95pct_cap_s'])}/{fmt(r['longest_at_95pct_cap_s'])}",r['minimum_swept_clearance_m']] for r in rows if r['batch']=='lowvz01_v4']),
        "", "## 安全失败与正式确认", ""]
    for row in rows:
        if row["classification"] == "unsafe_arrival":
            lines.append(f"- {row['batch']}/{row['trial']}：原始 ARRIVED，{fmt(row['sim_s'])} s；独立扫掠重叠 {fmt(row['swept_overlap_segments'])} 段、最小净距 {fmt(row['minimum_swept_clearance_m'])} m，因此记 UNSAFE_ARRIVAL。[原始审计](<{row['audit_path']}>)。")
    lines += ["", "此前 dense_wide_v6 与 confirm_moderate_v4 的地面拒绝使用保守 z=0 审计面。USD 地面实体顶面约 -0.20 m，因此不能仅凭 z=0 重叠就宣称穿过 USD 地面；也不能用较低实体平面消除既定审计失败。当前修复让认证空间直接受 z=0 约束。独立体素、地面、边界及运动限值仍须全部通过。", "",
        table(["正式组", "图/速度", "实际accepted/已结束", "已有记录", "重复通过"], [[g["group_id"][:12], f"{g['map']}/{fmt(g['speed_mps'])}", f"{g['formal_accepted']}/{g['formal_finished']}", g["formal_records"], g["repeated_pass_this_campaign"]] for g in formal]),
        "", "UNSAFE_ARRIVAL、完整 TIMEOUT、探索阈值触发 STALL、无轨迹启动失败分别保留。计划 3 次但遇失败停止，只计算实际已运行的条数。", "",
        "## 新相机投影阶段", "",
        "新协议同时记录请求HFOV和实际四路相机的焦距、孔径、分辨率、HFOV/VFOV、内参及话题/坐标系；v6另记录请求camera offset、实际机体坐标位置、光轴旋转及安装变换。sensor_profile_sha256来自创建渲染产品前的 USD 属性读回，参与严格group与跨图family；独立 ROS CameraInfo probe 另存，不能把 USD 读回写成逐轮 ROS 实测。",
        "", table(["批次/图", "请求HFOV°", "请求offset m/有效向量", "实际四路HFOV°", "实际四路VFOV°", "身份一致", "阶段/结果/用时s"],
                  [[r['batch']+'/'+r['map'], r['requested_depth_hfov_deg'],
                    f"{fmt(r['requested_camera_offset_m'])}/{fmt(r['effective_camera_offset_m'])}",
                    '/'.join(fmt(x) for x in r['actual_depth_hfov_deg']) if r['actual_depth_hfov_deg'] else None,
                    '/'.join(fmt(x) for x in r['actual_depth_vfov_deg']) if r['actual_depth_vfov_deg'] else None,
                    r['sensor_identity_consistent'], f"{r['phase']}/{r['reported_status']}/{fmt(r['sim_s'])}"] for r in sensor_rows]),
        "", "没有记录即未测试；HFOV请求null表示不覆盖原生投影，实际角度只取读回；v6 offset请求null使用原生.22 m，不能将v5缺字段冒充v6原生外参已测。四路ROS CameraInfo的独立实例见 [120°原图probe](wide120_v4/original_v4_a16_r01.camera_info.json)。扩大FOV后原地卡停，当前研究将四相机移至.60 m并同步核心camera_offset，以减少自身入镜；结果仍由新轨迹验收。新协议缺相机记录、冻结校验失败或请求不匹配均不能通过身份审计。",
        "",
        "## 算法、参数与性能", "",
        "四路深度和采集位姿生成保守自由空间；世界球面/深度候选、直达及调和场提出方向；静态观测、已证球与胶囊补充空间证据；每条候选须通过包含响应、加减速、机体与制动尾段的运动认证，并独立检查提前归零 E 路径。缓存只复用候选，不免除认证。命令桥按世界方向输出，执行器以 60 Hz 推进简化一阶速度模型。",
        "", "本轮重点修改：按实际步加速度收取扫掠膨胀、完整路径重启余量、精确已证静态胶囊保留及来源限制、机体半径与地面保护一致性；最新新增自由空间地面半空间相交。完整说明和手柄/goal覆盖关系见 [算法与方法](algorithm_and_method.md)。",
        "", "当前候选默认 a=16 m/s²、τ=.10 s、认证延迟0；测试显式固定垂速1、深度10、H=1、历史1 s/16条、净空奖励.018、方向率8、重启余量0、E开启、缓存TTL .15 s、速度平滑关闭。最终以每轮 effective 和协议为准。普通手柄核与 goal 核不同，固定目标到达不能替代任意手柄改向验收。", ""]
    accepted = [r for r in rows if r["accepted"] is True]
    chosen = []
    for name in ("original", "long"):
        vals = [r for r in accepted if r["map"] == name and finite(r["sim_s"])]
        if vals:
            chosen.append(min(vals, key=lambda r: r["sim_s"]))
    lines += [table(["最快单次批次/图", "callback均/P95/峰值ms", "Isaac/控制墙钟FPS", "95%档累计/最长s", "净距m", "停车/最长s"],
        [[r["batch"]+"/"+r["map"], "/".join(fmt(r[k]) for k in ("callback_mean_ms","callback_p95_ms","callback_max_ms")),
          f"{fmt(r['isaac_wall_fps'])}/{fmt(r['controller_wall_fps'])}", f"{fmt(r['time_at_95pct_cap_s'])}/{fmt(r['longest_at_95pct_cap_s'])}",
          r["minimum_swept_clearance_m"], f"{fmt(r['stopped_s'])}/{fmt(r['longest_stop_s'])}"] for r in chosen]), ""]
    computed = [r for r in rows if finite(r["callback_max_ms"])]
    if computed:
        worst = max(computed, key=lambda r: r["callback_max_ms"])
        lines.append(f"跨阶段已观测 callback 最高峰值 {fmt(worst['callback_max_ms'])} ms（{worst['batch']}/{worst['trial']}）。60 Hz 仿真采样不代表严格实时控制；均值、P95和峰值需一起看。")
    lines += ["", "任务仿真用时扣除2 s预热；飞行墙钟不包含整个启动/退出。认证 .025 s 梯形积分与实际 1/60 s 末速度积分不同；模型未包含真实推力、姿态及气动约束，结论只适用于已审计仿真轨迹。",
        "", "P0 提供瓶颈参考：原图最快35.033 s、扩图126.883 s来自不同快照；proposal_recovery原图正式2/3、扩图只有2/2，不能称双图各3次。P0不计入本轮成功率，见 [P0先验](prior_results.md)。",
        "", "复跑命令及完整阶段/参数/共同配置表见 [详细报告](detailed_report.md)。逐轮时间、达速、净距、扫/护/限、停车和性能见 [results.csv](results.csv)；完整分组见 [results.md](results.md) 和 [summary.json](summary.json)。独立核验从原始CSV/JSON/日志重算，结果见 [independent_check.json](independent_check.json)，没有把生产审计中的安全布尔值当作独立体素重算。",
        "", f"额外指标/身份交叉核验目前覆盖 {fmt(independent.get('checked_traces'))} 条轨迹、差异 {fmt(independent.get('discrepancy_count'))} 项，快照 {fmt(independent.get('generated_at'))}。后续新增轨迹依每轮独立audit记录验收，尚未纳入这份额外交叉核验；仿真采样期间未并发进行重型复审。",
        "", "生产工具回归 [153/153](floor_cli_candidate_v1/production_full_suite_v1.log) 已通过：test_speed_benchmark 7项、test_sim4_benchmark 142项、test_navigation_ground_guard 4项；此前144是同三文件未增加9项floor CLI检查时的集合。不同阶段的42/45与38/42包含不同测试文件，不能相加或互相覆盖。测试通过不代替飞行验收，最终源码默认仍待确认。", ""]
    return "\n".join(lines)


def detailed_report(summary):
    """A timestamped, regenerable report; only observed records fill results."""
    rows = summary["trials"]
    batches = collections.defaultdict(list)
    for row in rows:
        batches[row["batch"]].append(row)
    order = ["p1_baseline_v4", "bulge_only_v4", "reserve05_v4_a64", "reserve05_v4_a8",
             "immediate_v4_a64", "immediate_raw_v4", "immediate_raw_v3", "capsule_v4",
             "capsule_retry_v4", "capsule_v6", "static_v5", "static_v8", "dense_wide_v6", "confirm_v4",
             "dense_wide_reserve10_v6", "moderate_v4_a16", "moderate_v6_a16", "moderate_v8_a16",
             "moderate_v8_vz4", "moderate_v10_a16", "moderate_v12_a16", "confirm_moderate_v4",
             "floor_confirm_v4", "floor_confirm_v8", "wide140_v4", "wide120_v4",
             "wide120_offset60_v4", "wide140_offset60_v4", "fast256_wide120_v4",
             "fast256_h04_v4", "lowvz01_v4", "midcam42_v4", "raisedfloor1_v4", "fullvz_v4",
             "final_native_v4", "final_native_v2", "final_native_v6", "final_native_v8"]
    ordered = sorted(batches, key=lambda name: (order.index(name) if name in order else len(order), name))
    notes = {
        "p1_baseline_v4": "复制后P1基线；继承候选，延迟.2",
        "bulge_only_v4": "按当步加速度计算扫掠膨胀",
        "reserve05_v4_a64": "全路径增加.05重启净空",
        "reserve05_v4_a8": "a8/τ.22及.05重启净空",
        "immediate_v4_a64": "认证延迟0，保留.05重启净空",
        "immediate_raw_v4": "延迟0，重启净空0",
        "immediate_raw_v3": "同阶段3 m/s对照",
        "capsule_v4": "胶囊证据候选，launch构造失败",
        "capsule_retry_v4": "修复launch上下文后胶囊候选重试",
        "capsule_v6": "胶囊候选6 m/s探索",
        "static_v5": "静态证据树约束候选5 m/s探索",
        "static_v8": "静态证据树约束候选8 m/s探索",
        "dense_wide_v6": "历史.25/32、净空奖励.06、H=.667",
        "confirm_v4": "旧a64/τ.03正式确认；不是当前a16阶段",
        "dense_wide_reserve10_v6": "地面保护修复前的加宽余量探索",
        "moderate_v4_a16": "地面保护修复后a16/τ.10，4 m/s探索",
        "moderate_v6_a16": "a16/τ.10，6 m/s探索",
        "moderate_v8_a16": "a16/τ.10，8 m/s、垂速1探索",
        "moderate_v8_vz4": "a16/τ.10，8 m/s、垂速4探索",
        "moderate_v10_a16": "a16/τ.10，10 m/s探索",
        "moderate_v12_a16": "a16/τ.10，12 m/s探索",
        "confirm_moderate_v4": "旧moderate正式4；原图地面审计失败后停止",
        "floor_confirm_v4": "新认证地面半空间阶段；4 m/s正式确认",
        "floor_confirm_v8": "新认证地面半空间阶段；8 m/s正式确认",
        "wide140_v4": "140°相机、原生偏移；v5双图原地卡停",
        "wide120_v4": "120°相机、原生偏移；v5双图原地卡停",
        "wide120_offset60_v4": "v6，120°/.60 m相机，a16/τ.10",
        "wide140_offset60_v4": "v6，140°/.60 m相机，a16/τ.10",
        "fast256_wide120_v4": "120°/.60 m，a256/τ.025、H1",
        "fast256_h04_v4": "120°/.60 m，a256/τ.025、H.4",
        "lowvz01_v4": "90°/.22 m，a16/τ.10、垂速.1",
        "midcam42_v4": "120°/.42 m安装偏移探索",
        "raisedfloor1_v4": "认证floor1 m探索；不能与floor0合组",
        "fullvz_v4": "90°/.22 m、垂速4探索",
        "final_native_v4": "固定生产配置4 m/s三次筛查；阈值15，非正式",
        "final_native_v2": "固定生产配置2 m/s单次筛查；阈值15",
        "final_native_v6": "固定生产配置6 m/s单次筛查；阈值15",
        "final_native_v8": "固定生产配置8 m/s单次筛查；阈值15",
    }
    groups = summary["groups"]
    formal = [row for row in rows if row["phase"] == "formal"]
    repeated = [group for group in groups if group["repeated_pass_this_campaign"]]
    families = cross_map_successes(summary)
    lines = ["# EGO2P1 避障速度优化综合报告（数据快照，研究尚未宣布结束）", "",
        f"数据更新时间：{summary['generated_at']}。本页由 summarize.py 根据已有JSON生成，不预填计划中的最终确认。",
        "", f"当前记录 {summary['record_count']} 次启动，已结束实际飞行 {summary['completed_flight_count']} 次，启动失败且无导航轨迹 {summary['startup_failure_count']} 次，进行中/未完整 {summary['pending_count']} 次。安全且达速 accepted={summary['accepted']}，UNSAFE_ARRIVAL={summary['unsafe_arrival_count']}。这些跨版本合计不能作为某个配置成功率。",
        "", f"正式记录现有 {len(formal)} 条，满足同图同配置至少3条实际正式全通过的组有 {len(repeated)} 个。是否完成双图确认，以下方实际分组为准；同一批次名不自动表示同一版本，名称 confirm/final 也不代表通过。",
        "", "## 算法与本轮修改方向", "",
        "四路深度、相机内参和采集位姿先形成保守自由空间证据；世界球面/深度候选、直达候选及调和方向场提名运动；静态历史、已证球/管段支持连续运动；最终逐条完成含机体、响应、加减速、延迟及制动尾段的认证，并独立认证提前归零E路径。world命令桥保留方向，同一接收样本绑定速度和寿命，仿真60 Hz积分。候选缓存和超时恢复只复用候选，每次仍重新认证。",
        "", "本轮依次研究当步加速度扫掠膨胀、沿完整路径的重启余量、立即响应仿真的认证延迟、精确已证胶囊保留，以及静态证据来源的递归约束。胶囊只能在名义运动与E全部通过后提交；失败候选、纯查询和非静态证据不能生成永久空间授权。参数层继续测试加速度/τ、速度、历史采样/容量、净空奖励及前视H。不同源码阶段的结果保持分开。",
        "", "完整组成、手柄与goal参数优先级、数学与仿真边界见 [algorithm_and_method.md](algorithm_and_method.md)。手柄普通核与固定目标goal核不同，固定目标成功不替代任意手柄改向测试。",
        "", "## P0参考与P1证据范围", "",
        "P0原图最快35.033 s（a64/τ.03），扩图最快126.883 s（a8/τ.22）来自不同冻结配置，均只是单次成绩；没有同一最终版双图各3次稳定通过。P0 proposal_recovery原图正式2/3，扩图正式仅2/2，第3次未运行。此前原1.2 m/s²模型原图3 m/s正式3/3，但均73.989 s慢于2 m/s的69.667 s。因此更高速度与更短任务时间必须分别回答。P0旧数据不计入本轮成功率，详见 [prior_results.md](prior_results.md)。",
        "", "## 分阶段实际结果与失败", "",
        table(["阶段", "含义", "执行二进制SHA前12位", "原图实际结果", "扩图实际结果"],
              [[name, notes.get(name, "相机投影/外参阶段；以协议和USD读回为准" if any(r['sensor_evidence_required'] for r in batches[name]) else "新增阶段，以实际参数为准"),
                ", ".join(sorted({r["executable_sha256"][:12] for r in batches[name] if r["executable_sha256"]})) or "缺失",
                "; ".join(f"{r['trial'].rsplit('_',1)[-1]} {r['reported_status']} {fmt(r['sim_s'])}s ({r['phase']})" for r in batches[name] if r["map"] == "original") or "未记录",
                "; ".join(f"{r['trial'].rsplit('_',1)[-1]} {r['reported_status']} {fmt(r['sim_s'])}s ({r['phase']})" for r in batches[name] if r["map"] == "long") or "未记录"] for name in ordered]),
        "", "探索STALL按预设低速持续阈值提前结束，不是完整240/600 s超时。RUNTIME_ERROR无轨迹是启动失败；有轨迹则是运行中断。ARRIVED只是终点条件，必须再通过独立安全和达速审计；UNSAFE_ARRIVAL明确排除在最快安全时间与重复通过中。",
        "", "## 每阶段实际参数", "",
        table(["阶段/地图", "v/a/τ", "延迟/H/重启余量", "深度/垂速", "历史间隔/容量", "净空奖励", "设置SHA前12位", "请求/实际HFOV°", "实际sensor SHA前12位"],
              [[name + "/" + r["map"], "/".join(fmt(r[k]) for k in ("speed_mps", "accel_mps2", "tau_s")),
                "/".join(fmt(r[k]) for k in ("pipeline_delay_s", "horizon_s", "restart_clearance_m")),
                f"{fmt(r['max_depth_m'])}/{fmt(r['vertical_speed_mps'])}",
                f"{fmt(r['history_interval_s'])}/{fmt(r['history_max_observations'])}", r["clearance_reward"],
                r["controller_settings_sha256"][:12] if r["controller_settings_sha256"] else None,
                f"{fmt(r['requested_depth_hfov_deg'])}/{','.join(fmt(x) for x in r['actual_depth_hfov_deg']) if r['actual_depth_hfov_deg'] else '—'}",
                r['sensor_profile_sha256'][:12] if r['sensor_profile_sha256'] else None]
               for name in ordered for r in { (x["map"],x["group_id"]): x for x in batches[name] }.values()]),
        "", "v单位m/s、a单位m/s²、τ/延迟/H/历史间隔单位s，重启余量单位m。SHA列包含全量effective设置；输出路径和运行名不参与，故增加历史/奖励/H等参数会拆组。运行环境与执行脚本身份另入组键。表中的缺失不补0。",
        "", "## 最快安全单次、共同配置与正式重复", ""]
    for name in ("original", "long"):
        best = summary["fastest_single_accepted_by_map"].get(name)
        high = summary["highest_repeated_speed_by_map"].get(name)
        if best:
            lines.append(f"- {name}最快安全且达速单次：{fmt(best['sim_s'])}仿真s / {fmt(best['flight_wall_s'])}飞行墙钟s，请求{fmt(best['speed_mps'])} m/s，a={fmt(best['accel_mps2'])}、τ={fmt(best['tau_s'])}，{best['batch']}/{best['trial']}，阶段={best['phase']}。[审计](<{best['audit_path']}>)。")
        else:
            lines.append(f"- {name}当前无安全且达速单次。")
        lines.append(f"- {name}本轮最高重复通过速度：{fmt(high['speed_mps'])} m/s，正式{high['formal_accepted']}/{high['formal_finished']}；均时{fmt(high['formal_accepted_mean_sim_s'])} s。" if high else f"- {name}当前未建立正式至少3次全通过的速度档位。")
    lines += ["", "各图最快值可能属于不同源码或参数，不能拼成共同成绩。以下共同配置仅去掉地图路径与目标坐标后比较，其余有效设置、程序、执行源码和运行环境相同；它与单图严格重复组分别统计。", ""]
    if families:
        lines.append(table(["共同配置ID", "v/a/τ", "原图最快s/批次", "扩图最快s/批次", "两图均正式重复通过"],
                           [[f["family_id"][:12], f"{fmt(f['speed_mps'])}/{fmt(f['accel_mps2'])}/{fmt(f['tau_s'])}",
                             f"{fmt(f['best_by_map']['original']['sim_s'])}/{f['best_by_map']['original']['batch']}",
                             f"{fmt(f['best_by_map']['long']['sim_s'])}/{f['best_by_map']['long']['batch']}", f["both_maps_repeated_pass"]] for f in families]))
    else:
        lines.append("当前没有同一共同配置在双图均accepted的证据。")
    lines += ["", table(["正式组", "地图/速度", "已结束/已有正式记录", "accepted", "本轮重复通过", "均时/最佳s"],
                         [[g["group_id"][:12], f"{g['map']}/{fmt(g['speed_mps'])}", f"{g['formal_finished']}/{g['formal_records']}",
                           f"{g['formal_accepted']}/{g['formal_finished']}",g["repeated_pass_this_campaign"],
                           f"{fmt(g['formal_accepted_mean_sim_s'])}/{fmt(g['formal_accepted_best_sim_s'])}"] for g in groups if g["formal_records"]]),
        "", "未运行的重复不计入分母；只有已有实际正式记录至少3条、全部结束且全部accepted，才能标本轮重复通过。速度2/3/4/5/6/8/10中未出现的档位仍为未测，不能插值认定安全。有限扫描不是全局速度极限或绝对最短时间。",
        "", "## 安全失败与性能局限", ""]
    unsafe = [row for row in rows if row["classification"] == "unsafe_arrival"]
    for row in unsafe:
        lines.append(f"- **{row['batch']}/{row['trial']}：UNSAFE_ARRIVAL**。原始状态ARRIVED，用时{fmt(row['sim_s'])} s，但独立扫掠重叠{fmt(row['swept_overlap_segments'])}段，最小净距{fmt(row['minimum_swept_clearance_m'])} m，保护介入{fmt(row['collision_blocks'])}，限值违规{fmt(row['limit_violations_total'])}；不计安全到达。[审计](<{row['audit_path']}>)。")
    computed = [row for row in rows if finite(row["callback_max_ms"])]
    if computed:
        worst = max(computed, key=lambda row: row["callback_max_ms"])
        lines.append(f"- 当前已记录callback最大峰值{fmt(worst['callback_max_ms'])} ms，出现在{worst['batch']}/{worst['trial']}；尾延迟仍需逐轮报告，不能以采样60 Hz宣称严格60 Hz控制。")
    lines += ["", table(["安全达速轨迹", "callback均/P95/峰值ms", "Isaac/控制墙钟FPS", "95%档累计/最长s", "净距m", "停车/最长s", "路长/均速"],
                         [[row["batch"]+"/"+row["trial"], "/".join(fmt(row[key]) for key in ("callback_mean_ms","callback_p95_ms","callback_max_ms")),
                           f"{fmt(row['isaac_wall_fps'])}/{fmt(row['controller_wall_fps'])}",f"{fmt(row['time_at_95pct_cap_s'])}/{fmt(row['longest_at_95pct_cap_s'])}",
                           row["minimum_swept_clearance_m"],f"{fmt(row['stopped_s'])}/{fmt(row['longest_stop_s'])}",f"{fmt(row['path_m'])}/{fmt(row['route_average_mps'])}"]
                          for row in rows if row["accepted"] is True]),
        "", "完整失败的相同性能字段见results.csv/results.md。缺失为—。独立扫掠为体素AABB上的保守段包络，不是在线ESDF自证；保护介入、扫掠重叠或任一运动限值违规均拒绝。仿真任务时间已扣2s预热，飞行墙钟不含完整启动退出，进程墙钟另列。",
        "", "当前默认已调整为a16/τ.10、延迟0；旧a64/τ.03超时单列，不能称当前参数失败。模型没有完整多旋翼姿态、推力与气动约束；认证.025s梯形积分与实际1/60s末速度积分仍有差异，安全结论只覆盖已审计轨迹。",
        "", "## 复跑与证据入口", "",
        "先核对拟复跑版本的manifest、源码/二进制SHA、effective和sensor身份，确认没有活动仿真。以下是固定生产配置4 m/s的重复筛查示例；unique标签必须改成未使用的新名称。它不是正式确认命令，执行后结果仍须重新审计，不能保证到达。",
        "", "```bash", "cd /home/starry/isaac-data/EGO2P1", "bash scripts/build_isaac_ros_workspace.sh",
        "env -u FOV_GVF_CONTROLLER_EXECUTABLE \\",
        "  PYTHONNOUSERSITE=1 FOV_GVF_INSTALL=/tmp/fov_gvf_ego2p1_isaac_install \\",
        "  FOV_GVF_HISTORY_SAMPLE_INTERVAL=1 FOV_GVF_HISTORY_MAX_OBSERVATIONS=16 \\",
        "  ISAAC_DEPTH_HFOV_DEG=90 ISAAC_DEPTH_CAMERA_OFFSET_M=.22 \\",
        "  FOV_GVF_CLEARANCE_REWARD=.018 FOV_GVF_DIRECTION_RATE=8 \\",
        "  FOV_GVF_SHARED_OBSTACLES=0 FOV_GVF_SPHERICAL_MEMORY=0 \\",
        "  FOV_GVF_DEPTH_UNCERTAINTY=1 FOV_GVF_DEPTH_SIGMA0=.005 FOV_GVF_DEPTH_SIGMA_RANGE2=0 \\",
        "  /usr/bin/python3 src/pc_gvf/tools/sim4_benchmark.py unique_final_native_v4 \\",
        "  --maps original long --speed 4 --accel 16 --tau .10 --pipeline-delay 0 \\",
        "  --vertical-speed 1 --depth 10 --horizon 1 --restart-clearance 0 --floor-z 0 \\",
        "  --no-smooth-speed --cache --cache-ttl .15 --emergency \\",
        "  --repeats 3 --abort-stall 15", "",
        "PYTHONDONTWRITEBYTECODE=1 python3 docs/performance/p1_speed_20261015/summarize.py", "```", "",
        "调速时更换唯一label并显式改--speed；2/6/8单次筛查设--repeats 1。若执行正式确认，另用新标签并设--abort-stall 0。计划不进入实际分母，失败保留。主运行仍经scripts/run_isaac_fov_gvf_navigation.sh，不直接绕开共享锁。",
        "", "证据：[逐轮结果](results.md)、[CSV](results.csv)、[机器汇总](summary.json)、[算法与方法](algorithm_and_method.md)、[P0先验](prior_results.md)。每轮同名protocol/parameters/manifest/audit/trace/log保存实际依据；构建与回归日志只支持相应构建/测试结论，不作为到达验收。", ""]
    return "\n".join(lines)


def discover(roots, excluded=None):
    stems = set()
    for root in roots:
        if not root.exists():
            continue
        for suffix in SUFFIXES:
            for path in root.rglob("*" + suffix):
                if "frozen" in path.parts:
                    continue
                stem = Path(str(path)[:-len(suffix)])
                # The benchmark producer always uses these map-prefixed trial
                # names. Launch-construction/invalid-parameter fixtures are not
                # flight attempts, even when they save a .protocol.json file.
                if re.fullmatch(r"(?:original|long)_v.+_r\d+", stem.name):
                    stems.add(stem)
                elif excluded is not None:
                    excluded.add(str(stem))
    return sorted(stems)


def write_outputs(summary, output):
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    (output / "results.md").write_text(markdown(summary))
    (output / "report_draft.md").write_text(report_draft(summary))
    (output / "detailed_report.md").write_text(detailed_report(summary))
    rows = summary["trials"]
    fields = list(rows[0]) if rows else ["trial", "phase", "status"]
    with (output / "results.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, ensure_ascii=False, sort_keys=True)
                             if isinstance(value, (list, dict)) else value for key, value in row.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=HERE)
    parser.add_argument("--baseline", type=Path, default=BASELINE)
    parser.add_argument("--out", type=Path, default=HERE)
    args = parser.parse_args()
    errors = []
    excluded = set()
    rows = [read_trial(stem, errors) for stem in discover([args.root.resolve(), args.baseline.resolve()], excluded)]
    summary = build_summary(rows, errors)
    summary["cross_map_successful_configurations"] = cross_map_successes(summary)
    summary["excluded_nontrial_validation_stems"] = sorted(excluded)
    write_outputs(summary, args.out.resolve())
    print(json.dumps({key: summary[key] for key in ("record_count", "terminal_count", "actual_flight_count", "startup_failure_count", "pending_count", "safe_arrivals", "accepted", "phase_counts", "classification_counts")}, ensure_ascii=False))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
