#!/usr/bin/env python3
"""Isolated simulation trials with swept geometry and independently checked arrival."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

import numpy as np

from speed_benchmark import MAPS, ROOT, audit as legacy_audit


OUTPUT_ROOT = ROOT / 'docs/performance/p6_speed_20261019'
TAU = .10
FRAME_DT = 1. / 60.


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def value_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


SENSOR_PROTOCOL_VERSION = 'ego2p1_sim4_v6'
SENSOR_PROTOCOL_VERSIONS = ('ego2p1_sim4_v5', SENSOR_PROTOCOL_VERSION)
OFFSET_RECORD_VERSION = 'ego2p1_runtime_sensors_v2'
SENSOR_RECORD_VERSION = 'ego2p1_runtime_sensors_v1'
SENSOR_PATHS = {
    '/World/Robot/CameraMount/DepthCamera',
    '/World/Robot/CameraMountLeft/DepthCamera',
    '/World/Robot/CameraMountBack/DepthCamera',
    '/World/Robot/CameraMountRight/DepthCamera',
}
SENSOR_CAMERA_FIELDS = ('name', 'prim_path', 'depth_topic', 'camera_info_topic', 'frame_id',
    'width', 'height', 'focal_length', 'horizontal_aperture', 'vertical_aperture',
    'hfov_deg', 'vfov_deg', 'fx', 'fy', 'cx', 'cy', 'projection', 'lens_distortion_model',
    'horizontal_aperture_offset', 'vertical_aperture_offset')
SENSOR_EXTRINSIC_FIELDS = ('mount_path', 'mount_translation_robot_m',
    'camera_position_robot_m', 'robot_from_optical_rotation', 'camera_local_matrix4x4')


def requested_hfov(environment):
    raw = environment.get('ISAAC_DEPTH_HFOV_DEG')
    if raw is None:
        return None
    value = float(raw)
    if not math.isfinite(value) or not 0. < value < 180.:
        raise ValueError('ISAAC_DEPTH_HFOV_DEG must be finite and strictly between 0 and 180 degrees')
    return value


def requested_camera_offset(environment):
    raw = environment.get('ISAAC_DEPTH_CAMERA_OFFSET_M')
    if raw is None:
        return None
    value = float(raw)
    if not math.isfinite(value) or value <= 0.:
        raise ValueError('ISAAC_DEPTH_CAMERA_OFFSET_M must be finite and positive')
    return value


def sensor_profile(record):
    """Only stable, actual camera geometry; run IDs/scene paths are provenance."""
    fields = SENSOR_CAMERA_FIELDS + (SENSOR_EXTRINSIC_FIELDS
        if record['version'] == OFFSET_RECORD_VERSION else ())
    return dict(version=record['version'], render_width=record['render_width'],
        render_height=record['render_height'],
        cameras=[{key: camera[key] for key in fields}
                 for camera in sorted(record['cameras'], key=lambda item: item['prim_path'])])


def freeze_sensor_evidence(result, manifest):
    path = result.with_suffix('.sensors.json')
    if not path.is_file():
        manifest['sensor_evidence_error'] = 'missing runtime sensor record'
        return
    content = path.read_bytes()
    digest = hashlib.sha256(content).hexdigest()
    blob = result.parent / 'frozen' / 'sha256' / digest
    blob.parent.mkdir(parents=True, exist_ok=True)
    if not blob.exists():
        with blob.open('xb') as stream:
            stream.write(content)
    elif hashlib.sha256(blob.read_bytes()).hexdigest() != digest:
        raise ValueError('corrupt frozen sensor content: ' + str(blob))
    manifest.update(sensors_file_sha256=digest, sensors_blob=str(blob.relative_to(result.parent)))
    try:
        manifest['sensor_profile_sha256'] = value_sha256(sensor_profile(json.loads(content)))
    except (KeyError, TypeError, ValueError) as error:
        manifest['sensor_evidence_error'] = str(error)


def audit_camera_extrinsics(record, protocol, manifest, effective):
    """Bind requested, actual USD and controller geometry without authorizing unknown space."""
    checks = dict(camera_offset_request_matches=False, camera_extrinsics_valid=False,
                  camera_offset_parameter_matches=False)
    details = {}
    try:
        requested = protocol['depth_sensor_camera_offset_m']
        valid_request = requested is None or (type(requested) in (int, float) and
            math.isfinite(requested) and requested > 0.)
        requested_env = requested_camera_offset(manifest.get('requested_environment', {}))
        recorded = record['requested_camera_offset_m']
        checks['camera_offset_request_matches'] = bool(valid_request and requested_env == requested and
            (recorded is None if requested is None else type(recorded) in (int, float) and recorded == requested) and
            type(record['camera_offset_override_applied']) is bool and
            record['camera_offset_override_applied'] == (requested is not None))
        if not valid_request:
            return checks, details
        offset = .22 if requested is None else requested
        def array_close(actual, expected, tolerance=1e-6):
            def finite_numbers(value):
                if isinstance(value, list):
                    return all(finite_numbers(item) for item in value)
                return type(value) in (int, float) and math.isfinite(value)
            if not isinstance(actual, list) or not finite_numbers(actual):
                return False
            values = np.asarray(actual, dtype=float)
            return values.shape == expected.shape and bool(np.allclose(values, expected, rtol=0., atol=tolerance))
        checks['camera_offset_parameter_matches'] = array_close(
            effective.get('camera_offset'), np.array([offset, 0., .02]))
        front = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        local = np.eye(4)
        yaw_by_path = dict(zip(('/World/Robot/CameraMount' + suffix + '/DepthCamera'
            for suffix in ('', 'Left', 'Back', 'Right')), (0., 90., 180., -90.)))
        valid = (record['version'] == OFFSET_RECORD_VERSION and len(record['cameras']) == 4 and
            {camera['prim_path'] for camera in record['cameras']} == SENSOR_PATHS)
        for camera in record['cameras']:
            angle = math.radians(yaw_by_path[camera['prim_path']])
            c, sin = math.cos(angle), math.sin(angle)
            rz = np.array([[c, -sin, 0.], [sin, c, 0.], [0., 0., 1.]])
            position = rz @ np.array([offset, 0., .02])
            valid = bool(valid and camera['mount_path'] == camera['prim_path'].rsplit('/', 1)[0] and
                array_close(camera['mount_translation_robot_m'], position) and
                array_close(camera['camera_position_robot_m'], position) and
                array_close(camera['robot_from_optical_rotation'], rz @ front) and
                array_close(camera['camera_local_matrix4x4'], local, 1e-9))
        checks['camera_extrinsics_valid'] = valid
        details = dict(requested_depth_camera_offset_m=requested,
            actual_camera_positions_robot_m=[camera['camera_position_robot_m'] for camera in record['cameras']])
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        details['camera_extrinsic_evidence_error'] = str(error)
    return checks, details


def audit_sensor_evidence(result, protocol, manifest, effective=None):
    checks = dict(sensor_record_valid=False, sensor_request_matches=False,
                  sensor_frozen_matches=False)
    details = {}
    try:
        path = result.with_suffix('.sensors.json')
        content = path.read_bytes()
        record = json.loads(content)
        profile = sensor_profile(record)
        profile_sha = value_sha256(profile)
        digest = hashlib.sha256(content).hexdigest()
        blob = result.parent / manifest['sensors_blob']
        checks['sensor_frozen_matches'] = bool(digest == manifest['sensors_file_sha256'] and
            blob.is_file() and hashlib.sha256(blob.read_bytes()).hexdigest() == digest and
            profile_sha == manifest['sensor_profile_sha256'])
        requested = protocol['depth_sensor_hfov_deg']
        requested_env = requested_hfov(manifest.get('requested_environment', {}))
        valid_request = requested is None or (type(requested) in (int, float) and
            math.isfinite(requested) and 0. < requested < 180.)
        checks['sensor_request_matches'] = bool(valid_request and requested_env == requested and
            (record['requested_hfov_deg'] is None if requested is None else
             type(record['requested_hfov_deg']) in (int, float) and record['requested_hfov_deg'] == requested) and
            type(record['override_applied']) is bool and record['override_applied'] == (requested is not None))
        cameras = record['cameras']
        expected_record_version = (OFFSET_RECORD_VERSION if protocol.get('version') == SENSOR_PROTOCOL_VERSION
                                   else SENSOR_RECORD_VERSION)
        valid = (record['version'] == expected_record_version and
            type(record['render_width']) is int and record['render_width'] == 320 and
            type(record['render_height']) is int and record['render_height'] == 240 and
            len(cameras) == 4 and {camera['prim_path'] for camera in cameras} == SENSOR_PATHS)
        hfovs = []
        for camera in cameras:
            width, height = camera['width'], camera['height']
            f, ha, va = (camera[key] for key in ('focal_length', 'horizontal_aperture', 'vertical_aperture'))
            numeric = (f, ha, va, *(camera[key] for key in ('hfov_deg', 'vfov_deg', 'fx', 'fy', 'cx', 'cy',
                         'horizontal_aperture_offset', 'vertical_aperture_offset')))
            if not all(type(value) in (int, float) and math.isfinite(value) for value in numeric):
                valid = False
                continue
            if min(f, ha, va) <= 0.:
                valid = False
                continue
            hfov, vfov = math.degrees(2.*math.atan(ha/(2.*f))), math.degrees(2.*math.atan(va/(2.*f)))
            hfovs.append(hfov)
            close = lambda a, b: math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-4)
            valid = bool(valid and type(width) is int and width == 320 and
                type(height) is int and height == 240 and camera['projection'] == 'perspective' and
                camera['lens_distortion_model'] is None and camera['horizontal_aperture_offset'] == 0. and
                camera['vertical_aperture_offset'] == 0. and
                close(camera['hfov_deg'], hfov) and close(camera['vfov_deg'], vfov) and
                close(camera['fx'], width*f/ha) and close(camera['fy'], height*f/va) and
                close(camera['cx'], width/2.) and close(camera['cy'], height/2.) and
                close(ha/va, width/height) and close(camera['fx'], camera['fy']))
            valid = bool(valid and close(hfov, 90. if requested is None else requested))
            # Override explicitly keeps square pixels and the 4:3 render aspect.
            if requested is not None:
                valid = bool(valid and close(hfov, requested) and close(ha/va, width/height) and
                             close(camera['fx'], camera['fy']))
        checks['sensor_record_valid'] = bool(valid and len(hfovs) == 4 and max(hfovs)-min(hfovs) <= 1e-4)
        details = dict(sensor_profile_sha256=profile_sha, sensors_file_sha256=digest,
                       requested_depth_hfov_deg=requested, actual_depth_hfov_deg=hfovs)
        if protocol.get('version') == SENSOR_PROTOCOL_VERSION:
            offset_checks, offset_details = audit_camera_extrinsics(record, protocol, manifest, effective or {})
            checks.update(offset_checks)
            details.update(offset_details)
    except (OSError, KeyError, TypeError, ValueError, OverflowError) as error:
        details['sensor_evidence_error'] = str(error)
    return checks, details


def freeze_trial_evidence(result, protocol, command, environment):
    """Save content-addressed source/install bytes before starting this trial."""
    install = Path(environment.get('FOV_GVF_INSTALL', '/home/starry/isaac-data/EGO2P6/docs/performance/runtime/install'))
    executable = Path(environment.get('FOV_GVF_CONTROLLER_EXECUTABLE',
        '/home/starry/isaac-data/EGO2P6/docs/performance/runtime/install/pc_gvf/lib/pc_gvf/depth_angular_controller_goal'))
    manifest_path = result.with_suffix('.manifest.json')
    if manifest_path.exists():
        raise RuntimeError('refusing to overwrite ' + str(manifest_path))
    blobs = result.parent / 'frozen' / 'sha256'
    blobs.mkdir(parents=True, exist_ok=True)
    paths = {}
    for kind, base in [('source', ROOT / name) for name in ('src', 'scripts')] + [('install', install)]:
        for path in sorted(base.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.pyo'):
                paths[str(path.absolute())] = kind
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError('controller must exist and be executable: ' + str(executable))
    paths[str(executable.absolute())] = 'controller'
    files = {}
    for name, kind in sorted(paths.items()):
        path = Path(name)
        content = path.read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        blob = blobs / digest
        if not blob.exists():
            with blob.open('xb') as stream:
                stream.write(content)
        elif hashlib.sha256(blob.read_bytes()).hexdigest() != digest:
            raise ValueError('corrupt frozen content: ' + str(blob))
        files[name] = dict(kind=kind, sha256=digest, bytes=len(content),
            mode=path.stat().st_mode & 0o777, blob=str(blob.relative_to(result.parent)))
    source_identity = {str(Path(name).relative_to(ROOT)): entry['sha256']
                       for name, entry in files.items() if entry['kind'] == 'source'}
    environment_keys = ('FOV_GVF_INSTALL', 'FOV_GVF_CONTROLLER_EXECUTABLE',
        'FOV_GVF_BENCHMARK_ALGORITHM', 'FOV_GVF_SCENE', 'FOV_GVF_ESDF_OCCUPANCY',
        'FOV_GVF_COMMAND_FRAME', 'ISAAC_COMMAND_FRAME', 'ISAAC_TWIST_SAMPLE_SOURCE',
        'FOV_GVF_EMERGENCY_CERTIFICATE', 'FOV_GVF_SHARED_OBSTACLES',
        'FOV_GVF_SPHERICAL_MEMORY', 'FOV_GVF_DEPTH_UNCERTAINTY',
        'FOV_GVF_DEPTH_SIGMA0', 'FOV_GVF_DEPTH_SIGMA_RANGE2',
        'EGO5090_CONTROLLER_CPU_AFFINITY', 'EGO5090_MANUAL_POLICY', 'FOV_GVF_GOAL_CACHE_DISPLACEMENT',
        'ISAAC_EXPLORATION_STALL_S', 'ISAAC_DEPTH_HFOV_DEG', 'ISAAC_DEPTH_CAMERA_OFFSET_M', 'OMP_WAIT_POLICY', 'ROS_DOMAIN_ID')
    manifest = dict(version='ego2p1_trial_evidence_v2' if protocol.get('version') in SENSOR_PROTOCOL_VERSIONS else 'ego2p1_trial_evidence_v1', files=files,
        protocol_version=protocol.get('version'),
        source_tree_sha256=value_sha256(source_identity), command=command,
        prepared_protocol_sha256=value_sha256(protocol),
        requested_environment={key: environment[key] for key in environment_keys if key in environment},
        expected_executable=str(executable.absolute()),
        expected_executable_sha256=files[str(executable.absolute())]['sha256'])
    write_json(manifest_path, manifest)
    return manifest


def finish_trial_evidence(result, manifest):
    """Check the actual launcher metadata and detect edits/rebuilds during a run."""
    changed = [name for name, item in manifest['files'].items()
        if not Path(name).is_file() or hashlib.sha256(Path(name).read_bytes()).hexdigest() != item['sha256']]
    manifest['changed_files_during_run'] = changed
    parameters_path = result.with_suffix('.parameters.json')
    if parameters_path.is_file():
        parameters = json.loads(parameters_path.read_text())
        manifest['parameters_file_sha256'] = hashlib.sha256(parameters_path.read_bytes()).hexdigest()
        manifest['actual_executable'] = parameters.get('executable')
        manifest['actual_executable_sha256'] = parameters.get('executable_sha256')
        manifest['effective_parameters_sha256'] = value_sha256(parameters['effective'])
        # Run IDs and output paths are evidence locations, not controller settings.
        stable = {key: value for key, value in parameters['effective'].items()
                  if key not in ('performance_log_path', 'performance_run_id',
                                 'paper_replay_directory', 'paper_replay_dir')}
        manifest['controller_settings_sha256'] = value_sha256(stable)
        manifest['execution_matches_frozen'] = (
            parameters.get('algorithm') in ('ego2p1', 'ego2p2', 'ego2p3', 'ego2p4', 'ego2p5', 'ego2p6') and
            parameters.get('executable') == manifest['expected_executable'] and
            parameters.get('executable_sha256') == manifest['expected_executable_sha256'])
    else:
        manifest['execution_matches_frozen'] = False
    if manifest.get('protocol_version') in SENSOR_PROTOCOL_VERSIONS:
        freeze_sensor_evidence(result, manifest)
    manifest['complete'] = True
    write_json(result.with_suffix('.manifest.json'), manifest)
    return manifest


def protocol_for(args, map_name):
    return dict(version=SENSOR_PROTOCOL_VERSION, depth_sensor_hfov_deg=(args.camera_hfov if args.camera_hfov is not None else requested_hfov(os.environ)),
        depth_sensor_camera_offset_m=(args.camera_offset if args.camera_offset is not None else requested_camera_offset(os.environ)),
        start=[-64., -11.2, 2.8],
        goal=list(MAPS[map_name][2]), warmup_s=2.,
        timeout_s=600.,
        max_speed_mps=args.speed, max_vertical_speed_mps=args.vertical_speed,
        input_deadband_m=.5, arrival_radius_m=1., arrival_speed_mps=.1,
        arrival_hold_s=.5, body_radius_m=.48, plant_tau_s=args.tau,
        plant_accel_mps2=args.accel, final_repeats=args.repeats,
        free_space_floor_enabled=True, free_space_floor_z=args.floor_z,
        max_depth_m=args.depth, pipeline_delay_s=args.pipeline_delay,
        goal_proposal_cache=args.cache, goal_proposal_cache_ttl_s=args.cache_ttl,
        goal_speed_governor=False, restart_clearance_m=args.restart_clearance,
        emergency_braking_certificate=args.emergency,
        planning_horizon_s=args.horizon, smooth_speed=args.smooth_speed,
        perception_parameters=dict(
            paper_shared_obstacles=os.environ.get('FOV_GVF_SHARED_OBSTACLES', '0') == '1',
            paper_spherical_memory=os.environ.get('FOV_GVF_SPHERICAL_MEMORY', '0') == '1',
            depth_uncertainty_enabled=os.environ.get('FOV_GVF_DEPTH_UNCERTAINTY', '1') == '1',
            depth_uncertainty_sigma0=float(os.environ.get('FOV_GVF_DEPTH_SIGMA0', '.005')),
            depth_uncertainty_sigma_range2=float(os.environ.get('FOV_GVF_DEPTH_SIGMA_RANGE2', '0')),
            depth_uncertainty_pose_bound=.01))


def high_speed_duration(t, speed, threshold):
    """Attribute each integration interval to its executed, end-of-step velocity."""
    dt = np.diff(t)
    mask = speed[1:] >= threshold
    longest = streak = 0.
    for flag, delta in zip(mask, dt):
        streak = streak + float(delta) if flag else 0.
        longest = max(longest, streak)
    return float(dt[mask].sum()), longest


def working_bounds_checks(protocol, effective, positions, radius):
    """Declared operational bounds only; this is not an online free-space map."""
    bounds = protocol.get('working_bounds')
    if bounds is None:
        return {}
    keys = ['min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z']
    valid = (isinstance(bounds, dict) and type(bounds.get('enabled')) is bool and
             all(type(bounds.get(k)) in (int, float) and np.isfinite(bounds[k]) for k in keys))
    if valid:
        valid = all(bounds['min_' + axis] < bounds['max_' + axis] for axis in 'xyz')
    result = dict(working_bounds_protocol_valid=bool(valid), working_bounds_parameters_match=False,
                  working_bounds_physical_body=False)
    if not valid:
        return result
    expected = dict(paper_working_bounds_enabled=bounds['enabled'],
        paper_working_min_x=bounds['min_x'], paper_working_max_x=bounds['max_x'],
        paper_working_min_y=bounds['min_y'], paper_working_max_y=bounds['max_y'],
        paper_free_space_floor_enabled=True, paper_free_space_ceiling_enabled=True,
        paper_free_space_floor_z=bounds['min_z'], paper_free_space_ceiling_z=bounds['max_z'])
    result['working_bounds_parameters_match'] = all(effective.get(k) == v for k, v in expected.items())
    lo = np.array([bounds['min_' + k] for k in 'xyz'])
    hi = np.array([bounds['max_' + k] for k in 'xyz'])
    result['working_bounds_physical_body'] = bool(not bounds['enabled'] or
        (np.isfinite(positions).all() and np.all(positions-radius >= lo-1e-8) and
         np.all(positions+radius <= hi+1e-8)))
    return result


def audit_trial(result, scene, shape):
    """Keep the old geometry check, then verify the recorded execution contract."""
    report = legacy_audit(result, scene, shape)
    legacy_safe = bool(report['safe_arrival'])
    data = json.loads(result.read_text())
    protocol = data['protocol']
    expected = json.loads(result.with_suffix('.protocol.json').read_text())
    parameters = json.loads(result.with_suffix('.parameters.json').read_text())
    effective = parameters['effective']
    environment = parameters.get('runtime_environment', {})
    with result.with_suffix('.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    keys = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz']
    trajectory = np.array([[float(row[k]) for k in keys] for row in rows])
    if len(trajectory) < 3 or not np.isfinite(trajectory).all():
        raise ValueError('incomplete or nonfinite trajectory')
    t, p, v = trajectory[:, 0], trajectory[:, 1:4], trajectory[:, 4:7]
    dt = np.diff(t)
    speed = np.linalg.norm(v, axis=1)
    goal = np.asarray(protocol['goal'], float)
    final_position = np.asarray(data['final_position'], float)
    actual_distance = float(np.linalg.norm(p[-1] - goal))
    elapsed = max(0., float(t[-1]) - protocol['warmup_s'])
    conditions = ((np.linalg.norm(p - goal, axis=1) <= protocol['arrival_radius_m']) &
                  (speed <= protocol['arrival_speed_mps']) & (t >= protocol['warmup_s']))
    hold_start = len(t)
    while hold_start > 0 and conditions[hold_start - 1]:
        hold_start -= 1
    hold = float(t[-1] - t[hold_start]) if hold_start < len(t) else 0.
    integration_error = float(np.linalg.norm(np.diff(p, axis=0) - v[1:] * dt[:, None], axis=1).max())
    z_min, z_max = -.2, .4 * (shape[2] - 1) + .2
    radius = protocol['body_radius_m']
    checks = dict(
        protocol_matches_prepared=protocol == expected,
        tau_parameter_matches=bool(type(protocol['plant_tau_s']) in (int, float) and
            math.isfinite(protocol['plant_tau_s']) and protocol['plant_tau_s'] > 0. and
            type(effective.get('velocity_response_time')) in (int, float) and
            effective['velocity_response_time'] == protocol['plant_tau_s']),
        frame_cadence=bool(abs(t[0]) <= 1e-8 and np.all(np.abs(dt - FRAME_DT) <= 1e-7)),
        starts_at_protocol=bool(np.linalg.norm(p[0] - np.asarray(protocol['start'])) <= 1e-7 and
                                np.linalg.norm(v[0]) <= 1e-7),
        integration_matches_velocity=integration_error <= 1e-7,
        result_matches_trace=bool(final_position.shape == (3,) and np.isfinite(final_position).all() and
            np.linalg.norm(final_position - p[-1]) <= 1e-7 and
            data['goal'] == protocol['goal'] and
            math.isfinite(data['final_distance_m']) and
            abs(data['final_distance_m'] - actual_distance) <= 1e-7 and
            math.isfinite(data['simulation_elapsed_s']) and
            abs(data['simulation_elapsed_s'] - elapsed) <= 1e-7),
        arrival_hold_independent=bool(conditions[-1] and hold + 1e-7 >= protocol['arrival_hold_s']),
        timeout_respected=elapsed <= protocol['timeout_s'] + FRAME_DT + 1e-7,
        vertical_domain=bool(np.all(p[:, 2] - radius >= z_min) and np.all(p[:, 2] + radius <= z_max)),
        braking_parameter_matches=effective.get('brake_accel_limit') == protocol['plant_accel_mps2'],
        vertical_parameter_matches=effective.get('max_vertical_speed') == protocol['max_vertical_speed_mps'],
        delay_parameter_matches=effective.get('paper_pipeline_delay') == protocol['pipeline_delay_s'],
        world_execution=environment.get('ISAAC_COMMAND_FRAME') == 'world',
    )
    # Old protocols had no planner floor: missing fields retain disabled/zero
    # semantics. New prepared protocols explicitly bind both actual parameters.
    floor_fields = ('free_space_floor_enabled', 'free_space_floor_z')
    explicit_floor = protocol.get('version') in SENSOR_PROTOCOL_VERSIONS or any(key in protocol for key in floor_fields)
    floor_enabled = protocol.get('free_space_floor_enabled', False)
    floor_z = protocol.get('free_space_floor_z', 0.)
    actual_floor_enabled = effective.get('paper_free_space_floor_enabled', False)
    actual_floor_z = effective.get('paper_free_space_floor_z', 0.)
    checks['free_space_floor_protocol_complete'] = (
        not explicit_floor or all(key in protocol for key in floor_fields))
    checks['free_space_floor_parameters_present'] = (not explicit_floor or all(
        key in effective for key in ('paper_free_space_floor_enabled', 'paper_free_space_floor_z')))
    checks['free_space_floor_enabled_parameter_matches'] = (
        type(floor_enabled) is bool and type(actual_floor_enabled) is bool and
        floor_enabled == actual_floor_enabled)
    checks['free_space_floor_z_parameter_matches'] = (
        type(floor_z) in (int, float) and math.isfinite(floor_z) and
        type(actual_floor_z) in (int, float) and math.isfinite(actual_floor_z) and
        floor_z == actual_floor_z)
    if 'emergency_braking_certificate' in protocol:
        checks['emergency_parameter_matches'] = effective.get('paper_emergency_braking_certificate') == protocol['emergency_braking_certificate']
    if 'goal_proposal_cache' in protocol:
        checks['cache_parameter_matches'] = (effective.get('paper_goal_proposal_cache') == protocol['goal_proposal_cache'] and
            effective.get('paper_goal_proposal_cache_ttl') == protocol['goal_proposal_cache_ttl_s'] and
            effective.get('paper_goal_speed_governor') == protocol['goal_speed_governor'])
    if 'planning_horizon_s' in protocol:
        checks['planning_horizon_parameter_matches'] = effective.get('planning_horizon') == protocol['planning_horizon_s']
    if 'smooth_speed' in protocol:
        checks['smooth_speed_parameter_matches'] = effective.get('paper_smooth_speed') == protocol['smooth_speed']
    if 'perception_parameters' in protocol:
        checks['perception_parameters_match'] = all(
            ((type(effective.get(key)) is bool) if type(value) is bool else
             (type(effective.get(key)) in (int, float))) and effective.get(key) == value
            for key, value in protocol['perception_parameters'].items())
    if protocol.get('version') in ('ego2p1_sim4_v4', *SENSOR_PROTOCOL_VERSIONS):
        manifest = json.loads(result.with_suffix('.manifest.json').read_text())
        checks['restart_parameter_matches'] = effective.get('paper_restart_clearance') == protocol['restart_clearance_m']
        checks['proposal_tau_parameter_matches'] = effective.get('paper_proposal_response_time') == protocol['plant_tau_s']
        checks['frozen_execution_matches'] = bool(manifest.get('complete') and
            manifest.get('execution_matches_frozen') and not manifest.get('changed_files_during_run') and
            manifest['prepared_protocol_sha256'] == value_sha256(protocol) and
            manifest['parameters_file_sha256'] == hashlib.sha256(result.with_suffix('.parameters.json').read_bytes()).hexdigest())
    sensor_details = {}
    if protocol.get('version') in SENSOR_PROTOCOL_VERSIONS:
        sensor_checks, sensor_details = audit_sensor_evidence(result, protocol, manifest, effective)
        checks.update(sensor_checks)
    checks.update(working_bounds_checks(protocol, effective, p, radius))
    high_s, high_longest = high_speed_duration(t, speed, 3.8)
    cap_s, cap_longest = high_speed_duration(t, speed, .95 * protocol['max_speed_mps'])
    # Cap attainment is a separate requirement from collision-free arrival.
    cap_attained = bool(float(speed.max()) >= .95 * protocol['max_speed_mps'] and cap_s >= 1.)
    safe_arrival = bool(legacy_safe and all(checks.values()))
    report.update(audit_version=('sim4_v6' if protocol.get('version') == SENSOR_PROTOCOL_VERSION else
        'sim4_v5' if protocol.get('version') in SENSOR_PROTOCOL_VERSIONS else 'sim4_v4'), legacy_safe_arrival=legacy_safe,
        **sensor_details,
        safe_arrival=safe_arrival, accepted=bool(safe_arrival and cap_attained),
        additional_checks=checks, rejection_reasons=[name for name, ok in checks.items() if not ok],
        cap_attained=cap_attained, time_at_95pct_cap_s=cap_s,
        longest_at_95pct_cap_s=cap_longest, time_ge_3p8_s=high_s,
        longest_ge_3p8_s=high_longest, independent_arrival_hold_s=hold,
        position_integration_max_error_m=integration_error,
        requested_accel_mps2=protocol['plant_accel_mps2'], plant_tau_s=protocol['plant_tau_s'],
        exploration_stall_s=data.get('exploration_stall_s', 0.),
        body_radius_m=radius, actual_route_average_mps=report['path_m'] / elapsed if elapsed > 0 else 0.,
        mean_sample_speed_mps=float(np.mean(speed[t >= protocol['warmup_s']])),
        actual_final_distance_m=actual_distance)
    if not legacy_safe:
        report['rejection_reasons'].insert(0, 'legacy_geometry_limits_or_process')
    if not cap_attained:
        report['rejection_reasons'].append('requested_cap_not_attained')
    write_json(result.with_suffix('.audit.json'), report)
    return report


def run(args):
    goal_headless = os.environ.get('EGO2P6_GOAL_HEADLESS', '1')
    if goal_headless not in ('0', '1'):
        raise ValueError('EGO2P6_GOAL_HEADLESS must be 0 or 1')
    goal_rviz = os.environ.get('EGO2P6_GOAL_RVIZ', 'true' if goal_headless == '0' else 'false').lower()
    if goal_rviz not in ('true', 'false'):
        raise ValueError('EGO2P6_GOAL_RVIZ must be true or false')
    folder = args.out.resolve() / args.label
    folder.mkdir(parents=True, exist_ok=True)
    summary = folder / f'summary_r{args.repeat_start:02d}_n{args.repeats:02d}.json'
    if summary.exists():
        raise RuntimeError('refusing to overwrite ' + str(summary))
    for repeat in range(args.repeat_start, args.repeat_start + args.repeats):
        for map_name in args.maps:
            stem = f'{map_name}_v{args.speed:g}_a{args.accel:g}_r{repeat:02d}'
            if list(folder.glob(stem + '.*')) or (folder / (stem + '_replays')).exists():
                raise RuntimeError('refusing to overwrite ' + str(folder / stem))
    reports = []
    for repeat in range(args.repeat_start, args.repeat_start + args.repeats):
        for map_name in args.maps:
            scene, shape, _ = MAPS[map_name]
            stem = f'{map_name}_v{args.speed:g}_a{args.accel:g}_r{repeat:02d}'
            result = folder / (stem + '.json')
            if list(folder.glob(stem + '.*')) or (folder / (stem + '_replays')).exists():
                raise RuntimeError('refusing to overwrite ' + str(result))
            protocol = protocol_for(args, map_name)
            protocol_path = result.with_suffix('.protocol.json')
            write_json(protocol_path, protocol)
            environment = dict(os.environ, ISAAC_MANUAL_INPUT_MODE='goal', ISAAC_HEADLESS=goal_headless,
                FOV_GVF_RVIZ=goal_rviz, FOV_GVF_BENCHMARK_ALGORITHM='ego2p6',
                FOV_GVF_SCENE_MODE='cloud', FOV_GVF_SCENE=str(scene),
                FOV_GVF_ESDF_OCCUPANCY=str(scene.with_name('occupancy.bin')),
                ISAAC_GOAL_PROTOCOL=str(protocol_path), ISAAC_BENCHMARK_RESULT=str(result),
                ISAAC_ACCEPTANCE_TRACE=str(result.with_suffix('.csv')),
                FOV_GVF_RUN_ID=f'ego2p6_sim4_{args.label}_{stem}',
                FOV_GVF_PERFORMANCE_LOG=str(folder / 'PERFORMANCE.md'),
                FOV_GVF_REPLAY_DIR=(str(folder / (stem + '_replays'))
                    if os.environ.get('EGO2P6_RECORD_REPLAYS', '0') == '1' else ''),
                FOV_GVF_EMERGENCY_CERTIFICATE='1' if args.emergency else '0',
                FOV_GVF_COMMAND_FRAME='world', ISAAC_COMMAND_FRAME='world',
                ISAAC_MANUAL_TIMEOUT='0', ISAAC_EXPLORATION_STALL_S=str(args.abort_stall),
                ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1')
            if args.camera_hfov is not None:
                environment['ISAAC_DEPTH_HFOV_DEG'] = str(args.camera_hfov)
            if args.camera_offset is not None:
                environment['ISAAC_DEPTH_CAMERA_OFFSET_M'] = str(args.camera_offset)
            command = ['bash', str(ROOT / 'scripts/run_isaac_fov_gvf_navigation.sh')]
            manifest = freeze_trial_evidence(result, protocol, command, environment)
            started = time.perf_counter()
            print('START', args.label, stem, flush=True)
            with result.with_suffix('.log').open('x') as stream:
                child = subprocess.run(command,
                    cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT)
            process_wall_s = time.perf_counter() - started
            finish_trial_evidence(result, manifest)
            data = json.loads(result.read_text()) if result.exists() else dict(status='RUNTIME_ERROR')
            data.update(process_exit_code=child.returncode, process_wall_s=process_wall_s,
                driver_requested_environment={key: environment[key] for key in
                    ('FOV_GVF_COMMAND_FRAME', 'ISAAC_COMMAND_FRAME', 'FOV_GVF_EMERGENCY_CERTIFICATE', 'ISAAC_DEPTH_HFOV_DEG', 'ISAAC_DEPTH_CAMERA_OFFSET_M') if key in environment})
            write_json(result, data)
            try:
                report = audit_trial(result, scene, shape)
            except Exception as error:
                report = dict(audit_version='sim4_v6', result=str(result.relative_to(ROOT)),
                    status=data['status'], safe_arrival=False, cap_attained=False, accepted=False,
                    audit_error=str(error), rejection_reasons=['audit_error'])
                write_json(result.with_suffix('.audit.json'), report)
            reports.append(report)
            # Unique repeat-indexed summaries allow further trials in this batch without overwriting evidence.
            complete = len(reports) == len(args.maps) * args.repeats
            write_json(summary, dict(all_accepted=bool(complete and all(r['accepted'] for r in reports)),
                complete=complete, completed=len(reports), requested=len(args.maps) * args.repeats, reports=reports))
            print('END', stem, json.dumps(report, allow_nan=False), flush=True)
            if not report['accepted'] and args.stop_on_failure:
                return 1
    return 0 if reports and all(report['accepted'] for report in reports) else 1


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('label', help='unique batch name, e.g. accel4_explore')
    cli.add_argument('--out', type=Path, default=OUTPUT_ROOT, help='output root inside docs/performance/p6_speed_20261019')
    cli.add_argument('--maps', nargs='+', choices=MAPS, default=['original'])
    cli.add_argument('--repeats', type=int, default=1)
    cli.add_argument('--repeat-start', type=int, default=1)
    cli.add_argument('--accel', type=float, default=16.)
    cli.add_argument('--speed', type=float, default=4.)
    cli.add_argument('--camera-hfov', type=float, default=None,
        help='override four depth cameras horizontal FOV in degrees; unset preserves native 90 degrees')
    cli.add_argument('--camera-offset', type=float, default=None,
        help='radial depth-camera mount distance in meters; unset preserves native .22 meters')
    cli.add_argument('--tau', type=float, default=TAU, help='plant and certificate response time, at least .025 seconds')
    cli.add_argument('--vertical-speed', type=float, default=1., help='vertical speed limit in m/s')
    cli.add_argument('--pipeline-delay', type=float, default=0., help='modeled transport delay before new command, seconds; immediate simulation by default')
    cli.add_argument('--floor-z', type=float, default=0.,
        help='planner free-space floor height; independent geometry audit keeps its fixed z=0 plane')
    cli.add_argument('--restart-clearance', type=float, default=0., help='extra certified stopping corridor room, in meters')
    cli.add_argument('--depth', type=float, default=10.)
    cli.add_argument('--horizon', type=float, default=1., help='planner horizon in seconds; distinct from certified motion/braking proof')
    cli.add_argument('--smooth-speed', action=argparse.BooleanOptionalAction, default=False)
    cli.add_argument('--cache', action=argparse.BooleanOptionalAction, default=True)
    cli.add_argument('--cache-ttl', type=float, default=.15)
    cli.add_argument('--emergency', action=argparse.BooleanOptionalAction, default=True)
    cli.add_argument('--abort-stall', type=float, default=0., help='exploration only; formal runs use 0')
    cli.add_argument('--stop-on-failure', action='store_true')
    return cli


def validate_args(cli, args):
    try:
        sensor_environment = dict(os.environ)
        if args.camera_hfov is not None:
            sensor_environment['ISAAC_DEPTH_HFOV_DEG'] = str(args.camera_hfov)
        requested_hfov(sensor_environment)
        if args.camera_offset is not None:
            sensor_environment['ISAAC_DEPTH_CAMERA_OFFSET_M'] = str(args.camera_offset)
        requested_camera_offset(sensor_environment)
    except (TypeError, ValueError) as error:
        cli.error(str(error))
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.label):
        cli.error('label must be a simple batch name')
    if not args.out.resolve().is_relative_to(OUTPUT_ROOT.resolve()):
        cli.error('--out must remain inside ' + str(OUTPUT_ROOT))
    if args.repeats < 1 or args.repeat_start < 1 or len(args.maps) != len(set(args.maps)):
        cli.error('positive repeat counts and unique maps required')
    if any(not math.isfinite(value) or value <= 0 for value in (args.accel, args.speed, args.depth, args.cache_ttl, args.horizon, args.tau, args.vertical_speed)):
        cli.error('acceleration, speed, depth, cache TTL, horizon, tau and vertical speed must be positive and finite')
    if not math.isfinite(args.floor_z):
        cli.error('planner floor height must be finite')
    if not math.isfinite(args.pipeline_delay) or args.pipeline_delay < 0.:
        cli.error('pipeline delay must be finite and nonnegative')
    if args.tau < .025 or not math.isfinite(args.restart_clearance) or args.restart_clearance < 0.:
        cli.error('tau >= .025 and finite nonnegative restart clearance required')
    if args.accel < .1 or args.cache_ttl > 10. or not math.isfinite(args.abort_stall) or args.abort_stall < 0.:
        cli.error('accel >= .1, cache TTL <= 10 and finite nonnegative abort stall required')


def main():
    cli = parser()
    args = cli.parse_args()
    validate_args(cli, args)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
