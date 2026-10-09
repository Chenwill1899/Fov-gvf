#!/usr/bin/env python3
"""Verify a completed frozen comparison against its source/binary manifests."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat

p = argparse.ArgumentParser()
p.add_argument('label')
p.add_argument('--version')
a = p.parse_args()
a.version = a.version or a.label.split('_')[-1]
evidence = Path(__file__).resolve().parent
project = evidence.parents[1]
baseline = project.parent / 'EGO1P2'
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = json.loads((evidence / 'source_manifest.json').read_text())
changes = []
for name, expected in manifest.items():
    path = baseline / name
    try:
        actual_mode = path.lstat().st_mode
        if actual_mode != expected['mode']:
            changes.append([name, 'mode'])
        if 'sha256' in expected and sha(path) != expected['sha256']:
            changes.append([name, 'content'])
        if stat.S_ISLNK(actual_mode):
            target = expected.get('link', expected.get('target'))
            if target is not None and os.readlink(path) != target:
                changes.append([name, 'symlink'])
    except FileNotFoundError:
        changes.append([name, 'missing'])
extra = sorted(str(path.relative_to(baseline)) for path in baseline.rglob('*')
               if str(path.relative_to(baseline)) not in manifest)
frozen = evidence / (a.version + '_frozen')
sources = json.loads((frozen / 'sources_sha256.json').read_text())
binaries = json.loads((frozen / 'binary_sha256.json').read_text())
source_drift = [name for name, digest in sources.items() if sha(project / name) != digest]
binary_drift = [name for name, digest in binaries.items()
                if sha(frozen / name) != digest or sha(Path('/tmp/fov_gvf_ego1p3_isaac_install/pc_gvf/lib/pc_gvf') / name) != digest]
protected = json.loads((evidence / 'safety_and_platform_integrity_v12.json').read_text())
intentional_path = frozen / 'intentional_core_changes.json'
intentional = json.loads(intentional_path.read_text()) if intentional_path.exists() else {}
assert not (set(intentional) - set(protected)), 'unrecognized protected-file exception'
protected_drift = []
for name, row in protected.items():
    expected = intentional.get(name, {}).get('after', row['sha256'])
    if sha(baseline / name) != row['sha256'] or sha(project / name) != expected:
        protected_drift.append(name)
    if name in intentional and (intentional[name]['before'] != row['sha256'] or sources[name] != expected):
        protected_drift.append(name + ': invalid explicit change record')
manual = json.loads((evidence / (a.label + '_analysis.json')).read_text())
goalroot = project / 'performance/navigation_benchmark_20260929'
goals = json.loads((goalroot / ('omni_' + a.label + '_analysis.json')).read_text())
trial_checks = {}
trace_hashes = {}
controller_hashes = {}
scene_hash = sha(project / 'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd')
simulator_hash = sha(project / 'scripts/isaac/run_fov_gvf_navigation.py')
for run in manual['runs']:
    name = run['name']
    meta = json.loads((evidence / (name + '_process.json')).read_text())
    expected_binary = binaries['depth_angular_controller'] if run['algorithm'] == 'ego1p3' else sha(Path('/tmp/fov_gvf_ego1p2_isaac_install/pc_gvf/lib/pc_gvf/depth_angular_controller'))
    expected_launch = sha((project if run['algorithm'] == 'ego1p3' else baseline) / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py')
    trial_checks[name] = meta['returncode'] == 0 and meta['controller_sha256'] == expected_binary and meta['launch_sha256'] == expected_launch and meta['scene_sha256'] == scene_hash and meta['shared_simulator_sha256'] == simulator_hash
    trace_hashes.setdefault(run['case'], set()).add(meta['trace_sha256'])
    controller_hashes.setdefault(run['algorithm'], set()).add(meta['controller_sha256'])
goal_protocols = set()
for run in goals['trials']:
    name = run['run_id']
    meta = json.loads((goalroot / (name + '.parameters.json')).read_text())
    expected_binary = binaries['depth_angular_controller'] if run['algorithm'] == 'ego1p3' else sha(Path('/tmp/fov_gvf_ego1p2_isaac_install/pc_gvf/lib/pc_gvf/depth_angular_controller'))
    expected_launch = sha((project if run['algorithm'] == 'ego1p3' else baseline) / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py')
    common_ok = all(sha(project / file) == digest for file, digest in meta['common_source_sha256'].items())
    trial_checks[name] = run['process_exit_code'] == 0 and meta['executable_sha256'] == expected_binary and meta['launch_sha256'] == expected_launch and common_ok
    goal_protocols.add(json.dumps(run['protocol'], sort_keys=True))
report = {
    'label': a.label, 'frozen_version': a.version,
    'baseline_manifest_entries': len(manifest),
    'baseline_files_hashed': sum('sha256' in row for row in manifest.values()),
    'baseline_changes': changes, 'baseline_extra_entries': extra,
    'frozen_source_count': len(sources), 'frozen_source_drift': source_drift,
    'frozen_binary_drift': binary_drift, 'protected_file_count': len(protected),
    'protected_source_drift': protected_drift,
    'unchanged_protected_file_count': len(protected)-len(intentional),
    'intentional_proof_changes': intentional,
    'trial_metadata_checks': trial_checks,
    'input_sha256_by_case': {k: sorted(v) for k, v in trace_hashes.items()},
    'controller_sha256_by_algorithm': {k: sorted(v) for k, v in controller_hashes.items()},
    'identical_goal_protocol': len(goal_protocols) == 1,
}
report['integrity_pass'] = not (changes or extra or source_drift or binary_drift or protected_drift) and all(trial_checks.values()) and all(len(v) == 1 for v in trace_hashes.values()) and len(goal_protocols) == 1 and len(trial_checks) == 24
(evidence / (a.label + '_integrity.json')).write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['integrity_pass'] else 1)
