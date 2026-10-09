#!/usr/bin/env python3
"""Confirm a selected feature subset against baseline through the installed default routes."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time

PROFILE_ORDER = ('baseline', 'combined')
FEATURE_CHOICES = ('uncertainty', 'dynamic', 'memory', 'shared')
FEATURE_KEYS = dict(uncertainty='FOV_GVF_DEPTH_UNCERTAINTY', dynamic='FOV_GVF_DYNAMIC_OBSTACLES',
    incremental='FOV_GVF_INCREMENTAL_FIELD', memory='FOV_GVF_SPHERICAL_MEMORY',
    operator='FOV_GVF_OPERATOR_ASSISTANCE', shared='FOV_GVF_SHARED_OBSTACLES')



def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    parser.add_argument('--case', choices=('goal', 'jitter'), required=True)
    parser.add_argument('--profiles', nargs='+', choices=PROFILE_ORDER, default=list(PROFILE_ORDER))
    parser.add_argument('--features', nargs='+', choices=FEATURE_CHOICES, default=list(FEATURE_CHOICES),
                        help='features enabled only in combined; incremental/operator remain off')
    parser.add_argument('--repeats', type=int, default=1)
    args = parser.parse_args()
    if (args.repeats < 1 or len(set(args.profiles)) != len(args.profiles)
            or len(set(args.features)) != len(args.features)):
        parser.error('repeats must be positive; profiles and features must be unique')
    profile_features = {'baseline': set(), 'combined': set(args.features)}
    if Path(args.label).name != args.label or args.label in ('.', '..'):
        parser.error('label must be one new directory name')
    profiles = [p for p in PROFILE_ORDER if p in args.profiles]
    orders = [profiles if r % 2 == 0 else list(reversed(profiles)) for r in range(args.repeats)]
    root = Path(__file__).resolve().parents[1]
    install = Path('/tmp/fov_gvf_ego1p5_isaac_install')
    controller = (install / 'pc_gvf/lib/pc_gvf' /
                  ('depth_angular_controller_goal' if args.case == 'goal' else 'depth_angular_controller')).resolve(strict=True)
    if not controller.is_file() or not os.access(controller, os.X_OK):
        parser.error('default controller must be executable')
    output = root / 'performance/enable_gains_20261008/closed_loop' / args.label
    if output.exists():
        parser.error('refusing to overwrite existing batch output')
    launch = root / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    simulator = root / 'scripts/isaac/run_fov_gvf_navigation.py'
    scene = root / 'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'
    fixture = root / 'src/pc_gvf/test/fixtures/paper/isaac_operator_jitter_trace.json'
    motion_auditor = root / 'performance/retain_gains_20261008/assessment/audit_motion_limits.py'
    common_names = ['scripts/run_isaac_fov_gvf_navigation.sh', 'scripts/isaac/run_fov_gvf_navigation.py',
        'scripts/isaac/navigation_benchmark.py', 'scripts/isaac/manual_control_math.py',
        'src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py',
        'performance/navigation_benchmark_20260929/protocol.json']
    runtime_files = [launch, scene, fixture, motion_auditor, Path(__file__).resolve(),
        root / 'src/pc_gvf/launch/navigation_benchmark.launch.py',
        root / 'scenes/ego_swarm_cloud/occupancy.bin',
        root / 'tools/analyze_navigation_benchmark.py', root / 'tools/analyze_omni_comparison.py',
        root / 'tools/analyze_envelope_trace.py', *[root / name for name in common_names],
        install / 'pc_gvf/share/pc_gvf/launch/isaac_cloud_navigation.launch.py',
        install / 'pc_gvf/share/pc_gvf/launch/navigation_benchmark.launch.py']
    frozen_files = {str(p): sha(p) for p in runtime_files}
    for filename in ('isaac_cloud_navigation.launch.py', 'navigation_benchmark.launch.py'):
        if sha(root / 'src/pc_gvf/launch' / filename) != sha(install / 'pc_gvf/share/pc_gvf/launch' / filename):
            raise SystemExit('installed/source launch differs: ' + filename)
    frozen_binary = sha(controller)
    spec = importlib.util.spec_from_file_location('enabled_feature_motion_audit', motion_auditor)
    audit_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(audit_module)
    manifest = dict(case=args.case, variants={p: str(controller) for p in profiles},
        feature_flags='explicit per-profile; incremental/operator always off', twist_source='callback',
        combined_features=[feature for feature in FEATURE_CHOICES if feature in args.features],
        routing='installed default goal' if args.case == 'goal' else 'installed default manual',
        binary_sha256={p: frozen_binary for p in profiles}, file_sha256=frozen_files,
        order_by_repeat=orders, runs=[], complete=False)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'manifest.json', manifest)
    for repeat, order in enumerate(orders, 1):
        for profile in order:
            if sha(controller) != frozen_binary or not all(sha(path) == value for path, value in frozen_files.items()):
                raise SystemExit('frozen binary/source changed before run')
            flags = {key: str(int(feature in profile_features[profile])) for feature, key in FEATURE_KEYS.items()}
            name = f'{args.label}_{args.case}_{profile}_{repeat}'
            clean = {key: value for key, value in os.environ.items() if not key.startswith(('ISAAC_', 'FOV_GVF_'))}
            env = dict(clean, **flags, PYTHONNOUSERSITE='1', FOV_GVF_INSTALL=str(install),
                ISAAC_HEADLESS='1', FOV_GVF_RVIZ='false', FOV_GVF_SCENE_MODE='cloud',
                ISAAC_TWIST_SAMPLE_SOURCE='callback', ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1',
                FOV_GVF_PERFORMANCE_LOG=str(output / (name + '_performance.md')),
                FOV_GVF_REPLAY_DIR=str(output / (name + '_replays')))
            # No executable override: the effective case selects the normal route.
            meta = dict(algorithm='p5_enabled_' + profile, case=args.case, variant=profile, profile=profile,
                repeat=repeat, flags=flags, controller_sha256=frozen_binary, controller_executable=str(controller),
                controller_selection='default_' + args.case, launch_sha256=sha(launch),
                shared_simulator_sha256=sha(simulator), scene_sha256=sha(scene))
            if args.case == 'goal':
                label = f'{args.label}_{profile}'
                navigation = root / 'performance/navigation_benchmark_20260929'
                run_id = f'{label}_ego1p5_{repeat}'
                result = navigation / (run_id + '.json')
                if any(result.with_suffix(suffix).exists() for suffix in ('.json', '.csv', '.parameters.json')):
                    raise SystemExit('refusing to overwrite existing goal artifacts')
                env.update(ISAAC_MANUAL_INPUT_MODE='goal', FOV_GVF_BENCHMARK_ALGORITHM='ego1p5',
                    ISAAC_GOAL_PROTOCOL=str(navigation / 'protocol.json'), ISAAC_BENCHMARK_RESULT=str(result),
                    ISAAC_ACCEPTANCE_TRACE=str(result.with_suffix('.csv')), FOV_GVF_RUN_ID=run_id,
                    ISAAC_MANUAL_TIMEOUT='0', ISAAC_EXPLORATION_STALL_S='0')
                runtime_file = result.with_suffix('.parameters.json')
            else:
                env.update(ISAAC_MANUAL_INPUT_MODE='trace', ISAAC_INTENT_TRACE=str(fixture),
                    ISAAC_ACCEPTANCE_TRACE=str(output / (name + '.csv')), ISAAC_MANUAL_TIMEOUT='34',
                    FOV_GVF_RUN_ID=name, FOV_GVF_RUNTIME_MANIFEST=str(output / (name + '_runtime.json')))
                runtime_file = Path(env['FOV_GVF_RUNTIME_MANIFEST'])
                meta.update(duration_s=34, trace_sha256=sha(fixture))
            if 'FOV_GVF_CONTROLLER_EXECUTABLE' in env:
                raise SystemExit('unexpected controller override')
            meta.update(csv=env['ISAAC_ACCEPTANCE_TRACE'], performance=env['FOV_GVF_PERFORMANCE_LOG'],
                log=str(output / (name + '.log')),
                experiment_environment={k: v for k, v in env.items() if k.startswith(('ISAAC_', 'FOV_GVF_'))})
            print('START', name, flush=True)
            start = time.monotonic()
            with Path(meta['log']).open('x') as log:
                run = subprocess.run(['bash', str(root / 'scripts/run_isaac_fov_gvf_navigation.sh')],
                    cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            meta.update(returncode=run.returncode, process_wall_s=time.monotonic() - start)
            audit = subprocess.run(['/usr/bin/python3', str(root / 'tools/analyze_envelope_trace.py'), meta['csv']],
                cwd=root, env=env, text=True, capture_output=True)
            (output / (name + '_audit.json')).write_text(audit.stdout)
            (output / (name + '_audit.stderr')).write_text(audit.stderr)
            meta['audit_returncode'] = audit.returncode
            motion = audit_module.audit_csv(meta['csv'])
            write_json(output / (name + '_motion_limits.json'), motion)
            meta['motion_limits_pass'] = motion['status'] == 'PASS'
            errors = []
            try:
                audited = json.loads(audit.stdout)
                meta['audit_geometry_safe'] = (audited['swept_sphere_overlap_segments'] == 0 and
                    audited['external_collision_blocks'] == 0 and
                    audited['scene_sha256'] == frozen_files[str(scene)] and
                    audited['occupancy_sha256'] == frozen_files[str(root / 'scenes/ego_swarm_cloud/occupancy.bin')])
                runtime = json.loads(runtime_file.read_text())
                meta['runtime_manifest_sha256'] = sha(runtime_file)
                meta['runtime_launch_verified'] = runtime['launch_sha256'] == frozen_files[str(launch)]
                meta['runtime_common_sources_verified'] = all(sha(path) == value for path, value in frozen_files.items())
                if args.case == 'goal':
                    actual_common = runtime.get('common_source_sha256', {})
                    meta['runtime_common_sources_verified'] &= all(actual_common.get(n) == frozen_files[str(root / n)] for n in common_names)
                    expected_flags = dict(depth_uncertainty_enabled=flags[FEATURE_KEYS['uncertainty']] == '1',
                        paper_dynamic_obstacles=flags[FEATURE_KEYS['dynamic']] == '1',
                        paper_incremental_field=False, paper_spherical_memory=flags[FEATURE_KEYS['memory']] == '1',
                        operator_assistance=False, paper_shared_obstacles=flags[FEATURE_KEYS['shared']] == '1')
                    meta['runtime_flags_verified'] = all(runtime['effective'].get(k) is v for k, v in expected_flags.items())
                else:
                    # Manual runtime attests binary/launch; explicit flags and frozen launch bind selection.
                    meta['runtime_flags_verified'] = all(env[k] == v for k, v in flags.items())
                meta['runtime_binary_verified'] = (runtime['executable_sha256'] == frozen_binary and
                    Path(runtime['executable']).resolve() == controller and sha(controller) == frozen_binary)
                if args.case == 'goal':
                    data = json.loads(result.read_text())
                    data.update(process_exit_code=run.returncode, process_wall_s=meta['process_wall_s'])
                    write_json(result, data)
                    meta.update(result=data, result_path=str(result))
                    meta['safe_arrival'] = data['status'] == 'ARRIVED' and meta['audit_geometry_safe']
                    meta['audit_safe'] = meta['safe_arrival']
                    meta['expected_completion'] = data['status'] in ('ARRIVED', 'TIMEOUT')
                    nav = subprocess.run(['/usr/bin/python3', str(root / 'tools/analyze_navigation_benchmark.py'), label],
                        cwd=root, env=env, text=True, capture_output=True)
                    (output / (name + '_navigation_audit.stdout')).write_text(nav.stdout)
                    (output / (name + '_navigation_audit.stderr')).write_text(nav.stderr)
                    meta['navigation_audit_returncode'] = nav.returncode
                    analysis = json.loads((navigation / f'{label}_analysis.json').read_text())
                    trial = next(row for row in analysis['trials'] if row['run_id'] == run_id)
                    meta['navigation_audit_verified'] = (nav.returncode == 0 and trial['status'] == data['status'] and
                        trial['swept_overlap_segments'] == 0 and trial['collision_blocks'] == 0 and
                        bool(trial['safe_arrival']) == bool(meta['safe_arrival']))
                else:
                    meta['audit_safe'] = meta['audit_geometry_safe'] and abs(audited['duration_s'] - 34) < .2
                    meta['expected_completion'] = abs(audited['duration_s'] - 34) < .2
            except (KeyError, ValueError, TypeError, OSError, StopIteration) as error:
                errors.append(type(error).__name__ + ': ' + str(error))
            meta['verification_errors'] = errors
            passed = (run.returncode == 0 and audit.returncode == 0 and not errors and
                meta.get('audit_geometry_safe') and meta.get('motion_limits_pass') and meta.get('expected_completion') and
                meta.get('runtime_binary_verified') and meta.get('runtime_launch_verified') and
                meta.get('runtime_common_sources_verified') and meta.get('runtime_flags_verified') and
                (args.case != 'goal' or meta.get('navigation_audit_verified')))
            meta['hard_checks_pass'] = bool(passed)
            write_json(output / (name + '_process.json'), meta)
            manifest['runs'].append(meta); write_json(output / 'manifest.json', manifest)
            print('END', name, meta.get('result', {}).get('status', ''), 'HARD_PASS' if passed else 'HARD_FAIL', flush=True)
            if not passed:
                raise SystemExit('hard safety/runtime failure preserved; inspect before continuing')
    if sha(controller) != frozen_binary or not all(sha(path) == value for path, value in frozen_files.items()):
        raise SystemExit('frozen binary/source changed at batch end')
    manifest['complete'] = True  # Batch complete is not a claim that every goal arrived.
    manifest['goal_timeout_count'] = sum(r.get('result', {}).get('status') == 'TIMEOUT' for r in manifest['runs'])
    write_json(output / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
