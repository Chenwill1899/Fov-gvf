#!/usr/bin/env python3
"""Compare frozen baseline with the installed default goal controller; all six features remain off."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    parser.add_argument('--case', choices=['goal'], required=True)
    parser.add_argument('--variants', nargs='+', required=True,
                        help='exactly baseline=/absolute/frozen/controller and v4=/absolute/installed/default/goal/controller')
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error('at least three repetitions are required')
    install = Path('/tmp/fov_gvf_ego1p5_isaac_install')
    default_goal = install / 'pc_gvf/lib/pc_gvf/depth_angular_controller_goal'
    binaries = {}
    for item in args.variants:
        key, separator, value = item.partition('=')
        if not separator or key not in {'baseline', 'v4'} or key in binaries:
            parser.error('invalid or duplicate variant: ' + item)
        path = Path(value)
        if not path.is_absolute():
            parser.error('controller paths must be absolute: ' + item)
        try:
            path = path.resolve(strict=True)
        except OSError as error:
            parser.error(str(error))
        if not path.is_file() or not os.access(path, os.X_OK):
            parser.error('controller must be an executable file: ' + item)
        binaries[key] = path
    if set(binaries) != {'baseline', 'v4'}:
        parser.error('provide exactly baseline and v4')
    try:
        expected_goal = default_goal.resolve(strict=True)
    except OSError as error:
        parser.error(str(error))
    if binaries['v4'] != expected_goal:
        parser.error('v4 must be the installed default goal executable: ' + str(default_goal))
    variants = ['baseline', 'v4']
    order_indices = [[0, 1], [1, 0]]
    orders = [[variants[i] for i in order_indices[r % len(order_indices)]]
              for r in range(args.repeats)]
    root = Path(__file__).resolve().parents[1]
    output = root / 'performance/retain_gains_20261008/closed_loop' / args.label
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    frozen = {key: sha(path) for key, path in binaries.items()}
    launch = root / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    output.mkdir(parents=True, exist_ok=False)
    simulator = root / 'scripts/isaac/run_fov_gvf_navigation.py'
    scene = root / 'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'
    runtime_files = [launch, simulator, scene,
        root / 'src/pc_gvf/launch/navigation_benchmark.launch.py',
        root / 'scripts/run_isaac_fov_gvf_navigation.sh',
        root / 'scripts/isaac/navigation_benchmark.py',
        root / 'scripts/isaac/manual_control_math.py',
        root / 'src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py',
        root / 'performance/navigation_benchmark_20260929/protocol.json',
        root / 'scenes/ego_swarm_cloud/occupancy.bin', Path(__file__).resolve(),
        root / 'tools/analyze_navigation_benchmark.py', root / 'tools/analyze_omni_comparison.py',
        root / 'tools/analyze_envelope_trace.py',
        install / 'pc_gvf/share/pc_gvf/launch/isaac_cloud_navigation.launch.py',
        install / 'pc_gvf/share/pc_gvf/launch/navigation_benchmark.launch.py']
    frozen_files = {str(path): sha(path) for path in runtime_files}
    features = ['uncertainty', 'dynamic', 'incremental', 'memory', 'operator', 'shared']
    env_keys = dict(zip(features, [
        'FOV_GVF_DEPTH_UNCERTAINTY', 'FOV_GVF_DYNAMIC_OBSTACLES',
        'FOV_GVF_INCREMENTAL_FIELD', 'FOV_GVF_SPHERICAL_MEMORY',
        'FOV_GVF_OPERATOR_ASSISTANCE', 'FOV_GVF_SHARED_OBSTACLES']))
    results = []
    manifest = dict(case=args.case, variants={k: str(v) for k, v in binaries.items()},
                    feature_flags='all six off', twist_source='callback',
                    routing={'baseline': 'explicit frozen executable', 'v4': 'installed default goal executable'},
                    install=str(install), default_goal_executable=str(default_goal),
                    binary_sha256=frozen, file_sha256=frozen_files,
                    order_by_repeat=orders, runs=results, complete=False)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    for repeat, order in enumerate(orders, 1):
        for variant in order:
            assert all(sha(path) == frozen[key] for key, path in binaries.items()), 'binary changed'
            assert all(sha(Path(path)) == digest for path, digest in frozen_files.items()), 'runtime changed'
            chosen = []
            flags = {key: str(int(feature in chosen)) for feature, key in env_keys.items()}
            name = f'{args.label}_{args.case}_{variant}_{repeat}'
            clean_environment = {key: value for key, value in os.environ.items()
                                 if not key.startswith(('ISAAC_', 'FOV_GVF_'))}
            env = dict(clean_environment, **flags, PYTHONNOUSERSITE='1',
                       FOV_GVF_INSTALL=str(install),
                       ISAAC_HEADLESS='1', FOV_GVF_RVIZ='false', FOV_GVF_SCENE_MODE='cloud',
                       ISAAC_TWIST_SAMPLE_SOURCE='callback',
                       ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1')
            if variant == 'baseline':
                env['FOV_GVF_CONTROLLER_EXECUTABLE'] = str(binaries[variant])
            # v4 deliberately leaves the override absent: verify default launch routing.
            meta = dict(algorithm='p5_retained_' + variant, case=args.case, variant=variant,
                        repeat=repeat, flags=flags, controller_sha256=frozen[variant],
                        controller_executable=str(binaries[variant]),
                        launch_sha256=sha(launch), shared_simulator_sha256=sha(simulator),
                        scene_sha256=sha(scene),
                        controller_selection='explicit' if variant == 'baseline' else 'default_goal')
            label = f'{args.label}_{variant}'
            navigation = root / 'performance/navigation_benchmark_20260929'
            run_id = f'{label}_ego1p5_{repeat}'
            result = navigation / (run_id + '.json')
            if result.exists() or result.with_suffix('.csv').exists():
                raise SystemExit('refusing to overwrite existing goal result')
            env.update(ISAAC_MANUAL_INPUT_MODE='goal', FOV_GVF_BENCHMARK_ALGORITHM='ego1p5',
                       ISAAC_GOAL_PROTOCOL=str(navigation / 'protocol.json'),
                       ISAAC_BENCHMARK_RESULT=str(result),
                       ISAAC_ACCEPTANCE_TRACE=str(result.with_suffix('.csv')),
                       FOV_GVF_RUN_ID=run_id,
                       FOV_GVF_PERFORMANCE_LOG=str(output / (name + '_performance.md')),
                       FOV_GVF_REPLAY_DIR=str(output / (name + '_replays')),
                       ISAAC_MANUAL_TIMEOUT='0', ISAAC_EXPLORATION_STALL_S='0')
            command = ['bash', str(root / 'scripts/run_isaac_fov_gvf_navigation.sh')]
            meta.update(csv=env.get('ISAAC_ACCEPTANCE_TRACE'), performance=env.get('FOV_GVF_PERFORMANCE_LOG'), log=str(output / (name + '.log')))
            meta['experiment_environment'] = {key: value for key, value in env.items()
                                              if key.startswith(('ISAAC_', 'FOV_GVF_'))}
            print('START', name, flush=True)
            start = time.monotonic()
            with (output / (name + '.log')).open('x') as log:
                run = subprocess.run(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            meta.update(returncode=run.returncode, process_wall_s=time.monotonic() - start)
            result = root / 'performance/navigation_benchmark_20260929' / f'{label}_ego1p5_{repeat}.json'
            if result.exists():
                result_data = json.loads(result.read_text())
                result_data.update(process_exit_code=run.returncode, process_wall_s=meta['process_wall_s'])
                result.write_text(json.dumps(result_data, indent=2) + '\n')
                meta['result'] = result_data
                meta['result_path'] = str(result)
                audit_command = ['/usr/bin/python3', str(root / 'tools/analyze_navigation_benchmark.py'), label]
            else:
                audit_command = None
            if audit_command:
                audit = subprocess.run(audit_command, cwd=root, env=env, text=True, capture_output=True)
                (output / (name + '_audit.json')).write_text(audit.stdout)
                (output / (name + '_audit.stderr')).write_text(audit.stderr)
                meta['audit_returncode'] = audit.returncode
                try:
                    analysis = json.loads((result.parent / f'{label}_analysis.json').read_text())
                    trial = next(row for row in analysis['trials'] if row['run_id'] == f'{label}_ego1p5_{repeat}')
                    meta['audit_safe'] = bool(trial['safe_arrival'])
                    runtime_file = result.with_suffix('.parameters.json')
                    runtime = json.loads(runtime_file.read_text())
                    meta['runtime_launch_verified'] = runtime['launch_sha256'] == frozen_files[str(launch)]
                    meta['runtime_common_sources_verified'] = all(
                        sha(root / key) == digest for key, digest in runtime.get('common_source_sha256', {}).items())
                    meta['runtime_binary_verified'] = (runtime['executable_sha256'] == frozen[variant] and
                        Path(runtime['executable']).resolve() == binaries[variant])
                except (KeyError, ValueError, OSError, StopIteration) as error:
                    meta['verification_error'] = str(error)
                    meta['audit_safe'] = False
                    meta['runtime_binary_verified'] = False
            (output / (name + '_process.json')).write_text(json.dumps(meta, indent=2))
            results.append(meta)
            (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
            print('END', name, meta.get('result', {}).get('status', ''), flush=True)
            if (run.returncode or meta.get('audit_returncode', 1) or
                    not meta.get('audit_safe') or not meta.get('runtime_binary_verified') or
                    not meta.get('runtime_launch_verified') or not meta.get('runtime_common_sources_verified')):
                raise SystemExit('runtime/audit failure preserved; inspect before continuing')
    assert all(sha(path) == frozen[key] for key, path in binaries.items()), 'final binary changed'
    assert all(sha(Path(path)) == digest for path, digest in frozen_files.items()), 'final runtime changed'
    manifest['complete'] = True
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
