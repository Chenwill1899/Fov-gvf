#!/usr/bin/env python3
"""Run per-item and combined retained-gain ablations with immutable controllers."""
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
    parser.add_argument('--case', choices=['goal', 'jitter', 'spin', 'sweep'], required=True)
    parser.add_argument('--variants', nargs='+', required=True,
                        help='NAME=/absolute/controller; baseline,v2,v4,combined or baseline and one candidate')
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error('at least three repetitions are required')
    binaries = {}
    for item in args.variants:
        key, separator, value = item.partition('=')
        if not separator or key not in {'baseline', 'v2', 'v4', 'combined'} or key in binaries:
            parser.error('invalid or duplicate variant: ' + item)
        binaries[key] = Path(value).resolve(strict=True)
    if 'baseline' not in binaries or len(binaries) not in (2, 4):
        parser.error('provide baseline plus one candidate, or all four variants')
    variants = list(binaries)
    if len(variants) == 4:
        variants = ['baseline', 'v2', 'v4', 'combined']
        # Balanced Latin orders; three blocks are interleaved but incomplete,
        # while four blocks put every variant in every position once.
        order_indices = [[0, 1, 3, 2], [1, 2, 0, 3], [2, 3, 1, 0], [3, 0, 2, 1]]
    else:
        variants = ['baseline'] + [key for key in variants if key != 'baseline']
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
        Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/share/pc_gvf/launch/isaac_cloud_navigation.launch.py'),
        Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/share/pc_gvf/launch/navigation_benchmark.launch.py')]
    if args.case != 'goal':
        runtime_files.append(root / 'src/pc_gvf/test/fixtures/paper' / (
            'isaac_operator_jitter_trace.json' if args.case == 'jitter' else 'isaac_omni_' + args.case + '_trace.json'))
    frozen_files = {str(path): sha(path) for path in runtime_files}
    features = ['uncertainty', 'dynamic', 'incremental', 'memory', 'operator', 'shared']
    env_keys = dict(zip(features, [
        'FOV_GVF_DEPTH_UNCERTAINTY', 'FOV_GVF_DYNAMIC_OBSTACLES',
        'FOV_GVF_INCREMENTAL_FIELD', 'FOV_GVF_SPHERICAL_MEMORY',
        'FOV_GVF_OPERATOR_ASSISTANCE', 'FOV_GVF_SHARED_OBSTACLES']))
    results = []
    manifest = dict(case=args.case, variants={k: str(v) for k, v in binaries.items()},
                    feature_flags='all six off', twist_source='callback',
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
                       FOV_GVF_INSTALL='/tmp/fov_gvf_ego1p5_isaac_install',
                       FOV_GVF_CONTROLLER_EXECUTABLE=str(binaries[variant]),
                       ISAAC_HEADLESS='1', FOV_GVF_RVIZ='false', FOV_GVF_SCENE_MODE='cloud',
                       ISAAC_TWIST_SAMPLE_SOURCE='callback',
                       ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1')
            meta = dict(algorithm='p5_retained_' + variant, case=args.case, variant=variant,
                        repeat=repeat, flags=flags, controller_sha256=frozen[variant],
                        controller_executable=str(binaries[variant]),
                        launch_sha256=sha(launch), shared_simulator_sha256=sha(simulator),
                        scene_sha256=sha(scene))
            if args.case == 'goal':
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
            else:
                fixture = root / 'src/pc_gvf/test/fixtures/paper' / (
                    'isaac_operator_jitter_trace.json' if args.case == 'jitter' else 'isaac_omni_' + args.case + '_trace.json')
                duration = {'jitter': 34, 'spin': 64, 'sweep': 40}[args.case]
                env.update(FOV_GVF_RUNTIME_MANIFEST=str(output / (name + '_runtime.json')),
                           ISAAC_MANUAL_INPUT_MODE='trace', ISAAC_INTENT_TRACE=str(fixture),
                           ISAAC_ACCEPTANCE_TRACE=str(output / (name + '.csv')),
                           ISAAC_MANUAL_TIMEOUT=str(duration), FOV_GVF_RUN_ID=name,
                           FOV_GVF_REPLAY_DIR=str(output / (name + '_replays')),
                           FOV_GVF_PERFORMANCE_LOG=str(output / (name + '_performance.md')))
                meta.update(duration_s=duration, trace_sha256=sha(fixture))
                command = ['bash', str(root / 'scripts/run_isaac_fov_gvf_navigation.sh')]
            meta.update(csv=env.get('ISAAC_ACCEPTANCE_TRACE'), performance=env.get('FOV_GVF_PERFORMANCE_LOG'), log=str(output / (name + '.log')))
            meta['experiment_environment'] = {key: value for key, value in env.items()
                                              if key.startswith(('ISAAC_', 'FOV_GVF_'))}
            print('START', name, flush=True)
            start = time.monotonic()
            with (output / (name + '.log')).open('x') as log:
                run = subprocess.run(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            meta.update(returncode=run.returncode, process_wall_s=time.monotonic() - start)
            if args.case == 'goal':
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
            else:
                audit_command = ['/usr/bin/python3', str(root / 'tools/analyze_envelope_trace.py'),
                                 str(output / (name + '.csv'))]
            if audit_command:
                audit = subprocess.run(audit_command, cwd=root, env=env, text=True, capture_output=True)
                (output / (name + '_audit.json')).write_text(audit.stdout)
                (output / (name + '_audit.stderr')).write_text(audit.stderr)
                meta['audit_returncode'] = audit.returncode
                try:
                    if args.case == 'goal':
                        analysis = json.loads((result.parent / f'{label}_analysis.json').read_text())
                        trial = next(row for row in analysis['trials'] if row['run_id'] == f'{label}_ego1p5_{repeat}')
                        meta['audit_safe'] = bool(trial['safe_arrival'])
                        runtime_file = result.with_suffix('.parameters.json')
                    else:
                        audited = json.loads(audit.stdout)
                        meta['audit_safe'] = (audited['swept_sphere_overlap_segments'] == 0 and
                            audited['external_collision_blocks'] == 0 and
                            abs(audited['duration_s'] - duration) < .2)
                        runtime_file = output / (name + '_runtime.json')
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
