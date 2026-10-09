#!/usr/bin/env python3
"""One 64-second spin acceptance run through the installed default manual route."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[2]
    install = Path('/tmp/fov_gvf_ego1p5_isaac_install')
    output = here / 'manual_default'
    if output.exists():
        raise SystemExit('refusing to overwrite manual_default output')
    controller = (install / 'pc_gvf/lib/pc_gvf/depth_angular_controller').resolve(strict=True)
    if not controller.is_file() or not os.access(controller, os.X_OK):
        raise SystemExit('installed default manual controller is not executable')
    reference = root / 'performance/retain_gains_20261008/baseline/depth_angular_controller'
    reference_manifest = reference.parent / 'binary_sha256.json'
    expected = json.loads(reference_manifest.read_text())['depth_angular_controller']
    installed_sha = sha(controller)
    if installed_sha != expected or sha(reference) != expected:
        raise SystemExit('installed manual controller does not match frozen old baseline')
    launch = root / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    installed_launch = install / 'pc_gvf/share/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    if sha(launch) != sha(installed_launch):
        raise SystemExit('installed cloud launch differs from project source')
    simulator = root / 'scripts/isaac/run_fov_gvf_navigation.py'
    scene = root / 'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'
    fixture = root / 'src/pc_gvf/test/fixtures/paper/isaac_omni_spin_trace.json'
    runtime_files = [launch, installed_launch, simulator, scene, fixture,
        root / 'src/pc_gvf/launch/navigation_benchmark.launch.py',
        install / 'pc_gvf/share/pc_gvf/launch/navigation_benchmark.launch.py',
        root / 'scripts/run_isaac_fov_gvf_navigation.sh',
        root / 'scripts/isaac/navigation_benchmark.py',
        root / 'scripts/isaac/manual_control_math.py',
        root / 'src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py',
        root / 'performance/navigation_benchmark_20260929/protocol.json',
        root / 'scenes/ego_swarm_cloud/occupancy.bin', Path(__file__).resolve(),
        root / 'tools/analyze_navigation_benchmark.py',
        root / 'tools/analyze_omni_comparison.py', root / 'tools/analyze_envelope_trace.py',
        reference, reference_manifest]
    frozen_files = {str(path): sha(path) for path in runtime_files}
    flags = {key: '0' for key in (
        'FOV_GVF_DEPTH_UNCERTAINTY', 'FOV_GVF_DYNAMIC_OBSTACLES',
        'FOV_GVF_INCREMENTAL_FIELD', 'FOV_GVF_SPHERICAL_MEMORY',
        'FOV_GVF_OPERATOR_ASSISTANCE', 'FOV_GVF_SHARED_OBSTACLES')}
    name = 'p5_retain_20261008_manual_default_spin_baseline_1'
    clean_environment = {key: value for key, value in os.environ.items()
                         if not key.startswith(('ISAAC_', 'FOV_GVF_'))}
    env = dict(clean_environment, **flags, PYTHONNOUSERSITE='1',
               FOV_GVF_INSTALL=str(install), ISAAC_HEADLESS='1', FOV_GVF_RVIZ='false',
               FOV_GVF_SCENE_MODE='cloud', ISAAC_TWIST_SAMPLE_SOURCE='callback',
               ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1',
               FOV_GVF_RUNTIME_MANIFEST=str(output / (name + '_runtime.json')),
               ISAAC_MANUAL_INPUT_MODE='trace', ISAAC_INTENT_TRACE=str(fixture),
               ISAAC_ACCEPTANCE_TRACE=str(output / (name + '.csv')),
               ISAAC_MANUAL_TIMEOUT='64', FOV_GVF_RUN_ID=name,
               FOV_GVF_REPLAY_DIR=str(output / (name + '_replays')),
               FOV_GVF_PERFORMANCE_LOG=str(output / (name + '_performance.md')))
    # Exercise normal launch selection; a controller override must never leak in.
    if 'FOV_GVF_CONTROLLER_EXECUTABLE' in env:
        raise SystemExit('default route requires controller override to be absent')
    meta = dict(algorithm='p5_retained_baseline', case='spin', variant='baseline', repeat=1,
                flags=flags, controller_sha256=installed_sha, controller_executable=str(controller),
                controller_selection='default_manual', launch_sha256=sha(launch),
                shared_simulator_sha256=sha(simulator), scene_sha256=sha(scene),
                duration_s=64, trace_sha256=sha(fixture),
                csv=env['ISAAC_ACCEPTANCE_TRACE'], performance=env['FOV_GVF_PERFORMANCE_LOG'],
                log=str(output / (name + '.log')),
                experiment_environment={key: value for key, value in env.items()
                                        if key.startswith(('ISAAC_', 'FOV_GVF_'))})
    manifest = dict(case='spin', variants={'baseline': str(controller)},
                    feature_flags='all six off', twist_source='callback',
                    routing={'baseline': 'installed default manual executable'},
                    binary_sha256={'baseline': installed_sha}, file_sha256=frozen_files,
                    baseline_identity=dict(reference=str(reference), reference_manifest=str(reference_manifest),
                                           expected_sha256=expected, installed_sha256=installed_sha, matches=True),
                    order_by_repeat=[['baseline']], runs=[], complete=False)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'manifest.json', manifest)
    if sha(controller) != installed_sha or not all(sha(path) == digest for path, digest in frozen_files.items()):
        raise SystemExit('frozen binary/source changed before launch')
    command = ['bash', str(root / 'scripts/run_isaac_fov_gvf_navigation.sh')]
    print('START', name, flush=True)
    start = time.monotonic()
    with Path(meta['log']).open('x') as log:
        run = subprocess.run(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
    meta.update(returncode=run.returncode, process_wall_s=time.monotonic() - start)
    audit = subprocess.run(['/usr/bin/python3', str(root / 'tools/analyze_envelope_trace.py'), meta['csv']],
                           cwd=root, env=env, text=True, capture_output=True)
    (output / (name + '_audit.json')).write_text(audit.stdout)
    (output / (name + '_audit.stderr')).write_text(audit.stderr)
    meta['audit_returncode'] = audit.returncode
    try:
        audited = json.loads(audit.stdout)
        meta['audit_safe'] = (audited['swept_sphere_overlap_segments'] == 0 and
                              audited['external_collision_blocks'] == 0 and
                              abs(audited['duration_s'] - 64) < .2)
        runtime_path = Path(env['FOV_GVF_RUNTIME_MANIFEST'])
        runtime = json.loads(runtime_path.read_text())
        meta['runtime_manifest_sha256'] = sha(runtime_path)
        meta['runtime_launch_verified'] = runtime['launch_sha256'] == frozen_files[str(launch)]
        # Manual runtime attests binary/launch only; common files are frozen before/after.
        meta['runtime_common_sources_verified'] = all(sha(path) == digest for path, digest in frozen_files.items())
        meta['runtime_binary_verified'] = (runtime['executable_sha256'] == installed_sha and
            Path(runtime['executable']).resolve() == controller and sha(controller) == installed_sha)
    except (KeyError, ValueError, TypeError, OSError) as error:
        meta['verification_error'] = str(error)
        meta['audit_safe'] = False
        meta['runtime_binary_verified'] = False
    passed = (run.returncode == 0 and audit.returncode == 0 and meta.get('audit_safe') and
              meta.get('runtime_binary_verified') and meta.get('runtime_launch_verified') and
              meta.get('runtime_common_sources_verified'))
    write_json(output / 'process.json', meta)
    manifest['runs'].append(meta)
    manifest['complete'] = bool(passed)
    write_json(output / 'manifest.json', manifest)
    print('END', name, 'PASS' if passed else 'FAIL', flush=True)
    if not passed:
        raise SystemExit('runtime/audit failure preserved; inspect before continuing')


if __name__ == '__main__':
    main()
