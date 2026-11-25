#!/usr/bin/python3
"""Build the simulation-only app against current P3 headers, in an isolated prefix."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import tarfile
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / 'docs/performance/isaac_closed_loop_20261117'
APP = ROOT / 'src/isaac_closed_loop'
LIBRARY = ROOT / 'docs/performance/harmonic_block4_20261104_213849/deps/suitesparse/usr/lib/x86_64-linux-gnu'
SCENARIOS = ('straight', 'climb', 'offset_obstacle', 'blocking_obstacle')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_identity():
    files = set(APP.rglob('*.hpp')) | set(APP.rglob('*.cpp')) | {APP / 'CMakeLists.txt'}
    candidate = ROOT / 'src/new_controller/ldlt'
    files.update((candidate / 'include').rglob('*.hpp'))
    files.add(candidate / 'src/sparse_trial_backend.cpp')
    files.update(ROOT / 'src/new_controller/config/isaac_closed_loop' / (name + '.json') for name in SCENARIOS)
    files.update(ROOT / p for p in ['src/new_controller/config/base_step1.json', 'src/new_controller/config/navigation_step2.json',
        'scripts/build_isaac_closed_loop.py', 'scripts/run_isaac_closed_loop.py',
        'scripts/audit_isaac_closed_loop.py', 'scripts/audit_isaac_execution.py', 'scripts/isaac/run_closed_loop_simulation.py',
        'scripts/isaac/closed_loop_scene.py',
        'scripts/isaac/manual_control_math.py', 'scripts/isaac/depth_camera_projection.py',
        'scripts/base_config.py', 'docs/performance/isaac_closed_loop_20261117/IPC_SPEC.md'])
    return {str(p.relative_to(ROOT)): digest(p) for p in sorted(files)}


def production_identity():
    prefix = ROOT / 'docs/performance/runtime/install/new_controller'
    identity = json.loads((prefix / 'share/new_controller/build_identity.json').read_text())
    for relative, wanted in identity.items():
        if digest(ROOT / relative) != wanted:
            raise RuntimeError('active production source is stale: ' + relative)
    return {str(p.relative_to(ROOT)): digest(p) for p in sorted(prefix.rglob('*')) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests', action='store_true')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--check', action='store_true', help='validate inputs without compiling or starting applications')
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('build as the normal project user; do not use sudo')
    if not 1 <= args.jobs <= 16:
        parser.error('jobs must be 1..16')
    before = json.loads((EXPERIMENT / 'before/manifest.json').read_text())
    if digest(EXPERIMENT / 'before/source.tar.gz') != before['source_archive_sha256']:
        raise RuntimeError('pre-integration snapshot SHA mismatch')
    sources = source_identity()
    simulation_runtime = {str((ROOT.parent / 'isaacsim' / name).resolve()): digest(ROOT.parent / 'isaacsim' / name)
        for name in ['python.sh', 'setup_python_env.sh', 'kit/python/bin/python3', 'VERSION']}
    production_before = production_identity()
    if args.check:
        print(json.dumps(dict(source_files=len(sources), production_files=len(production_before),
            snapshot_verified=True, production_install_modified=False, simulation_started=False), indent=2))
        return
    output = EXPERIMENT / ('build_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    build = output / 'build'
    prefix = output / 'install'
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
               LD_LIBRARY_PATH=str(LIBRARY), PYTHONDONTWRITEBYTECODE='1')
    for key in ['LD_PRELOAD', 'LD_AUDIT', 'BASH_ENV', 'ENV']:
        env.pop(key, None)
    records = []

    def run(name, command):
        with (output / (name + '.log')).open('w') as log:
            result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        records.append(dict(name=name, command=command, exit_code=result.returncode))
        (output / 'build_execution.json').write_text(json.dumps(records, indent=2) + '\n')
        if result.returncode:
            raise RuntimeError(name + ' failed; see ' + str(output / (name + '.log')))

    run('configure', ['cmake', '-S', str(APP), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
        '-DBUILD_TESTING=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
    run('build', ['cmake', '--build', str(build), '-j', str(args.jobs)])
    if args.tests:
        run('ctest', ['ctest', '--test-dir', str(build), '--output-on-failure'])
    run('isolated_install', ['cmake', '--install', str(build), '--prefix', str(prefix)])
    if source_identity() != sources:
        raise RuntimeError('app/core/config/scripts changed during build; keep this evidence and rebuild')
    if production_identity() != production_before:
        raise RuntimeError('production installation changed during isolated build')
    binary = prefix / 'bin/isaac_closed_loop_controller'
    compile_dependencies = {}
    dependency_files = list(build.rglob('*.o.d'))
    if len(dependency_files) != 3:
        raise RuntimeError('expected dependency evidence for exactly three translation units')
    for dependency_file in dependency_files:
        words = shlex.split(dependency_file.read_text().replace('\\\n', ' '))
        for word in words[1:]:
            path = Path(word).resolve(strict=True)
            if not path.is_file():
                raise RuntimeError('compile dependency is not a file: ' + word)
            compile_dependencies[str(path)] = digest(path)
    dependencies = {}
    probe = subprocess.run(['/usr/bin/ldd', str(binary)], env=env, capture_output=True, text=True)
    (output / 'ldd.txt').write_text(probe.stdout + probe.stderr)
    if probe.returncode or 'not found' in probe.stdout + probe.stderr:
        raise RuntimeError('isolated executable has unresolved runtime dependencies')
    for line in probe.stdout.splitlines():
        words = line.split()
        paths = [Path(word) for word in words if word.startswith('/')]
        for path in paths:
            if path.is_file():
                dependencies[str(path.resolve())] = digest(path)
    source_archive = output / 'validated_source.tar.gz'
    with tarfile.open(source_archive, 'w:gz') as archive:
        for relative in sources:
            archive.add(ROOT / relative, arcname=relative, recursive=False)
    if source_identity() != sources:
        raise RuntimeError('validated inputs changed while recording build identity; retain evidence and rebuild')
    manifest = dict(schema=1, simulation_only=True, world_dimensions=3,
        source_identity=sources, binary=dict(path=str(binary.relative_to(ROOT)), sha256=digest(binary)),
        source_archive=dict(path=str(source_archive.relative_to(ROOT)), sha256=digest(source_archive)),
        supervisor_check=dict(path=str((build / 'supervisor_check').relative_to(ROOT)),
                              sha256=digest(build / 'supervisor_check')),
        scenarios={name: str(Path('src/new_controller/config/isaac_closed_loop') / (name + '.json')) for name in SCENARIOS},
        dependencies=dependencies, tests_run=args.tests, production_install_modified=False,
        compile_dependency_identity=compile_dependencies,
        simulation_runtime_identity=simulation_runtime,
        production_identity=production_before, snapshot_sha256=before['source_archive_sha256'],
        build_commands=records, compile_commands=json.loads((build / 'compile_commands.json').read_text()))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (EXPERIMENT / 'latest_build.json').write_text(json.dumps(dict(manifest=str((output / 'manifest.json').relative_to(ROOT))), indent=2) + '\n')
    print('MANIFEST=' + str(output / 'manifest.json'))


if __name__ == '__main__':
    main()
