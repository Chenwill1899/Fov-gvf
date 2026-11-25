#!/usr/bin/python3
"""Build isolated step-1 versus coarse-graph-cache candidates without installing."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / 'docs/performance/coarse_cache_20261116'
BASELINE = EXPERIMENT / 'baseline_source'
CANDIDATE = EXPERIMENT / 'candidate_source'
LIBRARY = ROOT / 'docs/performance/harmonic_block4_20261104_213849/deps/suitesparse/usr/lib/x86_64-linux-gnu'
DRIVER = CANDIDATE / 'candidates/ldlt_parallel/new_controller/test/coarse_cache_benchmark.cpp'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_baseline():
    identity = json.loads((EXPERIMENT / 'before/manifest.json').read_text())
    if digest(EXPERIMENT / 'before/source.tar.gz') != identity['source_snapshot_sha256']:
        raise RuntimeError('frozen baseline archive SHA mismatch')
    for relative, expected in identity['baseline_files'].items():
        if digest(BASELINE / relative) != expected:
            raise RuntimeError('frozen baseline changed: ' + relative)
    return dict(source_snapshot_sha256=identity['source_snapshot_sha256'],
                verified_baseline_files=len(identity['baseline_files']),
                measurement_patch_applied=False, production_install_modified=False)


def source_identity(source):
    # Include test sources as well as every translation unit/header/configuration.
    paths = set()
    for directory in ['src/new_controller', 'candidates/ldlt_parallel/new_controller', 'config']:
        paths.update(p for p in (source / directory).rglob('*')
                     if p.is_file() and (p.suffix in {'.cpp', '.hpp', '.h', '.json', '.xml', '.cmake'}
                                         or p.name == 'CMakeLists.txt'))
    return {str(p.relative_to(source)): digest(p) for p in sorted(paths)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests', action='store_true')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--prepare-only', action='store_true', help='read-only verification of the frozen source and source roots')
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('run the compiler as the project user, without sudo')
    if not 1 <= args.jobs <= 16:
        parser.error('jobs must be 1..16')
    for source in [BASELINE, CANDIDATE]:
        if not (source / 'src/new_controller/CMakeLists.txt').is_file():
            parser.error('missing source tree: ' + str(source))
    evidence = verify_baseline()
    evidence['source_roots'] = dict(BASELINE=str(BASELINE), CURRENT=str(CANDIDATE))
    evidence['driver_sha256'] = digest(DRIVER)
    if args.prepare_only:
        print(json.dumps(evidence, indent=2))
        return
    source_before = {variant: source_identity(source)
                     for variant, source in [('BASELINE', BASELINE), ('CURRENT', CANDIDATE)]}
    output = EXPERIMENT / ('build_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    (output / 'precheck.json').write_text(json.dumps(evidence, indent=2) + '\n')
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
               LD_LIBRARY_PATH=str(LIBRARY), PYTHONDONTWRITEBYTECODE='1')
    records = []

    def run(name, command, ros=False):
        actual = ['/bin/bash', '-c', 'source /opt/ros/humble/setup.bash && exec "$@"', 'p3-build', *command] if ros else command
        with (output / (name + '.log')).open('w') as log:
            result = subprocess.run(actual, env=env, stdout=log, stderr=subprocess.STDOUT)
        records.append(dict(name=name, command=actual, exit_code=result.returncode))
        (output / 'build_execution.json').write_text(json.dumps(records, indent=2) + '\n')
        if result.returncode:
            raise RuntimeError(f'{name} failed; see {output / (name + ".log")}')

    binaries = {}
    identities = {}
    for variant, source in [('BASELINE', BASELINE), ('CURRENT', CANDIDATE)]:
        build = output / variant.lower()
        run(variant + '_configure', ['cmake', '-S', str(source / 'src/new_controller'), '-B', str(build),
            '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'], True)
        run(variant + '_build', ['cmake', '--build', str(build), '-j', str(args.jobs)], True)
        if args.tests:
            run(variant + '_ctest', ['ctest', '--test-dir', str(build), '--output-on-failure'], True)
        binary = build / 'coarse_cache_benchmark'
        compatibility = ['-DEGO4P3_COARSE_CACHE_BASELINE=1'] if variant == 'BASELINE' else []
        # Neither source tree's CMake needs a benchmark target. Only the new
        # graph-cache diagnostic fields use the baseline compatibility macro.
        run(variant + '_common_driver', ['/usr/bin/g++', '-O3', '-DNDEBUG', '-std=c++17', '-fno-fast-math', '-pthread',
            *compatibility, '-DEGO4P2_STEP2=1', '-DEGO4P0_CURVE_GUARD=1', '-DEGO4P0_CORNER_SMALL_STEP=1',
            '-DEGO4P0_COARSE_PC_MODE=2', '-DAMGCL_NO_BOOST',
            '-I' + str(source / 'candidates/ldlt_parallel/new_controller/include'), '-I/usr/include/eigen3',
            str(DRIVER), str(build / 'libnavigation_ldlt_backend.a'), '-L' + str(LIBRARY),
            '-Wl,--disable-new-dtags,-rpath,' + str(LIBRARY), '-lcholmod', '-lyaml-cpp', '-o', str(binary)])
        current_identity = source_identity(source)
        if current_identity != source_before[variant] or digest(DRIVER) != evidence['driver_sha256']:
            raise RuntimeError('source or shared driver changed during build: ' + variant)
        cmake_identity = json.loads((build / 'build_identity.json').read_text())
        for relative, expected in cmake_identity.items():
            if digest(source / relative) != expected:
                raise RuntimeError('CMake identity differs from executed source: ' + relative)
        current_identity.update(cmake_identity)
        binaries[variant] = dict(path=str(binary.relative_to(ROOT)), sha256=digest(binary))
        identities[variant] = dict(source_root=str(source.relative_to(ROOT)), identity=current_identity)
    # Recheck both trees after both builds, preventing a concurrent edit from
    # producing a manifest for a different source than the measured executable.
    verify_baseline()
    for variant, source in [('BASELINE', BASELINE), ('CURRENT', CANDIDATE)]:
        if source_identity(source) != source_before[variant]:
            raise RuntimeError('source changed before manifest completion: ' + variant)
    inputs = [DRIVER, ROOT / 'src/new_controller/config/base_step1.json', ROOT / 'src/new_controller/config/navigation_step2.json',
              ROOT / 'docs/performance/current_baseline_20261105_141800/input/last_depth.bin',
              ROOT / 'docs/performance/current_baseline_20261105_141800/input/simulation.sensors.json',
              ROOT / 'docs/performance/step5_rolling_20261107/fixture.json',
              Path(__file__), ROOT / 'scripts/benchmarks/run_coarse_cache_fifo.py', EXPERIMENT / 'before/manifest.json']
    inputs.extend(p for p in LIBRARY.iterdir() if p.is_file() and '.so' in p.name)
    manifest = dict(binaries=binaries, source_identities=identities,
                    shared_inputs={str(p.relative_to(ROOT)): digest(p) for p in inputs},
                    driver_sha256=digest(DRIVER), tests_run=args.tests, world_dimensions=3,
                    source_snapshot_sha256=evidence['source_snapshot_sha256'],
                    instrumentation=dict(measurement_patch_applied=False,
                        description='Both sources already contain identical step-1 CPU instrumentation; only graph-cache diagnostics need a baseline shim.'),
                    production_install_modified=False,
                    comparison='validated local-fine topology planner versus exact coarse graph cache')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (EXPERIMENT / 'latest_build.json').write_text(json.dumps(dict(manifest=str((output / 'manifest.json').relative_to(ROOT))), indent=2) + '\n')
    print('MANIFEST=' + str(output / 'manifest.json'))


if __name__ == '__main__':
    main()
