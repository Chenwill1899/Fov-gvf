#!/usr/bin/python3
"""Build P3 and its frozen baseline using the same three-dimensional replay driver."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / 'docs/performance/planar_20261116'
FROZEN = EXPERIMENT / 'baseline_source'
DEPS = ROOT / 'docs/performance/harmonic_block4_20261104_213849/deps'
LIBRARY = DEPS / 'suitesparse/usr/lib/x86_64-linux-gnu'
DRIVER = ROOT / 'src/new_controller/ldlt/test/intrinsic_2d_benchmark.cpp'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests', action='store_true')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('run the compiler as the project user, without sudo')
    if not 1 <= args.jobs <= 16:
        parser.error('jobs must be 1..16')
    if not (FROZEN / 'src/new_controller/CMakeLists.txt').is_file():
        parser.error('missing retained baseline_source; restore before/source.tar.gz first')
    output = EXPERIMENT / 'intrinsic_build'
    output.mkdir(exist_ok=True)
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
    build_identities = {}
    for variant, source in [('BASELINE', FROZEN), ('CURRENT', ROOT)]:
        build = EXPERIMENT / 'baseline_build' if variant == 'BASELINE' else output / 'current'
        run(variant + '_configure', ['cmake', '-S', str(source / 'src/new_controller'), '-B', str(build),
            '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'], True)
        run(variant + '_build', ['cmake', '--build', str(build), '-j', str(args.jobs)], True)
        if args.tests:
            run(variant + '_ctest', ['ctest', '--test-dir', str(build), '--output-on-failure'], True)
        if variant == 'CURRENT':
            run('CURRENT_install', ['cmake', '--install', str(build), '--prefix',
                str(ROOT / 'docs/performance/runtime/install/new_controller')], True)
        binary = build / 'intrinsic_2d_benchmark'
        if variant == 'BASELINE':
            # This is the current common driver compiled solely against the
            # retained headers/library; no frozen source is rewritten.
            run('BASELINE_common_driver', ['/usr/bin/g++', '-O3', '-DNDEBUG', '-std=c++17', '-fno-fast-math', '-pthread',
                '-DEGO4P2_STEP2=1', '-DEGO4P0_CURVE_GUARD=1', '-DEGO4P0_CORNER_SMALL_STEP=1',
                '-DEGO4P0_COARSE_PC_MODE=2', '-DAMGCL_NO_BOOST',
                '-I' + str(source / 'candidates/ldlt_parallel/new_controller/include'), '-I/usr/include/eigen3',
                str(DRIVER), str(build / 'libnavigation_ldlt_backend.a'), '-L' + str(LIBRARY),
                '-Wl,--disable-new-dtags,-rpath,' + str(LIBRARY), '-lcholmod', '-lyaml-cpp', '-o', str(binary)])
        binaries[variant] = dict(path=str(binary.relative_to(ROOT)), sha256=digest(binary))
        build_identities[variant] = dict(source_root=str(source.relative_to(ROOT)),
            identity=json.loads((build / 'build_identity.json').read_text()))
    inputs = [DRIVER, ROOT / 'src/new_controller/config/base_step1.json', ROOT / 'src/new_controller/config/navigation_step2.json',
              ROOT / 'docs/performance/current_baseline_20261105_141800/input/last_depth.bin',
              ROOT / 'docs/performance/current_baseline_20261105_141800/input/simulation.sensors.json',
              ROOT / 'docs/performance/step5_rolling_20261107/fixture.json']
    # Record the libraries actually loaded as well as the shared measurement code.
    inputs.extend(p for p in LIBRARY.iterdir() if p.is_file() and '.so' in p.name)
    inputs.extend([Path(__file__), ROOT / 'scripts/benchmarks/run_intrinsic_2d_fifo.py'])
    installed = ROOT / 'docs/performance/runtime/install/new_controller'
    inputs.extend([installed / 'lib/new_controller/navigation_input',
                   installed / 'lib/new_controller/intrinsic_2d_benchmark',
                   installed / 'share/new_controller/build_identity.json'])
    manifest = dict(binaries=binaries, source_identities=build_identities,
                    shared_inputs={str(p.relative_to(ROOT)):digest(p) for p in inputs},
                    driver_sha256=digest(DRIVER), tests_run=args.tests,
                    world_dimensions=3, source_snapshot_sha256=digest(EXPERIMENT/'before/source.tar.gz'))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('MANIFEST=' + str(output / 'manifest.json'))


if __name__ == '__main__':
    main()
