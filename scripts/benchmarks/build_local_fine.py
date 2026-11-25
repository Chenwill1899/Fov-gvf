#!/usr/bin/python3
"""Build a fresh local-fine comparison; retain the original intrinsic experiment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import shutil
import difflib
from datetime import datetime
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / 'docs/performance/local_fine_20261116'
FROZEN = EXPERIMENT / 'baseline_source'
INSTRUMENTED = EXPERIMENT / 'instrumented_baseline'
DEPS = ROOT / 'docs/performance/harmonic_block4_20261104_213849/deps'
LIBRARY = DEPS / 'suitesparse/usr/lib/x86_64-linux-gnu'
DRIVER = ROOT / 'src/new_controller/ldlt/test/local_fine_benchmark.cpp'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_baseline():
    """Apply only symmetric CPU measurement to a derivative, never the frozen copy."""
    identity=json.loads((EXPERIMENT/'before/manifest.json').read_text())
    if digest(EXPERIMENT/'before/source.tar.gz')!=identity['source_snapshot_sha256']:
        raise RuntimeError('baseline archive SHA mismatch')
    for relative, expected in identity['baseline_files'].items():
        if digest(FROZEN/relative)!=expected:
            raise RuntimeError('frozen baseline changed: '+relative)
    header='candidates/ldlt_parallel/new_controller/include/new_controller/'
    changes={}
    def replace_once(text, old, new):
        if text.count(old)!=1:
            raise RuntimeError('instrumentation anchor is not unique: '+old)
        return text.replace(old,new,1)
    name=header+'harmonic_types.hpp';original=(FROZEN/name).read_text()
    changes[name]=replace_once(original,'int64_t domain_ns=0,solve_ns=0,cache_ns=0,','int64_t solve_cpu_ns=0;\n  int64_t domain_ns=0,solve_ns=0,cache_ns=0,')
    name=header+'harmonic_planner.hpp';text=(FROZEN/name).read_text()
    text=replace_once(text,'out.coarse.domain_ns=field_now_ns()-coarse_domain;coarse_solver_.solve(out.adaptive,out.coarse,config_);',
        'out.coarse.domain_ns=field_now_ns()-coarse_domain;\n    const auto coarse_solve_cpu_begin=field_thread_cpu_ns();\n    coarse_solver_.solve(out.adaptive,out.coarse,config_);\n    out.coarse.solve_cpu_ns=field_thread_cpu_ns()-coarse_solve_cpu_begin;')
    text=replace_once(text,'      if(external_fine_){external_fine_->solve_layer',
        '      const auto fine_solve_cpu_begin=field_thread_cpu_ns();\n      if(external_fine_){external_fine_->solve_layer')
    text=replace_once(text,'++out.fine_attempts;\n      out.fine.solve_ns+=old_solve;',
        '++out.fine_attempts;\n      out.fine.solve_cpu_ns+=field_thread_cpu_ns()-fine_solve_cpu_begin;\n      out.fine.solve_ns+=old_solve;')
    text=replace_once(text,'l.domain_ns=l.solve_ns=l.cache_ns=', 'l.solve_cpu_ns=0;l.domain_ns=l.solve_ns=l.cache_ns=')
    changes[name]=text
    if not INSTRUMENTED.exists():
        shutil.copytree(FROZEN,INSTRUMENTED,symlinks=True)
        for name,text in changes.items():(INSTRUMENTED/name).write_text(text)
    for relative, expected in identity['baseline_files'].items():
        desired=hashlib.sha256(changes[relative].encode()).hexdigest() if relative in changes else expected
        if digest(INSTRUMENTED/relative)!=desired:
            raise RuntimeError('instrumented baseline has unexpected edits: '+relative)
    patch=''.join(''.join(difflib.unified_diff((FROZEN/name).read_text().splitlines(True),text.splitlines(True),
        fromfile='baseline_source/'+name,tofile='instrumented_baseline/'+name)) for name,text in changes.items())
    patch_file=EXPERIMENT/'baseline_measurement.patch'
    if patch_file.exists() and patch_file.read_text()!=patch:
        raise RuntimeError('refuse to rewrite a different measurement patch')
    patch_file.write_text(patch)
    evidence=dict(purpose='CPU instrumentation only; identical numerical algorithm to frozen intrinsic baseline',
        frozen_snapshot_sha256=identity['source_snapshot_sha256'],patch_sha256=digest(patch_file),
        changed_files={name:dict(before_sha256=digest(FROZEN/name),after_sha256=digest(INSTRUMENTED/name)) for name in changes},
        verified_baseline_files=len(identity['baseline_files']))
    (EXPERIMENT/'baseline_measurement.json').write_text(json.dumps(evidence,indent=2)+'\n')
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests', action='store_true')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--prepare-only', action='store_true', help='verify the frozen source and create the CPU-instrumented derivative')
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('run the compiler as the project user, without sudo')
    if not 1 <= args.jobs <= 16:
        parser.error('jobs must be 1..16')
    if not (FROZEN / 'src/new_controller/CMakeLists.txt').is_file():
        parser.error('missing retained baseline_source; restore before/source.tar.gz first')
    instrumentation=prepare_baseline()
    if args.prepare_only:
        print(json.dumps(instrumentation,indent=2));return
    output = EXPERIMENT / ('build_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
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
    for variant, source in [('BASELINE', INSTRUMENTED), ('CURRENT', ROOT)]:
        build = output / variant.lower()
        run(variant + '_configure', ['cmake', '-S', str(source / 'src/new_controller'), '-B', str(build),
            '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=ON', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'], True)
        run(variant + '_build', ['cmake', '--build', str(build), '-j', str(args.jobs)], True)
        if args.tests:
            run(variant + '_ctest', ['ctest', '--test-dir', str(build), '--output-on-failure'], True)
        binary = build / 'local_fine_benchmark'
        if variant == 'BASELINE':
            # This is the current common driver compiled solely against the
            # retained headers/library; no frozen source is rewritten.
            run('BASELINE_common_driver', ['/usr/bin/g++', '-O3', '-DNDEBUG', '-std=c++17', '-fno-fast-math', '-pthread',
                '-DEGO4P3_LEGACY_BASELINE=1', '-DEGO4P2_STEP2=1', '-DEGO4P0_CURVE_GUARD=1', '-DEGO4P0_CORNER_SMALL_STEP=1',
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
    inputs.extend([Path(__file__), ROOT / 'scripts/benchmarks/run_local_fine_fifo.py',EXPERIMENT/'baseline_measurement.patch',EXPERIMENT/'baseline_measurement.json'])
    inputs.extend(p for p in (EXPERIMENT/'before/original_binaries').iterdir() if p.is_file())
    manifest = dict(binaries=binaries, source_identities=build_identities,
                    shared_inputs={str(p.relative_to(ROOT)):digest(p) for p in inputs},
                    driver_sha256=digest(DRIVER), tests_run=args.tests,
                    world_dimensions=3, source_snapshot_sha256=digest(EXPERIMENT/'before/source.tar.gz'),
                    instrumentation=instrumentation,production_install_modified=False,
                    original_binary_identity=json.loads((EXPERIMENT/'before/original_binaries/identity.json').read_text()))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    # A pointer selects the most recent complete build; all build directories
    # and their evidence remain immutable when another build is requested.
    (EXPERIMENT/'latest_build.json').write_text(json.dumps(dict(manifest=str((output/'manifest.json').relative_to(ROOT))),indent=2)+'\n')
    print('MANIFEST=' + str(output / 'manifest.json'))


if __name__ == '__main__':
    main()
