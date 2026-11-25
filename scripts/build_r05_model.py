#!/usr/bin/python3
"""Build the original R05 experimental adapter with the current P3 C++ core."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'src/r05_model'
EXPERIMENT = ROOT / 'docs/performance/r05_model_sync_20261117'
ASSETS = ROOT / 'scenes/r05_cloud'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity():
    core = ROOT / 'src/new_controller/ldlt'
    files = set(APP.rglob('*.cpp')) | set(APP.rglob('*.hpp')) | set(APP.rglob('*.inc')) | set(APP.rglob('*.py'))
    files.add(APP / 'CMakeLists.txt')
    files.update((core / 'include').rglob('*.hpp'))
    files.add(core / 'src/sparse_trial_backend.cpp')
    files.update(p for p in ASSETS.iterdir() if p.is_file())
    files.update((ROOT / 'scenes/robots/arl_robot_1').iterdir())
    files.update(ROOT / p for p in ['scripts/build_r05_model.py', 'scripts/run_r05_model.py',
        'scripts/run_isaac_closed_loop.py', 'scripts/base_config.py',
        'scripts/isaac/manual_control_math.py', 'scripts/isaac/depth_camera_projection.py',
        'src/new_controller/config/base_step1.json', 'src/new_controller/config/navigation_step2.json'])
    return {str(p.relative_to(ROOT)): digest(p) for p in sorted(files)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if os.geteuid() == 0 or not 1 <= args.jobs <= 16:
        parser.error('build as the normal user, jobs must be 1..16')
    sources = identity()
    output = EXPERIMENT / ('build_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True)
    for label, cmd in [('configure', ['cmake', '-S', str(APP), '-B', str(output / 'build'), '-DCMAKE_BUILD_TYPE=Release']),
                       ('build', ['cmake', '--build', str(output / 'build'), '-j', str(args.jobs)])]:
        with (output / (label + '.log')).open('w') as log:
            result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(str(output / (label + '.log')))
    binary = output / 'build/r05_controller'
    dependencies = {}
    probe = subprocess.run(['/usr/bin/ldd', str(binary)], capture_output=True, text=True, check=True)
    (output / 'ldd.txt').write_text(probe.stdout)
    if 'not found' in probe.stdout:
        raise RuntimeError('missing dynamic dependency')
    for word in probe.stdout.split():
        if word.startswith('/') and Path(word).is_file():
            path = Path(word).resolve(); dependencies[str(path)] = digest(path)
    if sources != identity():
        raise RuntimeError('sources changed while building')
    manifest = dict(scope='R05 model + original experimental guidance adapter + current P3 core',
        source_identity=sources, dependencies=dependencies,
        binary=dict(path=str(binary.relative_to(ROOT)), sha256=digest(binary)),
        algorithm_core='current P3 root; no P2 frozen ELF or pipeline overlay',
        production_install_modified=False)
    path = output / 'manifest.json';path.write_text(json.dumps(manifest, indent=2)+'\n')
    (EXPERIMENT / 'latest_build.json').write_text(json.dumps(dict(manifest=str(path.relative_to(ROOT))), indent=2)+'\n')
    print('MANIFEST='+str(path))

if __name__ == '__main__':
    main()
