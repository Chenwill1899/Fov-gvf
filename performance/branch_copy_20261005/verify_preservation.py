#!/usr/bin/env python3
"""Verify preserved EGO1P1 and explicitly scoped EGO1P2 copy adaptations."""
import hashlib
import json
import os
import stat
from datetime import datetime
from pathlib import Path

folder = Path(__file__).resolve().parent
destination = folder.parents[1]
source = destination.parent / 'EGO1P1'
manifest = json.loads((folder / 'source_manifest.json').read_text())


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def inventory(root):
    result = {}
    for parent, dirs, files in os.walk(root, followlinks=False):
        for name in sorted(dirs + files):
            path = Path(parent) / name
            status = path.lstat()
            entry = {'mode': stat.S_IMODE(status.st_mode)}
            if path.is_symlink():
                entry.update(type='symlink', target=os.readlink(path))
            elif path.is_dir():
                entry.update(type='directory')
            else:
                assert path.is_file(), path
                entry.update(type='file', size=status.st_size,
                             sha256=file_sha256(path))
            result[str(path.relative_to(root))] = entry
    return result


current_source = inventory(source)
source_diff = sorted(k for k in manifest.keys() | current_source.keys()
                     if manifest.get(k) != current_source.get(k))
assert not source_diff, source_diff
current = inventory(destination)
removed = sorted(manifest.keys() - current.keys())
assert not removed, removed
changed = sorted(k for k in manifest if manifest[k] != current[k])
allowed = {
    'AGENTS.md', 'README.md', 'WORK_LOG.md',
    'scripts/build_isaac_ros_workspace.sh', 'scripts/run_isaac_fov_gvf_navigation.sh',
    'src/pc_gvf/launch/navigation_benchmark.launch.py',
    'tools/run_navigation_benchmark.py', 'tools/analyze_navigation_benchmark.py',
    'tools/verify_navigation_replays.py', 'tools/ros_vector_guidance_probe.py',
}
automatic = [k for k in changed if '__pycache__' in Path(k).parts or '.pytest_cache' in Path(k).parts]
assert set(changed) <= allowed | set(automatic), changed
preserved_prefixes = ('.git/', 'scenes/', 'performance/')
preserved = [k for k in manifest if k.startswith(preserved_prefixes)]
assert all(manifest[k] == current[k] for k in preserved)
source_code = [k for k in manifest if k.startswith('src/')
               and k != 'src/pc_gvf/launch/navigation_benchmark.launch.py'
               and '__pycache__' not in Path(k).parts and '.pytest_cache' not in Path(k).parts]
assert all(manifest[k] == current[k] for k in source_code)
shared = []
for name, entry in manifest.items():
    if entry['type'] != 'file':
        continue
    a, b = (source / name).stat(), (destination / name).stat()
    if a.st_dev == b.st_dev and a.st_ino == b.st_ino:
        shared.append(name)
assert not shared, shared
result = dict(verified_at=datetime.now().astimezone().isoformat(),
              source=str(source), destination=str(destination),
              source_unchanged=True, source_difference_paths=source_diff,
              source_manifest_entries=len(manifest), source_files=sum(e['type']=='file' for e in manifest.values()),
              inherited_files_removed=removed, intentional_adaptation_paths=[k for k in changed if k not in automatic],
              regenerated_cache_paths=automatic, shared_file_inodes=shared,
              inherited_git_scenes_experiments_unchanged=True,
              controller_algorithm_and_main_launch_unchanged=True,
              added_paths=sorted(current.keys() - manifest.keys()))
(folder / 'final_preservation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('added_paths','regenerated_cache_paths')}, ensure_ascii=False, indent=2))
