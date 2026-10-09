#!/usr/bin/env python3
"""Assemble completed v22 evidence without changing the predeclared gates."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parent
read = lambda name: json.loads((root / name).read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
acceptance = read('final_v22_acceptance.json')
integrity = read('final_v22_integrity.json')
manual = read('final_v22_analysis.json')
source = read('final_v22_source_checks.json')
ros = read('ros_omni_probe_v22_metadata.json')
replay = read('v21_replay_effects.json')
goal_path = root.parent / 'navigation_benchmark_20260929/omni_final_v22_analysis.json'
goal = json.loads(goal_path.read_text())
assert integrity['integrity_pass'], 'source/binary/baseline drift must be resolved'
assert len(manual['runs']) == 14 and len(goal['trials']) == 10
assert '100% tests passed, 0 tests failed out of 22' in (root / 'ctest_v22.log').read_text()
assert 'Summary: 3 packages finished' in (root / 'build_v22.log').read_text()
assert ros['exit_code'] == 0 and not (root / 'ros_omni_probe_v22.stderr').read_text()
assert ros['controller_sha256'] == read('v22_frozen/binary_sha256.json')['depth_angular_controller']
assert sha(root / 'v22_frozen/paper_replay') == sha(root / 'v21_frozen/paper_replay')
assert source['frozen_sha256_unchanged'] and source['git_diff_check']['returncode'] == 0
failed = [k for k, value in acceptance['checks'].items() if not value]
report = {
    'label': 'final_v22',
    'implementation': 'Continuous world-frame spherical depth proposals; fixed-goal world candidate pool; certified exact-goal early return; conservative union depth 5 to 7 with unchanged 1024 split budget.',
    'status': 'implemented_and_comparatively_tested',
    'all_predeclared_performance_checks_pass': acceptance['all_predeclared_checks_pass'],
    'failed_predeclared_checks': failed,
    'build': {'success': True, 'packages': 3, 'log': 'build_v22.log'},
    'core_tests': {'passed': 22, 'failed': 0, 'log': 'ctest_v22.log'},
    'platform_tests': {'passed': 15, 'failed': 0, 'log': 'pytest_platform.log', 'scope': 'Executed earlier in this task; protected platform source unchanged afterward.'},
    'baseline_core_tests': {'passed': 20, 'scope': 'Executed earlier in this task against original P2.'},
    'ros_probe': ros,
    'replay_decision_effects': {k: replay[k] for k in ['count', 'same_decisions', 'newly_certified', 'all_finite', 'scope']},
    'v22_replay_binary_identical_to_tested_v21': True,
    'manual_trials': len(manual['runs']),
    'goal_trials': len(goal['trials']),
    'manual_summary': manual['summary'],
    'goal_summary': goal['groups'],
    'all_safety_and_speed_checks_pass': all(value for key, value in acceptance['checks'].items() if key.endswith('/safe') or key.endswith('/speed_contract') or key.endswith('/count_and_safe')),
    'independent_geometry_audit': 'Swept 0.48 m body sphere against original occupied voxels and ground; no simulator collision protection allowed.',
    'integrity': integrity,
    'source_checks': source,
    'visual_review': read('final_v22_visual_review.json'),
    'unvalidated': ['Actual physical joystick input', 'GUI/RViz performance under full visualization load', 'Other maps, start-goal pairs and moving obstacles', 'Real aircraft', 'Universal non-stalling or per-cycle real-time guarantee'],
    'notes': ['No exploration trial or earlier failed frozen group included in final_v22 statistics.', 'Unknown observations remain unknown; physical camera blind regions remain.', 'Angle error is conditional on moving; all-active RMS, stopped time and projected progress are reported alongside it.', 'Only three sweep/spin repetitions and one recovery per algorithm; no statistical-significance claim.', 'Original replay snapshots omit native hit clouds and ROS frontend timing.', 'All strict failed gates are retained; no threshold was relaxed after measurement.'],
}
evidence_files = ['final_v22_analysis.json', 'final_v22_acceptance.json', 'final_v22_integrity.json', 'final_v22_source_checks.json', 'ros_omni_probe_v22_metadata.json', 'ros_omni_controller_v22.log', 'v21_replay_effects.json', 'build_v22.log', 'ctest_v22.log', 'pytest_platform.log', 'v22_diff_vs_EGO1P2.patch', 'v22_runtime_changes.json', 'final_v22_visual_review.json']
report['evidence_sha256'] = {name: sha(root / name) for name in evidence_files}
report['evidence_sha256']['../navigation_benchmark_20260929/omni_final_v22_analysis.json'] = sha(goal_path)
(root / 'final_v22_validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'trials': 24, 'safety_and_limits_pass': report['all_safety_and_speed_checks_pass'], 'strict_performance_pass': not failed, 'failed': failed}, indent=2))
