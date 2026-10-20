#!/usr/bin/env python3
"""Staged Isaac speed trials and an independent swept-geometry/limits audit."""
import argparse
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'docs/performance/p6_speed_envelope_20261019'
BASE_PROTOCOL = ROOT / 'src/pc_gvf/config/navigation_protocol.json'
# There is one current Cloud map. Historical maps live only in the merge archive.
MAPS = {
    'original': (ROOT / 'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd',
                 (1200, 300, 50), [384., 11.2, 2.8]),
}


def audit(result, scene, shape):
    """No online ESDF values are used to declare safe arrival."""
    data = json.loads(result.read_text())
    rows = list(csv.DictReader(result.with_suffix('.csv').open()))
    if len(rows) < 3:
        raise ValueError('incomplete trajectory')
    def matrix(keys):
        a = np.array([[float(r[k]) for k in keys] for r in rows])
        if not np.isfinite(a).all():
            raise ValueError('nonfinite trajectory')
        return a
    t = matrix(['t']).ravel()
    p, v = matrix(['x', 'y', 'z']), matrix(['vx', 'vy', 'vz'])
    command = matrix(['applied_target_x', 'applied_target_y', 'applied_target_z'])
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError('nonmonotonic timestamps')
    grid = np.fromfile(scene.with_name('occupancy.bin'), np.uint8).reshape(shape)
    origin = np.array([-80., -60., 0.])
    radius = data['protocol']['body_radius_m']
    minimum, overlaps = float('inf'), 0
    for a, b in zip(p[:-1], p[1:]):
        center = (a + b) * .5
        enclosing = radius + np.linalg.norm(b - a) * .5
        index = np.floor((center - origin) / .4 + .5).astype(int)
        reach = int(np.ceil((enclosing + .4) / .4)) + 1
        lo, hi = np.maximum(0, index - reach), np.minimum(shape, index + reach + 1)
        # Ground and finite occupancy bounds are part of the independent audit.
        clearance = min(center[2] - enclosing,
            np.min(center[:2] - (origin[:2] - .2)) - enclosing,
            np.min(origin[:2] + .4 * (np.array(shape[:2]) - 1) + .2 - center[:2]) - enclosing)
        if np.any(lo >= hi):
            clearance = float('-inf')
        else:
            occupied = np.argwhere(grid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]) + lo
            if len(occupied):
                delta = np.maximum(np.abs(origin + .4 * occupied - center) - .2, 0)
                clearance = min(clearance, float(np.linalg.norm(delta, axis=1).min()) - enclosing)
        minimum = min(minimum, clearance)
        overlaps += int(clearance < 0)
    speed = np.linalg.norm(v, axis=1)
    acc = np.diff(v, axis=0) / dt[:, None]
    jerk = np.linalg.norm(np.diff(acc, axis=0) / dt[1:, None], axis=1)
    limits = data['protocol']
    violations = dict(speed=int(np.sum(speed > limits['max_speed_mps'] + 1e-5)),
        vertical=int(np.sum(np.abs(v[:, 2]) > limits['max_vertical_speed_mps'] + 1e-5)),
        acceleration=int(np.sum(np.linalg.norm(acc, axis=1) > limits['plant_accel_mps2'] + 1e-4)),
        applied_command=int(np.sum(np.linalg.norm(command, axis=1) > limits['max_speed_mps'] + 1e-5)))
    active = (t[:-1] >= limits['warmup_s']) & (np.linalg.norm(matrix(['qx','qy','qz'])[:-1], axis=1) > .05)
    stopped = active & (speed[:-1] < .05)
    longest = streak = 0.
    for flag, delta in zip(stopped, dt):
        streak = streak + delta if flag else 0.
        longest = max(longest, streak)
    statuses = collections.Counter()
    for row, delta in zip(rows, dt):
        statuses[row['status']] += delta
    parameters = json.loads(result.with_suffix('.parameters.json').read_text())
    effective = parameters['effective']
    config_ok = (effective['max_speed'] == limits['max_speed_mps'] and
        effective['command_accel_limit'] == limits['plant_accel_mps2'] and
        effective['body_radius'] == radius and effective['max_depth'] == limits.get('max_depth_m', 10.))
    log = result.with_suffix('.log').read_text(errors='replace')
    perf = re.findall(r'\[PERF CTRL FINAL\].*?avoid_ms\(mean/p50/p95/max\)=([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+).*?callback_to_publish_ms\(mean/p95/max\)=([\d.]+)/([\d.]+)/([\d.]+)', log)
    performance = dict(zip(('avoid_mean_ms','avoid_p50_ms','avoid_p95_ms','avoid_max_ms',
                            'callback_mean_ms','callback_p95_ms','callback_max_ms'), map(float, perf[-1]))) if perf else {}
    peak = float(speed.max())
    travelled = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    report = dict(result=os.path.relpath(result, ROOT), status=data['status'],
        scene=os.path.relpath(scene, ROOT), requested_speed_mps=limits['max_speed_mps'],
        max_depth_m=limits.get('max_depth_m', 10.),
        safe_arrival=data['status']=='ARRIVED' and overlaps==0 and data['collision_blocks']==0 and
            not any(violations.values()) and config_ok and data.get('process_exit_code')==0,
        sim_s=data['simulation_elapsed_s'], wall_s=data['wall_elapsed_s'],
        actual_peak_mps=peak, actual_p95_mps=float(np.percentile(speed, 95)),
        path_m=travelled, route_average_mps=travelled/data['simulation_elapsed_s'],
        time_at_95pct_cap_s=float(dt[speed[:-1] >= .95*limits['max_speed_mps']].sum()),
        swept_overlap_segments=int(overlaps), minimum_swept_clearance_m=float(minimum),
        collision_blocks=data['collision_blocks'], limits_violations=violations, config_ok=config_ok,
        acceleration_max_mps2=float(np.linalg.norm(acc, axis=1).max()),
        jerk_rms_mps3=float(np.sqrt(np.mean(jerk*jerk))), stopped_s=float(dt[stopped].sum()),
        longest_stop_s=float(longest), status_seconds=dict(statuses),
        trace_max_gap_s=float(dt.max()), performance=performance,
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        occupancy_sha256=hashlib.sha256(scene.with_name('occupancy.bin').read_bytes()).hexdigest())
    report['cap_attained']=peak >= .95*limits['max_speed_mps'] and report['time_at_95pct_cap_s']>=1.
    result.with_suffix('.audit.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def run(args):
    folder = OUT / args.label
    folder.mkdir(parents=True, exist_ok=True)
    for speed in args.speeds:
        stage_ok = True
        for repeat in range(1, args.repeats + 1):
            for name in args.maps:
                scene, shape, goal = MAPS[name]
                stem = f'{name}_v{speed:g}_r{repeat:02d}'
                result = folder / (stem + '.json')
                if result.exists() or result.with_suffix('.log').exists():
                    raise RuntimeError('refusing to overwrite ' + stem)
                protocol = json.loads(BASE_PROTOCOL.read_text())
                protocol.update(version='ego2p1_speed_v1', max_speed_mps=speed, goal=goal,
                    timeout_s=600., max_depth_m=args.depth,
                    final_repeats=args.repeats,goal_proposal_cache=args.cache,restart_clearance_m=args.restart,
                    goal_proposal_cache_ttl_s=args.cache_ttl,goal_speed_governor=args.governor)
                protocol_path = folder / (stem + '.protocol.json')
                protocol_path.write_text(json.dumps(protocol, indent=2) + '\n')
                env = dict(os.environ, ISAAC_MANUAL_INPUT_MODE='goal', ISAAC_HEADLESS='1',
                    FOV_GVF_RVIZ='false', FOV_GVF_BENCHMARK_ALGORITHM='ego2p6',
                    FOV_GVF_SCENE=str(scene), ISAAC_GOAL_PROTOCOL=str(protocol_path),
                    ISAAC_BENCHMARK_RESULT=str(result), ISAAC_ACCEPTANCE_TRACE=str(result.with_suffix('.csv')),
                    FOV_GVF_RUN_ID=f'p2_speed_{args.label}_{stem}',
                    FOV_GVF_PERFORMANCE_LOG=str(folder/'PERFORMANCE.md'),
                    FOV_GVF_REPLAY_DIR=(str(folder/(stem+'_replays'))
                        if os.environ.get('EGO2P6_RECORD_REPLAYS', '0') == '1' else ''), ISAAC_MANUAL_TIMEOUT='0',
                    ISAAC_EXPLORATION_STALL_S=str(args.abort_stall),
                    ROS_DOMAIN_ID='42', FOV_GVF_WAIT_FOR_LOCK='1')
                if args.emergency:
                    env['FOV_GVF_EMERGENCY_CERTIFICATE']='1'
                start = time.perf_counter()
                print('START', args.label, stem, flush=True)
                with result.with_suffix('.log').open('w') as stream:
                    child = subprocess.run(['bash', str(ROOT/'scripts/run_isaac_fov_gvf_navigation.sh')],
                        cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
                data = json.loads(result.read_text()) if result.exists() else dict(status='RUNTIME_ERROR')
                data.update(process_exit_code=child.returncode, process_wall_s=time.perf_counter()-start)
                result.write_text(json.dumps(data, indent=2) + '\n')
                try:
                    report = audit(result, scene, shape)
                except Exception as exc:
                    report = dict(safe_arrival=False, status=data['status'], audit_error=str(exc))
                    result.with_suffix('.audit.json').write_text(json.dumps(report, indent=2) + '\n')
                print('END', stem, json.dumps(report), flush=True)
                if not report['safe_arrival'] and args.stop_on_failure:
                    print('STOP: profile rejected on this failed repeat; remaining repeats not run',flush=True)
                    return
                stage_ok &= report['safe_arrival'] and (not args.require_cap or report.get('cap_attained',False))
        if not stage_ok and not args.continue_after_failure:
            print('STOP: failed arrival or requested cap not attained; no automatic increase', flush=True)
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    parser.add_argument('--speeds', type=float, nargs='+', default=[2., 3., 4., 5., 6.])
    parser.add_argument('--maps', nargs='+', choices=MAPS, default=['original'])
    parser.add_argument('--depth', type=float, default=10.)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--continue-after-failure', action='store_true')
    parser.add_argument('--stop-on-failure', action='store_true',help='reject a confirmation profile immediately after a failed repeat')
    parser.add_argument('--require-cap', action='store_true', help='before increasing, require >=95%% cap for at least 1 second')
    parser.add_argument('--emergency', action='store_true', help='certify both turning and straight watchdog stop')
    parser.add_argument('--cache', action='store_true', help='reuse a short-lived world target; certify every command')
    parser.add_argument('--governor', action='store_true', help='certify high-speed direct room; use 2 m/s policy in tight bypasses')
    parser.add_argument('--cache-ttl', type=float, default=.15, help='proposal lifetime only; fresh motion proof still mandatory')
    parser.add_argument('--restart', type=float, default=0., help='extra certified room along the whole stopping corridor')
    parser.add_argument('--abort-stall', type=float, default=0., help='exploration only: explicit failure after stationary seconds; 0 keeps full timeout')
    args = parser.parse_args()
    if args.repeats < 1 or not np.isfinite(args.abort_stall) or args.abort_stall<0 or not np.isfinite(args.cache_ttl) or not 0<args.cache_ttl<=10 or not np.isfinite(args.restart) or args.restart<0 or not np.isfinite(args.depth) or args.depth <= 0 or any(not np.isfinite(s) or s <= 0 for s in args.speeds):
        parser.error('positive finite speed, depth and repeat count required')
    run(args)
