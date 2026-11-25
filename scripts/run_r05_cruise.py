#!/usr/bin/python3
"""Reproduce and check the fixed R05 waypoint-204 FIFO5 cruise experiment."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess

PROJECT = Path(__file__).resolve().parents[1]
EXPERIMENT = PROJECT / 'docs/performance/wp207_fifo_20261119'
REFERENCE = EXPERIMENT / 'run_20261119_142131_038887/simulation'
SUPERVISOR = EXPERIMENT / 'supervise.py'
MANIFEST_SHA = '7402fca316eba0d32d42b0ab4c368d1409a9f1c0e160216462cfc6498e688e48'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def identity():
    manifest = (PROJECT / load(EXPERIMENT / 'latest_build.json')['manifest']).resolve()
    require(manifest == EXPERIMENT / 'manifest.json' and digest(manifest) == MANIFEST_SHA,
            'pinned R05 experiment manifest changed')
    require(digest(SUPERVISOR) == load(manifest)['source_identity'][str(SUPERVISOR.relative_to(PROJECT))],
            'supervisor identity changed')
    subprocess.run(['/usr/bin/python3', '-I', SUPERVISOR, '--check'], check=True,
                   stdout=subprocess.DEVNULL)


def statistics_ms(values):
    samples = sorted(v / 1e6 for v in values)
    return dict(n=len(samples), mean=statistics.mean(samples),
                p99=samples[math.ceil(.99*len(samples))-1], maximum=samples[-1],
                over_5=sum(t > 5 for t in samples))


def verify(folder):
    run = folder.resolve()
    require(run.parent == EXPERIMENT, 'expected a run of the pinned experiment')
    e = load(run / 'execution.json')
    require(e['audit_passed'] and e['exit_code'] == 0 and e['manifest_sha256'] == MANIFEST_SHA,
            'incomplete run or different build')
    threads = e['thread_observations']
    require(e['global_rt_settings_unchanged'] and sorted(t['cpus'] for t in threads) == [[n] for n in range(5)]
            and all(t['policy'] == 1 and t['priority'] == 5 and t['uid'] != 0 and not t['cap_eff']
                    for t in threads), 'FIFO5 thread readback failed')
    require(e['supervisor_policy'] == 0 and any(x['controller_seen'] for x in e['simulator_readbacks'])
            and all(set(x['policies']) <= {0, 3, 5} for x in e['simulator_readbacks']),
            'simulator scheduling check failed')
    sim = run / 'simulation'
    audit, result = load(sim / 'audit.json'), load(sim / 'result.json')
    require(result['status'] == 'ARRIVED' and result['final_speed_mps'] == 0 and
            result['terminal_hold_steps'] == 100 and audit['at_goal'] and
            all(audit[k] == 0 for k in ('swept_overlap_segments','speed_violations','acceleration_violations')),
            'arrival or independent swept-volume audit failed')
    t, c = rows(sim / 'trajectory.csv'), rows(sim / 'controller.csv')
    require(digest(REFERENCE/'trajectory.csv') == '8b3516cfcce0a7f2e1b12cfb54123c7f25ee8f6146aee82918bd241245f7983b'
            and digest(REFERENCE/'controller.csv') == '691a9d633326c06c852c91d3324dc9f5c6104e94e477d72a584126271423f794',
            'reference logs changed')
    for filename, observed, fields in [
        ('trajectory.csv', t, 'step t x y z vx vy vz ux uy uz mode waypoint yaw_rad guide_cap'),
        ('controller.csv', c, 'step mode field_attempted angular_ready spatial_ready reference_commit'),
    ]:
        expected = rows(REFERENCE / filename)
        require(len(expected) == len(observed) and all(a[k] == b[k] for a,b in zip(expected, observed)
                for k in fields.split()), 'trajectory or branch mismatch in ' + filename)
    # Identical trajectory and branch sequence imply identical spatial gate crossings.
    cruise = t[221:5197]
    error = max(abs(math.hypot(*(float(x[k]) for k in ('vx','vy','vz')))-12) for x in cruise)
    require(len(cruise) == 4976 and error <= .001 and all(x['mode'] not in {'3','4','5'} for x in cruise),
            'constant-speed cruise regression')
    clock = rows(sim / 'sensor_times.csv')
    require(len(clock) == len(c) == 5319 and all(abs(float(x[k])-float(x['physics_s'])) < 1e-7
            for x in clock for k in ('front_s','left_s','back_s','right_s')), 'four-camera clock mismatch')
    rolling = [x for x in c if x['mode'] == '1']
    def three(x):
        return sum(int(x[k]) for k in ('planning_ns','certification_ns','control_ns'))
    return dict(run=run.name, manifest_sha256=MANIFEST_SHA, cruise_steps=len(cruise),
                max_speed_error_mps=error, sim_s=result['simulation_s'], swept_collisions=audit['swept_overlap_segments'],
                no_field=statistics_ms(three(x) for x in rolling if x['field_attempted']=='0'),
                field=statistics_ms(three(x) for x in rolling if x['field_attempted']=='1'),
                rolling=statistics_ms(map(three, rolling)),
                takeover=statistics_ms(int(x['step_compute_ns']) for x in c if x['mode']=='7'),
                all_decisions=statistics_ms(int(x['step_compute_ns']) for x in c))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='check pinned inputs without starting Isaac')
    mode.add_argument('--verify', type=Path, help='audit an existing FIFO5 run')
    args = parser.parse_args()
    identity()
    if args.check:
        print('Pinned R05 identities verified')
        return
    run = args.verify
    if run is None:
        require(os.getuid() != 0, 'launch as the regular project user')
        command = ['/usr/bin/sudo','/usr/bin/prlimit','--rtprio=0:5','--rttime=5000000:6000000',
                   '--','/usr/bin/setpriv',f'--reuid={os.getuid()}',f'--regid={os.getgid()}','--init-groups',
                   '--bounding-set=-all','--inh-caps=-all','--ambient-caps=-all','--no-new-privs',
                   '--','/usr/bin/python3','-I',SUPERVISOR,'--headless','--seconds','32','--time-scale','1']
        with subprocess.Popen(command, stdout=subprocess.PIPE, text=True) as process:
            for line in process.stdout:
                print(line, end='', flush=True)
                if line.startswith('RESULT_DIR='):
                    run = Path(line.strip().partition('=')[2])
            require(process.wait() == 0 and run is not None, 'FIFO5 simulation failed')
    report = verify(run)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
