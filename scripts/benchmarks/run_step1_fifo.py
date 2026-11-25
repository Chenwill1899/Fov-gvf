#!/usr/bin/env python3
"""FIFO5 step-1 acceptance: stationary sensors, then independent offline LDLT replay."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import pwd
import resource
import shlex
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from base_config import load_config


def stats(values):
    v = sorted(values)
    if not v:
        raise ValueError('no valid samples')
    return dict(n=len(v), mean=sum(v)/len(v), p99=v[math.ceil(.99*len(v))-1],
                maximum=v[-1], over_5ms=sum(x > 5 for x in v))


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def check_build(config):
    subprocess.run(['/usr/bin/python3', '-I', str(ROOT/'scripts/base_config.py'), '--check'],
                   check=True, stdout=subprocess.DEVNULL)
    install = ROOT/'docs/performance/runtime/install'
    identity = json.loads((install/'new_controller/share/new_controller/build_identity.json').read_text())
    if any(hashlib.sha256((ROOT/k).read_bytes()).hexdigest() != v for k, v in identity.items()):
        raise RuntimeError('source/build identity differs; rebuild P2 before testing')
    exe = install/'new_controller/lib/new_controller/depth_reader'
    command = 'source /opt/ros/humble/setup.bash\nexec ' + shlex.join([str(exe), '--print-config'])
    compiled = json.loads(subprocess.check_output(['/bin/bash', '-c', command], text=True))
    if compiled != config:
        raise RuntimeError('compiled C++ configuration differs from JSON')
    return exe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='read-only configuration/build check')
    parser.add_argument('--phase', choices=('all', 'sensors', 'offline'), default='all')
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--repeats', type=int, default=200)
    args = parser.parse_args()
    if not 1 <= args.rounds <= 4 or not 1 <= args.repeats <= 1000:
        parser.error('rounds must be 1..4 and repeats 1..1000')
    config, sha = load_config()
    exe = check_build(config)
    scope = dict(base_config=config, base_config_sha256=sha, phase=args.phase,
                 fifo_priority=5, camera_cpus=[1, 2, 3, 4], coordinator_cpu=0,
                 sensor_window_sim_s=.2, rounds=args.rounds, repeats=args.repeats,
                 root_algorithm=False, motion_output_enabled=False,
                 global_settings_changed=False,
                 sensor_stage='input/omega only; LDLT remains an independent offline candidate')
    if args.check:
        print(json.dumps(dict(scope=scope, compiled_config_matches=True), indent=2))
        return 0
    if os.geteuid() != 0:
        parser.error('run sudo /usr/bin/python3 -I ' + str(Path(__file__).resolve()))
    if os.sched_getscheduler(0) != os.SCHED_OTHER:
        parser.error('the supervisor must remain SCHED_OTHER')
    owner = pwd.getpwuid(ROOT.stat().st_uid)
    if owner.pw_uid == 0:
        raise RuntimeError('project must have a non-root owner')
    session = ROOT/'docs/performance'/('step1_fifo_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    session.mkdir()
    os.chown(session, owner.pw_uid, owner.pw_gid)
    env = dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin', LANG='C.UTF-8')
    desktop = {}
    runtime = Path('/run/user')/str(owner.pw_uid)
    if runtime.is_dir():
        desktop['XDG_RUNTIME_DIR'] = str(runtime)
    for auth in (runtime/'gdm/Xauthority', Path(owner.pw_dir)/'.Xauthority'):
        if auth.is_file() and auth.stat().st_uid == owner.pw_uid:
            desktop['XAUTHORITY'] = str(auth)
            break
    sockets = sorted(Path('/tmp/.X11-unix').glob('X[0-9]*'))
    if len(sockets) == 1:
        desktop['DISPLAY'] = ':' + sockets[0].name[1:]

    def save(name, value):
        target = session/name
        target.write_text(json.dumps(value, indent=2)+'\n')
        os.chown(target, owner.pw_uid, owner.pw_gid)

    def user_command(command):
        return ['/usr/bin/setpriv', '--reuid', str(owner.pw_uid), '--regid', str(owner.pw_gid),
                '--init-groups', '--reset-env', '--no-new-privs', '--bounding-set=-all',
                '--inh-caps=-all', '--ambient-caps=-all', '--', '/usr/bin/env',
                *[k+'='+v for k, v in desktop.items()], *map(str, command)]

    save('scope.json', scope)
    result = dict(status='running', motion_output_enabled=False)
    job = None
    known_children = {}
    try:
        if args.phase in ('all', 'sensors'):
            sensor = session/'sensors'
            command = user_command(['/usr/bin/prlimit', '--rtprio=0:0', '--', '/bin/bash',
                                    ROOT/'scripts/run_depth_step1.sh', '--output', sensor,
                                    '--reader-cpu', '0', '--fifo-priority', '5', '--spin-us', '200',
                                    '--record-physics', '--scheduler-gate', '--no-lock-memory'])
            print('stationary sensor check: 200 sim Hz, 5ms physics, .02 speed; no motion output', flush=True)
            with (session/'sensors_console.log').open('w') as log:
                job = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                       start_new_session=True)
                started = time.monotonic()
                last_notice = 0
                authorized = False
                observed = []
                while job.poll() is None:
                    elapsed = time.monotonic()-started
                    if elapsed > 600:
                        raise RuntimeError('stationary sensor check exceeded 600s')
                    try:
                        for child_pid in Path(f'/proc/{job.pid}/task/{job.pid}/children').read_text().split():
                            proc = Path('/proc')/child_pid
                            if proc.stat().st_uid == owner.pw_uid:
                                known_children[int(child_pid)] = (proc/'stat').read_text().rsplit(')', 1)[1].split()[19]
                    except FileNotFoundError:
                        pass
                    gate = sensor/'reader_scheduler_ready' 
                    if not authorized and gate.exists():
                        pid = int(gate.read_text())
                        task = Path('/proc')/str(pid)
                        direct = Path(f'/proc/{job.pid}/task/{job.pid}/children').read_text().split()
                        if str(pid) not in direct or task.stat().st_uid != owner.pw_uid or (task/'exe').resolve() != exe:
                            raise RuntimeError('reader process identity/ancestry mismatch')
                        resource.prlimit(pid, resource.RLIMIT_RTPRIO, (5, 5))
                        resource.prlimit(pid, resource.RLIMIT_RTTIME, (5_000_000, 6_000_000))
                        subprocess.run(user_command(['/usr/bin/touch', sensor/'reader_scheduler_authorized']),
                                       env=env, check=True)
                        authorized = True
                        result['reader_rtprio_limit'] = list(resource.prlimit(pid, resource.RLIMIT_RTPRIO))
                    if authorized and (sensor/'reader_ready').exists():
                        try:
                            tasks = []
                            for t in task.joinpath('task').iterdir():
                                tid = int(t.name)
                                name = (t/'comm').read_text().strip()
                                if tid == pid or name in ['depth_cam'+str(c) for c in range(4)]:
                                    tasks.append(dict(tid=tid, name=name, policy=os.sched_getscheduler(tid),
                                                      priority=os.sched_getparam(tid).sched_priority,
                                                      cpus=sorted(os.sched_getaffinity(tid))))
                            if len(tasks) == 5:
                                if any(t['policy'] != os.SCHED_FIFO or t['priority'] != 5 for t in tasks):
                                    raise RuntimeError('five algorithm threads are not FIFO5')
                                if any(t['cpus'] != [0 if t['tid'] == pid else int(t['name'][-1])+1] for t in tasks):
                                    raise RuntimeError('algorithm CPU affinity differs')
                                observed = tasks
                        except (FileNotFoundError, ProcessLookupError):
                            pass
                    if elapsed-last_notice >= 10:
                        print(f'stationary sensors: {elapsed:.0f}s; '+('FIFO5 read back' if observed else 'waiting for startup'), flush=True)
                        last_notice = elapsed
                    time.sleep(.05)
                if job.returncode:
                    raise RuntimeError('sensor check failed; see sensors_console.log and sensors/orchestrator_error.json')
                if not observed:
                    raise RuntimeError('no live five-thread FIFO5 readback')
            save('sensor_fifo_readback.json', observed)
            m = json.loads((sensor/'reader_meta.json').read_text())
            sim = json.loads((sensor/'simulation.json').read_text())
            cal = json.loads((sensor/'simulation.sensors.json').read_text())
            if json.loads((sensor/'base_config.json').read_text()) != config or sim['base_config'] != config or m['base_config'] != config or m['base_config_sha256'] != sha or sim['base_config_sha256'] != sha:
                raise RuntimeError('runtime profile readback differs')
            if m['actual_scheduler_policy'] != os.SCHED_FIFO or m['actual_priority'] != 5:
                raise RuntimeError('reader FIFO metadata differs')
            # Export metadata and observed policy must independently agree.
            workers = m['direction_depth_queries']
            if workers['worker_policies'] != [os.SCHED_FIFO]*4 or workers['worker_priorities'] != [5]*4 or workers['worker_cpus'] != [1,2,3,4]:
                raise RuntimeError('worker FIFO export differs from live readback')
            if m['local_field_preparation']['local_range_m'] != 4 or m['desired_motion']['max_speed_mps'] != 12:
                raise RuntimeError('runtime range/speed limits differ')
            if m['requested_period_ns'] != 5_000_000 or m['motion_publishers_created'] != 0:
                raise RuntimeError('reader period/output mismatch')
            if any(not math.isclose(sim[k], .005, abs_tol=1e-8) for k in ('actual_physics_dt', 'actual_render_dt')):
                raise RuntimeError('actual simulator step differs from 5ms')
            if cal['actual_sensor_tick_rate'] != [200.]*4 or sim['command_publishers_created'] != 0:
                raise RuntimeError('camera frequency or output mismatch')
            if math.dist(sim['initial_position'], sim['final_position']) > 1e-9:
                raise RuntimeError('read-only sensor test moved the body')
            updates = read_rows(sensor/'simulator_updates.csv')
            deltas = [float(r['sim_end_s'])-float(r['sim_start_s']) for r in updates]
            positive = [v for v in deltas if v > 1e-10]
            physics = [float(r['step_dt_s']) for r in read_rows(sensor/'physics_events.csv')]
            if not positive or not physics or any(not math.isclose(v, .005, abs_tol=1e-8) for v in positive+physics):
                raise RuntimeError('observed timeline/physics callbacks are not 5ms')
            reads = read_rows(sensor/'reads.csv')
            camera_periods = {}
            received = read_rows(sensor/'received.csv')
            for camera in range(4):
                stamps = sorted({int(r['image_stamp_ns']) for r in received if int(r['camera']) == camera})
                gaps = [b-a for a,b in zip(stamps,stamps[1:])]
                if not gaps or any(g <= 0 or abs(g-round(g/5_000_000)*5_000_000)>100 for g in gaps):
                    raise RuntimeError('received camera timestamps do not follow the 5ms simulation grid')
                camera_periods[str(camera)] = dict(interval_ms=stats([g/1e6 for g in gaps]),
                                                   missing_intervals=sum(round(g/5_000_000)-1 for g in gaps))
            scheduled = [int(r['scheduled_ns']) for r in reads]
            if any((b-a) <= 0 or (b-a) % 5_000_000 for a,b in zip(scheduled,scheduled[1:])):
                raise RuntimeError('reader absolute releases do not follow the 5ms wall grid')
            result['sensors'] = dict(camera_timestamp_intervals=camera_periods, actual_sim_step_ms=stats([1000*v for v in positive]),
                                     actual_reader_interval_ms=stats([(int(b['wall_monotonic_ns'])-int(a['wall_monotonic_ns']))/1e6 for a,b in zip(reads,reads[1:])]),
                                     physics_callback_count=len(physics), fifo_threads=5, motion_publishers=0)
            print('stationary configuration readbacks passed; reader wall jitter is reported separately', flush=True)

        if args.phase in ('all', 'offline'):
            records = []
            for round_id in range(1, args.rounds+1):
                output = session/f'offline_r{round_id}'
                command = ['/usr/bin/python3', '-I', str(ROOT/'scripts/benchmarks/run_ldlt_candidate_benchmark.py'),
                           str(ROOT/'docs/performance/current_baseline_20261105_141800/input'), str(output),
                           '--repeats', str(args.repeats)]
                subprocess.run(command, check=True, env=env)
                run = output/'run'
                if json.loads((run/'base_config.json').read_text()) != config:
                    raise RuntimeError('candidate compiled profile differs')
                timing_rows = read_rows(run/'timings.csv')
                if any(r['status'] != 'valid' or r['termination'] != 'exit' for r in timing_rows if r['case'].startswith('supported_')):
                    raise RuntimeError('supported real replay stopped early; review before reporting timing')
                releases = read_rows(run/'releases.csv')
                records.extend(dict(r, round=str(round_id)) for r in releases)
                print(f'offline_r{round_id}: FIFO5 verified, 5ms releases, {len(releases)} records', flush=True)
            summary = {}
            for mode in ('cold', 'boundary_change', 'new_group_reuse'):
                # Supported real snapshots are distinct from synthetic/unknown early exits.
                rows = [r for r in records if r['mode'] == mode and r['case'].startswith('supported_')]
                summary[mode] = dict(compute_ms=stats([float(r['compute_ms']) for r in rows]),
                                     response_ms=stats([float(r['response_ms']) for r in rows]),
                                     skipped_releases=sum(int(r['skipped_releases']) for r in rows))
            result['offline_supported_real'] = summary
            save('offline_releases.json', records)
        result['status'] = 'passed_parameter_acceptance'
        result['new_motion_implemented'] = False
        result['online_ldlt_integrated'] = False
        result['long_term_deadline_guarantee'] = False
        save('summary.json', result)
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        result.update(status='failed', error=str(exc))
        save('summary.json', result)
        raise
    finally:
        if job is not None and job.poll() is None:
            os.killpg(job.pid, signal.SIGINT)
            try:
                job.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(job.pid, signal.SIGKILL)
                job.wait()
        # Child sessions were created by this wrapper; guard against PID reuse.
        for child_pid, start_ticks in known_children.items():
            try:
                proc = Path('/proc')/str(child_pid)
                if proc.stat().st_uid == owner.pw_uid and (proc/'stat').read_text().rsplit(')', 1)[1].split()[19] == start_ticks:
                    os.killpg(child_pid, signal.SIGTERM)
            except (FileNotFoundError, ProcessLookupError):
                pass
        for f in session.iterdir():
            if f.is_file():
                os.chown(f, owner.pw_uid, owner.pw_gid)
        print('RESULT_DIR='+str(session), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
