#!/usr/bin/env python3
"""Default: Step3 scan planning and Step5 rolling-reference replay; explicit legacy flag retains the old builder."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import pwd
import resource
import signal
import subprocess
import time

PROJECT = Path(__file__).resolve().parents[2]


def child_limits(fifo):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    resource.setrlimit(resource.RLIMIT_RTPRIO, (5, 5) if fifo else (0, 0))
    resource.setrlimit(resource.RLIMIT_RTTIME, (5_000_000, 6_000_000))
    resource.setrlimit(resource.RLIMIT_CPU, (90, 95))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def inventory(pid):
    result = []
    for task in (Path('/proc') / str(pid) / 'task').iterdir():
        tid = int(task.name)
        status = dict(line.split(':', 1) for line in (task / 'status').read_text().splitlines() if ':' in line)
        result.append(dict(tid=tid, name=(task / 'comm').read_text().strip(),
                           policy=os.sched_getscheduler(tid),
                           priority=os.sched_getparam(tid).sched_priority,
                           cpus=sorted(os.sched_getaffinity(tid)),
                           uid=status['Uid'].split()[0], cap_eff=status['CapEff'].strip()))
    return sorted(result, key=lambda item: item['tid'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archived_input', type=Path, nargs='?',
                        default=PROJECT / 'docs/performance/current_baseline_20261105_141800/input')
    parser.add_argument('new_output', type=Path, nargs='?')
    parser.add_argument('--repeats', type=int, default=200)
    parser.add_argument('--input-mode', choices=('parallel', 'serial'), default='parallel')
    parser.add_argument('--scheduler', choices=('fifo5', 'normal'), default='fifo5',
                        help='normal is an explicit diagnostic comparison, never an automatic fallback')
    parser.add_argument('--exports', action='store_true')
    parser.add_argument('--legacy-input-only', action='store_true', help='explicitly run the retained pre-unified-input body-only benchmark')
    parser.add_argument('--check', action='store_true', help='print the planned scope without building/running')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--source-dir', type=Path, help='project-local frozen source for a controlled comparison')
    parser.add_argument('--benchmark-source', type=Path, help='project-local common measurement driver')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 1000:
        parser.error('repeats must be between 1 and 1000')
    source_input = args.archived_input.resolve()
    for name in ('first_depth.bin', 'last_depth.bin', 'simulation.sensors.json'):
        if not (source_input / name).is_file():
            parser.error(f'missing input: {source_input / name}')
    candidate = (args.source_dir or PROJECT / 'src/new_controller/ldlt').resolve()
    benchmark_source = (args.benchmark_source or candidate / 'test/harmonic_fields_benchmark.cpp').resolve()
    if not candidate.is_relative_to(PROJECT) or not benchmark_source.is_relative_to(PROJECT):
        parser.error('comparison source and driver must be inside EGO4P2')
    if not (candidate / 'include/new_controller/trial_field_solver.hpp').is_file() or not benchmark_source.is_file():
        parser.error('candidate source or measurement driver is missing')
    if not args.source_dir:
        subprocess.run(['/usr/bin/python3', '-I', str(PROJECT/'scripts/base_config.py'), '--check'],
                       check=True, stdout=subprocess.DEVNULL)
    # The ordinary entry follows the current unified chain; old frozen source
    # comparisons remain explicit. Do not silently measure the .10m old input.
    if (PROJECT/'src/new_controller/config/navigation_step2.json').exists() and not args.legacy_input_only and not args.source_dir and not args.benchmark_source and not args.build_only:
        if args.input_mode!='parallel' or args.scheduler!='fifo5' or args.exports or args.new_output or source_input!=(PROJECT/'docs/performance/current_baseline_20261105_141800/input').resolve():
            parser.error('current unified FIFO comparison: run scripts/benchmarks/run_step5_fifo.py; retained old benchmark requires --legacy-input-only')
        new_command=['/usr/bin/python3','-I',str(PROJECT/'scripts/benchmarks/run_step5_fifo.py'),'--repeats',str(args.repeats)]
        if args.check:new_command.append('--check')
        os.execv(new_command[0],new_command)
    fifo = args.scheduler == 'fifo5'
    root = os.geteuid() == 0
    uid = int(os.environ.get('SUDO_UID', PROJECT.stat().st_uid)) if root else os.getuid()
    if uid == 0:
        parser.error('the algorithm and compiler must run as the non-root project user')
    owner = pwd.getpwuid(uid)
    output = (args.new_output or PROJECT / 'docs/performance' /
              ('fifo_run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    policy = os.SCHED_FIFO if fifo else os.SCHED_OTHER
    priority = 5 if fifo else 0
    scope = dict(input=str(source_input), output=str(output), scheduler=args.scheduler,
                 policy=policy, priority=priority, coordinator_cpu=0,
                 worker_cpus=[1, 2, 3, 4] if args.input_mode == 'parallel' else [],
                 owner_uid=owner.pw_uid, owner_gid=owner.pw_gid,
                 root_supervisor=root, algorithm_runs_as_root=False,
                 repeats=args.repeats, input_mode=args.input_mode,
                 build_only=args.build_only, wall_timeout_s=180,
                 child_rtprio_limit=5 if fifo else 0,
                 child_rttime_us=[5_000_000, 6_000_000],
                 global_settings_changed=False, fallback_to_normal=False)
    scope.update(candidate_source=str(candidate), benchmark_source=str(benchmark_source))
    if not args.source_dir:
        scope['base_config'] = json.loads((PROJECT/'src/new_controller/config/base_step1.json').read_text())
        if not args.benchmark_source:
            scope['release_protocol'] = 'periodic5ms_skip_missed_no_catchup'
    if args.check:
        print(json.dumps(scope, indent=2))
        return
    if fifo and not args.build_only and not root and resource.getrlimit(resource.RLIMIT_RTPRIO)[0] < 5:
        parser.error('FIFO5 permission is required; run sudo /usr/bin/python3 -I ' + str(Path(__file__).resolve()) +
                     ' with the same arguments. No algorithm was started.')
    if root and os.sched_getscheduler(0) != os.SCHED_OTHER:
        parser.error('the supervising Python process must use SCHED_OTHER')
    if output.exists():
        parser.error('choose a new output directory; existing evidence is preserved')
    output.mkdir(parents=True)
    for folder in (output, output / 'bin', output / 'logs'):
        folder.mkdir(exist_ok=True)
        if root:
            os.chown(folder, owner.pw_uid, owner.pw_gid)
    historical = PROJECT / 'docs/performance/harmonic_block4_20261104_213849'
    library = historical / 'deps/suitesparse/usr/lib/x86_64-linux-gnu'
    env = dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin', LANG='C.UTF-8',
               HOME=owner.pw_dir, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
               LD_LIBRARY_PATH=str(library))
    records = []

    def save(path, value):
        path.write_text(json.dumps(value, indent=2) + '\n')
        if root:
            os.chown(path, owner.pw_uid, owner.pw_gid)

    def translate(command):
        original = '/home/starry/isaac-data/EGO4P0/docs/performance/harmonic_block4_20261104_213849'
        result = [part.replace(original, str(historical))
                  .replace(str(historical / 'work/EGO4P0/src/new_controller'), str(candidate))
                  .replace(str(historical / 'bin'), str(output / 'bin')) for part in command]
        assert not any('/home/starry/isaac-data/EGO4P0/' in part for part in result)
        return result

    def as_owner(command):
        if not root:
            return command
        return ['/usr/bin/setpriv', '--reuid', str(owner.pw_uid), '--regid', str(owner.pw_gid),
                '--init-groups', '--reset-env', '--no-new-privs', '--bounding-set=-all',
                '--inh-caps=-all', '--ambient-caps=-all', '--', '/usr/bin/env'] + [
                    key + '=' + value for key, value in env.items()] + command

    def execute(name, command, benchmark=False):
        real_command = as_owner((['/usr/bin/taskset', '-c', '0-4'] if benchmark else []) + command)
        started = time.monotonic()
        record = dict(name=name, command=real_command, requested_policy=policy if benchmark else os.SCHED_OTHER,
                      requested_priority=priority if benchmark else 0)
        child = None
        try:
            with (output / 'logs' / (name + '.log')).open('w') as log:
                # User-owned numerical libraries are added by env only after setpriv.
                launch_env = dict(PATH=env['PATH'], LANG=env['LANG']) if root else env
                child = subprocess.Popen(real_command, env=launch_env, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True,
                                         preexec_fn=(lambda: child_limits(fifo)) if benchmark else None)
                observed = []
                while child.poll() is None:
                    if time.monotonic() - started > 180:
                        raise RuntimeError('child wall timeout')
                    if benchmark and (output / 'run/preprocess_execution.json').exists():
                        try:
                            tasks = inventory(child.pid)
                            expected = 5 if args.input_mode == 'parallel' else 1
                            if len(tasks) == expected:
                                if any(t['policy'] != policy or t['priority'] != priority for t in tasks):
                                    raise RuntimeError('requested scheduling policy was not maintained')
                                if any(int(t['uid']) != owner.pw_uid or int(t['cap_eff'], 16) for t in tasks):
                                    raise RuntimeError('algorithm privilege drop was not confirmed')
                                by_name = {t['name']: t for t in tasks}
                                if args.input_mode == 'parallel' and any(
                                        by_name.get('depth_cam' + str(c), {}).get('cpus') != [c + 1]
                                        for c in range(4)):
                                    raise RuntimeError('camera affinity changed during the run')
                                if next(t for t in tasks if t['tid'] == child.pid)['cpus'] != [0]:
                                    raise RuntimeError('coordinator affinity changed during the run')
                                observed = tasks
                        except (FileNotFoundError, ProcessLookupError):
                            pass
                    time.sleep(.01)
                if child.returncode:
                    raise RuntimeError(f'{name} exited with {child.returncode}; see {output / "logs"}')
                if benchmark:
                    metadata = json.loads((output / 'run/preprocess_execution.json').read_text())
                    if not args.source_dir and not args.benchmark_source:
                        compiled = json.loads((output/'run/base_config.json').read_text())
                        if compiled != scope['base_config'] or metadata['period_ns'] != 5_000_000:
                            raise RuntimeError('candidate compiled configuration differs from the base profile')
                    if metadata['coordinator_scheduler'] != policy or metadata['coordinator_priority'] != priority:
                        raise RuntimeError('coordinator policy readback mismatch')
                    if args.input_mode == 'parallel' and (metadata['worker_policies'] != [policy] * 4 or
                            metadata['worker_priorities'] != [priority] * 4 or metadata['worker_cpus'] != [1, 2, 3, 4]):
                        raise RuntimeError('camera policy/priority/affinity readback mismatch')
                    record.update(verified_metadata=metadata, last_observed_threads=observed)
                record['exit_code'] = child.returncode
        except BaseException as error:
            if child is not None and child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
            record.update(exit_code=None if child is None else child.returncode, error=repr(error))
            raise
        finally:
            record['wall_s'] = time.monotonic() - started
            records.append(record)
            save(output / 'execution.json', records)
            if root:
                os.chown(output / 'logs' / (name + '.log'), owner.pw_uid, owner.pw_gid)

    save(output / 'scope.json', scope)
    hashes = {str(path.relative_to(candidate)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in candidate.rglob('*') if path.is_file()}
    save(output / 'source_sha256.json', hashes)
    save(output / 'measurement_source_sha256.json', {
        str(benchmark_source): hashlib.sha256(benchmark_source.read_bytes()).hexdigest()})
    execute('build_backend', translate(json.loads((historical / 'backend_build.json').read_text())['command']))
    pipeline_command = translate(json.loads((historical / 'pipeline_build_commands.json').read_text())[1])
    pipeline_command[pipeline_command.index(str(candidate / 'test/harmonic_fields_benchmark.cpp'))] = str(benchmark_source)
    execute('build_pipeline', pipeline_command)
    save(output / 'binary_sha256.json', {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                       for path in (output / 'bin').iterdir() if path.is_file()})
    if not args.build_only:
        command = [str(output / 'bin/pipeline_small'), str(source_input), str(output / 'run'),
                   str(args.repeats), 'eigen_ldlt_both', '4']
        if not args.exports:
            command.append('no_exports')
        command.extend([args.input_mode, args.scheduler])
        original_affinity = os.sched_getaffinity(0)
        try:
            if 5 in original_affinity:
                os.sched_setaffinity(0, {5})
            execute('benchmark', command, benchmark=True)
        finally:
            os.sched_setaffinity(0, original_affinity)
    print('RESULT_DIR=' + str(output))


if __name__ == '__main__':
    main()
