#!/usr/bin/python3
"""Launch isolated visible P3 simulation; only five C++ threads may use FIFO5."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import pwd
import resource
import shutil
import signal
import stat
import subprocess
import tempfile
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / 'docs/performance/isaac_closed_loop_20261117'
SHARED_CACHE = EXPERIMENT / 'shared_cache'
ISAAC = ROOT.parent / 'isaacsim'
SIMULATOR = ROOT / 'scripts/isaac/run_closed_loop_simulation.py'
AUDITOR = ROOT / 'scripts/audit_isaac_closed_loop.py'
EXECUTION_AUDITOR = ROOT / 'scripts/audit_isaac_execution.py'
SCENARIOS = ('straight', 'climb', 'offset_obstacle', 'blocking_obstacle')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_manifest(manifest):
    checked = 0
    def local(relative, wanted):
        nonlocal checked
        path = (ROOT / relative).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file() or digest(path) != wanted:
            raise RuntimeError('stale or missing validated project input: ' + relative)
        checked += 1
    if not manifest.get('simulation_only') or manifest.get('world_dimensions') != 3:
        raise RuntimeError('manifest is not the P3 simulation-only build')
    for relative, wanted in manifest['source_identity'].items():
        local(relative, wanted)
    local(manifest['binary']['path'], manifest['binary']['sha256'])
    local(manifest['supervisor_check']['path'], manifest['supervisor_check']['sha256'])
    local(manifest['source_archive']['path'], manifest['source_archive']['sha256'])
    for relative, wanted in manifest['production_identity'].items():
        local(relative, wanted)
    for filename, wanted in manifest['dependencies'].items():
        if digest(Path(filename)) != wanted:
            raise RuntimeError('runtime dependency changed: ' + filename)
        checked += 1
    for filename, wanted in manifest['simulation_runtime_identity'].items():
        if digest(Path(filename)) != wanted:
            raise RuntimeError('simulation runtime entry changed: ' + filename)
        checked += 1
    for filename, wanted in manifest['compile_dependency_identity'].items():
        if digest(Path(filename)) != wanted:
            raise RuntimeError('compiled source/header dependency changed: ' + filename)
        checked += 1
    local(str((EXPERIMENT / 'before/source.tar.gz').relative_to(ROOT)), manifest['snapshot_sha256'])
    return checked


def finite_tree(value):
    if isinstance(value, float) and not math.isfinite(value):
        return False
    if isinstance(value, dict):
        return all(finite_tree(v) for v in value.values())
    if isinstance(value, list):
        return all(finite_tree(v) for v in value)
    return True


def validate_scenario(scenario, base):
    if not finite_tree(scenario) or scenario.get('schema') != 1 or scenario.get('static_scene') is not True:
        raise ValueError('scenario requires finite values, schema 1 and a fixed static scene')
    expected = dict(physics_dt_s=base['physics_dt_s'], body_radius_m=base['body_radius_m'],
                    camera_offset_m=0., camera_near_m=.1, camera_far_m=50.,
                    image_width=320, image_height=240, camera_hfov_deg=90.)
    for name, value in expected.items():
        if scenario.get(name) != value:
            raise ValueError('scenario model differs from validated contract: ' + name)
    for name in ['start', 'goal', 'bounds_min', 'bounds_max']:
        if not isinstance(scenario.get(name), list) or len(scenario[name]) != 3:
            raise ValueError('scenario needs a 3-D ' + name)
    if not 0 < scenario['cruise_speed_mps'] <= base['max_speed_mps']:
        raise ValueError('invalid scenario cruise speed')
    if not base['body_radius_m'] < scenario['seed_radius_m'] <= base['local_range_m']:
        raise ValueError('invalid certified-spawn sphere radius')
    if not 0 < scenario['simulation_timeout_s'] <= 300 or not 0 < scenario['wall_timeout_s'] <= 7200:
        raise ValueError('scenario time limits must be finite and bounded')
    for i in range(3):
        if not scenario['bounds_min'][i] < scenario['bounds_max'][i]:
            raise ValueError('invalid room bounds')
        for name in ['start', 'goal']:
            if not scenario['bounds_min'][i] + base['body_radius_m'] < scenario[name][i] < scenario['bounds_max'][i] - base['body_radius_m']:
                raise ValueError(name + ' lies outside the finite room')
    for box in scenario['boxes']:
        if len(box['min']) != 3 or len(box['max']) != 3 or any(a >= b for a, b in zip(box['min'], box['max'])):
            raise ValueError('invalid box bounds')


def inventory(pid):
    result = []
    for path in (Path('/proc') / str(pid) / 'task').iterdir():
        try:
            tid = int(path.name)
            status = dict(line.split(':', 1) for line in (path / 'status').read_text().splitlines() if ':' in line)
            result.append(dict(tid=tid, name=(path / 'comm').read_text().strip(),
                policy=os.sched_getscheduler(tid), priority=os.sched_getparam(tid).sched_priority,
                cpus=sorted(os.sched_getaffinity(tid)), uid=int(status['Uid'].split()[0]),
                uids=list(map(int, status['Uid'].split())),
                cap_eff=int(status['CapEff'].strip(), 16),
                cap_prm=int(status['CapPrm'].strip(), 16),
                no_new_privs=int(status['NoNewPrivs'].strip())))
        except (FileNotFoundError, ProcessLookupError):
            # Kit creates transient ordinary workers; a vanished thread is not a
            # scheduling mismatch. The controller still requires exactly five.
            continue
    return sorted(result, key=lambda row: row['tid'])


def process_identity(pid, libraries=False):
    process = Path('/proc') / str(pid)
    executable = (process / 'exe').resolve(strict=True)
    limits = {}
    for line in (process / 'limits').read_text().splitlines():
        words = line.split()
        if words[:3] == ['Max', 'realtime', 'priority']:
            limits['rtprio'] = words[3:5]
        elif words[:3] == ['Max', 'realtime', 'timeout']:
            limits['rttime_us'] = words[3:5]
    result = dict(executable=str(executable), executable_sha256=digest(executable),
                cmdline=(process / 'cmdline').read_bytes().decode().rstrip('\0').split('\0'),
                limits=limits)
    if libraries:
        mapped = set()
        for line in (process / 'maps').read_text().splitlines():
            words = line.split(maxsplit=5)
            if len(words) == 6 and words[5].startswith('/') and '.so' in words[5]:
                mapped.add(Path(words[5]).resolve(strict=True))
        result['mapped_libraries_sha256'] = {str(path): digest(path) for path in sorted(mapped)}
    return result


def thread_violations(threads, uid, policy, priority, expected_cpus=None):
    """Return field-level evidence for the explicitly requested scheduling scope."""
    violations = []
    if not threads:
        violations.append(dict(field='thread_count', expected='at least one', observed=0))
    if expected_cpus is not None:
        names = sorted(thread['name'] for thread in threads)
        if names != sorted(expected_cpus):
            violations.append(dict(field='thread_names', expected=sorted(expected_cpus), observed=names))
    for thread in threads:
        expected = dict(uids=[uid] * 4, cap_eff=0, cap_prm=0, no_new_privs=1,
                        policy=policy, priority=priority)
        if expected_cpus is not None and thread['name'] in expected_cpus:
            expected['cpus'] = [expected_cpus[thread['name']]]
        for field, wanted in expected.items():
            mismatch = thread.get(field) not in wanted if field == 'policy' and isinstance(wanted, list) else thread.get(field) != wanted
            if mismatch:
                violations.append(dict(tid=thread['tid'], name=thread['name'], field=field,
                                       expected=wanted, observed=thread.get(field)))
    return violations


def child_limits(algorithm, fifo, cpu_seconds):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    resource.setrlimit(resource.RLIMIT_RTPRIO, (5, 5) if algorithm and fifo else (0, 0))
    resource.setrlimit(resource.RLIMIT_RTTIME, (5_000_000, 6_000_000) if algorithm and fifo else (0, 0))
    resource.setrlimit(resource.RLIMIT_CPU, (cpu_seconds, cpu_seconds + 5))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def ordinary_or_limited_command(argv, env, owner, root):
    prefix = ['/usr/bin/setpriv']
    if root:
        prefix += ['--reuid', str(owner.pw_uid), '--regid', str(owner.pw_gid), '--init-groups',
                   '--reset-env', '--bounding-set=-all', '--inh-caps=-all', '--ambient-caps=-all']
    prefix += ['--no-new-privs', '--', '/usr/bin/env']
    return prefix + [k + '=' + v for k, v in env.items()] + list(map(str, argv))


def stop_group(child, grace=5.):
    if child is None:
        return
    # A wrapper may exit while its Python/Kit descendants are still alive.
    for sig, wait in [(signal.SIGTERM, grace), (signal.SIGKILL, 2.)]:
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
        if sig == signal.SIGKILL:
            break
        # Reap any surviving members of our own process group as well.
        try:
            os.killpg(child.pid, 0)
        except ProcessLookupError:
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS, default='climb')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-overlay', action='store_true')
    parser.add_argument('--check', action='store_true', help='identity/dependency/argument checks only; never starts applications')
    parser.add_argument('--correctness-only', action='store_true', help='explicit non-root SCHED_OTHER run; no performance conclusion')
    parser.add_argument('--steps', type=int, help='maximum simulation steps; defaults to the scenario time limit')
    parser.add_argument('--time-scale', type=float, help='wall-time pacing scale, default base_step1.json (.02)')
    parser.add_argument('--wall-timeout', type=float, help='bounded run time after camera readiness; initialization separately bounded by scenario wall_timeout_s')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--manifest', type=Path)
    args = parser.parse_args()
    manifest_path = (args.manifest or ROOT / json.loads((EXPERIMENT / 'latest_build.json').read_text())['manifest']).resolve()
    if not manifest_path.is_relative_to(EXPERIMENT):
        parser.error('choose a build manifest inside the isolated simulation experiment')
    manifest = json.loads(manifest_path.read_text())
    sha_checks = verify_manifest(manifest)
    base = json.loads((ROOT / 'src/new_controller/config/base_step1.json').read_text())
    scenario_path = ROOT / manifest['scenarios'][args.scenario]
    scenario = json.loads(scenario_path.read_text())
    validate_scenario(scenario, base)
    scene_spec = importlib.util.spec_from_file_location('p3_closed_loop_scene', ROOT / 'scripts/isaac/closed_loop_scene.py')
    scene_module = importlib.util.module_from_spec(scene_spec)
    scene_spec.loader.exec_module(scene_module)
    # Pure geometry checks verify seed/start/goal; this module defers USD imports.
    scene_module.read_scenario(scenario_path)
    steps = args.steps if args.steps is not None else math.ceil(scenario['simulation_timeout_s'] / base['physics_dt_s'])
    time_scale = args.time_scale if args.time_scale is not None else base['time_scale']
    wall_timeout = args.wall_timeout if args.wall_timeout is not None else scenario['wall_timeout_s']
    initialization_timeout = scenario['wall_timeout_s']
    if not 1 <= steps <= 200_000 or not math.isfinite(time_scale) or not 0 < time_scale <= 1:
        parser.error('steps must be 1..200000 and time-scale finite in (0,1]')
    if not math.isfinite(wall_timeout) or not 1 <= wall_timeout <= 7200:
        parser.error('wall timeout must be 1..7200 seconds')
    if not (ISAAC / 'python.sh').is_file() or not (ISAAC / 'kit/kit').is_file():
        parser.error('local Isaac installation is unavailable')
    root = os.geteuid() == 0
    uid = ROOT.stat().st_uid
    if uid == 0:
        parser.error('project must belong to a non-root user')
    owner = pwd.getpwuid(uid)
    if not root and os.getuid() != uid:
        parser.error('run as the project owner')
    display = os.environ.get('DISPLAY', ':0')
    xauthority = Path(os.environ.get('XAUTHORITY') or f'/run/user/{uid}/gdm/Xauthority')
    if not args.headless and (not display or not xauthority.is_file()):
        parser.error('visible mode requires the user desktop DISPLAY and XAUTHORITY')
    if os.sched_getscheduler(0) != os.SCHED_OTHER:
        parser.error('supervisor must use SCHED_OTHER')
    if not set(range(5)).issubset(os.sched_getaffinity(0)):
        parser.error('the supervisor must inherit access to CPUs 0 through 4 before the worker pool is created')
    fifo = not args.correctness_only
    scope = dict(simulation_only=True, scenario=args.scenario, scenario_sha256=digest(scenario_path),
        manifest=str(manifest_path), source_sha_checks=sha_checks, world_dimensions=3,
        plant_model='fixed-step first-order kinematic velocity model; no rigid-body quadrotor dynamics',
        headless=args.headless, overlay_enabled=not args.no_overlay, steps=steps, time_scale=time_scale,
        physics_dt_s=base['physics_dt_s'], wall_timeout_s=wall_timeout,
        initialization_timeout_s=initialization_timeout,
        total_wall_bound_s=initialization_timeout + wall_timeout,
        total_wall_bound_scope='application initialization plus run; cleanup and two audits are separate',
        cleanup_grace_s=14, each_audit_timeout_s=60,
        scheduler='FIFO5' if fifo else 'correctness-only SCHED_OTHER',
        performance_run=fifo, ordinary_run_has_performance_conclusion=False,
        algorithm_threads=5, coordinator_cpu=0, camera_cpus=[1, 2, 3, 4],
        supervisor_policy='SCHED_OTHER', simulator_policy='non-real-time', simulator_rtprio_limit=0,
        simulator_main_policy=0, simulator_worker_policies=[0, 3, 5], simulator_thread_priority=0,
        algorithm_rtprio_limit=5 if fifo else 0, algorithm_uid=uid, simulator_uid=uid,
        global_rt_settings_modified=False, production_install_modified=False,
        production_motion_flag=base['motion_output_enabled'], simulation_motion_authorized=True,
        old_controller_bridge_used=False, cadence='lockstep simulation steps, wall-time pacing recorded separately',
        simulator_cache=str(SHARED_CACHE), simulator_cache_scope='shared experiment cache; run config/data/logs remain isolated',
        isaac_version=(ISAAC / 'VERSION').read_text().strip())
    if args.check:
        print(json.dumps(dict(scope=scope, manifest_verified=True, applications_started=False,
            formal_privilege_present=root, tests_run=manifest['tests_run']), indent=2))
        return 0
    if fifo and not root:
        parser.error('FIFO5 requires sudo /usr/bin/python3 -I ' + str(Path(__file__).resolve()) + '; no scheduling fallback')
    if args.correctness_only and root:
        parser.error('correctness-only must run as the non-root project owner')
    if not manifest['tests_run']:
        parser.error('run build_isaac_closed_loop.py --tests before starting applications')
    output = (args.output or EXPERIMENT / ('run_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    if not output.is_relative_to(EXPERIMENT) or output.exists():
        parser.error('choose a new output directory inside this experiment')
    lock_path = '/tmp/fov_gvf_user_navigation.lock'
    try:
        lock_fd = os.open(lock_path, os.O_RDWR | os.O_NOFOLLOW)
    except FileNotFoundError:
        lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o660)
        if root:
            os.fchown(lock_fd, uid, owner.pw_gid)
    if not stat.S_ISREG(os.fstat(lock_fd).st_mode):
        raise RuntimeError('shared navigation lock is not a regular file')
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        raise RuntimeError('another navigation session holds the shared lock; no applications started')
    output.mkdir(mode=0o750)
    if root:
        os.chown(output, uid, owner.pw_gid)

    def save(path, value):
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        if root:
            os.chown(path, uid, owner.pw_gid)

    def copy(source, target):
        shutil.copyfile(source, target)
        if root:
            os.chown(target, uid, owner.pw_gid)

    for name, source in [('scenario.json', scenario_path), ('base_step1.json', ROOT / 'src/new_controller/config/base_step1.json'),
                         ('navigation_step2.json', ROOT / 'src/new_controller/config/navigation_step2.json')]:
        copy(source, output / name)
    copy(manifest_path, output / 'source_manifest.json')
    input_hashes = {name: digest(output / name) for name in ['scenario.json', 'base_step1.json', 'navigation_step2.json']}
    save(output / 'scope.json', scope)
    kernel = lambda: {key: Path('/proc/sys/kernel/' + key).read_text().strip()
                      for key in ['sched_rt_runtime_us', 'sched_rt_period_us']}
    execution = dict(scope=scope, input_sha256=input_hashes, kernel_before=kernel(), processes={},
                     started_realtime_ns=time.time_ns(), started_monotonic_ns=time.monotonic_ns(),
                     supervisor_before=inventory(os.getpid()),
                     controller_thread_observations=[], simulator_thread_observations=[], status='STARTING')
    socket_dir = Path(tempfile.mkdtemp(prefix='p3-sim-'))
    if root:
        os.chown(socket_dir, uid, owner.pw_gid)
    socket_path = socket_dir / 'controller.sock'
    execution['private_socket_directory'] = str(socket_dir)
    runtime = output / 'runtime'
    for name in ['', 'config', 'data', 'tmp']:
        folder = runtime / name
        folder.mkdir(exist_ok=True)
        if root:
            os.chown(folder, uid, owner.pw_gid)
    for name in ['', 'cache', 'cuda', 'shader', 'warp', 'kit']:
        folder = SHARED_CACHE / name
        if not folder.exists():
            folder.mkdir()
            if root:
                os.chown(folder, uid, owner.pw_gid)
        if folder.is_symlink() or not folder.is_dir() or folder.stat().st_uid != uid:
            raise RuntimeError('shared simulator cache must be a project-owned real directory')
    execution['cache'] = dict(root=str(SHARED_CACHE), isaac_version=scope['isaac_version'],
        source_seed_receipt=str(EXPERIMENT / 'shared_cache_seed.json'),
        source_seed_receipt_sha256=digest(EXPERIMENT / 'shared_cache_seed.json') if (EXPERIMENT / 'shared_cache_seed.json').is_file() else None,
        preparation_receipts_sha256={str(path): digest(path) for path in sorted(EXPERIMENT.glob('shared_cache*.json')) if path.is_file()},
        mutable_runtime_data=True, included_in_algorithm_identity=False, previous_caches_deleted=False)
    common_env = dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin', LANG='C.UTF-8', HOME=owner.pw_dir,
        USER=owner.pw_name, LOGNAME=owner.pw_name, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
        OMP_WAIT_POLICY='PASSIVE', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    sim_env = dict(common_env, DISPLAY=display, XAUTHORITY=str(xauthority), XDG_RUNTIME_DIR=f'/run/user/{uid}',
        DBUS_SESSION_BUS_ADDRESS=os.environ.get('DBUS_SESSION_BUS_ADDRESS', f'unix:path=/run/user/{uid}/bus'),
        EGO4P0_ROOT=str(ROOT), EGO4P2_ROOT=str(ROOT), EGO4P3_ROOT=str(ROOT),
        EGO4P0_RUNTIME=str(runtime), EGO4P2_RUNTIME=str(runtime), EGO4P3_RUNTIME=str(runtime),
        XDG_CACHE_HOME=str(SHARED_CACHE / 'cache'), XDG_CONFIG_HOME=str(runtime / 'config'),
        XDG_DATA_HOME=str(runtime / 'data'), CUDA_CACHE_PATH=str(SHARED_CACHE / 'cuda'),
        __GL_SHADER_DISK_CACHE_PATH=str(SHARED_CACHE / 'shader'), WARP_CACHE_PATH=str(SHARED_CACHE / 'warp'),
        EGO4P3_SIM_SHARED_CACHE=str(SHARED_CACHE),
        TMPDIR=str(runtime / 'tmp'))
    sim_args = [ISAAC / 'python.sh', SIMULATOR, '--scenario', output / 'scenario.json',
        '--output', output / 'simulation', '--socket', socket_path, '--time-scale', str(time_scale),
        '--wall-timeout', str(wall_timeout)]
    if args.steps is not None:
        sim_args += ['--steps', str(steps)]
    if args.headless:
        sim_args.append('--headless')
    if args.no_overlay:
        sim_args.append('--no-overlay')
    simulator = controller = None
    sim_log = controller_log = None
    actual_sim_pid = None
    controller_verified = False
    controller_eof_at = None
    last_sim_sample = last_controller_sample = last_save = 0.
    started = time.monotonic()
    running_started = None
    caught = None

    def on_signal(signum, frame):
        raise KeyboardInterrupt('supervisor received signal ' + str(signum))

    old_handlers = {sig: signal.signal(sig, on_signal) for sig in [signal.SIGINT, signal.SIGTERM]}
    try:
        print('RESULT_DIR=' + str(output), flush=True)
        gpu_command = ordinary_or_limited_command(['/usr/bin/nvidia-smi',
            '--query-gpu=name,uuid,driver_version,pci.bus_id', '--format=csv,noheader'], common_env, owner, root)
        gpu = subprocess.run(gpu_command, env={'PATH': common_env['PATH'], 'LANG': common_env['LANG']},
            capture_output=True, text=True, timeout=20, preexec_fn=lambda: child_limits(False, False, 20))
        execution['cache']['gpu_identity'] = dict(command=gpu_command, exit_code=gpu.returncode,
            output=gpu.stdout.strip(), error=gpu.stderr.strip())
        if gpu.returncode or not gpu.stdout.strip():
            raise RuntimeError('GPU identity probe failed before simulator launch')
        sim_command = ordinary_or_limited_command(sim_args, sim_env, owner, root)
        sim_log = (output / 'simulation_console.log').open('w')
        simulator = subprocess.Popen(sim_command, env={'PATH': common_env['PATH'], 'LANG': common_env['LANG']},
            stdout=sim_log, stderr=subprocess.STDOUT, start_new_session=True,
            preexec_fn=lambda: child_limits(False, False, math.ceil(initialization_timeout + wall_timeout) * 16 + 120))
        execution['processes']['simulator'] = dict(command=sim_command, wrapper_pid=simulator.pid,
            expected_main_policy=0, expected_worker_policies=[0, 3, 5], priority=0, rtprio_limit=0, uid=uid)
        save(output / 'execution.json', execution)
        ready_path = output / 'simulation/sensors_ready.json'
        while not ready_path.is_file():
            if simulator.poll() is not None:
                raise RuntimeError('simulator exited before publishing actual camera calibration')
            if time.monotonic() - started > initialization_timeout:
                raise RuntimeError('camera initialization wall timeout')
            time.sleep(.05)
        ready = json.loads(ready_path.read_text())
        calibration = output / 'simulation/simulation.sensors.json'
        if Path(ready['calibration']).resolve() != calibration.resolve() or not calibration.is_file():
            raise RuntimeError('simulator calibration path mismatch')
        if ready['scenario_sha256'] != input_hashes['scenario.json'] or ready['width'] != scenario['image_width'] or ready['height'] != scenario['image_height']:
            raise RuntimeError('actual sensor identity/dimensions disagree with the scenario')
        if ready['physics_dt_s'] != base['physics_dt_s'] or ready['time_scale'] != time_scale or ready['headless'] != args.headless:
            raise RuntimeError('simulator timing or display configuration mismatch')
        actual_sim_pid = int(ready['pid'])
        if os.getpgid(actual_sim_pid) != simulator.pid:
            raise RuntimeError('sensor-ready PID is not in the launched simulator process group')
        execution['processes']['simulator'].update(pid=actual_sim_pid, sensors_ready=ready,
            calibration_sha256=digest(calibration), actual=process_identity(actual_sim_pid))
        actual_sim = execution['processes']['simulator']['actual']
        if Path(actual_sim['executable']) != (ISAAC / 'kit/python/bin/python3').resolve() or actual_sim['limits']['rtprio'] != ['0', '0']:
            raise RuntimeError('simulator executable or RT priority limit readback failed')
        running_started = time.monotonic()
        execution['sensors_ready_elapsed_s'] = running_started - started
        execution['sensors_ready_monotonic_ns'] = time.monotonic_ns()
        binary = ROOT / manifest['binary']['path']
        controller_args = [binary, '--calibration', calibration, '--scenario', output / 'scenario.json',
                           '--output', output / 'controller', '--socket', socket_path]
        if args.correctness_only:
            controller_args.append('--correctness-only')
        controller_command = ordinary_or_limited_command(controller_args, common_env, owner, root)
        controller_log = (output / 'controller_console.log').open('w')
        controller = subprocess.Popen(controller_command, env={'PATH': common_env['PATH'], 'LANG': common_env['LANG']},
            stdout=controller_log, stderr=subprocess.STDOUT, start_new_session=True,
            preexec_fn=lambda: child_limits(True, fifo, math.ceil(wall_timeout) * 5 + 120))
        execution['processes']['controller'] = dict(command=controller_command, pid=controller.pid,
            binary_sha256=digest(binary), expected_policy=1 if fifo else 0, priority=5 if fifo else 0,
            rtprio_limit=5 if fifo else 0, rttime_us=[5_000_000, 6_000_000] if fifo else [0, 0], uid=uid)
        execution['status'] = 'RUNNING'
        while simulator.poll() is None:
            now = time.monotonic()
            if now - running_started > wall_timeout:
                raise RuntimeError('bounded simulation wall timeout')
            controller_running = controller.poll() is None
            if not controller_running:
                summary_path = output / 'controller/execution_summary.json'
                normal_eof = controller.returncode == 0 and summary_path.is_file() and json.loads(summary_path.read_text()).get('normal_eof') is True
                if not normal_eof:
                    # Allow the adapter to save a failed-IPC result, then stop.
                    try:
                        simulator.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        pass
                    raise RuntimeError('controller exited without a normal simulator disconnect')
                # Kit can still be closing its GPU/UI after the socket closes.
                # A successful native EOF is distinct from an early crash.
                if controller_eof_at is None:
                    controller_eof_at = now
                    execution['controller_normal_eof_elapsed_s'] = now - started
                if now - controller_eof_at > 30.:
                    raise RuntimeError('simulator did not finish within 30 seconds after normal native EOF')
            metadata_path = output / 'controller/controller_ready.json'
            if controller_running and metadata_path.is_file():
                try:
                    if 'ready' not in execution['processes']['controller']:
                        controller_ready = json.loads(metadata_path.read_text())
                        expected_ready = dict(simulation_only=True, correctness_only=not fifo,
                            coordinator_policy=1 if fifo else 0, coordinator_priority=5 if fifo else 0,
                            coordinator_cpu=0, worker_policies=[1 if fifo else 0] * 4,
                            worker_priorities=[5 if fifo else 0] * 4, worker_cpus=[1, 2, 3, 4],
                            algorithm_threads=5)
                        if any(controller_ready.get(k) != v for k, v in expected_ready.items()):
                            raise RuntimeError('controller-ready metadata disagrees with the requested scope')
                        execution['processes']['controller']['ready'] = controller_ready
                        actual_controller = process_identity(controller.pid, libraries=True)
                        execution['processes']['controller']['actual'] = actual_controller
                        if Path(actual_controller['executable']) != binary.resolve() or actual_controller['executable_sha256'] != manifest['binary']['sha256'] or actual_controller['limits']['rtprio'] != [str(5 if fifo else 0)] * 2:
                            raise RuntimeError('actual controller ELF or RT priority limit readback failed')
                        if actual_controller['mapped_libraries_sha256'] != manifest['dependencies']:
                            raise RuntimeError('actual controller shared libraries differ from validated dependencies')
                    threads = inventory(controller.pid)
                    expected_cpus = {'p3_sim_coord': 0, **{'depth_cam' + str(i): i + 1 for i in range(4)}}
                    violations = thread_violations(threads, uid, 1 if fifo else 0, 5 if fifo else 0, expected_cpus)
                    if violations and controller.poll() is not None:
                        raise ProcessLookupError('controller exited during thread sampling')
                    if now - last_controller_sample >= .25 or violations:
                        execution['controller_thread_observations'].append(dict(elapsed_s=now-started,
                            threads=threads, validation_passed=not violations, violations=violations))
                        last_controller_sample = now
                    if violations:
                        save(output / 'execution.json', execution)
                        raise RuntimeError('controller thread readback failed: ' + json.dumps(violations))
                    controller_verified = True
                except (FileNotFoundError, ProcessLookupError):
                    if controller.poll() is None:
                        raise
            if now - last_sim_sample >= 1.:
                try:
                    threads = inventory(actual_sim_pid)
                    observation = dict(elapsed_s=now-started, threads=threads)
                    execution['simulator_thread_observations'].append(observation)
                    violations = thread_violations(threads, uid, [0, 3, 5], 0)
                    main_threads = [thread for thread in threads if thread['tid'] == actual_sim_pid]
                    if len(main_threads) != 1:
                        violations.append(dict(field='main_thread_count', expected=1, observed=len(main_threads)))
                    else:
                        violations.extend(thread_violations(main_threads, uid, 0, 0))
                    observation.update(validation_passed=not violations, violations=violations)
                    last_sim_sample = now
                    if violations:
                        save(output / 'execution.json', execution)
                        raise RuntimeError('simulator thread readback failed: ' + json.dumps(violations))
                except (FileNotFoundError, ProcessLookupError):
                    if simulator.poll() is None:
                        raise
            if now - last_save >= 2.:
                save(output / 'execution.json', execution)
                last_save = now
            time.sleep(.02)
        if simulator.returncode:
            raise RuntimeError('simulator failed with exit ' + str(simulator.returncode))
        try:
            controller.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            raise RuntimeError('controller did not exit after simulator disconnected')
        if controller.returncode or not controller_verified or not execution['simulator_thread_observations']:
            raise RuntimeError('run lacks successful controller exit and verified process policies')
        execution['status'] = 'COMPLETED'
    except BaseException as error:
        caught = error
        execution['status'] = 'FAILED'
        execution['error'] = repr(error)
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        stop_group(controller)
        stop_group(simulator)
        for log in [sim_log, controller_log]:
            if log is not None:
                log.close()
        for name, child in [('controller', controller), ('simulator', simulator)]:
            if child is not None:
                execution['processes'][name]['exit_code'] = child.returncode
        execution['wall_s'] = time.monotonic() - started
        execution['run_wall_s'] = None if running_started is None else time.monotonic() - running_started
        execution['ended_realtime_ns'] = time.time_ns()
        execution['ended_monotonic_ns'] = time.monotonic_ns()
        execution['supervisor_after'] = inventory(os.getpid())
        execution['controller_threads_verified'] = controller_verified
        execution['kernel_after'] = kernel()
        if execution['kernel_before'] != execution['kernel_after']:
            execution['status'] = 'FAILED'
            execution['kernel_error'] = 'global RT settings changed during run'
        changed_inputs = [name for name, wanted in input_hashes.items() if digest(output / name) != wanted]
        execution['input_files_unchanged'] = not changed_inputs
        if changed_inputs:
            execution['status'] = 'FAILED'
            execution['changed_inputs'] = changed_inputs
        try:
            execution['final_identity_sha_checks'] = verify_manifest(manifest)
        except Exception as error:
            execution['status'] = 'FAILED'
            execution['final_identity_error'] = repr(error)
        for source, name in [(output / 'simulation/trajectory.csv', 'trajectory.csv'),
                             (output / 'simulation/result.json', 'result.json'),
                             (output / 'controller/controller.csv', 'controller.csv')]:
            if source.is_file():
                copy(source, output / name)
        if not (output / 'result.json').is_file():
            save(output / 'result.json', dict(status='LAUNCH_FAILED', success=False, arrived=False,
                completed_steps=0, failures=[execution.get('error', execution['status'])]))
        save(output / 'execution.json', execution)
        shutil.rmtree(socket_dir)
        try:
            audit_command = ordinary_or_limited_command(['/usr/bin/python3', '-I', AUDITOR, output], common_env, owner, root)
            with (output / 'audit_console.log').open('w') as log:
                audit = subprocess.run(audit_command, env={'PATH': common_env['PATH'], 'LANG': common_env['LANG']},
                    stdout=log, stderr=subprocess.STDOUT, timeout=60,
                    preexec_fn=lambda: child_limits(False, False, 60))
            execution['audit'] = dict(command=audit_command, exit_code=audit.returncode)
        except Exception as error:
            execution['audit'] = dict(error=repr(error), exit_code=None)
        save(output / 'execution.json', execution)
        try:
            audit_command = ordinary_or_limited_command(['/usr/bin/python3', '-I', EXECUTION_AUDITOR, output], common_env, owner, root)
            with (output / 'execution_audit_console.log').open('w') as log:
                audit = subprocess.run(audit_command, env={'PATH': common_env['PATH'], 'LANG': common_env['LANG']},
                    stdout=log, stderr=subprocess.STDOUT, timeout=60,
                    preexec_fn=lambda: child_limits(False, False, 60))
            execution['execution_audit'] = dict(command=audit_command, exit_code=audit.returncode)
        except Exception as error:
            execution['execution_audit'] = dict(error=repr(error), exit_code=None)
        if root:
            for path in output.iterdir():
                if path.is_file():
                    os.chown(path, uid, owner.pw_gid)
        save(output / 'execution.json', execution)
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        os.close(lock_fd)
    print('STATUS=' + execution['status'], flush=True)
    print('RESULT_DIR=' + str(output), flush=True)
    if caught is not None:
        print('ERROR=' + str(caught), flush=True)
    return 0 if execution['status'] == 'COMPLETED' and execution['audit']['exit_code'] == 0 and execution['execution_audit']['exit_code'] == 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
