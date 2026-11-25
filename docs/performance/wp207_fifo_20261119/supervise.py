#!/usr/bin/python3
"""Run the original Cloud/R05 model in P3 with current core and explicit route guidance.

The original experimental fallback/guidance wrapper is retained. This is not a
pure-goal autonomous-navigation test of the production P3 rolling controller.
"""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import pwd
import resource
import signal
import stat
import subprocess
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT = Path(__file__).resolve().parent
ASSETS = ROOT / 'scenes/r05_cloud'
ISAAC = ROOT.parent / 'isaacsim'
APP = ROOT / 'src/r05_model'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, default=110.)
    parser.add_argument('--time-scale', type=float, default=.02)
    parser.add_argument('--correctness-only', action='store_true', help='explicit ordinary-scheduler functional check')
    parser.add_argument('--direct-only', action='store_true', help='disable the experimental search branch')
    parser.add_argument('--goal-only', action='store_true', help='omit the archived geometric guide; success is not guaranteed')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 150 or not math.isfinite(args.time_scale) or not 0 < args.time_scale <= 1:
        parser.error('seconds must be (0,150], time-scale must be (0,1]')
    # Reuse scheduling observation/privilege-drop helpers, never its scenario controller.
    import sys
    sys.path.insert(0, str(ROOT / 'scripts'))
    import run_isaac_closed_loop as helper
    record = json.loads((EXPERIMENT / 'latest_build.json').read_text())
    manifest_path = ROOT / record['manifest']
    manifest = json.loads(manifest_path.read_text())
    for relative, wanted in manifest['source_identity'].items():
        path = (ROOT / relative).resolve(strict=True)
        if not path.is_relative_to(ROOT) or digest(path) != wanted:
            raise RuntimeError('stale build input: '+relative+'; rebuild with build_r05_model.py')
    binary = ROOT / manifest['binary']['path']
    if digest(binary) != manifest['binary']['sha256']:
        raise RuntimeError('controller identity mismatch')
    for path, wanted in manifest['dependencies'].items():
        if digest(Path(path)) != wanted:
            raise RuntimeError('dynamic dependency mismatch: '+path)
    assets_expected = {'ego_swarm_cloud_navigation.usd':'cfcc22694241eb2ca5e506060c136a2f9fe5b3b5e2efd76cf0d44a1da26ed1ca',
        'occupancy.bin':'e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8'}
    for name, wanted in assets_expected.items():
        if digest(ASSETS / name) != wanted:raise RuntimeError('original R05 asset mismatch: '+name)
    fifo = not args.correctness_only
    scope = dict(model='original R05 Cloud/camera rig, EGO2P6 ARL-Robot-1 session appearance',
        algorithm_core='current P3', wrapper='original R05 experimental guidance/search and safety adapter',
        pure_goal_autonomy=False, geometric_guide=not args.goal_only,
        guide_speed_policy='route speed applies to guidance fallback; certified cruise tracking can continue at 12 m/s',
        production_install_modified=False, production_motion_enabled=False,
        scheduler='FIFO5' if fifo else 'correctness-only SCHED_OTHER',
        seconds=args.seconds, time_scale=args.time_scale, physics_dt_s=.005,
        no_hit_adapter='Isaac +Inf -> 4.5m after far-clip readback; navigation range 4m')
    if args.check:
        print(json.dumps(dict(passed=True, scope=scope, source_files=len(manifest['source_identity']),
            model_sha256=assets_expected, applications_started=False), ensure_ascii=False, indent=2));return
    root = os.geteuid() == 0
    if root:raise RuntimeError('experiment supervisor must be unprivileged')
    if fifo and resource.getrlimit(resource.RLIMIT_RTPRIO)!=(0,5):parser.error('FIFO5 requires sudo /usr/bin/python3 -I '+str(Path(__file__).resolve()))
    if args.correctness_only and root:parser.error('correctness-only must run as the ordinary user')
    uid = int(os.environ.get('SUDO_UID', ROOT.stat().st_uid)) if root else os.getuid()
    if uid == 0:raise RuntimeError('run under sudo from the project owner, not a root login')
    owner = pwd.getpwuid(uid)
    lock = '/tmp/fov_gvf_user_navigation.lock'
    try:
        fd = os.open(lock, os.O_RDWR | os.O_NOFOLLOW)
    except FileNotFoundError:
        fd = os.open(lock, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o660)
        if root:os.fchown(fd,uid,owner.pw_gid)
    if not stat.S_ISREG(os.fstat(fd).st_mode):raise RuntimeError('invalid navigation lock')
    fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
    output = EXPERIMENT / ('run_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    def save(name, value):
        path=output/name;path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
        if root:os.chown(path,uid,owner.pw_gid)
    if root:os.chown(output,uid,owner.pw_gid)
    cache = EXPERIMENT / 'cache'
    runtime = output / 'runtime'
    for folder in [cache,runtime]+[cache/x for x in ['cuda','shader','warp','kit']]+[runtime/x for x in ['config','data','tmp']]:
        folder.mkdir(exist_ok=True)
        if root:os.chown(folder,uid,owner.pw_gid)
    env=dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin', LANG='C.UTF-8', HOME=owner.pw_dir,
        USER=owner.pw_name, LOGNAME=owner.pw_name, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
        OMP_WAIT_POLICY='PASSIVE', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
        EGO4P0_RUNTIME=str(runtime), EGO4P4_ROOT=str(ROOT), XDG_CACHE_HOME=str(cache),
        XDG_CONFIG_HOME=str(runtime/'config'), XDG_DATA_HOME=str(runtime/'data'),
        CUDA_CACHE_PATH=str(cache/'cuda'), __GL_SHADER_DISK_CACHE_PATH=str(cache/'shader'),
        WARP_CACHE_PATH=str(cache/'warp'), TMPDIR=str(runtime/'tmp'),
        DISPLAY=os.environ.get('DISPLAY',':0'), XAUTHORITY=os.environ.get('XAUTHORITY',str(Path(owner.pw_dir)/'.Xauthority')),
        XDG_RUNTIME_DIR=f'/run/user/{uid}')
    argv=[ISAAC/'python.sh',EXPERIMENT/'trial_sim.py',ASSETS,binary,output/'simulation',
        '--seconds',str(args.seconds),'--time-scale',str(args.time_scale),'--isaac-no-hit']
    if not args.goal_only:argv += ['--guide',ASSETS/'r05_guidance.csv']
    if args.headless:argv.append('--headless')
    if fifo:argv.append('--fifo')
    if args.direct_only:argv.append('--direct-only')
    command=helper.ordinary_or_limited_command(argv,env,owner,root)
    kernel=lambda:{k:Path('/proc/sys/kernel/'+k).read_text().strip() for k in ['sched_rt_runtime_us','sched_rt_period_us']}
    execution=dict(scope=scope,command=command,kernel_before=kernel(),manifest=str(manifest_path),
                   manifest_sha256=digest(manifest_path),thread_observations=[],supervisor_policy=os.sched_getscheduler(0))
    save('execution.json',execution)
    child=None;started=time.monotonic();seen=False
    def limits():
        # Kit requests RR1 for its own workers when the soft RT limit permits it.
        # Leave the soft limit at zero until the separate controller is exec'd.
        os.sched_setscheduler(0,os.SCHED_OTHER,os.sched_param(0))
        resource.setrlimit(resource.RLIMIT_RTPRIO,(0,5) if fifo else (0,0))
        resource.setrlimit(resource.RLIMIT_RTTIME,(5_000_000,6_000_000) if fifo else (0,0))
        resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    try:
        with (output/'console.log').open('w') as log:
            child=subprocess.Popen(command,env={'PATH':env['PATH'],'LANG':env['LANG']},stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True,preexec_fn=limits)
            if root:os.chown(output/'console.log',uid,owner.pw_gid)
            last=0;last_sim_check=None;simulator_checked_with_controller=False
            execution['simulator_readbacks']=[]
            while child.poll() is None:
                elapsed=time.monotonic()-started
                if elapsed>args.seconds/.02+600:raise RuntimeError('bounded simulator wall timeout')
                sim_pid=output/'simulation/simulator_pid.json'
                if sim_pid.exists() and (last_sim_check is None or elapsed-last_sim_check>=5 or (seen and not simulator_checked_with_controller)):
                    try:
                        sim_threads=helper.inventory(json.loads(sim_pid.read_text())['pid'])
                        if not sim_threads:
                            raise ProcessLookupError('empty simulator thread snapshot')
                        if any(t['policy'] not in (os.SCHED_OTHER,os.SCHED_BATCH,os.SCHED_IDLE) or t['priority']!=0 or t['uid']!=uid or t['cap_eff']!=0 for t in sim_threads):
                            execution['rejected_simulator_threads']=sim_threads
                            raise RuntimeError('simulator thread policy or privilege mismatch')
                        execution['simulator_threads']=sim_threads
                        execution['simulator_readbacks'].append(dict(wall_s=elapsed,threads=len(sim_threads),
                            policies=sorted({t['policy'] for t in sim_threads}),controller_seen=seen))
                        last_sim_check=elapsed
                        simulator_checked_with_controller |= seen
                    except (FileNotFoundError,ProcessLookupError,json.JSONDecodeError):pass
                ready=output/'simulation/controller_ready.json'
                if ready.exists():
                    try:
                        metadata=json.loads(ready.read_text());threads=helper.inventory(metadata['pid'])
                        expected=1 if fifo else 0
                        # Thread teardown races with /proc enumeration after the final
                        # controller reply. Validate complete five-thread snapshots;
                        # final success still requires an earlier complete readback.
                        if len(threads)==5:
                            if any(t['policy']!=expected or t['priority']!=(5 if fifo else 0) or t['uid']!=uid or t['cap_eff']!=0 for t in threads):
                                raise RuntimeError('actual algorithm scheduling/owner mismatch')
                            if sorted(t['cpus'] for t in threads)!=[[i] for i in range(5)]:raise RuntimeError('actual affinity mismatch')
                            if not seen:
                                execution['thread_observations']=threads
                                execution['controller_metadata']=metadata;seen=True;save('execution.json',execution)
                    except (ProcessLookupError,FileNotFoundError,json.JSONDecodeError):pass
                if elapsed-last>30:
                    print(f'R05 model: {elapsed:.0f}s wall; five threads verified={seen}',flush=True);last=elapsed
                time.sleep(.02)
        execution['exit_code']=child.returncode
        failure=output/'simulation/failure.json'
        if failure.exists():raise RuntimeError('Isaac failure: '+json.loads(failure.read_text())['error']+'; see '+str(failure))
        if child.returncode:raise RuntimeError('simulation failed; see '+str(output/'console.log'))
        if not seen or not simulator_checked_with_controller:raise RuntimeError('no actual algorithm/simulator thread readback observed')
        report=json.loads((output/'simulation/result.json').read_text());execution['result']=report
        if report['frame']<1:raise RuntimeError('no simulation step recorded')
        # Independent response/sweep audit also accepts bounded runs, without claiming ARRIVED.
        audit=helper.ordinary_or_limited_command([ISAAC/'python.sh',APP/'audit.py',output/'simulation',ASSETS/'occupancy.bin'],env,owner,root)
        with (output/'audit.log').open('w') as log:subprocess.run(audit,env={'PATH':env['PATH'],'LANG':env['LANG']},stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        execution['audit_passed']=True
    except BaseException as error:
        execution['error']=repr(error)
        raise
    finally:
        helper.stop_group(child)
        execution['kernel_after']=kernel();execution['wall_s']=time.monotonic()-started
        execution['global_rt_settings_unchanged']=execution['kernel_before']==execution['kernel_after']
        save('execution.json',execution)
        os.close(fd)
        print('RESULT_DIR='+str(output),flush=True)
    if not execution['global_rt_settings_unchanged']:raise RuntimeError('kernel realtime settings changed externally')

if __name__=='__main__':
    main()
