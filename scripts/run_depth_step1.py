#!/usr/bin/env python3
"""Orchestrate one bounded read-only Isaac/ROS observation test."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import tarfile
import time
from zoneinfo import ZoneInfo
from base_config import load_config
BASE, BASE_SHA = load_config()


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=root/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=.2)
    parser.add_argument('--camera-hz', type=int, default=BASE["camera_hz"])
    parser.add_argument('--pace', choices=('realtime','unlimited'), default='realtime')
    parser.add_argument('--time-scale', type=float, default=BASE["time_scale"],
                        help='simulation seconds per wall second; realtime pacing only')
    parser.add_argument('--sensor-mode', choices=('rate','auto'), default='rate')
    parser.add_argument('--disable-viewport', action='store_true',default=True)
    parser.add_argument('--enable-viewport', action='store_false',dest='disable_viewport')
    parser.add_argument('--render-threads', type=int, default=8)
    parser.add_argument('--profile-stages', action='store_true')
    parser.add_argument('--record-physics', action='store_true')
    parser.add_argument('--read-mode', choices=('reference','copy'), default='reference')
    parser.add_argument('--desired-source', choices=('velocity','goal','none'), default='velocity')
    parser.add_argument('--goal-speed-mps', type=float, default=BASE["max_speed_mps"])
    parser.add_argument('--azimuth-resolution-deg',type=float,default=1.)
    parser.add_argument('--elevation-resolution-deg',type=float,default=1.)
    parser.add_argument('--duration-clock', choices=('sim','wall'), default='sim')
    parser.add_argument('--reader-cpu', type=int, default=0)
    parser.add_argument('--spin-us', type=int, default=200)
    parser.add_argument('--fifo-priority', type=int, default=5)
    parser.add_argument('--timer-slack-ns', type=int, default=1)
    parser.add_argument('--lock-memory', action='store_true',default=True)
    parser.add_argument('--no-lock-memory', action='store_false',dest='lock_memory')
    parser.add_argument('--scheduler-gate', action='store_true')
    parser.add_argument('--wall-timeout', type=float, default=300.)
    args=parser.parse_args()
    if args.camera_hz != BASE["camera_hz"] or args.time_scale != BASE["time_scale"] or args.pace != "realtime" or args.sensor_mode != "rate":
        parser.error("step-1 requires 200 sim Hz, 5ms physics, rate sensors and .02 time scale")
    if args.fifo_priority != 5 or args.reader_cpu != 0:
        parser.error("step-1 requires reader CPU0 and FIFO5; use the verified FIFO launcher")
    import math
    if not math.isfinite(args.goal_speed_mps) or not 0 <= args.goal_speed_mps <= BASE["max_speed_mps"]:
        parser.error('goal-speed-mps must be finite and within [0,12]')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 120:
        parser.error('duration must be finite and within (0,120] simulation seconds')
    if not math.isfinite(args.time_scale) or not 0 < args.time_scale <= 1:
        parser.error('time-scale must be finite and within (0,1]')
    if args.pace == 'unlimited' and args.time_scale != 1:
        parser.error('unlimited pacing requires --time-scale 1')
    if not 1 <= args.camera_hz <= 1000 or not 0 <= args.spin_us <= 1000:
        parser.error('invalid camera rate or spin time')
    if not 0 <= args.timer_slack_ns <= 1000000:
        parser.error('invalid timer slack')
    if not (math.isfinite(args.azimuth_resolution_deg) and 0<args.azimuth_resolution_deg<=360 and math.isfinite(args.elevation_resolution_deg) and 0<args.elevation_resolution_deg<=180):
        parser.error('invalid angular resolutions in degrees')
    alpha_bins=360/args.azimuth_resolution_deg;beta_bins=180/args.elevation_resolution_deg
    if not math.isfinite(alpha_bins*beta_bins) or math.ceil(alpha_bins)*math.ceil(beta_bins)>1048576:
        parser.error('angular grid exceeds 1048576-cell allocation limit')
    run_id=f'depth_cpp_{args.camera_hz}hz_{args.read_mode}_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    out=(args.output or root/'docs/performance/depth_step1'/run_id).resolve()
    out.mkdir(parents=True, exist_ok=False)
    gpu_query=subprocess.run(['nvidia-smi','--query-gpu=index,name,uuid','--format=csv,noheader'],
        check=True,text=True,capture_output=True)
    gpu_rows=[line.split(',',2) for line in gpu_query.stdout.strip().splitlines()]
    if not any(row[0].strip()=='0' and '5070' in row[1] for row in gpu_rows):
        raise RuntimeError('GPU0 must be the local RTX 5070 for this test platform')
    gpu_record={'host':'local','index':0,'name':gpu_rows[0][1].strip(),
                'uuid':gpu_rows[0][2].strip(),'physics_device':'cuda:0','renderer_active_gpu':0}
    (out/'gpu_device.json').write_text(json.dumps(gpu_record,indent=2)+'\n')
    lock=open('/tmp/fov_gvf_user_navigation.lock','a')
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Match the shared navigation domain and reject existing simulation/control sources.
    import rclpy
    rclpy.init(args=[])
    probe=rclpy.create_node('ego4p0_depth_step1_preflight')
    end=time.monotonic()+2.
    while time.monotonic()<end:
        rclpy.spin_once(probe, timeout_sec=.1)
    topics=('/position_cmd','/sim/cmd_vel','/sim/depth/image_raw',
            '/sim/depth_left/image_raw','/sim/depth_back/image_raw','/sim/depth_right/image_raw','/clock')
    counts={t:probe.count_publishers(t) for t in topics}
    probe.destroy_node(); rclpy.shutdown()
    if any(counts.values()):
        raise RuntimeError(f'existing publishers in ROS domain: {counts}')
    files=[root/'scripts/isaac/run_depth_input_simulation.py',root/'scripts/isaac/depth_camera_projection.py',
           root/'scripts/run_depth_step1.sh',Path(__file__).resolve(),
           *sorted((root/'src/new_controller/src').rglob('*.cpp')),
           *sorted((root/'src/new_controller/test').rglob('*.cpp')),
           *sorted((root/'src/new_controller/include').rglob('*.hpp')),
           root/'src/new_controller/CMakeLists.txt',root/'src/new_controller/package.xml',
           root/'scripts/analysis/analyze_depth_cpp.py',root/'scripts/analysis/verify_depth_preprocessing.py',root/'scripts/analysis/verify_direction_queries.py',root/'scripts/analysis/verify_local_field.py',root/'scripts/analysis/plot_depth_stage_timing.py',root/'scripts/analysis/analyze_depth_pacing.py']
    files.extend([root/'src/new_controller/config/base_step1.json',root/'scripts/base_config.py',root/'scripts/analysis/verify_virtual_panorama.py'])
    executable=root/'docs/performance/runtime/install/new_controller/lib/new_controller/depth_reader'
    if not executable.is_file():
        raise RuntimeError('build the C++ new_controller package before running')
    identity=json.loads((root/'docs/performance/runtime/install/new_controller/share/new_controller/build_identity.json').read_text())
    if any(hashlib.sha256((root/k).read_bytes()).hexdigest()!=v for k,v in identity.items()):
        raise RuntimeError('C++ source differs from build identity; rebuild before running')
    compiled=json.loads(subprocess.check_output([str(executable),'--print-config'],text=True))
    if compiled != BASE:
        raise RuntimeError('compiled configuration differs from base_step1.json')
    installed={str(executable):hashlib.sha256(executable.read_bytes()).hexdigest()}
    background_cpus=None
    if args.reader_cpu >= 0:
        allowed=set(os.sched_getaffinity(0))
        if args.reader_cpu not in allowed:
            raise RuntimeError('requested reader CPU is unavailable')
        siblings=Path(f'/sys/devices/system/cpu/cpu{args.reader_cpu}/topology/thread_siblings_list').read_text().strip()
        reserved=set()
        for chunk in siblings.split(','):
            if '-' in chunk:
                a,b=map(int,chunk.split('-'));reserved.update(range(a,b+1))
            else:reserved.add(int(chunk))
        background_cpus=','.join(map(str,sorted(allowed-reserved)))
        if not background_cpus:raise RuntimeError('no CPU left for renderer and ROS')
    manifest=dict(run_id=run_id, output=str(out), scene=str(args.scene.resolve()),
        duration_s=args.duration, camera_hz=args.camera_hz,read_hz=1/BASE["control_period_s"],read_period_ms=1000*BASE["control_period_s"],base_config=BASE,base_config_sha256=BASE_SHA,pace=args.pace,time_scale=args.time_scale,
        camera_time_domain="simulation",reader_time_domain="wall",
        read_mode=args.read_mode,duration_clock=args.duration_clock,reader_cpu=args.reader_cpu,
        desired_source=args.desired_source,goal_speed_mps=args.goal_speed_mps,
        profile_stages=args.profile_stages,record_physics=args.record_physics,sensor_mode=args.sensor_mode,disable_viewport=args.disable_viewport,render_threads=args.render_threads,
        spin_us=args.spin_us,fifo_priority=args.fifo_priority,timer_slack_ns=args.timer_slack_ns,lock_memory=args.lock_memory,scheduler_gate=args.scheduler_gate,background_cpus=background_cpus,ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
        azimuth_resolution_deg=args.azimuth_resolution_deg,elevation_resolution_deg=args.elevation_resolution_deg,
        gpu=gpu_record,preflight_publishers=counts, installed_module_sha256=installed,
        source_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    shutil.copy2(executable,out/'depth_reader.executed')
    (out/'build_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    with tarfile.open(out/'source_at_run.tar.gz','w:gz') as archive:
        for p in files:archive.add(p,arcname=str(p.relative_to(root)))
    print(f'[OUTPUT] {out}',flush=True)
    env=os.environ.copy()
    for key in ('PYTHONPATH','OLD_PYTHONPATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH'):
        env.pop(key,None)
    isaac=root.parent/'isaacsim'
    env['LD_LIBRARY_PATH']=str(isaac/'exts/isaacsim.ros2.core/humble/lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['ROS_DISTRO']='humble'
    env.setdefault('OMP_WAIT_POLICY','PASSIVE')
    children=[]
    def stop_child(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL);p.wait()
    try:
        with (out/'simulator_console.log').open('w') as log:
            sim_command=[str(isaac/'python.sh'),str(root/'scripts/isaac/run_depth_input_simulation.py'),
                '--scene',str(args.scene.resolve()),'--output',str(out),'--run-id',run_id,
                '--camera-hz',str(args.camera_hz),'--pace',args.pace,'--time-scale',str(args.time_scale),
                '--sensor-mode',args.sensor_mode,'--render-threads',str(args.render_threads)]
            if args.profile_stages:sim_command.append('--profile-stages')
            if args.record_physics:sim_command.append('--record-physics')
            if args.disable_viewport:sim_command.append('--disable-viewport')
            if background_cpus:sim_command=['taskset','-c',background_cpus]+sim_command
            sim=subprocess.Popen(sim_command,
                stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            children.append(sim)
            end=time.monotonic()+300
            while not (out/'simulator_ready').exists():
                if sim.poll() is not None:raise RuntimeError(f'Isaac exited during startup: {sim.returncode}')
                if time.monotonic()>end:raise RuntimeError('Isaac startup timeout')
                time.sleep(.1)
            with (out/'reader_console.log').open('w') as read_log:
                reader_command=[str(executable),'--output',str(out),'--run-id',run_id,
                    '--duration',str(args.duration),'--duration-clock',args.duration_clock,
                    '--wall-timeout',str(args.wall_timeout),'--read-mode',args.read_mode,
                    '--reader-cpu',str(args.reader_cpu),'--spin-us',str(args.spin_us),
                    '--desired-source',args.desired_source,'--goal-speed-mps',str(args.goal_speed_mps),
                    '--azimuth-resolution-deg',str(args.azimuth_resolution_deg),'--elevation-resolution-deg',str(args.elevation_resolution_deg),
                    '--fifo-priority',str(args.fifo_priority),
                    '--timer-slack-ns',str(args.timer_slack_ns),'--lock-memory',str(int(args.lock_memory)),
                    '--scheduler-gate',str(int(args.scheduler_gate))]
                if background_cpus:reader_command=['taskset','-c',background_cpus]+reader_command
                reader=subprocess.Popen(reader_command,
                    stdout=read_log,stderr=subprocess.STDOUT,start_new_session=True)
                children.append(reader)
                end=time.monotonic()+args.wall_timeout+20
                while reader.poll() is None:
                    if sim.poll() is not None:raise RuntimeError('Isaac exited before reader completed')
                    if time.monotonic()>end:raise RuntimeError('reader process timeout')
                    time.sleep(.1)
                sim.wait(timeout=40)
                codes=dict(reader=reader.returncode,simulator=sim.returncode)
                (out/'process_result.json').write_text(json.dumps(codes,indent=2)+'\n')
                if any(codes.values()):raise RuntimeError(f'child process failed: {codes}')
        subprocess.run(['/usr/bin/python3','-B',str(root/'scripts/analysis/analyze_depth_cpp.py'),str(out)],check=True)
        print((out/'reader_summary.json').read_text(),flush=True)
        return 0
    except BaseException as exc:
        (out/'orchestrator_error.json').write_text(json.dumps({'error':str(exc)},indent=2)+'\n')
        raise
    finally:
        for child in reversed(children):stop_child(child)
        lock.close()


if __name__=='__main__':
    raise SystemExit(main())
