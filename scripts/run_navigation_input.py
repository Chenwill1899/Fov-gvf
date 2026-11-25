#!/usr/bin/python3
"""Launch the installed no-motion production input chain with five verified FIFO5 threads."""
import argparse,hashlib,importlib.util,json,os,pwd,signal,subprocess,time
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
P=Path(__file__).resolve().parents[1]
ROS_SETUP=Path('/opt/ros/humble/setup.bash')
def ros_command(argv):
 if not ROS_SETUP.is_file():raise RuntimeError('ROS setup script is missing: '+str(ROS_SETUP))
 return ['/bin/bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash && exec "$@"','ego4p3-ros',*map(str,argv)]
def check_runtime_dependencies(binary):
 env=dict(os.environ)
 for key in ['LD_PRELOAD','LD_AUDIT','PYTHONPATH','BASH_ENV','ENV']:env.pop(key,None)
 command=ros_command(['/usr/bin/ldd',str(binary)])
 probe=subprocess.run(command,env=env,capture_output=True,text=True,timeout=30)
 missing=[line.strip() for line in (probe.stdout+'\n'+probe.stderr).splitlines() if 'not found' in line]
 if probe.returncode or missing:raise RuntimeError('ROS runtime dependency check failed: '+('; '.join(missing) or probe.stderr.strip() or 'ldd exit '+str(probe.returncode)))
 return dict(command=command,exit_code=probe.returncode,missing=missing,stdout=probe.stdout,stderr=probe.stderr,resolved_library_count=sum(' => ' in line for line in probe.stdout.splitlines()))
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--calibration',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--duration-sim',type=float,default=.2);p.add_argument('--wall-timeout',type=float,default=60);tracking=p.add_mutually_exclusive_group();tracking.add_argument('--track-fixed',action='store_true');tracking.add_argument('--track-rolling',action='store_true');p.add_argument('--source-generation',type=int);p.add_argument('--source-session',help='publisher-generated lowercase UUID hex, defaults to the calibration input_session');p.add_argument('--check',action='store_true');a=p.parse_args()
 runtime=P/'docs/performance/runtime/install/new_controller';binary=runtime/'lib/new_controller/navigation_input';identity=json.loads((runtime/'share/new_controller/build_identity.json').read_text())
 for file,digest in identity.items():
  path=(P/file).resolve();assert path.is_relative_to(P) and hashlib.sha256(path.read_bytes()).hexdigest()==digest,('stale production build',file)
 calibration=a.calibration.resolve();assert calibration.is_file()
 metadata=json.loads(calibration.read_text())
 session=a.source_session or metadata.get('input_session','')
 generation=a.source_generation if a.source_generation is not None else metadata.get('source_generation',0)
 if type(generation)!=int or not 0<=generation<=2**32-1 or (generation and not session):p.error('invalid source generation')
 if session and (len(session)!=32 or any(c not in '0123456789abcdef' for c in session)):p.error('invalid source session UUID')
 if not ROS_SETUP.is_file():p.error('ROS setup script is missing: '+str(ROS_SETUP))
 scope=dict(executable=str(binary),calibration=str(calibration),duration_sim_s=a.duration_sim,wall_timeout_s=a.wall_timeout,algorithm_threads=5,coordinator_cpu=0,camera_cpus=[1,2,3,4],scheduler='FIFO5',ROS_receiving_threads='SCHED_OTHER',motion_publishers=0,fixed_tracking_shadow=a.track_fixed,rolling_tracking_shadow=a.track_rolling,reference_kind='spatial_reference',spatial_reference_enabled=True,continuous_tube_check_enabled=True,simulation_started_by_launcher=False,global_rt_settings_modified=False)
 scope['source_generation']=generation
 scope['source_session']=session or None
 scope['legacy_rewind_policy']='quarantine'
 scope['ros_setup']=str(ROS_SETUP)
 if a.check:
  probe=check_runtime_dependencies(binary);scope['runtime_dependencies']=dict(verified=True,resolved_library_count=probe['resolved_library_count']);print(json.dumps(scope,indent=2));return
 if os.geteuid()!=0:p.error('FIFO5 requires sudo /usr/bin/python3 -I '+str(Path(__file__).resolve())+' with the same arguments')
 assert os.sched_getscheduler(0)==os.SCHED_OTHER
 uid=int(os.environ.get('SUDO_UID',P.stat().st_uid));assert uid!=0;owner=pwd.getpwuid(uid)
 out=(a.output or P/'docs/performance'/('production_input_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))).resolve();assert out.is_relative_to(P) and not out.exists();out.mkdir();os.chown(out,uid,owner.pw_gid)
 spec=importlib.util.spec_from_file_location('fifo_helper',P/'scripts/benchmarks/run_ldlt_candidate_benchmark.py');helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
 env=dict(os.environ,PATH='/usr/sbin:/usr/bin:/sbin:/bin',HOME=owner.pw_dir,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','42'),ROS_LOG_DIR=str(out/'ros_log'),ROS_HOME=str(out/'ros_home'))
 for key in ['LD_PRELOAD','LD_AUDIT','PYTHONPATH','BASH_ENV','ENV']:env.pop(key,None)
 for name in ['ros_log','ros_home']:(out/name).mkdir();os.chown(out/name,uid,owner.pw_gid)
 command=['/usr/bin/setpriv','--reuid',str(uid),'--regid',str(owner.pw_gid),'--init-groups','--no-new-privs','--bounding-set=-all','--inh-caps=-all','--ambient-caps=-all','--']+ros_command([str(binary),'--calibration',str(calibration),'--output',str(out/'run'),'--duration-sim',str(a.duration_sim),'--wall-timeout',str(a.wall_timeout)])
 if session:command+=['--source-session',session,'--source-generation',str(generation)]
 if a.track_fixed:command.append('--track-fixed')
 if a.track_rolling:command.append('--track-rolling')
 child=None;observed=[];started=time.monotonic()
 try:
  with (out/'console.log').open('w') as log:
   child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True,preexec_fn=lambda:helper.child_limits(True))
   while child.poll() is None:
    if time.monotonic()-started>a.wall_timeout+10:raise RuntimeError('production input wall timeout')
    try:
     threads=helper.inventory(child.pid);algorithm=[t for t in threads if t['name']=='ego4p2_coord' or t['name'].startswith('depth_cam')]
     if len(algorithm)==5 and (out/'run/preprocess_execution.json').exists():
      byname={t['name']:t for t in algorithm};assert byname['ego4p2_coord']['cpus']==[0]
      for c in range(4):assert byname['depth_cam'+str(c)]['cpus']==[c+1]
      assert all(t['policy']==1 and t['priority']==5 and t['uid']==str(uid) and int(t['cap_eff'],16)==0 for t in algorithm)
      assert all(t['policy']==0 and t['priority']==0 for t in threads if t not in algorithm);observed=threads
    except (FileNotFoundError,ProcessLookupError):pass
    time.sleep(.02)
   if child.returncode or not observed:raise RuntimeError('production input did not finish with verified FIFO5; see '+str(out))
 finally:
  if child is not None and child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
  (out/'execution.json').write_text(json.dumps(dict(scope=scope,command=command,exit_code=None if child is None else child.returncode,threads=observed,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()),indent=2)+'\n')
  for file in out.glob('*'):
   if file.is_file():os.chown(file,uid,owner.pw_gid)
 print('RESULT_DIR='+str(out))
if __name__=='__main__':main()
