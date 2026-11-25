#!/usr/bin/python3
"""Verified five-thread FIFO5, same-binary Step2 versus optimized Step3 comparison; no motion."""
import argparse,hashlib,importlib.util,json,math,os,pwd,signal,subprocess,time
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
P=Path(__file__).resolve().parents[2]
E=P/'docs/performance/step3_optimized_20261107'
VARIANTS=('step2','step3')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true');parser.add_argument('--rounds',type=int,default=3);parser.add_argument('--repeats',type=int,default=200);a=parser.parse_args()
 if not 1<=a.rounds<=4 or not 1<=a.repeats<=1000:parser.error('rounds 1..4, repeats 1..1000')
 validation=json.loads((E/'validation_summary.json').read_text());assert validation['passed'] and validation['ready_for_user_fifo'] and not validation['new_fifo_performance_run']
 manifest=json.loads((E/'fifo_manifest.json').read_text());mismatches=[p for p,h in manifest.items() if not Path(p).is_file() or sha(Path(p))!=h]
 if mismatches:raise RuntimeError('source/input/binary differs from validated build: '+str(mismatches[:4]))
 scope=dict(rounds=a.rounds,repeats=a.repeats,period_ms=5,variants=list(VARIANTS),coordinator_cpu=0,camera_cpus=[1,2,3,4],scheduler='SCHED_FIFO/5',motion_output_enabled=False,spatial_reference_modes=['step3'],continuous_tube_check_modes=['step3'],same_binary=True,simulation_started=False,stationary_archive_with_explicit_synthetic_state=True,variant_definition=dict(step2='unified input to Q, .30m envelope',step3='same Q, then optimized continuous space reference and raw-depth tube'),input_chain='4 persistent camera workers -> 4x4 -> 3x3 -> panorama/Omega -> two LDLT -> Q',global_rt_settings_modified=False)
 if a.check:print(json.dumps(dict(scope=scope,validation_passed=True,hashes_checked=len(manifest)),ensure_ascii=False,indent=2));return
 if os.geteuid()!=0:parser.error('FIFO5 needs the authorized launcher: sudo /usr/bin/python3 -I '+str(Path(__file__).resolve()))
 if os.sched_getscheduler(0)!=os.SCHED_OTHER:raise RuntimeError('supervisor must remain SCHED_OTHER')
 uid=int(os.environ.get('SUDO_UID',P.stat().st_uid));assert uid!=0;owner=pwd.getpwuid(uid)
 spec=importlib.util.spec_from_file_location('fifo_helper',P/'scripts/benchmarks/run_ldlt_candidate_benchmark.py');helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
 output=E/('run_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));output.mkdir();os.chown(output,owner.pw_uid,owner.pw_gid)
 def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');os.chown(path,owner.pw_uid,owner.pw_gid)
 kernel=lambda:{n:Path('/proc/sys/kernel/'+n).read_text().strip() for n in ['sched_rt_runtime_us','sched_rt_period_us']}
 save(output/'scope.json',scope);save(output/'kernel_before.json',kernel());save(output/'measurement_manifest.json',manifest)
 records=[];library=P/'docs/performance/harmonic_block4_20261104_213849/deps/suitesparse/usr/lib/x86_64-linux-gnu'
 env=dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin',LANG='C.UTF-8',HOME=owner.pw_dir,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',LD_LIBRARY_PATH=str(library))
 def as_owner(cmd):return ['/usr/bin/setpriv','--reuid',str(owner.pw_uid),'--regid',str(owner.pw_gid),'--init-groups','--reset-env','--no-new-privs','--bounding-set=-all','--inh-caps=-all','--ambient-caps=-all','--','/usr/bin/env']+[k+'='+v for k,v in env.items()]+cmd
 for round_id in range(1,a.rounds+1):
  order=VARIANTS if round_id%2 else tuple(reversed(VARIANTS))
  for variant in order:
   label=f'r{round_id}_{variant}';folder=output/label;folder.mkdir();os.chown(folder,owner.pw_uid,owner.pw_gid)
   spatial=variant=='step3'
   binary=E/'ready_bins/navigation_replay'
   command=as_owner(['/usr/bin/taskset','-c','0-4',str(binary),str(P/'docs/performance/current_baseline_20261105_141800/input'),str(folder/'run'),str(E/'fixture.json'),str(a.repeats),'fifo5']+(['--spatial'] if spatial else []))
   record=dict(label=label,round=round_id,variant=variant,command=command,binary_sha256=sha(binary));child=None;started=time.monotonic();observed=[]
   try:
    with (folder/'console.log').open('w') as log:
     child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=dict(PATH=env['PATH'],LANG=env['LANG']),start_new_session=True,preexec_fn=lambda:helper.child_limits(True))
     while child.poll() is None:
      if time.monotonic()-started>180:raise RuntimeError('child wall timeout')
      if (folder/'run/preprocess_execution.json').is_file():
       try:
        threads=helper.inventory(child.pid)
        if len(threads)==5:
         assert all(t['policy']==os.SCHED_FIFO and t['priority']==5 and t['cpus']==[i] and t['uid']==str(uid) and int(t['cap_eff'],16)==0 for i,t in enumerate(threads)),threads
         observed=threads
       except (FileNotFoundError,ProcessLookupError):pass
      time.sleep(.01)
     record['exit_code']=child.returncode
     if child.returncode:raise RuntimeError(f'{label} failed ({child.returncode}); see {folder}')
     metadata=json.loads((folder/'run/preprocess_execution.json').read_text());assert observed and metadata['coordinator_scheduler']==1 and metadata['coordinator_priority']==5 and metadata['worker_policies']==[1]*4 and metadata['worker_priorities']==[5]*4 and metadata['worker_cpus']==[1,2,3,4] and metadata['worker_affinity_errors']==[0]*4 and metadata['worker_scheduler_errors']==[0]*4 and metadata['period_ns']==5000000
     assert math.isclose(metadata['omega_radius_m'],.3,abs_tol=1e-12)
     assert json.loads((folder/'run/base_config.json').read_text())==json.loads((P/'src/new_controller/config/base_step1.json').read_text())
     assert metadata['secondary_summary_in_camera_threads'] is True
     assert metadata['variant']==(2 if spatial else 1)
     assert metadata['spatial_reference_enabled'] is spatial and metadata['continuous_tube_check_enabled'] is spatial
     assert [metadata[k] for k in ('spatial_ramp_time_s','spatial_ramp_length_m','spatial_grid_m')]==[.10,1.20,.05]
     assert json.loads((folder/'run/navigation_config.json').read_text())==json.loads((E/'configuration.json').read_text())
     record.update(threads=observed,metadata=metadata,wall_s=time.monotonic()-started);print(f'{label}: confirmed five FIFO5 threads; {record["wall_s"]:.2f}s',flush=True)
   except BaseException as error:
    if child is not None and child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
    record.update(error=repr(error),exit_code=None if child is None else child.returncode);raise
   finally:
    records.append(record);save(output/'execution.json',records);os.chown(folder/'console.log',owner.pw_uid,owner.pw_gid)
 save(output/'kernel_after.json',kernel())
 if json.loads((output/'kernel_after.json').read_text())!=json.loads((output/'kernel_before.json').read_text()):raise RuntimeError('external kernel RT settings changed during comparison')
 cmd=as_owner(['/usr/bin/python3','-I',str(E/'analyze_fifo.py'),str(output)]);subprocess.run(cmd,check=True,env=dict(PATH=env['PATH'],LANG=env['LANG']))
 print('RESULT_DIR='+str(output),flush=True)
if __name__=='__main__':main()
