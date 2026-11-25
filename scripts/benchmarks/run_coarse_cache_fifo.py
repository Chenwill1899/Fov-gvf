#!/usr/bin/python3
"""Compare validated local-fine planning and its coarse graph cache; isolated evidence and verified FIFO5."""
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
import signal
import statistics
import subprocess
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / 'docs/performance/coarse_cache_20261116'
LIBRARY = ROOT / 'docs/performance/harmonic_block4_20261104_213849/deps/suitesparse/usr/lib/x86_64-linux-gnu'
RESOURCE_FIELDS = {'minor_faults','major_faults','voluntary_switches','involuntary_switches'}
MODES = ['cold','new_group_reuse','same_group','intent_change','anchor_translation']
DATASET_CASES = {'archive':['archive_turn','archive_straight','open90','open180','open3d','all_unknown'],
                 'synthetic_known_near':['synthetic_known_near_straight','synthetic_known_near_3d']}
CASES = [case for cases in DATASET_CASES.values() for case in cases]

def measurement_field(key):
    return key.endswith(('_ms','_ns')) or key=='skipped_releases' or key in RESOURCE_FIELDS


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(pid):
    result = []
    for path in (Path('/proc') / str(pid) / 'task').iterdir():
        tid = int(path.name)
        status = dict(line.split(':', 1) for line in (path/'status').read_text().splitlines() if ':' in line)
        result.append(dict(tid=tid, name=(path/'comm').read_text().strip(), policy=os.sched_getscheduler(tid),
                           priority=os.sched_getparam(tid).sched_priority, cpus=sorted(os.sched_getaffinity(tid)),
                           uid=int(status['Uid'].split()[0]), cap_eff=int(status['CapEff'].strip(), 16)))
    return sorted(result, key=lambda item:item['tid'])


def child_limits():
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    resource.setrlimit(resource.RLIMIT_RTPRIO, (5, 5))
    resource.setrlimit(resource.RLIMIT_RTTIME, (5_000_000, 6_000_000))
    resource.setrlimit(resource.RLIMIT_CPU, (90, 95))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def verify_manifest(manifest):
    checked = 0
    def check(relative, expected):
        nonlocal checked
        checked += 1
        path = (ROOT / relative).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file() or digest(path) != expected:
            raise RuntimeError('stale or missing validated input: ' + str(relative))
    for relative, expected in manifest['shared_inputs'].items():
        check(relative, expected)
    for binary in manifest['binaries'].values():
        check(binary['path'], binary['sha256'])
    for identity in manifest['source_identities'].values():
        for relative, expected in identity['identity'].items():
            check(str(Path(identity['source_root']) / relative), expected)
    check(str((EXPERIMENT/'before/source.tar.gz').relative_to(ROOT)),manifest['source_snapshot_sha256'])
    frozen=json.loads((EXPERIMENT/'before/manifest.json').read_text())
    for relative,expected in frozen['baseline_files'].items():
        check(str((EXPERIMENT/'baseline_source'/relative).relative_to(ROOT)),expected)
    return dict(sha_checks=checked,frozen_baseline_files=len(frozen['baseline_files']),
                source_roots={name:identity['source_root'] for name,identity in manifest['source_identities'].items()})


def analyze(output, records, performance, repeats):
    rows = {}
    expected = {(case,mode,str(i)) for case in CASES for mode in MODES for i in range(repeats)}
    release_checks = {}
    for record in records:
        with (output/record['label']/'run/timings.csv').open() as stream:
            data = list(csv.DictReader(stream))
        local_cases=DATASET_CASES[record['dataset']]
        local_expected={(case,mode,str(i)) for case in local_cases for mode in MODES for i in range(repeats)}
        keyed = {(row['case'],row['mode'],row['iteration']):row for row in data}
        if len(data)!=len(local_expected) or set(keyed)!=local_expected:
            raise RuntimeError('incomplete or duplicate workload rows: '+record['label'])
        if record['dataset']=='synthetic_known_near':
            for case in local_cases:
                if not any(row['case']==case and row['spatial_ready']=='1' and int(row['observed_sections'])>0 for row in data):
                    raise RuntimeError('synthetic fixture did not provide observed-clear section evidence: '+case)
        intervals=[];release_steps=[]
        if performance:
            for case in local_cases:
                for mode in MODES:
                    previous=None
                    for index in range(repeats):
                        row=keyed[(case,mode,str(index))]
                        release,begin,end,next_release=(int(row[key]) for key in ['scheduled_ns','begin_ns','end_ns','next_release_ns'])
                        skipped=int(row['skipped_releases'])
                        if not release<=begin<=end or skipped!=(end-release)//5000000 or next_release!=release+(skipped+1)*5000000:
                            raise RuntimeError('invalid release timestamps: '+record['label'])
                        for key,value in [('compute_ms',end-begin),('response_ms',end-release),('lateness_ms',begin-release)]:
                            if abs(float(row[key])*1e6-value)>1.:
                                raise RuntimeError('duration disagrees with absolute clock: '+key)
                        if previous is not None:
                            if release!=int(previous['next_release_ns']):
                                raise RuntimeError('release sequence changed inside a workload')
                            intervals.append((begin-int(previous['begin_ns']))/1e6)
                            release_steps.append((release-int(previous['scheduled_ns']))/1e6)
                        previous=row
        release_checks[record['label']]=dict(rows=len(data),inter_start_samples=len(intervals),
            minimum_inter_start_ms=min(intervals) if intervals else None,
            maximum_inter_start_ms=max(intervals) if intervals else None,
            scheduled_release_step_ms=sorted(set(release_steps)))
        rows[record['label']]=keyed
    mismatches=[];difference_count=0;expected_differences={};compared=0;max_error=0.
    topology_rows=0
    first=next(iter(next(iter(rows.values())).values()))
    keys=sorted(key for key in first if not measurement_field(key))
    for round_id in sorted({record['round'] for record in records}):
        old={key:row for record in records if record['round']==round_id and record['variant']=='BASELINE' for key,row in rows[record['label']].items()}
        new={key:row for record in records if record['round']==round_id and record['variant']=='CURRENT' for key,row in rows[record['label']].items()}
        if set(old)!=expected or set(new)!=expected:raise RuntimeError('dataset merge is incomplete')
        for identity in sorted(expected):
            a,b=old[identity],new[identity];compared+=1
            # Both variants are the already validated local-fine topology
            # algorithm. No old numerical/status exception remains in step 2.
            for variant,row in [('BASELINE',a),('CURRENT',b)]:
                topology_ready=row['coarse_status']=='topology_ready'
                early=row['coarse_status']=='no_input' and row['angular_ready']=='0' and row['case']=='all_unknown'
                if row['coarse_topology_only']!='1' or row['coarse_topology_failure']!='none' or not (topology_ready or early):
                    raise RuntimeError('unexpected topology mode/status: '+str(identity))
                no_numerics=(math.isnan(float(row['coarse_residual']))
                    and all(int(row[key])==0 for key in row if key.startswith('coarse_') and key.endswith('_ns') and key not in {'coarse_domain_ns','coarse_topology_ns','coarse_graph_cache_ns'})
                    and int(row['coarse_iterations'])==0
                    and all(float(row[key])==0. for key in row if key.startswith('coarse_backend_')))
                if not no_numerics:raise RuntimeError('topology-only planner claims coarse numerical evidence: '+str(identity))
                hit=row['coarse_graph_reused']=='1'
                if row['coarse_graph_reused'] not in {'0','1'}:raise RuntimeError('invalid graph cache marker')
                if variant=='BASELINE' and (hit or int(row['coarse_graph_cache_ns'])!=0):
                    raise RuntimeError('baseline cannot claim the new graph cache')
                if hit and (not topology_ready or int(row['coarse_topology_ns'])!=0):
                    raise RuntimeError('graph hit did not preserve the topology-only path')
                if early and any(int(row[key])!=0 for key in ['coarse_topology_ns','coarse_domain_ns','coarse_graph_cache_ns','coarse_graph_reused']):
                    raise RuntimeError('early reject performed coarse graph work')
                if topology_ready and not hit and int(row['coarse_topology_ns'])<=0:
                    raise RuntimeError('graph miss must validate the topology')
            if b['coarse_status']=='topology_ready':topology_rows+=1
            for key in keys:
                if a[key]==b[key]:continue
                if key=='coarse_graph_reused':
                    item=expected_differences.setdefault(key,dict(count=0,first_baseline=a[key],first_current=b[key]))
                    item['count']+=1
                    continue
                try:
                    av,bv=float(a[key]),float(b[key])
                    if math.isfinite(av) and math.isfinite(bv):
                        max_error=max(max_error,abs(av-bv))
                        if av==bv:continue
                    elif math.isnan(av) and math.isnan(bv):continue
                except ValueError:pass
                difference_count+=1
                if len(mismatches)<100:mismatches.append(dict(round=round_id,case=a['case'],mode=a['mode'],iteration=a['iteration'],field=key,baseline=a[key],current=b[key]))
    result=dict(performance_run=performance,world_dimensions=3,motion_output_enabled=False,
        compared_rows=compared,functional_fields_compared=keys,functional_match=difference_count==0,
        functional_difference_count=difference_count,first_mismatches=mismatches,
        tolerance=dict(relative=0.,absolute=0.),maximum_non_diagnostic_numeric_difference=max_error,
        expected_graph_cache_diagnostics=expected_differences,validated_topology_only_rows=topology_rows,
        translation_cache_exceptions=False,release_validation=release_checks,
        geometry_ready_is_executable_clearance=False)
    if performance:
        def summary(values):
            data=sorted(values)
            if not data:return None
            return dict(count=len(data),mean_ms=statistics.fmean(data),p99_ms=data[math.ceil(.99*len(data))-1],maximum_ms=data[-1],over_5ms=sum(v>5 for v in data))
        duration_keys=[key for key in first if key.endswith('_ms') or (key.endswith('_ns') and key not in {'scheduled_ns','begin_ns','end_ns','next_release_ns'})]
        def describe(selected):
            answer=dict(count=len(selected),angular_geometry_ready_count=sum(row['angular_ready']=='1' for row in selected),
                spatial_geometry_ready_count=sum(row['spatial_ready']=='1' for row in selected),
                rejected_count=sum(row['angular_ready']!='1' for row in selected),
                all_sections_observed_clear_count=sum(int(row['observed_sections'])>0 and all(int(row[k])==0 for k in ['unknown_sections','obstacle_sections','outside_sections']) for row in selected))
            for label,group in [('all_samples',selected),('angular_geometry_ready',[r for r in selected if r['angular_ready']=='1']),('spatial_geometry_ready',[r for r in selected if r['spatial_ready']=='1'])]:
                answer[label]={key:summary(float(row[key])/(1e6 if key.endswith('_ns') else 1.) for row in group) for key in duration_keys}
                answer[label]['skipped_releases']=sum(int(row['skipped_releases']) for row in group)
                answer[label]['resources']={key:sum(int(row[key]) for row in group) for key in RESOURCE_FIELDS}
            return answer
        aggregate={}
        for variant in ['BASELINE','CURRENT']:
            merged=[row for record in records if record['variant']==variant for row in rows[record['label']].values()]
            aggregate[variant]={'all_loads':describe(merged),'by_dataset':{dataset:describe([row for row in merged if row['case'] in cases]) for dataset,cases in DATASET_CASES.items()},'by_mode':{mode:describe([row for row in merged if row['mode']==mode]) for mode in MODES},
                'by_case_mode':{case:{mode:describe([row for row in merged if row['case']==case and row['mode']==mode]) for mode in MODES} for case in CASES}}
        result['timings']=aggregate
        result['by_dataset_mode_round']=[dict(variant=variant,dataset=dataset,round=round_id,mode=mode,
            summary=describe([row for record in records if record['variant']==variant and record['dataset']==dataset and record['round']==round_id
                              for row in rows[record['label']].values() if row['mode']==mode]))
            for dataset in DATASET_CASES for mode in MODES for round_id in sorted({r['round'] for r in records})
            for variant in ['BASELINE','CURRENT']]
        result['duration_summary_units']='All duration summaries use milliseconds; source columns ending _ns retain integer nanoseconds in CSV.'
    else:
        result['timing_conclusion']='not measured: ordinary scheduling is used only for correctness'
    (output/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--correctness',action='store_true',help='ordinary-scheduling correctness only; no performance result')
    parser.add_argument('--rounds',type=int,default=3)
    parser.add_argument('--repeats',type=int,default=100)
    parser.add_argument('--manifest',type=Path,help='choose a retained completed build manifest')
    args=parser.parse_args()
    if not 1<=args.rounds<=4 or args.repeats<1:
        parser.error('rounds 1..4, repeats must be positive')
    manifest_path=(args.manifest or ROOT/json.loads((EXPERIMENT/'latest_build.json').read_text())['manifest']).resolve()
    if not manifest_path.is_relative_to(EXPERIMENT):parser.error('manifest must belong to this coarse-cache experiment')
    manifest=json.loads(manifest_path.read_text())
    verification=verify_manifest(manifest)
    fifo=not args.correctness
    scope=dict(rounds=args.rounds,repeats=args.repeats,scheduler='FIFO5' if fifo else 'correctness-only SCHED_OTHER',
               coordinator_cpu=0,camera_cpus=[1,2,3,4],world_dimensions=3,period_ns=5000000,
               motion_output_enabled=False,simulation_started=False,global_rt_settings_modified=False,
               comparison='same driver and CPU instrumentation; validated local-fine planner versus exact coarse graph cache',
               manifest=str(manifest_path),absolute_release_timestamps=True,cpu_scope='coordinator thread; excludes camera-worker CPU',
               production_install_modified=False,datasets=DATASET_CASES,synthetic_calibration_is_measured=False,
               functional_tolerance=0.,expected_diagnostic_difference='coarse_graph_reused only')
    if args.check:
        print(json.dumps(dict(scope=scope,manifest_verified=True,verification=verification),indent=2));return
    if fifo and os.geteuid()!=0:
        parser.error('FIFO5 needs sudo /usr/bin/python3 -I '+str(Path(__file__).resolve())+'; no ordinary-scheduling fallback')
    if args.correctness and os.geteuid()==0:
        parser.error('correctness checks must run as the project user')
    if os.sched_getscheduler(0)!=os.SCHED_OTHER:
        parser.error('supervisor must remain SCHED_OTHER')
    uid=int(os.environ.get('SUDO_UID',ROOT.stat().st_uid)) if fifo else os.getuid()
    owner=pwd.getpwuid(uid)
    if uid==0:
        parser.error('the algorithm must run as the non-root project user')
    output=EXPERIMENT/('coarse_cache_'+('fifo_' if fifo else 'correctness_')+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    def save(path,value):
        path.write_text(json.dumps(value,indent=2)+'\n')
        if fifo:os.chown(path,uid,owner.pw_gid)
    if fifo:os.chown(output,uid,owner.pw_gid)
    kernel=lambda:{key:Path('/proc/sys/kernel/'+key).read_text().strip() for key in ['sched_rt_runtime_us','sched_rt_period_us']}
    save(output/'scope.json',scope);save(output/'manifest.json',manifest);save(output/'precheck.json',verification);save(output/'kernel_before.json',kernel())
    env=dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin',LANG='C.UTF-8',HOME=owner.pw_dir,LD_LIBRARY_PATH=str(LIBRARY),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    records=[]
    for round_id in range(1,args.rounds+1):
        for variant,dataset in [(variant,dataset) for dataset in DATASET_CASES for variant in (['BASELINE','CURRENT'] if round_id%2 else ['CURRENT','BASELINE'])]:
            label=f'r{round_id}_{variant}_{dataset}';folder=output/label;folder.mkdir()
            if fifo:os.chown(folder,uid,owner.pw_gid)
            binary=ROOT/manifest['binaries'][variant]['path']
            command=[str(binary),str(ROOT/'docs/performance/current_baseline_20261105_141800/input'),str(folder/'run'),str(ROOT/'docs/performance/step5_rolling_20261107/fixture.json'),str(args.repeats),'fifo5' if fifo else 'correctness',variant,dataset]
            if fifo:
                command=['/usr/bin/setpriv','--reuid',str(uid),'--regid',str(owner.pw_gid),'--init-groups','--reset-env','--no-new-privs','--bounding-set=-all','--inh-caps=-all','--ambient-caps=-all','--','/usr/bin/env',*[k+'='+v for k,v in env.items()],*command]
            record=dict(label=label,round=round_id,variant=variant,dataset=dataset,command=command,binary_sha256=digest(binary))
            child=None;observed=[];thread_samples=[];last_sample=0.;started=time.monotonic()
            try:
                with (folder/'console.log').open('w') as log:
                    child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=dict(PATH=env['PATH'],LANG=env['LANG']) if fifo else env,start_new_session=True,preexec_fn=child_limits if fifo else None)
                    while child.poll() is None:
                        if time.monotonic()-started>max(180.,len(DATASET_CASES[dataset])*len(MODES)*(args.repeats+10)*.005*3+60):raise RuntimeError('benchmark wall timeout')
                        if (folder/'run/execution_metadata.json').is_file():
                            try:
                                threads=inventory(child.pid)
                                if len(threads)==5:
                                    for index,thread in enumerate(threads):
                                        if thread['cpus']!=[index] or thread['uid']!=uid or thread['cap_eff']!=0 or thread['policy']!=(1 if fifo else 0) or thread['priority']!=(5 if fifo else 0):raise RuntimeError('algorithm thread policy/affinity/privilege mismatch')
                                    observed=threads
                                    if time.monotonic()-last_sample>=.25:
                                        thread_samples.append(dict(elapsed_s=time.monotonic()-started,threads=threads));last_sample=time.monotonic()
                            except (FileNotFoundError,ProcessLookupError):pass
                        time.sleep(.01)
                    if child.returncode:raise RuntimeError(f'{label} failed ({child.returncode}); see {folder}')
                metadata=json.loads((folder/'run/execution_metadata.json').read_text())
                if metadata['dataset']!=dataset or metadata['explicit_synthetic_calibration']!=(dataset=='synthetic_known_near'):raise RuntimeError('dataset calibration provenance mismatch')
                if not observed or metadata['worker_policies']!=[1 if fifo else 0]*4 or metadata['worker_priorities']!=[5 if fifo else 0]*4 or metadata['worker_cpus']!=[1,2,3,4] or metadata['worker_scheduler_errors']!=[0]*4 or metadata['worker_affinity_errors']!=[0]*4:raise RuntimeError('five verified algorithm threads required')
                record.update(exit_code=0,threads=observed,thread_observations=thread_samples,metadata=metadata,wall_s=time.monotonic()-started)
                print(label+': verified five '+('FIFO5' if fifo else 'correctness-only')+' threads',flush=True)
            except BaseException as error:
                if child is not None and child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
                record.update(error=repr(error),exit_code=None if child is None else child.returncode)
                raise
            finally:
                records.append(record);save(output/'execution.json',records)
                if fifo:os.chown(folder/'console.log',uid,owner.pw_gid)
    save(output/'kernel_after.json',kernel())
    if json.loads((output/'kernel_before.json').read_text())!=json.loads((output/'kernel_after.json').read_text()):raise RuntimeError('kernel RT settings changed during comparison')
    result=analyze(output,records,fifo,args.repeats)
    if fifo:os.chown(output/'analysis.json',uid,owner.pw_gid)
    print('RESULT_DIR='+str(output),flush=True)
    if not result['functional_match']:raise RuntimeError('functional differences require review; see analysis.json')


if __name__=='__main__':
    main()
