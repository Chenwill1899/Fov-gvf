#!/usr/bin/python3
"""Compare the frozen and intrinsic-2D spatial implementations; 3-D flight is preserved."""
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
EXPERIMENT = ROOT / 'docs/performance/planar_20261116'
MANIFEST = EXPERIMENT / 'intrinsic_build/manifest.json'
LIBRARY = ROOT / 'docs/performance/harmonic_block4_20261104_213849/deps/suitesparse/usr/lib/x86_64-linux-gnu'
TIME_FIELDS = {'compute_ms','response_ms','lateness_ms','skipped_releases','preprocess_ms','capture_ms','field_ms','spatial_ms','occupancy_ms'}


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
    def check(relative, expected):
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
    if digest(EXPERIMENT/'before/source.tar.gz') != manifest['source_snapshot_sha256']:
        raise RuntimeError('frozen snapshot identity changed')


def analyze(output, records, performance):
    rows = {}
    for record in records:
        with (output/record['label']/'run/timings.csv').open() as stream:
            rows[record['label']] = list(csv.DictReader(stream))
    mismatches = []
    expected_cache_differences = {}
    expected_cache_deltas = {}
    compared = 0
    max_error = 0.
    for round_id in sorted({record['round'] for record in records}):
        old, new = rows[f'r{round_id}_BASELINE'], rows[f'r{round_id}_CURRENT']
        if len(old) != len(new):
            raise RuntimeError('benchmark row counts differ')
        for a,b in zip(old,new):
            compared += 1
            for key in a.keys() - TIME_FIELDS:
                if a[key] == b[key]:
                    continue
                if a['mode']=='anchor_translation' and key in {'field_reused','field_version','curve_version'}:
                    expected_cache_differences[key]=expected_cache_differences.get(key,0)+1
                    expected_cache_deltas.setdefault(key,[]).append(float(a[key])-float(b[key]))
                    continue
                try:
                    av,bv = float(a[key]),float(b[key])
                    if math.isfinite(av) and math.isfinite(bv):
                        max_error = max(max_error,abs(av-bv))
                        if math.isclose(av,bv,rel_tol=2e-9,abs_tol=2e-9):
                            continue
                except ValueError:
                    pass
                if len(mismatches)<100:
                    mismatches.append(dict(round=round_id,case=a['case'],mode=a['mode'],iteration=a['iteration'],field=key,baseline=a[key],current=b[key]))
    result = dict(performance_run=performance,world_dimensions=3,motion_output_enabled=False,
                  compared_rows=compared,functional_fields_compared=sorted(next(iter(rows.values()))[0].keys()-TIME_FIELDS),
                  tolerance=dict(relative=2e-9,absolute=2e-9),maximum_non_diagnostic_numeric_difference=max_error,
                  functional_match=not mismatches,first_mismatches=mismatches,
                  expected_anchor_translation_cache_differences=expected_cache_differences,
                  expected_cache_baseline_minus_current_ranges={key:dict(minimum=min(values),maximum=max(values)) for key,values in expected_cache_deltas.items()})
    if performance:
        def summary(data):
            data=sorted(data)
            if not data:return None
            return dict(count=len(data),mean_ms=statistics.fmean(data),p99_ms=data[math.ceil(.99*len(data))-1],maximum_ms=data[-1],over_5ms=sum(x>5 for x in data))
        def describe(selected):
            answer=dict(count=len(selected),angular_ready_count=sum(row['angular_ready']=='1' for row in selected),
                        spatial_ready_count=sum(row['spatial_ready']=='1' for row in selected),
                        rejected_count=sum(row['angular_ready']!='1' for row in selected))
            for label, group in [('all_samples',selected),('angular_success',[row for row in selected if row['angular_ready']=='1']),
                                 ('spatial_success',[row for row in selected if row['spatial_ready']=='1'])]:
                answer[label] = {key:summary([float(row[key]) for row in group])
                    for key in ['compute_ms','response_ms','preprocess_ms','capture_ms','field_ms','spatial_ms','occupancy_ms']}
                answer[label]['skipped_releases']=sum(int(row['skipped_releases']) for row in group)
            return answer
        aggregate = {}
        for variant in ['BASELINE','CURRENT']:
            merged=[row for label,group in rows.items() if label.endswith(variant) for row in group]
            aggregate[variant] = {'all_loads':describe(merged)}
            for scenario in ['cold','new_group_reuse','same_group','intent_change','anchor_translation']:
                selected=[row for row in merged if row['mode']==scenario]
                aggregate[variant][scenario] = describe(selected)
            aggregate[variant]['by_case_mode']={name:{scenario:describe([row for row in merged if row['case']==name and row['mode']==scenario])
                for scenario in ['cold','new_group_reuse','same_group','intent_change','anchor_translation']} for name in sorted({row['case'] for row in merged})}
        result['timings'] = aggregate
    else:
        result['timing_conclusion'] = 'not measured: ordinary scheduling is used only for correctness'
    (output/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--correctness',action='store_true',help='ordinary-scheduling correctness only; no performance result')
    parser.add_argument('--rounds',type=int,default=3)
    parser.add_argument('--repeats',type=int,default=100)
    args=parser.parse_args()
    if not 1<=args.rounds<=4 or not 1<=args.repeats<=1000:
        parser.error('rounds 1..4, repeats 1..1000')
    if args.correctness:
        args.rounds,args.repeats=1,2
    manifest=json.loads(MANIFEST.read_text())
    verify_manifest(manifest)
    fifo=not args.correctness
    scope=dict(rounds=args.rounds,repeats=args.repeats,scheduler='FIFO5' if fifo else 'correctness-only SCHED_OTHER',
               coordinator_cpu=0,camera_cpus=[1,2,3,4],world_dimensions=3,period_ns=5000000,
               motion_output_enabled=False,simulation_started=False,global_rt_settings_modified=False,
               comparison='same driver and input; frozen source versus intrinsic 2-D representation with 3-D outputs',
               functional_tolerance=2e-9)
    if args.check:
        print(json.dumps(dict(scope=scope,manifest_verified=True),indent=2));return
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
    output=EXPERIMENT/('intrinsic_'+('fifo_' if fifo else 'correctness_')+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    def save(path,value):
        path.write_text(json.dumps(value,indent=2)+'\n')
        if fifo:os.chown(path,uid,owner.pw_gid)
    if fifo:os.chown(output,uid,owner.pw_gid)
    kernel=lambda:{key:Path('/proc/sys/kernel/'+key).read_text().strip() for key in ['sched_rt_runtime_us','sched_rt_period_us']}
    save(output/'scope.json',scope);save(output/'manifest.json',manifest);save(output/'kernel_before.json',kernel())
    env=dict(PATH='/usr/sbin:/usr/bin:/sbin:/bin',LANG='C.UTF-8',HOME=owner.pw_dir,LD_LIBRARY_PATH=str(LIBRARY),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    records=[]
    for round_id in range(1,args.rounds+1):
        for variant in (['BASELINE','CURRENT'] if round_id%2 else ['CURRENT','BASELINE']):
            label=f'r{round_id}_{variant}';folder=output/label;folder.mkdir()
            if fifo:os.chown(folder,uid,owner.pw_gid)
            binary=ROOT/manifest['binaries'][variant]['path']
            command=[str(binary),str(ROOT/'docs/performance/current_baseline_20261105_141800/input'),str(folder/'run'),str(ROOT/'docs/performance/step5_rolling_20261107/fixture.json'),str(args.repeats),'fifo5' if fifo else 'correctness',variant]
            if fifo:
                command=['/usr/bin/setpriv','--reuid',str(uid),'--regid',str(owner.pw_gid),'--init-groups','--reset-env','--no-new-privs','--bounding-set=-all','--inh-caps=-all','--ambient-caps=-all','--','/usr/bin/env',*[k+'='+v for k,v in env.items()],*command]
            record=dict(label=label,round=round_id,variant=variant,command=command,binary_sha256=digest(binary))
            child=None;observed=[];started=time.monotonic()
            try:
                with (folder/'console.log').open('w') as log:
                    child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=dict(PATH=env['PATH'],LANG=env['LANG']) if fifo else env,start_new_session=True,preexec_fn=child_limits if fifo else None)
                    while child.poll() is None:
                        if time.monotonic()-started>180:raise RuntimeError('benchmark wall timeout')
                        if (folder/'run/execution_metadata.json').is_file():
                            try:
                                threads=inventory(child.pid)
                                if len(threads)==5:
                                    for index,thread in enumerate(threads):
                                        if thread['cpus']!=[index] or thread['uid']!=uid or thread['cap_eff']!=0 or thread['policy']!=(1 if fifo else 0) or thread['priority']!=(5 if fifo else 0):raise RuntimeError('algorithm thread policy/affinity/privilege mismatch')
                                    observed=threads
                            except (FileNotFoundError,ProcessLookupError):pass
                        time.sleep(.01)
                    if child.returncode:raise RuntimeError(f'{label} failed ({child.returncode}); see {folder}')
                metadata=json.loads((folder/'run/execution_metadata.json').read_text())
                if not observed or metadata['worker_policies']!=[1 if fifo else 0]*4 or metadata['worker_priorities']!=[5 if fifo else 0]*4 or metadata['worker_cpus']!=[1,2,3,4] or metadata['worker_scheduler_errors']!=[0]*4 or metadata['worker_affinity_errors']!=[0]*4:raise RuntimeError('five verified algorithm threads required')
                record.update(exit_code=0,threads=observed,metadata=metadata,wall_s=time.monotonic()-started)
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
    result=analyze(output,records,fifo)
    if fifo:os.chown(output/'analysis.json',uid,owner.pw_gid)
    print('RESULT_DIR='+str(output),flush=True)
    if not result['functional_match']:raise RuntimeError('functional differences require review; see analysis.json')


if __name__=='__main__':
    main()
