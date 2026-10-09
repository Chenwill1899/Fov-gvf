"""Lightweight collector tests: no ROS, simulator, C++ or benchmark execution."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile
import pytest
sys.path.insert(0,str(Path(__file__).parent))
import collect_adaptive as c


def put(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data))


def freeze(path,files,binary_sha):
    path.mkdir()
    put(path/'source_sha256.json',{k:hashlib.sha256(v).hexdigest() for k,v in files.items()})
    put(path/'binary_sha256.json',{'depth_angular_controller':binary_sha})
    with tarfile.open(path/'sources.tar.gz','w:gz') as f:
        for name,data in files.items():
            info=tarfile.TarInfo(name);info.size=len(data);f.addfile(info,io.BytesIO(data))


@pytest.fixture
def fixture(tmp_path):
    project=tmp_path/'project';project.mkdir()
    cloud=project/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    benchmark=project/'src/pc_gvf/launch/navigation_benchmark.launch.py'
    cloud.parent.mkdir(parents=True);cloud.write_text('frozen cloud');benchmark.write_text('frozen benchmark')
    install=tmp_path/'install';target=install/'pc_gvf/share/pc_gvf/launch';target.mkdir(parents=True)
    (target/cloud.name).write_bytes(cloud.read_bytes());(target/benchmark.name).write_bytes(benchmark.read_bytes())
    binary=project/'controller';binary.write_bytes(b'controller')
    fz=tmp_path/'freeze';freeze(fz,{'a.cpp':b'core'},c.sha(binary))
    output=tmp_path/'runset';output.mkdir()
    rows=[];results=[]
    for i,side in enumerate(['before','after']):
        rid=f'jitter_{side}_1';log=output/(rid+'.log');log.write_text('[PERF] early waiting\n[INFO] [launch]: All log files can be found below /tmp/log/2026-10-07-23-00-0'+str(i)+'-123456-machine-42\n')
        runtime=output/(rid+'_runtime.json');put(runtime,{'executable':str(binary),'executable_sha256':c.sha(binary),'launch_sha256':c.sha(cloud)})
        csv=output/(rid+'.csv');csv.write_text('placeholder')
        perf=output/(rid+'_performance.md');perf.write_text('placeholder')
        env=dict(FOV_GVF_RUN_ID=rid,FOV_GVF_INSTALL=str(install),FOV_GVF_RUNTIME_MANIFEST=str(runtime),FOV_GVF_TRIAL_MANUAL_CONTINUOUS=str(i),FOV_GVF_CONTROLLER_EXECUTABLE=str(binary),ISAAC_TWIST_SAMPLE_SOURCE='callback')
        meta=dict(case='jitter',variant=side,repeat=1,controller_executable=str(binary),controller_sha256=c.sha(binary),launch_sha256=c.sha(cloud),experiment_environment=env,log=str(log),performance=str(perf),csv=str(csv),runtime_binary_verified=True,audit_safe=True,returncode=0)
        put(output/(rid+'_process.json'),meta);rows.append(meta)
        results.append(dict(name=rid,case='jitter',returncode=0))
    manifest=dict(case='jitter',complete=True,binary_sha256={'before':c.sha(binary),'after':c.sha(binary)},file_sha256={str(cloud):c.sha(cloud),str(benchmark):c.sha(benchmark)},runs=rows)
    mf=output/'manifest.json';put(mf,manifest);put(output/'runset_analysis.json',dict(runs=results))
    return dict(project=project,freeze=fz,manifest=mf,output=output,rows=rows,install=target)


def collect(f,**kw):
    return c.collect([f['manifest']],{'before':f['freeze'],'after':f['freeze']},project=f['project'],**kw)


def test_collect_variant_options_differ_conditions_equal(fixture):
    s,o=collect(fixture)
    b=s['runs']['jitter_before_1']['metadata'];a=s['runs']['jitter_after_1']['metadata']
    assert b['freeze_verified'] and a['freeze_verified']
    assert b['variant_sha256']!=a['variant_sha256']
    assert b['conditions_sha256']==a['conditions_sha256']
    assert (b['sequence'],a['sequence'])==(1,2)
    assert sum(map(len,o.values()))==2
    assert s['collection_audit']['contract_issues']['before'] # Missing inventory isn't silently passed.


def test_actual_installed_launch_drift_invalidates(fixture):
    (fixture['install']/'isaac_cloud_navigation.launch.py').write_text('stale installed launch')
    s,_=collect(fixture)
    assert all(not r['metadata']['freeze_verified'] for r in s['runs'].values())
    assert any('installed launch' in e for r in s['collection_audit']['run_audits'].values() for e in r['freeze']['errors'])


def test_manifest_process_mismatch_preserved(fixture):
    process=fixture['output']/'jitter_before_1_process.json';x=c.load(process);x['returncode']=-11;put(process,x)
    s,_=collect(fixture)
    assert any('process/manifest mismatch' in e for e in s['collection_audit']['issues'])
    assert not s['runs']['jitter_before_1']['metadata']['freeze_verified']


def test_incomplete_requires_explicit_flag(fixture):
    x=c.load(fixture['manifest']);x['complete']=False;put(fixture['manifest'],x)
    with pytest.raises(ValueError,match='incomplete'):collect(fixture)
    s,_=collect(fixture,allow_incomplete=True)
    assert all(not r['metadata']['freeze_verified'] for r in s['runs'].values())


def test_missing_analysis_run_retained(fixture):
    file=fixture['output']/'runset_analysis.json';put(file,dict(runs=[dict(name='jitter_before_1',case='jitter',returncode=0)]))
    s,o=collect(fixture)
    assert len(o['after'])==1
    assert any('lacks individual analysis' in e for e in s['collection_audit']['issues'])


def test_log_after_lock_timestamp_used(fixture):
    path=fixture['output']/'jitter_before_1.log'
    assert c.launch_time(path)=='2026-10-07T23:00:00.123456'
    path.write_text('[PERF] Started: 2026-10-07 22:00:00 CST\n[LOCK] waiting')
    with pytest.raises(ValueError,match='exactly one'):c.launch_time(path)


def test_archive_mismatch_never_verified(fixture):
    file=fixture['freeze']/'source_sha256.json';put(file,{'a.cpp':'f'*64})
    s,_=collect(fixture)
    assert not s['runs']['jitter_before_1']['metadata']['freeze_verified']


def test_incidental_output_path_does_not_change_conditions():
    meta=dict(case='jitter',experiment_environment={'FOV_GVF_RUN_ID':'a','FOV_GVF_PERFORMANCE_LOG':'/a','FOV_GVF_TRIAL_MANUAL_CONTINUOUS':'0','ISAAC_TWIST_SAMPLE_SOURCE':'callback'})
    manifest={'file_sha256':{}}
    b=c.conditions_payload(meta,manifest,{})
    meta['experiment_environment'].update(FOV_GVF_RUN_ID='b',FOV_GVF_PERFORMANCE_LOG='/b',FOV_GVF_TRIAL_MANUAL_CONTINUOUS='1')
    assert c.conditions_payload(meta,manifest,{})==b
    meta['experiment_environment']['ISAAC_TWIST_SAMPLE_SOURCE']='omnigraph'
    assert c.conditions_payload(meta,manifest,{})!=b


def test_duplicated_actual_run_rejected(fixture):
    with pytest.raises(ValueError,match='duplicate'):
        c.collect([fixture['manifest'],fixture['manifest']],{'before':fixture['freeze'],'after':fixture['freeze']},project=fixture['project'])
