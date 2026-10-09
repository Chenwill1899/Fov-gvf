#!/usr/bin/env python3
"""Serial Isaac comparisons through the authorized main shell entrypoint."""
import hashlib,json,os,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[2]
out=Path(__file__).resolve().parent
binary=Path('/tmp/fov_gvf_ego1p2_isaac_install/pc_gvf/lib/pc_gvf/depth_angular_controller')
for name,flag,fixture,duration in [
    ('recovery_off','0','isaac_envelope_recovery_trace.json',94),
    ('recovery_on','1','isaac_envelope_recovery_trace.json',94),
    ('manual_final','1','isaac_stall_repair_trace.json',72),
]:
    trace=root/'src/pc_gvf/test/fixtures/paper'/fixture
    env=dict(os.environ,ISAAC_MANUAL_INPUT_MODE='trace',ISAAC_HEADLESS='1',FOV_GVF_RVIZ='false',
        FOV_GVF_SCENE_MODE='cloud',ISAAC_INTENT_TRACE=str(trace),ISAAC_ACCEPTANCE_TRACE=str(out/(name+'.csv')),
        ISAAC_MANUAL_TIMEOUT=str(duration),FOV_GVF_RUN_ID='stall_fix_'+name,
        FOV_GVF_REPLAY_DIR=str(out/(name+'_replays')),FOV_GVF_PERFORMANCE_LOG=str(out/'PERFORMANCE.md'),
        FOV_GVF_LOCAL_HISTORY_REPAIR=flag,ROS_DOMAIN_ID='42',PYTHONNOUSERSITE='1')
    print('START',name,flush=True)
    with (out/(name+'.log')).open('x') as log:
        result=subprocess.run(['bash','scripts/run_isaac_fov_gvf_navigation.sh'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
    metadata={'returncode':result.returncode,'local_history_repair':flag,'duration_s':duration,
        'trace_sha256':hashlib.sha256(trace.read_bytes()).hexdigest(),
        'controller_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
    (out/(name+'_process.json')).write_text(json.dumps(metadata,indent=2)+'\n')
    if result.returncode:raise SystemExit(f'{name} failed: {result.returncode}')
    audit=subprocess.run(['/usr/bin/python3','tools/analyze_envelope_trace.py',str(out/(name+'.csv'))],cwd=root,env=env,text=True,capture_output=True,check=True)
    (out/(name+'_audit.json')).write_text(audit.stdout)
    data=json.loads(audit.stdout)
    print('END',name,json.dumps({k:data[k] for k in ('stopped_with_input_s','longest_stopped_with_input_s','distance_travelled_m','swept_sphere_overlap_segments','external_collision_blocks')}),flush=True)
