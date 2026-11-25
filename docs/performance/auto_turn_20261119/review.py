"""Evaluate every completed flight. Cruise windows use fixed spatial boundaries in both maps."""
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

here=Path(__file__).resolve().parent
root=here.parents[2]
cfg=json.loads((root/'src/auto_cruise/scenarios.json').read_text())
def load(path):
    return json.loads(path.read_text())
def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))
def xyz(row, keys):
    return tuple(float(row[k]) for k in keys)
def stats(rows):
    ms=sorted(int(r['step_compute_ns'])/1e6 for r in rows)
    if ms:
        return dict(n=len(ms),mean=statistics.mean(ms),p99=ms[math.ceil(.99*len(ms))-1],maximum=ms[-1],over5=sum(t>5 for t in ms))

def gates(track):
    manifest=load(root/'scenes/reference_hangar/manifest.json')
    passed=[]
    start=1
    for gate,sign in zip(manifest['gates'],(1,1,1,-1)):
        c=gate['center']; axis=gate['normal'].index(1); k='xyz'[axis]
        for i in range(start,len(track)):
            a,b=track[i-1],track[i]
            u,v=(float(row[k])-c[axis] for row in (a,b))
            if not sign*u<0<=sign*v:
                continue
            pos=[float(a[q])+(float(b[q])-float(a[q]))*(-u)/(v-u) for q in 'xyz']
            horizontal=abs(pos[1-axis]-c[1-axis]);vertical=abs(pos[2]-c[2])
            if horizontal+.10<=gate['inner_width']/2 and vertical+.10<=gate['inner_height']/2:
                passed.append(dict(gate=gate['name'],step=i,position=pos))
                start=i+1
                break
    return passed

def summarize(path,fixed=False):
    sim=path/'simulation'
    e=load(path/'execution.json')
    result=load(sim/'result.json')
    audit=load(sim/'audit.json')
    scene=e['scope'].get('scenario','r05')
    c=read(sim/'controller.csv'); t=read(sim/'trajectory.csv')
    clocks=read(sim/'sensor_times.csv'); threads=e['thread_observations']
    assert e['audit_passed'] and e['exit_code']==0 and e['global_rt_settings_unchanged']
    assert sorted(th['cpus'] for th in threads)==[[n] for n in range(5)]
    assert all(th['policy']==1 and th['priority']==5 and th['uid']!=0 and th['cap_eff']==0 for th in threads)
    assert e['supervisor_policy']==0 and any(s['controller_seen'] for s in e['simulator_readbacks'])
    assert all(set(s['policies'])<={0,3,5} for s in e['simulator_readbacks'])
    assert len(clocks)==len(c) and all(abs(float(row[k])-float(row['physics_s']))<1e-7 for row in clocks for k in ('front_s','left_s','back_s','right_s'))
    assert all(abs(float(b['physics_s'])-float(a['physics_s'])-.005)<1e-7 for a,b in zip(clocks,clocks[1:]))
    assert result['scene_sha256']==cfg[scene]['scene_sha256'] and result['occupancy_sha256']==cfg[scene]['occupancy_sha256']
    start_x=-39.84206745243012 if scene=='r05' else -10.
    start=next((int(r['step']) for r in t if float(r['x'])>=start_x-1e-8),len(t))
    end=(5197 if fixed else next((int(r['step']) for r in c if r['terminal_demand']=='1'),len(t)))
    cruise=t[start:end]
    speed=[math.hypot(*xyz(r,('vx','vy','vz'))) for r in cruise]
    active=[r for r in c if r['mode']=='7']
    if not fixed:
        assert all((r['turn_check']=='1')==(r['mode'] in ('7','8')) for r in c)
    first=int(active[0]['step']) if active else None
    signature=hashlib.sha256()
    for row in t:
        signature.update((','.join(row[k] for k in 'step t x y z vx vy vz ux uy uz mode waypoint guide_cap yaw_rad'.split())+'\n').encode())
    switches=[int(r['step']) for i,r in enumerate(c) if r['mode']=='7' and (i==0 or c[i-1]['mode']!='7')]
    releases=[int(r['step']) for i,r in enumerate(c) if i and c[i-1]['mode']=='7' and r['mode']!='7']
    model=load(root/'src/new_controller/config/base_step1.json')
    nav=load(root/'src/new_controller/config/navigation_step2.json')
    residual=0.
    for before,after in zip(t,t[1:]):
        v=xyz(before,('vx','vy','vz'))
        w=xyz(after,('vx','vy','vz'))
        command=xyz(after,('ux','uy','uz'))
        diff=tuple(u-x for u,x in zip(command,v))
        scale=1. if math.hypot(*command)<1e-8 else min(1.,model['physics_dt_s']/model['plant_tau_s'])
        if math.hypot(*diff)>1e-12:
            scale=min(scale,model['plant_acceleration_mps2']*model['physics_dt_s']/math.hypot(*diff))
        residual=max(residual, math.dist(w,[x+scale*dx for x,dx in zip(v,diff)]))
        if after['mode'] in ('7','8'):
            assert math.hypot(*command)<=nav['compensation_input_limit_mps']+1e-8
            assert math.dist(command,xyz(before,('ux','uy','uz')))<=nav['compensation_slew_limit_mps2']*.005+1e-8
    assert residual<1e-8
    build=load(path/'build_manifest.json') if (path/'build_manifest.json').exists() else load(path.parent/'manifest.json')
    if (path/'build_manifest.json').exists():
        assert hashlib.sha256((path/'build_manifest.json').read_bytes()).hexdigest()==e['manifest_sha256']
    pair=None
    if scene=='r05' and not fixed:
        ref=here.parent/'wp207_fifo_20261119/run_20261119_213433_672565/simulation'
        rc,rt=read(ref/'controller.csv'),read(ref/'trajectory.csv')
        length=0
        for a,b,u,v in zip(c,rc,t[1:],rt[1:]):
            if any(a[k]!=b[k] for k in ('mode','field_attempted','angular_ready','spatial_ready','reference_commit')):break
            if any(u[k]!=v[k] for k in 'step x y z vx vy vz ux uy uz yaw_rad waypoint'.split()):break
            length+=1
        pair=dict(n=length,fixed=stats(rc[:length]),candidate=stats(c[:length]))
    return dict(run=str(path.relative_to(root)),scenario=scene,elf=build['binary']['sha256'],manifest=e['manifest_sha256'],physics_command_sha256=signature.hexdigest(),
                matched_prefix=pair,status=result['status'],arrived=audit['at_goal'],sim_s=result['simulation_s'],distance_m=audit['distance_m'],
                goal_error_m=result['remaining_m'],final_speed=result['final_speed_mps'],hold=result['terminal_hold_steps'],
                overlap=audit['swept_overlap_segments'],blocked=result['independent_collision_blocks'],speed_violations=audit['speed_violations'],
                accel_violations=audit['acceleration_violations'],min_clearance=audit['minimum_swept_clearance_lower_bound_m'],
                cruise_first=start,cruise_last=end-1,cruise_n=len(cruise),at12=sum(abs(v-12)<=.001 for v in speed),
                min_speed=min(speed,default=None),max_error=max((abs(v-12) for v in speed),default=None),
                cruise_modes=dict(Counter(r['mode'] for r in cruise)),first_turn=dict(step=first,t=float(t[first]['t']),
                wp=int(t[first]['waypoint']),pos=xyz(t[first-1],'xyz')) if first else None,
                refinement_tests=sum(int(r.get('refine_attempts',0)) for r in c),refinement_clear=sum(int(r.get('refine_clear',0)) for r in c),turn_entry_steps=switches,turn_exit_steps=releases,plant_max_residual=residual,checks=dict(Counter(r.get('turn_check') for r in c)) if not fixed else {},
                gate_crossings=gates(t) if scene=='hangar' else None,sensor_groups=len(clocks),
                all_decisions=stats(c),adaptive_decisions=stats(active),straight_decisions=stats([r for r in c if r['mode']=='8']),rolling_decisions=stats([r for r in c if r['mode']=='1']))

if __name__=='__main__':
    base=here.parent/'wp207_fifo_20261119'
    data={'fixed':[],'auto':[], 'incomplete':[], 'fault_runs':[]}
    for k,folder in [('fixed',base),('auto',here)]:
        for run in sorted(folder.glob('run_*')):
            e=load(run/'execution.json')
            if e['scope']['scheduler']!='FIFO5':
                continue
            result=e.get('result',{})
            if result.get('injected_loss_step') is not None or result.get('execution_fault') or result.get('reentries'):
                data['fault_runs'].append(dict(run=run.name,scenario=e['scope']['scenario'],
                    elf=load(run/'build_manifest.json')['binary']['sha256'],status=result.get('status'),fault=result.get('execution_fault'),
                    injected_step=result.get('injected_loss_step'),audit_passed=e.get('audit_passed',False)))
                continue
            if e.get('audit_passed'):
                data[k].append(summarize(run,k=='fixed'))
            else:
                data['incomplete'].append([run.name,e.get('error','running')])
    current=load(root/load(here/'latest_build.json')['manifest'])['binary']['sha256']
    data['current_elf']=current
    data['repeat_check']={}
    for name in cfg:
        series=[r for r in data['auto'] if r['elf']==current and r['scenario']==name]
        if not series:continue
        data['repeat_check'][name]=dict(runs=len(series),
            identical_motion_and_commands=len(series)>=2 and len({r['physics_command_sha256'] for r in series})==1,
            full_cruise_at_12=all(r['cruise_n']>0 and r['cruise_n']==r['at12'] for r in series),
            completed=not any(r['elf']==current and r['scenario']==name and r['injected_step'] is None
                              for r in data['fault_runs']) and all(r['arrived'] and r['hold']==100 and r['final_speed']==0 for r in series),
            passed_sweep=all(r['blocked']==r['overlap']==r['accel_violations']==r['speed_violations']==0 for r in series),
            passed_gates=all(len(r['gate_crossings'])==4 for r in series) if name=='hangar' else None)
    (here/'comparison.json').write_text(json.dumps(data,indent=2)+'\n')
    for row in data['auto']:
        print(row['scenario'],row['status'],row['at12'],row['cruise_n'],row['first_turn'],row['all_decisions'])
