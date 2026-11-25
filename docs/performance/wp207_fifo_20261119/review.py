from pathlib import Path
import csv,zipfile,io,json,math,statistics as st
from collections import Counter
d=Path(__file__).resolve().parent
archive=d.parent/'r05_wp207_cruise_20261118/R05_wp207_evidence.zip'
z=zipfile.ZipFile(archive)
def rows(f):return list(csv.DictReader(f))
def saved(name):return rows(io.StringIO(z.read('live_retry/'+name).decode()))
def disk(path): 
    with path.open() as f:return rows(f)
old=saved('controller.csv');original=saved('trajectory.csv')
def ns(r,col):return int(r[col])/1e6
def ms(r,k):
    if k=='three':return sum(ns(r,c) for c in ('planning_ns','certification_ns','control_ns'))
    if k=='pool_extra':return ns(r,'pool_ns')-ns(r,'cam_wall_ns')
    if k=='post_pool':return ns(r,'pre_ns')-ns(r,'pool_ns')
    if k=='worker_offcpu':return ns(r,'cam_wall_ns')-ns(r,'cam_cpu_ns')
    return ns(r,k)
def stats(r,k):
    a=sorted(ms(x,k) for x in r)
    return {'n':len(a),'avg':round(st.mean(a),6),'p99':round(a[math.ceil(.99*len(a))-1],6),'max':round(a[-1],6),'over5':sum(t>5 for t in a)}
def prefix(r,b):return [x for x in r if x['mode']=='1' and (b is None or x['field_attempted']==b)]
groups={'rolling':None,'no_field':'0','field':'1'}
out={'archived_other':{k:stats(prefix(old,v),'three') for k,v in groups.items()}}
base=d.parent/'r05_model_sync_20261117/run_20261118_205118_030389/simulation'
formal=disk(base/'controller.csv')
out['archived_fifo']={k:stats(prefix(formal[:1805],v),'three') for k,v in groups.items()}
cols='step_compute_ns pre_ns pool_ns cam_wall_ns cam_cpu_ns cam_repair_ns cam_compress_ns cam_project_ns pool_extra post_pool worker_offcpu field_ns spatial_ns occupancy_ns planning_ns certification_ns control_ns'.split()
for p in sorted(d.glob('run_*')):
    e=json.loads((p/'execution.json').read_text())
    if not e.get('audit_passed'):continue
    sim=p/'simulation';a=json.loads((sim/'audit.json').read_text());t=disk(sim/'trajectory.csv');r=disk(sim/'controller.csv')
    assert len(t)==len(original) and len(r)==len(old)
    delta={k:max(abs(float(x[k])-float(y[k])) for x,y in zip(t,original)) for k in ('x','y','z','vx','vy','vz','ux','uy','uz','yaw_rad','waypoint','mode')}
    assert all(x['mode']==y['mode'] and x['field_attempted']==y['field_attempted'] for x,y in zip(r,old))
    cruise=t[221:5197];errors=[abs(math.hypot(*(float(x[k]) for k in ('vx','vy','vz')))-12) for x in cruise]
    assert len(cruise)==4976 and max(errors)<=.001 and a['at_goal'] and a['swept_overlap_segments']==0
    threads=e['thread_observations'];fifo=e['scope']['scheduler']=='FIFO5'
    assert e['global_rt_settings_unchanged'] and len(threads)==5 and sorted(x['cpus'] for x in threads)==[[i] for i in range(5)]
    assert all(x['policy']==int(fifo) and x['priority']==(5 if fifo else 0) and not x['cap_eff'] for x in threads)
    s={'run':p.name,'deltas':delta,'steps_in_cruise':len(errors),'max_speed_error':max(errors),'audit':a,'sim_s':e['result']['simulation_s'],'policy':e['scope']['scheduler'],'simulator_readbacks':len(e['simulator_readbacks'])}
    def summarize(rr,names,metric):
        q={k:stats(rr,k) for k in names}
        def one(x):
            i=int(x['step']);return {'step':i,'t':t[i]['t'],'wp':t[i]['waypoint'],'field':x['field_attempted'],**{k:round(ms(x,k),6) for k in names}}
        q['worst']=one(max(rr,key=lambda x:ms(x,metric)))
        return q
    for k,v in groups.items():s[k]=summarize(prefix(r,v),['three']+cols,'three')
    rr=[x for x in r if x['mode']=='7'];assert len(rr)==3391
    s['cruise7']=summarize(rr,['step_compute_ns','prepare7_ns','steer7_ns','curve7_ns','stop7_ns']+cols[1:11],'step_compute_ns')
    s['step325']={k:ms(r[324],k) for k in ['three']+cols}
    s['all']=stats(r,'step_compute_ns')
    out['fifo' if fifo else 'other']=s
    if fifo:
        screenshot=d.parent/'r05_model_sync_20261117/run_20261118_195738_748686/simulation'
        previous=disk(screenshot/'controller.csv')
        trajectory=disk(screenshot/'trajectory.csv')
        aligned=[]
        keys=('x','y','z','vx','vy','vz','ux','uy','uz','yaw_rad','waypoint')
        for i,(left,right) in enumerate(zip(previous,r)):
            if left['mode']!=right['mode'] or left['mode']!='1' or left['field_attempted']!=right['field_attempted']:continue
            if all(trajectory[j][k]==t[j][k] for j in (i,i+1) for k in keys):
                aligned.append((left,right))
        out['screenshot_comparison']={
            'all_old':{k:stats(prefix(previous,v),'three') for k,v in groups.items()},
            'common_steps':[int(x['step']) for x,_ in aligned],
            'common':{version:{k:stats(prefix([pair[j] for pair in aligned],v),'three') for k,v in groups.items()} for j,version in enumerate(('old','new'))},
            'step325_archived_other':ms(old[324],'three')
        }
(d/'results.json').write_text(json.dumps(out,indent=2)+'\n')
for k,s in out.items():
 if k=='screenshot_comparison':continue
 print(k,[(g,s[g].get('three',s[g])) for g in groups])
 if 'cruise7' in s:print('M7',s['cruise7']['step_compute_ns'])
