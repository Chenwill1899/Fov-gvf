"""Compare identical physical/control records and audited FIFO decision costs across builds."""
import importlib.util
import json
import math
from pathlib import Path
import re
import statistics

out=Path(__file__).resolve().parent
root=out.parents[2]
exp=out.parent/'auto_turn_20261119'
spec=importlib.util.spec_from_file_location('flight_audit',exp/'review.py')
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
def load(p): return json.loads(p.read_text())
def binary(pointer): return load(root/load(pointer)['manifest'])['binary']['sha256']
versions={'before':binary(out/'before/latest_build.json'),'clearance_only':binary(out/'clearance_only/latest_build.json'),
          'image':load(exp/'run_20261119_224624_075095/build_manifest.json')['binary']['sha256']}
def metrics(values):
    values=sorted(values)
    return dict(mean_us=statistics.mean(values),p99_us=values[math.ceil(.99*len(values))-1],max_us=values[-1])
def equal_controls(actual,original):
    timings={k for k in original[0] if k.endswith('_ns')}
    exclude=timings|{'lateral_demand_m'}
    keys=sorted(set(original[0])-exclude)
    assert len(actual)==len(original)
    for a,b in zip(actual,original):
        if any(a[k]!=b[k] for k in keys):
            raise ValueError(f"control difference at {a['step']}")
    return max(abs(float(a['lateral_demand_m'])-float(b['lateral_demand_m'])) for a,b in zip(actual,original))
def image_usage(rows):
    if 'image_lateral_queries' not in rows[0]:return None
    return dict(total=2*len(rows),in_image=sum(int(r['image_lateral_queries']) for r in rows),
                attached_total=2*sum(r['mode'] in ('7','8') for r in rows),
                attached_in_image=sum(int(r['image_lateral_queries']) for r in rows if r['mode'] in ('7','8')))
data={'builds':versions,'runs':[],'live_parity':[]}
refs={}
for run in sorted(exp.glob('run_*')):
    m=run/'build_manifest.json'
    if not m.is_file():continue
    elf=load(m)['binary']['sha256']
    tag=next((k for k,v in versions.items() if v==elf),None)
    if tag is None:continue
    e=load(run/'execution.json')
    if not e.get('audit_passed'):continue
    r=audit.summarize(run);scene=r['scenario']
    assert r['arrived'] and r['final_speed']==0
    assert r['hold']==100 and r['cruise_n']==r['at12'] and r['blocked']==r['overlap']==r['accel_violations']==r['speed_violations']==0
    if scene=='hangar':assert len(r['gate_crossings'])==4
    rows=audit.read(run/'simulation/controller.csv')
    if scene not in refs:
        assert tag=='before'
        refs[scene]=(r,rows)
    ref,old=refs[scene]
    assert ref['physics_command_sha256']==r['physics_command_sha256']
    r['max_logged_lateral_difference_m']=equal_controls(rows,old)
    active=[x for x in rows if x['mode'] in ('7','8')]
    r['image_lateral']=image_usage(rows)
    r['curve_and_stop']=metrics([(int(x['curve7_ns'])+int(x['stop7_ns']))/1000 for x in active])
    r['variant']=tag
    data['runs'].append(r)
for run in sorted((out/'parity').glob('run_*')):
    e=load(run/'execution.json')
    if not e.get('audit_passed'):continue
    assert e['scope']['scheduler']=='correctness-only SCHED_OTHER' and e['exit_code']==0
    scene=e['scope']['scenario'];sim=run/'simulation'
    assert load(sim/'result.json')['status']=='ARRIVED' and load(sim/'audit.json')['swept_overlap_segments']==0
    line=next(x for x in (sim/'controller.log').read_text().splitlines() if x.startswith('BALL_PARITY '))
    counters={k:float(v) if k=='max_miss_error' else int(v) for k,v in re.findall(r'(\w+)=([\d.eE+-]+)',line)}
    rows=audit.read(sim/'controller.csv')
    equal_controls(rows,refs[scene][1])
    assert counters['frames']==len(rows) and counters['surface_hit']+counters['surface_free']==sum(int(r['refine_attempts']) for r in rows)
    assert counters['off_image']+counters['image_misses']==2*len(rows)
    data['live_parity'].append(dict(run=str(run.relative_to(root)),scene=scene,**counters))
for kind in ('r05','hangar'):
    file=out/('fixtures_'+kind+'.log')
    if file.exists():
        line=next(x for x in file.read_text().splitlines() if x.startswith('BALL_PARITY '))
        data.setdefault('fixtures',{})[kind]={k:float(v) if k=='max_miss_error' else int(v) for k,v in re.findall(r'(\w+)=([\d.eE+-]+)',line)}
if (out/'paired_query_timings.csv').exists():
    samples=audit.read(out/'paired_query_timings.csv')
    assert samples and len(samples)==20*len({(r['sequence'],r['query']) for r in samples})
    data['paired_fifo_us']={col:metrics([int(r[col])/1000 for r in samples]) for col in
                            ('old_raw_ns','image_raw_ns','old_surface_ns','image_surface_ns')}
    data['paired_fifo_counts']={'frames':len({r['sequence'] for r in samples}),'queries':len(samples)//20,'pairs':len(samples)}
(out/'comparison.json').write_text(json.dumps(data,indent=2)+'\n')
for r in data['runs']:
    print(r['variant'],r['scenario'],r['run'].split('/')[-1],r['at12'],r['all_decisions'],r['curve_and_stop'],r['image_lateral'])
for r in data['live_parity']:print(r)
