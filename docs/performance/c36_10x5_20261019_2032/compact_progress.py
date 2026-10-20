from pathlib import Path
import json,datetime
T=Path('/home/ubuntu5090/dhm/EGO2P6/performance/c36_10x5_20261019_2032');R=T.parents[1];s=json.loads((T/'state.json').read_text());rows=json.loads((T/'records.json').read_text()) if (T/'records.json').exists() else [];c=s.get('current',{});p=R/'performance/p6_speed_20261019/rtx5090_c36'/c.get('label','none')/'original_v10_a16_r01.log';live=[]
if p.exists():
 with p.open('rb') as f:f.seek(max(0,p.stat().st_size-150000));text=f.read().decode(errors='replace')
 live=[x for x in text.splitlines() if '[NAV]' in x][-1:]
try:proc=Path('/proc/'+(T/'queue.pid').read_text().strip()+'/stat').read_text().split(') ')[1].split()[0]
except OSError:proc='absent'
out=dict(now=datetime.datetime.now().astimezone().isoformat(),status=s['status'],queue_process=proc,flights=len([x for x in rows if x['has_trajectory']]),safe_arrivals=sum(x['safe_arrival'] for x in rows),attempts=len(rows),current={k:c.get(k) for k in ['route','repeat','kind']},live_nav=live,last=[{k:x.get(k) for k in ['route','repeat','status','sim_s','avoid_mean_ms','avoid_max_ms','boundary_review_candidate']} for x in rows[-1:]],failed=[{k:x.get(k) for k in ['route','repeat','status','boundary_review_candidate']} for x in rows if not x['safe_arrival']]);print(json.dumps(out,ensure_ascii=False))
