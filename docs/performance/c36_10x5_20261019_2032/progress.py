from pathlib import Path
import json,os,csv
T=Path('/home/ubuntu5090/dhm/EGO2P6/performance/c36_10x5_20261019_2032');R=T.parents[1]
s=json.loads((T/'state.json').read_text());rows=json.loads((T/'records.json').read_text()) if (T/'records.json').exists() else []
try:pid=int((T/'queue.pid').read_text());s['queue_process_state']=Path(f'/proc/{pid}/stat').read_text().split(') ')[1].split()[0]
except Exception:s['queue_process_state']='absent'
s['routes']={f'r{i:02d}':str(sum(x['safe_arrival'] for x in rows if x['route']==f'r{i:02d}'))+'/'+str(sum(x['has_trajectory'] for x in rows if x['route']==f'r{i:02d}')) for i in range(1,11)}
s['last_results']=[{k:x.get(k) for k in ['route','repeat','status','sim_s','avoid_mean_ms','avoid_p95_ms','avoid_max_ms','boundary_review_candidate']} for x in rows[-3:]]
if s.get('current'):
 p=R/'performance/p6_speed_20261019/rtx5090_c36'/s['current']['label']/'original_v10_a16_r01.log'
 if p.exists():
  with p.open('rb') as f:f.seek(max(0,p.stat().st_size-2500));s['active_log_tail']=f.read().decode(errors='replace').splitlines()[-3:]
print(json.dumps(s,ensure_ascii=False,indent=2))
