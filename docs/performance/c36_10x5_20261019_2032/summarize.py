#!/usr/bin/env python3
from pathlib import Path
import json,csv,statistics,datetime,re,hashlib
R=Path('/home/ubuntu5090/dhm/EGO2P6');T=R/'performance/c36_10x5_20261019_2032'
rows=json.loads((T/'records.json').read_text());ex=json.loads((T/'exclusions.json').read_text()) if (T/'exclusions.json').exists() else {}
for r in rows:
 r['exclusion']=ex.get(r['label'],{}).get('reason','')
 p=R/r['result'];perf=p.parent/'PERFORMANCE.md';counts=re.findall(r'Timer callbacks / command frames / guidance frames: (\d+) / (\d+) / (\d+)',perf.read_text()) if perf.exists() else []
 r['samples_complete']=bool(counts and int(counts[-1][2])==r['guidance_samples'])
 r['process_wall_s']=r['wall_s'];r['wall_s']=r['audit'].get('wall_s');r['sim_s']=r['audit'].get('sim_s')
fields=['route','formal_repeat','repeat','kind','attempt','label','status','has_trajectory','safe_arrival','accepted','exclusion','sim_s','wall_s','process_wall_s','path_m','actual_route_average_mps','actual_peak_mps','actual_final_distance_m','avoid_mean_ms','avoid_p95_ms','avoid_max_ms','guidance_samples','deadline_samples','samples_complete','collision_blocks','swept_overlap_segments','minimum_swept_clearance_m','limits_violations','frozen_unchanged','sensor_hash','result']
with (T/'all_attempts.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
formal=[]
for i in range(1,11):
 rid=f'r{i:02d}';eligible=[x for x in rows if x['route']==rid and x['has_trajectory'] and not x['exclusion']];assert len(eligible)<=5,(rid,'more than five included flights; do not select only five');
 for j,item in enumerate(eligible,1):item['formal_repeat']=j
 formal.extend(eligible)
with (T/'formal_results.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(formal)
with (T/'failures_and_exclusions.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows([x for x in rows if not x['safe_arrival'] or x['exclusion']])
groups={}
for i in range(1,11):
 rid=f'r{i:02d}';allr=[x for x in rows if x['route']==rid and x['has_trajectory']];rr=[x for x in formal if x['route']==rid];vals=[x['avoid_mean_ms'] for x in rr if x['avoid_mean_ms'] is not None]
 groups[rid]=dict(formal=len(rr),safe_arrivals=sum(x['safe_arrival'] for x in rr),failed=sum(not x['safe_arrival'] for x in rr),raw_flights=len(allr),excluded=sum(bool(x['exclusion']) for x in allr),mean_sim_s=statistics.mean([x['sim_s'] for x in rr if x['sim_s'] is not None]) if rr else None,mean_avoid_ms=statistics.mean(vals) if vals else None,max_avoid_ms=max((x['avoid_max_ms'] for x in rr if x['avoid_max_ms'] is not None),default=None),deadline_samples=sum(x['deadline_samples'] for x in rr))
summary=dict(updated=datetime.datetime.now().astimezone().isoformat(),algorithm='c36',server='RTX5090/i9-14900K/Jazzy',headless=True,rviz=False,video=False,source_path=str(R),requested=50,formal_flights=len(formal),formal_safe_arrivals=sum(x['safe_arrival'] for x in formal),formal_failed=sum(not x['safe_arrival'] for x in formal),raw_attempts=len(rows),raw_flights=sum(x['has_trajectory'] for x in rows),raw_safe_arrivals=sum(x['safe_arrival'] for x in rows),excluded=sum(bool(x['exclusion']) for x in rows),startup_attempts=sum(not x['has_trajectory'] for x in rows),all_frozen_unchanged=all(x['frozen_unchanged'] for x in rows if x['has_trajectory']),all_samples_complete=all(x['samples_complete'] for x in rows if x['has_trajectory']),groups=groups,all_formal_flight_mean_avoid_ms=statistics.mean([x['avoid_mean_ms'] for x in formal if x['avoid_mean_ms'] is not None]) if formal else None,worst_avoid_ms=max((x['avoid_max_ms'] for x in formal if x['avoid_max_ms'] is not None),default=None),swept_overlap_segments=sum(x.get('swept_overlap_segments') or 0 for x in rows),collision_blocks=sum(x.get('collision_blocks') or 0 for x in rows),deadline_samples=sum(x['deadline_samples'] for x in formal),complete=all(x['formal']==5 for x in groups.values()))
raw=[x for x in rows if x['has_trajectory']]
summary['raw_mean_avoid_ms']=statistics.mean([x['avoid_mean_ms'] for x in raw if x['avoid_mean_ms'] is not None]) if raw else None
summary['raw_worst_avoid_ms']=max((x['avoid_max_ms'] for x in raw if x['avoid_max_ms'] is not None),default=None)
summary['raw_deadline_samples']=sum(x['deadline_samples'] for x in raw)
(T/'completion_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
fmt=lambda x:f'{x:.3f}' if isinstance(x,(float,int)) else '—'
lines=['# 5090 c36：10条路线各5次','',f'更新：{summary["updated"]}；正式完成{len(formal)}/50，安全到达{summary["formal_safe_arrivals"]}，未安全到达{summary["formal_failed"]}；原始飞行{summary["raw_flights"]}，排除{summary["excluded"]}，无轨迹启动{summary["startup_attempts"]}。','','| 路线 | 正式安全到达 | 排除 | 平均仿真秒 | 全程避障逐次均值再平均 ms | 最坏样本 ms | 计算超时帧 |','|---|---:|---:|---:|---:|---:|---:|']
for k,g in groups.items():lines.append(f'|{k.upper()}|{g["safe_arrivals"]}/{g["formal"]}|{g["excluded"]}|{fmt(g["mean_sim_s"])}|{fmt(g["mean_avoid_ms"])}|{fmt(g["max_avoid_ms"])}|{g["deadline_samples"]}|')
lines+=['','[逐次正式数据](formal_results.csv) · [全部尝试](all_attempts.csv) · [摘要](completion_summary.json) · [原始记录](records.json)','','条件：同一已部署c36、原R01–R10约300m/Z10，10m/s请求上限、垂向1、a16/tau.10、模型延迟.20s、depth20/H.5/cache.3/.75/history16/1s，默认不绑核。每次全新仿真进程，headless/无RViz/无视频；四水平相机，算法与数值在整批内保持。','', 'CSV的avoid统计由整次流式samples.csv计算，含失败与计算超时；汇总均值每次运行等权。wall_s为仿真记录墙钟时间，process_wall_s含启动、退出和审计。普通未到达照常计失败；仅经复核的用户顶/底斜角围困另账排除并同路补跑，原结果不删除。统计不能当作任意路线/严格每帧1ms验收。']
(T/'RESULTS.md').write_text('\n'.join(lines)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='groups'},ensure_ascii=False))
