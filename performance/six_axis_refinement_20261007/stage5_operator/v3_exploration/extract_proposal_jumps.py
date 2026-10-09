#!/usr/bin/env python3
"""Recorded proposal jumps; raw q is simulator sample, not timestamp-matched DDS input."""
from pathlib import Path
import csv,json,math,statistics
out=Path(__file__).resolve().parent;root=out.parents[3];folder=root/'performance/six_axis_refinement_20261007/closed_loop/p5_refine_formal_operator_20261007'
norm=lambda x:math.sqrt(sum(v*v for v in x))
def angle(a,b):
 if norm(a)*norm(b)<1e-12:return 0.
 return math.degrees(math.atan2(norm([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]),sum(x*y for x,y in zip(a,b))))
def yp(v):return [math.degrees(math.atan2(v[1],v[0])),math.degrees(math.atan2(v[2],math.hypot(v[0],v[1])))]
events=[];groups=[]
for f in sorted(folder.glob('*.csv')):
 rows=list(csv.DictReader(f.open()));t=[float(r['t']) for r in rows];arr=lambda names:[[float(r[k] or '0') for k in names] for r in rows];target=arr(['target_x','target_y','target_z']);raw=arr(['qx','qy','qz']);filtered=arr(['input_x','input_y','input_z']);name=f.stem.split('jitter_')[1];group='before' if '_before_' in f.name else 'after';ix=[]
 for i in range(1,len(rows)):
  if 4<=t[i]<8 and angle(target[i-1],target[i])>2:
   prev=target[i-1];curr=target[i];rawstep=angle(raw[i-1],raw[i]);filteredstep=angle(filtered[i-1],filtered[i]);recapture=angle(curr,filtered[i])<1e-5
   neighbourhood=[k for k in range(1,len(rows)) if abs(t[k]-t[i])<=.1]
   event=dict(run=name,group=group,t=t[i],diagnostic_stamp=float(rows[i]['diagnostic_stamp']),previous_diagnostic_stamp=float(rows[i-1]['diagnostic_stamp']),proposal_jump_deg=angle(prev,curr),raw_sample_jump_deg=rawstep,filtered_sample_jump_deg=filteredstep,raw_max_step_within_100ms_deg=max(angle(raw[k-1],raw[k]) for k in neighbourhood),before_yaw_pitch_deg=yp(prev),after_yaw_pitch_deg=yp(curr),proposal_before=prev,proposal_after=curr,raw_at_row=raw[i],raw_previous_row=raw[i-1],filtered_at_row=filtered[i],new_proposal_error_to_filtered_deg=angle(curr,filtered[i]),recapture_exact_intent=recapture,status=rows[i]['status']);events.append(event);ix.append(event)
 groups.append(dict(run=name,group=group,count=len(ix),max_jump_deg=max(r['proposal_jump_deg'] for r in ix),sum_jump_deg=sum(r['proposal_jump_deg'] for r in ix),recaptures=sum(r['recapture_exact_intent'] for r in ix)))
(out/'proposal_jumps_4to8s.json').write_text(json.dumps(dict(events=events,groups=groups,timestamp_caveat='raw_sample is q at simulation CSV row; diagnostics are independently received and have separate stamp. raw_max_step_within_100ms covers nearby raw input changes; no exact input/output DDS causality is claimed.'),indent=2)+'\n')
keys=['run','t','diagnostic_stamp','proposal_jump_deg','raw_sample_jump_deg','filtered_sample_jump_deg','raw_max_step_within_100ms_deg','recapture_exact_intent','new_proposal_error_to_filtered_deg']
with (out/'proposal_jumps_4to8s.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,keys);w.writeheader();w.writerows({k:r[k] for k in keys} for r in events)
for g in groups:print(g)
for r in events:print(r['run'],f"t={r['t']:.3f} jump={r['proposal_jump_deg']:.3f}deg raw={r['raw_sample_jump_deg']:.3f}deg filtered={r['filtered_sample_jump_deg']:.4f}deg",r['before_yaw_pitch_deg'],'->',r['after_yaw_pitch_deg'],'recapture',r['recapture_exact_intent'])
