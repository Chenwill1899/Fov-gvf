#!/usr/bin/env python3
"""Offline decomposition of recorded Isaac velocity, applied target and proposal."""
from pathlib import Path
import csv,math,json,statistics
root=Path(__file__).resolve().parents[4];folder=root/'performance/six_axis_refinement_20261007/closed_loop/p5_refine_formal_operator_20261007'
out=Path(__file__).resolve().parent
norm=lambda a:math.sqrt(sum(x*x for x in a))
angle=lambda a,b:math.atan2(norm([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]),sum(x*y for x,y in zip(a,b))) if norm(a)*norm(b)>1e-12 else 0.
phases=[('clear_motion',2,4),('first_avoidance',4,8),('post_avoidance',8,14.8),('first_release_onset',14.8,15.2),('first_release_end',15.2,16),('slow_turn_steady',18,22.8),('reverse_braking',22.8,24),('release2',25.8,26.8),('release3',31.8,33)]
results=[];spikes=[]
for f in sorted(folder.glob('*.csv')):
 rows=list(csv.DictReader(f.open()));group='before' if '_before_' in f.name else 'after';t=[float(r['t']) for r in rows]
 arr=lambda ks:[[float(r[k] or '0') for k in ks] for r in rows]
 v=arr(['vx','vy','vz']);ap=arr(['applied_target_x','applied_target_y','applied_target_z']);proposal=arr(['target_x','target_y','target_z']);filtered=arr(['input_x','input_y','input_z'])
 derivative=lambda v:[[ (v[i+1][j]-v[i][j])/(t[i+1]-t[i]) for j in range(3)] for i in range(len(v)-1)]
 a=derivative(v);jerk=[[ (a[i+1][j]-a[i][j])/((t[i+2]-t[i])/2) for j in range(3)] for i in range(len(a)-1)];j2=[norm(j)**2 for j in jerk]
 jumps=[i for i in range(1,len(rows)) if angle(proposal[i-1],proposal[i])>math.radians(2)]
 for phase,lo,hi in phases:
  ix=[i for i in range(len(j2)) if lo<=t[i+1]<hi];ri=[i for i in range(1,len(rows)) if lo<=t[i]<hi]
  def tv(series):return sum(norm([series[i][j]-series[i-1][j] for j in range(3)]) for i in ri)
  candidate_jumps=[i for i in jumps if lo<=t[i]<hi]
  near_energy=sum(j2[i] for i in ix if any(abs(t[i+1]-t[j])<.15 for j in candidate_jumps))
  results.append(dict(file=f.name,group=group,phase=phase,jerk_rms=math.sqrt(sum(j2[i] for i in ix)/len(ix)),jerk_energy=sum(j2[i] for i in ix),filtered_intent_tv=tv(filtered),proposal_tv=tv(proposal),applied_target_tv=tv(ap),proposal_jumps_over_2deg=len(candidate_jumps),jerk_energy_near_proposal_jump=near_energy))
 for i in sorted([i for i in range(len(j2)) if 2<t[i+1]<14.8],key=lambda i:j2[i],reverse=True)[:10]:
  spikes.append(dict(file=f.name,t=t[i+1],jerk=norm(jerk[i]),filtered=filtered[i+1],proposal=proposal[i+1],applied=ap[i+1],previous_applied=ap[i],next_applied=ap[i+2],nearest_candidate_jump_s=min((abs(t[i+1]-t[j]) for j in jumps),default=None)))
summary=[]
for phase,_,_ in phases:
 summary.append(dict(phase=phase,**{g:{k:statistics.mean(r[k] for r in results if r['group']==g and r['phase']==phase) for k in ['jerk_rms','jerk_energy','filtered_intent_tv','proposal_tv','applied_target_tv','proposal_jumps_over_2deg','jerk_energy_near_proposal_jump']} for g in ['before','after']}))
(out/'refined_phase_metrics.json').write_text(json.dumps(dict(results=results,summary=summary,steady_spikes=spikes),indent=2)+'\n')
for r in summary:print(json.dumps(r))
