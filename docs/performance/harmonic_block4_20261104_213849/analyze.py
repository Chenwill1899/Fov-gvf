#!/usr/bin/python3
import pathlib,json,csv,collections
import numpy as np
r=pathlib.Path(__file__).resolve().parent
rows=[]
for round_id in [1,2,3]:
 for policy in ['legacy','small']:
  for edge in [3,4]:
   with (r/f'round_{round_id}'/f'{policy}_edge{edge}'/'timings.csv').open() as f:
    for d in csv.DictReader(f):d.update(round=round_id,policy=policy,edge=edge);rows.append(d)
metrics=['preprocess_ms','capture_ms','total_ms','input_field_total_ms','coarse_domain_ms','coarse_solve_ms','fine_domain_ms','fine_solve_ms','curve_ms']
def stats(a,k):
 v=np.array([float(d[k]) for d in a]);return dict(mean_ms=float(v.mean()),median_ms=float(np.median(v)),p99_ms=float(np.percentile(v,99)),max_ms=float(v.max()),above_2ms=int(sum(v>2)))
summary={}
for policy in ['legacy','small']:
 summary[policy]={}
 for edge in [3,4]:
  summary[policy][str(edge)]={}
  for case in sorted(set(d['case'] for d in rows)):
   summary[policy][str(edge)][case]={}
   for mode in ['cold','boundary_change','new_group_reuse']:
    a=[d for d in rows if d['policy']==policy and d['edge']==edge and d['case']==case and d['mode']==mode]
    z={k:stats(a,k) for k in metrics};z.update(count=len(a),status=dict(collections.Counter(d['status']+'/'+d['termination'] for d in a)),coarse_nodes=sorted(set(int(d['coarse_nodes']) for d in a)),fine_nodes=sorted(set(int(d['fine_nodes']) for d in a)),fine_attempts=sorted(set(int(d['fine_attempts']) for d in a)),reused=sum(int(d['reused']) for d in a),coarse_max_residual=max(float(d['coarse_residual']) for d in a),fine_max_residual=max(float(d['fine_residual']) for d in a))
    summary[policy][str(edge)][case][mode]=z
(r/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
fields=['policy','edge','case','mode','count','status','coarse_nodes','fine_nodes','reused']+[f'{k}_{s}' for k in metrics for s in ['mean_ms','p99_ms','max_ms']]
with (r/'summary.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
 for pol,byedge in summary.items():
  for edge,bycase in byedge.items():
   for case,bymode in bycase.items():
    for mode,z in bymode.items():
     o={k:z[k] for k in ['count','status','coarse_nodes','fine_nodes','reused']};o.update(policy=pol,edge=edge,case=case,mode=mode)
     for k in metrics:
      for s in ['mean_ms','p99_ms','max_ms']:o[k+'_'+s]=z[k][s]
     w.writerow(o)
# Models are not equivalent. Quantify state transitions without asserting nesting.
diffs={}
for case in ['real_first','real_last','supported_first','supported_last','angular_open','angular_gap','angular_narrow']:
 a=r/'round_3/small_edge3'/case;b=r/'round_3/small_edge4'/case
 x=np.fromfile(a/'sample_panorama_state.bin',dtype=np.uint8);y=np.fromfile(b/'sample_panorama_state.bin',dtype=np.uint8)
 z=dict(transitions={f'{i}->{j}':int(np.sum((x==i)&(y==j))) for i in range(3) for j in range(3)},changed_cells=int(np.sum(x!=y)),old_counts=dict(collections.Counter(map(int,x))),new_counts=dict(collections.Counter(map(int,y))),q0_edge3=json.load(open(a/'input.json'))['q0'],q0_edge4=json.load(open(b/'input.json'))['q0'])
 assert z['q0_edge3']==z['q0_edge4']
 diffs[case]=z
(r/'domain_changes.json').write_text(json.dumps(diffs,indent=2)+'\n')
counts={'formal_rows':len(rows),'states':dict(collections.Counter(d['status']+'/'+d['termination'] for d in rows)),'edge_counts':{str(e):dict(collections.Counter(d['status']+'/'+d['termination'] for d in rows if d['edge']==e)) for e in [3,4]},'old_first':34240,'new_first':19200,'first_reduction_pct':100*(1-19200/34240),'old_second':3888,'new_second':2160,'second_reduction_pct':100*(1-2160/3888)}
(r/'counts.json').write_text(json.dumps(counts,indent=2)+'\n');print(json.dumps(counts,indent=2))
for pol in ['legacy','small']:
 for edge in ['3','4']:
  print(pol,edge,'supported_first',{mode:{'field':round(summary[pol][edge]['supported_first'][mode]['total_ms']['mean_ms'],4),'full':round(summary[pol][edge]['supported_first'][mode]['input_field_total_ms']['mean_ms'],4),'status':summary[pol][edge]['supported_first'][mode]['status']} for mode in ['cold','boundary_change','new_group_reuse']})
# Static scientific figure: comparable scope, CPU0 serial input baseline.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'axes.unicode_minus':False,'svg.fonttype':'none'})
fig,ax=plt.subplots(1,2,figsize=(12,5.2));colors=['#3c78a8','#d78332']
for e,(edge,color) in enumerate(zip(['3','4'],colors)):
 z=summary['small'][edge]['supported_first']['cold'];bottom=0
 for k,label,alpha in [('preprocess_ms','补值/摘要/Ω',.5),('capture_ms','快照/二级汇总',.75),('total_ms','场与曲线',1)]:
  v=z[k]['mean_ms'];ax[0].bar(e,v,bottom=bottom,color=color,alpha=alpha,label=(label if e==0 else None));ax[0].text(e,bottom+v/2,f'{v:.3f}',ha='center',va='center',fontsize=10);bottom+=v
 ax[0].text(e,bottom+.05,f'{bottom:.3f} ms',ha='center')
 xs=np.arange(3)+(e-.5)*.32;ys=[summary['small'][edge]['supported_first'][m]['total_ms']['mean_ms'] for m in ['cold','boundary_change','new_group_reuse']];ax[1].bar(xs,ys,width=.32,color=color,label=f'{edge}×{edge} → 3×3')
 for x,v in zip(xs,ys):ax[1].text(x,v+.03,f'{v:.3f}',ha='center',fontsize=9)
ax[0].set_xticks([0,1],['3×3 → 3×3','4×4 → 3×3']);ax[0].set_title('真实存档首组：输入处理至曲线的平均耗时',fontproperties=font);ax[0].set_ylabel('ms');ax[0].legend(prop=font,loc='upper right');ax[0].set_ylim(0,5.7)
ax[1].set_xticks(range(3),['冷算','边界变化','同域复用'],fontproperties=font);ax[1].set_title('场与曲线耗时（不含输入处理）',fontproperties=font);ax[1].set_ylabel('ms');ax[1].legend();ax[1].axhline(2,color='gray',ls='--',lw=1)
for a in ax:a.spines[['right','top']].set_visible(False);a.grid(axis='y',alpha=.18);a.set_axisbelow(True)
fig.suptitle('两层 Eigen LDLT＋严格缓存｜三轮各40次｜离线、CPU0、small积分策略',fontproperties=font,fontsize=13)
fig.text(.5,.01,'边界变化：3×3有60/120次停滞，4×4为120/120到出口；两者均值不代表相同完成任务。',ha='center',fontproperties=font,fontsize=10)
fig.tight_layout(rect=(0,.055,1,1));fig.savefig(r/'block4_comparison.png',dpi=160);fig.savefig(r/'block4_comparison.svg');plt.close(fig)
