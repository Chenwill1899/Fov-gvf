from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
with (root/'closed_loop.csv').open() as f:
    rows=list(csv.DictReader(f))
cases=['pillar','disturbance','feasible_shift','no_feedback','no_continuation']
fig=plt.figure(figsize=(15,5))
fig.subplots_adjust(left=.035,right=.99,bottom=.15,top=.90,wspace=.42)
ax=fig.add_subplot(131,projection='3d')
for name in cases[:3]:
    data=[r for r in rows if r['case']==name]
    ax.plot(*[np.array([float(r[k]) for r in data]) for k in ['x','y','z']],label=name)
u,v=np.mgrid[0:2*np.pi:18j,0:np.pi:12j]
ax.plot_wireframe(3.5+.6*np.cos(u)*np.sin(v),.6*np.sin(u)*np.sin(v),.6*np.cos(v),color='.6',linewidth=.4)
ax.set(xlabel='x [m]',ylabel='y [m]',zlabel='z [m]',title='Depth-only closed-loop fixtures')
ax.legend(fontsize=8)
bx=fig.add_subplot(132)
for name in ['disturbance','no_feedback']:
    data=[r for r in rows if r['case']==name]
    y=[abs(float(r['chi'])) if r['tracking']=='1' else np.nan for r in data]
    bx.plot([float(r['t']) for r in data],y,label=name)
bx.set(xlabel='t [s]',ylabel='|chi| [chart pixels]',title='Valid tracking samples only')
bx.legend(fontsize=8);bx.grid(alpha=.2)
cx=fig.add_subplot(133)
for name in cases:
    data=np.sort([float(r['compute_ms']) for r in rows if r['case']==name])
    cx.plot(data,np.arange(1,len(data)+1)/len(data),label=name)
cx.set(xlabel='Planner compute [ms]',ylabel='Empirical CDF',title='500 control steps per case')
cx.legend(fontsize=7);cx.grid(alpha=.2)
fig.savefig(root/'closed_loop.png',dpi=170)
