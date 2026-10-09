#!/usr/bin/env python3
"""Plot frozen directional-control trials; also emit raw per-run summary table."""
from pathlib import Path
import csv,json,argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('label');args=parser.parse_args()
report=json.loads((root/(args.label+'_analysis.json')).read_text())
colors={'ego1p2':'#a76037','ego1p3':'#127f9a'}
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,3,figsize=(13,7.3),layout='constrained')
metrics=[('moving_angle_mean_deg','Mean direction error (degrees)'),
         ('stop_s','Stopped with input (seconds)'),
         ('active_projected_progress_m','Progress along intent (meters)'),
         ('active_vector_error_rms_mps','Velocity vector error RMS (m/s)'),
         ('jerk_rms_mps3','Measured jerk RMS (m/s^3)'),
         ('compute_ms','Mean avoidance compute (ms)')]
for ax,(key,title) in zip(axs.ravel(),metrics):
 for a,algorithm in enumerate(('ego1p2','ego1p3')):
  means=[];stds=[]
  for case in ('sweep','spin'):
   runs=[r for r in report['runs'] if r['case']==case and r['algorithm']==algorithm]
   values=[r[key]['mean'] if key=='compute_ms' else r[key] for r in runs]
   means.append(np.mean(values));stds.append(np.std(values,ddof=1) if len(values)>1 else 0)
  x=np.arange(2)+(a-.5)*.32
  bars=ax.bar(x,means,.3,yerr=stds,capsize=3,label=algorithm.upper(),color=colors[algorithm])
  for b,v,sd in zip(bars,means,stds):ax.annotate(f'{v:.2f}',(b.get_x()+b.get_width()/2,b.get_height()+sd),xytext=(0,5),textcoords='offset points',ha='center',fontsize=9)
 ax.set_xticks([0,1],['360 sweep','Body spin + reverse']);ax.set_title(title);ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True);ax.margins(y=.2)
axs[0,0].legend(frameon=False)
fig.suptitle('EGO1P2 vs EGO1P3 | Frozen Cloud trials | Mean and sample standard deviation',fontsize=14)
fig.savefig(root/(args.label+'_comparison.png'),dpi=160)
plt.close(fig)
# Compare the first paired trajectory, using the same axes and actual recorded state.
fig,axs=plt.subplots(1,2,figsize=(13,5.5),layout='constrained')
for ax,case in zip(axs,('sweep','spin')):
 for algorithm in ('ego1p2','ego1p3'):
  path=root/f'{args.label}_{case}_{algorithm}_1.csv'
  rows=list(csv.DictReader(path.open()))
  x=np.array([float(r['x']) for r in rows]);y=np.array([float(r['y']) for r in rows])
  ax.plot(x,y,color=colors[algorithm],label=algorithm.upper(),lw=1.5)
  ax.scatter(x[-1],y[-1],c=colors[algorithm],s=30)
 ax.scatter(-64,-11.2,c='black',marker='*',s=100,label='Start');ax.set_aspect('equal',adjustable='datalim')
 ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.set_title(case+' | first frozen pair (illustration)');ax.grid(alpha=.2);ax.legend(frameon=False)
fig.savefig(root/(args.label+'_trajectories.png'),dpi=160)
