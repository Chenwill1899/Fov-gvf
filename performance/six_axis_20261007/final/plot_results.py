"""Plot saved metrics only; never alter experiments or aggregate away failures."""
from pathlib import Path
import csv,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[3];out=Path(__file__).resolve().parent
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.titleweight':'bold'})
def row(name):
 with (out.parent/'accepted_optional/ablations'/f'{name}.csv').open() as f:return next(csv.DictReader(f))
u=row('depth_uncertainty_check');i=row('incremental_field_check');m=row('spherical_memory_check');o=row('operator_intent_check');s=row('shared_obstacles_check')
panels=[('1. Depth uncertainty','False-free body certificates',[float(u['baseline_false_free_envelope']),float(u['robust_false_free_envelope'])],'Lower is better'),('2. Dynamic prediction','Collisions in 5 crossing trials',[5,0],'Lower is better'),('3. Incremental harmonic field','Iterations in 100 frames',[float(i['cold_iterations']),float(i['incremental_iterations'])],'Lower is better'),('4. Spherical memory','Usable previously seen directions',[float(m['sparse_history_accepted']),float(m['spherical_memory_accepted'])],'Higher is better; expired = 0'),('5. Operator intent (revised)','Command total variation',[float(o['baseline_command_total_variation']),float(o['assisted_total_variation'])],'Lower jitter; no added release delay'),('6. Shared occupied evidence','Unsafe local acceptances',[float(s['independent_map_accepted']),float(s['shared_map_accepted'])],'Lower is better; local unknown refused')]
fig,axes=plt.subplots(2,3,figsize=(13.8,8.1));fig.subplots_adjust(top=.87,hspace=.52,wspace=.30,bottom=.08)
for ax,(title,label,values,note) in zip(axes.ravel(),panels):
 bars=ax.bar(['Off','On'],values,color=['#adb7c5','#246e9b'],width=.55);ax.set_title(title,loc='left',pad=13);ax.set_ylabel(label);ax.set_ylim(0,max(values)*1.27 or 1)
 for bar,v in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+max(values)*.035,f'{v:,.6g}',ha='center',va='bottom',fontsize=11)
 ax.text(0,-.19,note,transform=ax.transAxes,fontsize=9,color='#405064')
fig.suptitle('EGO1P5 | Six optional mechanism ablations',x=.08,ha='left',fontsize=20)
fig.text(.08,.91,'Production core; controlled synthetic scenes and numerical tests. These are not six flight-success claims.',fontsize=11,color='#405064')
fig.savefig(out/'six_mechanisms.png',dpi=180);fig.savefig(out/'six_mechanisms.pdf');plt.close(fig)
nav=root/'performance/navigation_benchmark_20260929'
profiles=['baseline','default','spherical']; data=[]
for profile in profiles:
 p=nav/f'p5_six_final_20261007_{profile}_analysis.json'
 if not p.exists():continue
 d=json.loads(p.read_text());g=d['groups']['ego1p5'];data.append((profile,g,d['trials']))
if len(data)==3:
 fig,axes=plt.subplots(1,3,figsize=(13,4.5));fig.subplots_adjust(top=.77,bottom=.17,wspace=.32)
 for ax,key,title in zip(axes,['simulation_elapsed_s','stopped_with_input_s','jerk_rms_mps3'],['Arrival time (simulation s)','Stopped with input (s)','Jerk RMS (m/s3)']):
  for x,(name,g,trials) in enumerate(data):
   vals=[t[key] for t in trials if key in t];mean=sum(vals)/len(vals)
   ax.bar(x,mean,width=.55,color=['#adb7c5','#246e9b','#65a894'][x]);ax.scatter([x]*len(vals),vals,color='#182633',s=28,zorder=3);ax.text(x,max(vals)+max(.03,max(vals)*.025),f'{mean:.3f}',ha='center')
  ax.set_xticks(range(3),['Baseline','3+5','3+4+5']);ax.set_title(title);ax.margins(y=.18)
 fig.suptitle('First frozen Isaac batch | Regressions retained',x=.07,ha='left',fontsize=18)
 fig.text(.07,.85,'Bars: means; dots: individual trials. Small sample, one route. Safety and failures are reported separately.',fontsize=10)
 fig.savefig(out/'cloud_goals.png',dpi=180);fig.savefig(out/'cloud_goals.pdf');plt.close(fig)
