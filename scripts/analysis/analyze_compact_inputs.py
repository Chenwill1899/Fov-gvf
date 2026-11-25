#!/usr/bin/env python3
"""Analyze matched offline compact-input benchmarks; never launch a simulator."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties


def stats(values):
    v = np.asarray(values, dtype=float)
    return dict(count=len(v), mean_ms=float(v.mean()), p50_ms=float(np.quantile(v,.5)),
                p99_ms=float(np.quantile(v,.99)), p999_ms=float(np.quantile(v,.999)),
                max_ms=float(v.max()), above_2ms=int((v>2).sum()), above_2_2ms=int((v>2.2).sum()))


def collect(out, execution):
    grouped = {}
    for run in execution['runs']:
        if run['returncode'] != 0:
            raise ValueError('unsuccessful benchmark')
        key = (run['scenario'], run['mode'])
        grouped.setdefault(key, []).extend(dict(r) for r in csv.DictReader((Path(run['directory'])/'timings.csv').open()))
    result = {}
    for (scenario, mode), rows in grouped.items():
        result.setdefault(scenario,{})[mode] = {k: stats([float(r[k]) for r in rows]) for k in rows[0] if k != 'iteration'}
    return result, grouped


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args();out=args.directory.resolve()
    execution=json.loads((out/'verified_execution.json').read_text());assert execution['passed'] and execution['sources_unchanged']
    formal, rows=collect(out,execution)
    intermediate={}
    for label,name in [('mean_hint','intermediate_mean_hint_execution.json'),('private_counters','intermediate_private_counters_final_execution.json')]:
        intermediate[label]=collect(out,json.loads((out/name).read_text()))[0]
    sizes=json.loads((out/'type_sizes.json').read_text());cells=360*180;pixels=320*240;blocks=107*80
    old=2*(cells+1)*sizes['uint32']+4*pixels*sizes['uint32']+4*blocks*sizes['uint32']+2*cells+4*blocks*sizes['FreeRayEvidence']
    new=cells*(sizes['LocalCell']+sizes['uint32'])
    memory=dict(scope='per protected frame slot local dynamic-array element payload only; excludes images, static tables, worker scratch, vector objects and RSS',before_bytes=old,after_bytes=new,reduction_percent=100*(1-new/old),old_formula='2*(cells+1)*4 + 4*pixels*4 + 4*blocks*4 + 2*cells + 4*blocks*sizeof(FreeRayEvidence)',new_formula='cells*(sizeof(LocalCell)+4)',cells=cells,pixels_per_camera=pixels,blocks_per_camera=blocks)
    summary=dict(scope='matched offline archived and explicitly synthetic numerical inputs, three rounds; ordinary scheduling, CPU0 coordinator and CPU1-4 workers; no new Isaac/FIFO',formal=formal,intermediate=intermediate,memory=memory,
                 archived_initial_local_obstacles=0,dense_synthetic_local_obstacles=307200,thin_synthetic_local_obstacles=34240,
                 timing_scope='camera_work_ms includes four parallel repair/compression/projection/local workers and rendezvous; index_merge and local_merge follow sequentially. shared_projection_local_max_ms overlaps camera_work_ms and includes representative projection; old local_camera_max_ms is a different subset and must not be directly compared or added.',new_simulation=False,new_fifo=False,strict_2ms_guarantee=False)
    for case,data in formal.items():
        oldmean=data['baseline_v1']['all_input_ms']['mean_ms'];newmean=data['compact_v2']['all_input_ms']['mean_ms']
        data['mean_reduction_percent']=100*(1-newmean/oldmean);data['speedup']=oldmean/newmean
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10})
    from matplotlib.font_manager import fontManager
    fontManager.addfont(font.get_file())
    cases=['archived','dense_synthetic','thin_synthetic'];names=['真实静止存档（局部障碍0）','密集近障碍（合成）','单像素近障碍／远块均值（合成）'];colors={'baseline_v1':'#718096','compact_v2':'#087f8c'}
    fig,axes=plt.subplots(3,1,figsize=(11,9),constrained_layout=True)
    for ax,case,title in zip(axes,cases,names):
        for mode,label in [('baseline_v1','旧版'),('compact_v2','优化版')]:
            v=[float(r['all_input_ms']) for r in rows[case,mode]];ax.plot(v,lw=.9,color=colors[mode],alpha=.85,label=label)
        ax.axhline(2,color='#c44e52',ls='--',lw=1,label='2 ms');ax.set_title(title);ax.set_ylabel('整组处理 / ms');ax.grid(alpha=.18);ax.legend(loc='upper right',ncol=3)
    axes[-1].set_xlabel('强制处理样本序号（分轮拼接，不是在线周期）')
    fig.suptitle('整组输入处理耗时：冻结旧版与优化版离线对照',fontsize=14)
    fig.savefig(out/'compact_input_curves.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,5.2))
    fig.subplots_adjust(left=.06,right=.99,top=.80,bottom=.22,wspace=.28)
    parts=[('camera_work_ms','四路工作及等待','#198e9b'),('index_merge_ms','代表观测计数归并','#f3a64b'),('local_merge_ms','局部约束归并','#7c6fba')]
    for ax,case,title in zip(axes,cases,names):
        for pos,mode in enumerate(['baseline_v1','compact_v2']):
            bottom=0
            for key,label,color in parts:
                value=formal[case][mode][key]['mean_ms'];ax.bar(pos,value,bottom=bottom,color=color,label=label if pos==0 else None);bottom+=value
            ax.text(pos,bottom+.03,f"{formal[case][mode]['all_input_ms']['mean_ms']:.3f}",ha='center',va='bottom')
        ax.set_xticks([0,1],['旧版','优化版']);ax.set_title(title,fontsize=10);ax.set_ylabel('平均经过时间 / ms');ax.grid(axis='y',alpha=.18);ax.margins(y=.17)
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=3,bbox_to_anchor=(.5,.02))
    fig.suptitle('可相加的分项均值；并行核最大值不重复计入',fontsize=13)
    fig.savefig(out/'compact_stage_means.png',dpi=160,bbox_inches='tight');plt.close(fig)
    print(json.dumps({k:{'old_ms':v['baseline_v1']['all_input_ms']['mean_ms'],'new_ms':v['compact_v2']['all_input_ms']['mean_ms'],'reduction_percent':v['mean_reduction_percent']} for k,v in formal.items()},indent=2))


if __name__=='__main__':
    main()
