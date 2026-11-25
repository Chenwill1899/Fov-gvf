#!/usr/bin/env python3
"""Offline comparison of accepted virtual-center and historical camera models."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import fontManager,FontProperties
from matplotlib.patches import Patch
from analyze_compact_inputs import stats


def collect(ex):
    rows={};first={}
    for r in ex['runs']:
        assert r['returncode']==0;key=(r['scenario'],r['mode']);directory=Path(r['directory']);rows.setdefault(key,[]).extend(list(csv.DictReader((directory/'timings.csv').open())))
        if r['mode']=='virtual_v4':first.setdefault(r['scenario'],directory)
    groups={}
    for (case,mode),data in rows.items():groups.setdefault(case,{})[mode]={key:stats([float(r[key]) for r in data]) for key in data[0] if key!='iteration'}
    for case,g in groups.items():g['mean_change_percent']=100*(g['virtual_v4']['all_input_ms']['mean_ms']/g['translated_v3']['all_input_ms']['mean_ms']-1)
    return rows,groups,first


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);args=p.parse_args();out=args.directory.resolve()
    ex=json.loads((out/'execution.json').read_text());assert ex['passed'] and ex['sources_unchanged'];assert json.loads((out/'independent_comparison.json').read_text())['passed']
    rows,formal,first=collect(ex);_,initial,_=collect(json.loads((out/'initial_virtual_execution.json').read_text()))
    states={}
    for case,directory in first.items():
        m=json.loads((directory/'sample_local_field.json').read_text());states[case]={k:m[k] for k in ['available_cells','excluded_cells','unknown_cells','excluded_blocks']};states[case]['unavailable_cells']=64800-m['available_cells']-m['unknown_cells']
    pilot=[float(r['all_input_ms']) for r in csv.DictReader((out/'pilot_dense_virtual/timings.csv').open())]
    summary=dict(scope='same-session frozen old/new offline inputs, different geometric models and outputs; 3 rounds, 10 warmups/100 forced builds per input/mode/round; no online/FIFO/Isaac',formal=formal,initial_virtual=initial,exploratory_dense_pilot=stats(pilot),domain_states=states,
                 approximation=dict(translation_ignored=True,physical_near_clip_checked=False,rotation_retained=True,rho_formula='Z*norm(K^-1 [u v 1])',distance_bin_m=.05,body_radius_m=.1,local_range_m=3,virtual_domain_clearance_m=3.1,uncovered='unknown',partial_raw='no free evidence from repair alone'),
                 compatibility='7 optical payloads unchanged against translated baseline; 27 model payloads unchanged against initial virtual implementation; geometric equivalence to real camera centers is not asserted',strict_2ms_guarantee=False,new_simulation=False,new_fifo=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');fontManager.addfont(font.get_file());plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10})
    cases=['archived','dense_synthetic','thin_synthetic'];titles=['真实静止存档','密集近障碍（合成）','每块一个细近像素（合成）']
    fig,axes=plt.subplots(3,1,figsize=(11,8),constrained_layout=True)
    for ax,case,title in zip(axes,cases,titles):
        for mode,label,color in [('translated_v3','原光心模型（无Ω分类）','#8492a6'),('virtual_v4','虚拟中心＋Ω','#087f8c')]:ax.plot([float(r['all_input_ms']) for r in rows[case,mode]],color=color,label=label,lw=.85)
        ax.axhline(2,color='#c44e52',ls='--',lw=.9,label='2 ms');ax.set_title(title);ax.set_ylabel('整组处理 / ms');ax.grid(alpha=.2);ax.legend(ncol=3,loc='upper right',fontsize=9)
    axes[-1].set_xlabel('强制重建序号（分轮拼接，并非在线周期）');fig.suptitle('近似模型更换后的离线耗时：包含全部长尾',fontsize=13);fig.savefig(out/'virtual_center_timings.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(11,7.5),constrained_layout=True);cmap=ListedColormap(['#dfe5eb','#db8034','#177d98'])
    for ax,case,title in zip(axes,cases,titles):
        array=np.fromfile(first[case]/'sample_panorama_state.bin',dtype=np.uint8).reshape(180,360)
        ax.imshow(array,origin='lower',extent=[-180,180,-90,90],aspect='auto',cmap=cmap,vmin=0,vmax=2,interpolation='nearest');ax.set_ylabel('俯仰角 / °');ax.set_title(f"{title}：可用 {states[case]['available_cells']}，不可用 {states[case]['unavailable_cells']}，未知 {states[case]['unknown_cells']}")
    axes[-1].set_xlabel('机体系方位角 / °');fig.legend([Patch(color=c) for c in ['#dfe5eb','#db8034','#177d98']],['未知','保守不可用','虚拟模型内可用'],loc='outside lower center',ncol=3) if matplotlib.__version__>='3.7' else None
    fig.suptitle('候选域Ω：忽略平移；灰＝未知，橙＝不可用，蓝＝模型内可用',fontsize=12);fig.savefig(out/'virtual_center_domain.png',dpi=150);plt.close(fig)
    print(json.dumps({case:{'old_mean_ms':g['translated_v3']['all_input_ms']['mean_ms'],'new_mean_ms':g['virtual_v4']['all_input_ms']['mean_ms'],'change_percent':g['mean_change_percent'],'new_p99_ms':g['virtual_v4']['all_input_ms']['p99_ms'],'new_max_ms':g['virtual_v4']['all_input_ms']['max_ms'],'over_2ms':g['virtual_v4']['all_input_ms']['above_2ms']} for case,g in formal.items()},indent=2))


if __name__=='__main__':main()
