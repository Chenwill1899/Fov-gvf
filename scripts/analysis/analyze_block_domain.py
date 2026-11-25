#!/usr/bin/env python3
"""Offline timing and conservative candidate-mask plots; no simulator launch."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager,FontProperties
from matplotlib.colors import ListedColormap
from analyze_compact_inputs import stats

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args();out=args.directory.resolve()
    ex=json.loads((out/'execution.json').read_text());verify=json.loads((out/'independent_comparison.json').read_text());assert ex['passed'] and ex['sources_unchanged'] and verify['passed']
    data={};first={}
    for r in ex['runs']:
        directory=Path(r['directory']);key=(r['scenario'],r['mode']);data.setdefault(key,[]).extend(list(csv.DictReader((directory/'timings.csv').open())))
        if r['mode']=='block_v3':first.setdefault(r['scenario'],directory)
    formal={}
    for (scenario,mode),rows in data.items():formal.setdefault(scenario,{})[mode]={k:stats([float(r[k]) for r in rows]) for k in rows[0] if k!='iteration'}
    for scenario,groups in formal.items():
        old=groups['compact_v2']['all_input_ms']['mean_ms'];new=groups['block_v3']['all_input_ms']['mean_ms'];groups['reduction_percent']=100*(1-new/old)
    masks={}
    for case,directory in first.items():
        local=json.loads((directory/'sample_local_field.json').read_text());validation=json.loads((directory/'local_validation.json').read_text())['local'];masks[case]={k:validation[k] for k in ['excluded_cells','excluded_blocks','complete_far_blocks','unknown_blocks','inflated_pixel_samples_verified']};masks[case]['excluded_fraction']=local['excluded_cells']/64800
    summary=dict(scope='matched offline; three rounds, 10 warmups/100 forced group rebuilds per input/mode/round; CPU0 coordinator, CPU1-4 ordinary workers; no new Isaac/FIFO',formal=formal,masks=masks,hot_near_pixel_transforms=0,mask_semantics='unavailable from conservative block exclusion; unexcluded remains unknown until an actual raw-image body-envelope query succeeds',geometry_scope='min Z screens possible near blocks; cached max Z tightens inflated frustum ball; body radius .10m; local surface range 3m',strict_2ms_guarantee=False,new_simulation=False,new_fifo=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');fontManager.addfont(font.get_file());plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10})
    cases=['archived','dense_synthetic','thin_synthetic'];titles=['真实静止存档（3m内障碍0）','密集近障碍（合成）','每块一个细近像素（合成）']
    fig,axes=plt.subplots(3,1,figsize=(11,8),constrained_layout=True)
    for ax,case,title in zip(axes,cases,titles):
        for mode,label,color in [('compact_v2','逐像素局部归格','#8492a6'),('block_v3','块级保守排除','#087f8c')]:
            ax.plot([float(r['all_input_ms']) for r in data[case,mode]],label=label,color=color,lw=.85)
        ax.axhline(2,color='#c44e52',ls='--',lw=.9,label='2 ms');ax.set_title(title);ax.set_ylabel('整组耗时 / ms');ax.grid(alpha=.2);ax.legend(loc='upper right',ncol=3,fontsize=9)
    axes[-1].set_xlabel('强制重建样本序号（分轮拼接，并非在线读取周期）');fig.suptitle('冻结旧/新版本：块级保守排除的离线处理耗时',fontsize=13);fig.savefig(out/'block_domain_timings.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(11,7.5),constrained_layout=True);cmap=ListedColormap(['#dfe5eb','#db8034'])
    for ax,case,title in zip(axes,cases,titles):
        array=np.fromfile(first[case]/'sample_local_excluded_camera_mask.bin',dtype=np.uint8).reshape(180,360)
        image=ax.imshow(array!=0,origin='lower',extent=[-180,180,-90,90],aspect='auto',cmap=cmap,vmin=0,vmax=1,interpolation='nearest')
        ax.set_title(f"{title}：排除 {masks[case]['excluded_cells']}/64800 格");ax.set_ylabel('俯仰角 / °')
    axes[-1].set_xlabel('机体系方位角 / °');cb=fig.colorbar(image,ax=axes,shrink=.75,ticks=[.25,.75],pad=.02);cb.ax.set_yticklabels(['默认未知','保守不可用'])
    fig.suptitle('候选方向排除掩码；灰色不能直接当成自由域',fontsize=13);fig.savefig(out/'block_domain_masks.png',dpi=150);plt.close(fig)
    print(json.dumps({case:dict(old_ms=d['compact_v2']['all_input_ms']['mean_ms'],new_ms=d['block_v3']['all_input_ms']['mean_ms'],reduction_percent=d['reduction_percent'],new_p99_ms=d['block_v3']['all_input_ms']['p99_ms'],new_max_ms=d['block_v3']['all_input_ms']['max_ms'],new_above_2ms=d['block_v3']['all_input_ms']['above_2ms']) for case,d in formal.items()},indent=2))

if __name__=='__main__':main()
