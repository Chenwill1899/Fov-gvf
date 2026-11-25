#!/usr/bin/env python3
"""Offline stage 5/6 timing and angular evidence figures, not a flight result."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np


def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(count=len(a),mean_ms=float(a.mean()),p50_ms=float(np.percentile(a,50)),
                p99_ms=float(np.percentile(a,99)),p999_ms=float(np.percentile(a,99.9)),max_ms=float(a.max()),above_2ms=int((a>2).sum()))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);out=parser.parse_args().directory.resolve()
    execution=json.loads((out/'execution.json').read_text());assert execution['passed']
    data={'off':[],'on':[]};query=[]
    for run in execution['runs']:
        assert run['returncode']==0
        directory=Path(run['directory']);data[run['mode']].extend(list(csv.DictReader((directory/'timings.csv').open())))
        if run['mode']=='on':query.extend(float(row['query_ns'])/1e6 for row in csv.DictReader((directory/'size_query_timings.csv').open()))
    summary=dict(scope=execution['scope'],formal={mode:{k:stats([float(r[k]) for r in rows]) for k in list(rows[0])[1:]} for mode,rows in data.items()},
                 sphere_query=stats(query),sphere_query_scope='artificial 2000 centers per on run; query cost includes status/geometry and full-resolution ROI when projected; no trajectory generation',
                 desired_ns_scope='benchmark consumes only desired speed; do not interpret as complete desired-direction cost',online_fifo_status='not_run_for_steps567')
    pilot=list(csv.DictReader((out/'pilot_on/timings.csv').open()));summary['exploratory_pilot']=stats([float(row['all_input_ms']) for row in pilot])
    sample=out/'r1_first_on';meta=json.loads((sample/'sample_local_field.json').read_text());summary['sample']=meta
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'font.size':10,'axes.unicode_minus':False})
    fig,axes=plt.subplots(1,3,figsize=(14,4.8))
    means=[summary['formal'][k]['all_input_ms']['mean_ms'] for k in ('off','on')]
    axes[0].bar(range(2),means,color=['#7594a4','#368b7b']);axes[0].set_xticks(range(2),['关闭局部整理','启用第五／六步'])
    for i,v in enumerate(means):axes[0].text(i,v+.02,f'{v:.3f} ms',ha='center')
    axes[0].set_ylim(0,max(means)*1.2);axes[0].set_ylabel('完整输入处理平均 / ms');axes[0].grid(axis='y',alpha=.2)
    for key,label in [('off','关闭局部整理'),('on','启用第五／六步')]:axes[1].plot([float(r['all_input_ms']) for r in data[key]],lw=.7,label=label)
    axes[1].axhline(2,color='#ba5656',ls='--',lw=.8,label='2ms参考线');axes[1].legend(fontsize=8);axes[1].set_xlabel('缓存强制重建编号');axes[1].set_ylabel('整组处理 / ms');axes[1].grid(alpha=.2)
    stages=['camera_work_ms','index_merge_ms','local_merge_ms'];values=[summary['formal']['on'][k]['mean_ms'] for k in stages]
    axes[2].bar(range(3),values,color=['#427b99','#b69351','#408e76']);axes[2].set_xticks(range(3),['四路并行墙钟','原方向归并','局部来源归并']);axes[2].set_ylabel('启用后分段平均 / ms');axes[2].grid(axis='y',alpha=.2)
    for i,v in enumerate(values):axes[2].text(i,v+.01,f'{v:.3f}',ha='center')
    axes[2].set_ylim(0,max(values)*1.2)
    fig.suptitle('第五／六步离线开销：各600次，初始存档3m内无局部障碍',fontsize=14)
    fig.text(.5,.02,'普通调度CPU0协调／CPU1～4工作；探索样本另有2.009ms长尾；新版本FIFO在线未运行。',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.06,1,.94))
    for suffix in ('png','pdf'):fig.savefig(out/f'local_field_timings.{suffix}',dpi=180)
    plt.close(fig)
    index=json.loads((sample/'sample_observation_index.json').read_text());na,nb=index['azimuth_bins'],index['elevation_bins']
    obs=np.diff(np.fromfile(sample/'sample_local_obstacle_offsets.bin',dtype='<u4')).reshape(nb,na)
    rays=np.diff(np.fromfile(sample/'sample_local_free_offsets.bin',dtype='<u4')).reshape(nb,na)
    uncertain=np.fromfile(sample/'sample_local_uncertain.bin',dtype=np.uint8).reshape(nb,na)
    no_evidence=(obs+rays)==0
    fig,axes=plt.subplots(2,2,figsize=(13,7.6))
    for ax,array,title,cmap in zip(axes.ravel(),[obs,rays,no_evidence,uncertain],['局部障碍像素来源数：本存档为0','射线证据来源数：条件证据，非整格自由','无观测证据的方向格','覆盖或近裁面依据不完整'],['magma','viridis','Greys','Oranges']):
        plot=ax.imshow(array,origin='lower',extent=(-180,180,-90,90),aspect='auto',cmap=cmap,vmin=0,vmax=max(1,int(array.max())))
        ax.set_title(title);ax.set_xlabel('机体系方位角 / °');ax.set_ylabel('俯仰角 / °');fig.colorbar(plot,ax=ax,shrink=.85)
    fig.suptitle('局部场构造输入：保留来源与未知边界，不生成直行距离图',fontsize=14)
    fig.text(.5,.02,'旧存档未记录近裁面，begin_known=false；原始深度覆盖与补值来源分开，不能据图判定整格可通行。',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.05,1,.94))
    for suffix in ('png','pdf'):fig.savefig(out/f'local_field_evidence.{suffix}',dpi=180)
    plt.close(fig)
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:summary[k] for k in ('formal','sphere_query','exploratory_pilot')},indent=2))


if __name__=='__main__':main()
