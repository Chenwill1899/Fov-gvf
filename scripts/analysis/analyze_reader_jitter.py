#!/usr/bin/env python3
"""Offline comparison of archived reader timing cases; no runtime control."""
import argparse,csv,json,os
from pathlib import Path
import numpy as np


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output',type=Path)
    ap.add_argument('--case',action='append',required=True,help='label=run_directory')
    args=ap.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    results=[];series=[]
    for entry in args.case:
        label,path=entry.split('=',1);run=Path(path).resolve()
        with (run/'reads.csv').open() as f:rows=list(csv.DictReader(f))
        meta=json.loads((run/'reader_summary.json').read_text())
        t=np.array([int(r['wall_monotonic_ns']) for r in rows],dtype=np.int64)
        dt=np.diff(t)/1e6
        assert len(dt)>0 and np.isfinite(dt).all()
        assert json.loads((run/'validation.json').read_text())['passed']
        assert sum(int(r['skipped_slots']) for r in rows)==meta['skipped_periods']
        usage=100*meta['reader_thread_cpu_ns']/meta['reader_run_wall_ns']
        result=dict(label=label,run=str(run),reads=len(rows),intervals=len(dt),
            over_2_2ms=int(np.sum(dt>2.2)),below_2_2_percent=float(100*np.mean(dt<2.2)),
            p99_ms=float(np.percentile(dt,99)),p999_ms=float(np.percentile(dt,99.9)),
            max_ms=float(np.max(dt)),skipped=meta['skipped_periods'],groups=meta['complete_sets'],
            incomplete=meta['partial_discarded'],reader_cpu_percent=usage,
            timer_slack_ns=meta['timer_slack_actual_ns'],memory_lock_error=meta['memory_lock_error'],
            scheduler_policy=meta['actual_scheduler_policy'],priority=meta['actual_priority'],spin_us=meta['spin_us'],
            voluntary_switches=meta['reader_voluntary_switches'],involuntary_switches=meta['reader_involuntary_switches'])
        results.append(result);series.append(((t[1:]-t[0])/1e9,dt))
    with (out/'jitter_comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(results[0]));writer.writeheader();writer.writerows(results)
    (out/'jitter_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
    os.environ.setdefault('MPLCONFIGDIR',str(out/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
                         'axes.unicode_minus':False,'axes.grid':True,'grid.alpha':.25,'font.size':10})
    fig,axes=plt.subplots(2,2,figsize=(14,9),constrained_layout=True)
    for result,(time,dt) in zip(results,series):
        axes[0,0].plot(time,dt,lw=.7,alpha=.7,label=result['label'])
        a=np.sort(dt);tail=(len(a)-np.arange(len(a)))/len(a)
        axes[0,1].step(a,tail,where='post',lw=1,label=result['label'])
    axes[0,0].axhline(2.2,ls='--',color='red',label='2.2ms阈值');axes[0,0].set_title('A  每次相邻读取间隔（保留全部长尾）',loc='left')
    axes[0,0].set_xlabel('现实时间 / s');axes[0,0].set_ylabel('间隔 / ms');axes[0,0].legend(fontsize=8)
    axes[0,1].set_yscale('log');axes[0,1].set_xlim(2, max(r['max_ms'] for r in results)+.1)
    axes[0,1].axvline(2.2,ls='--',color='red');axes[0,1].set_title('B  读取间隔长尾分布',loc='left')
    axes[0,1].set_xlabel('间隔阈值 / ms');axes[0,1].set_ylabel('达到或超过阈值的比例（对数）');axes[0,1].legend(fontsize=8)
    labels=[r['label'] for r in results]
    counts=[r['over_2_2ms'] for r in results]
    axes[1,0].bar(labels,counts,color=plt.get_cmap('tab10').colors[:len(labels)])
    for i,n in enumerate(counts):axes[1,0].text(i,n+.25,f'{n}/{results[i]["intervals"]}',ha='center',fontsize=9)
    axes[1,0].set_ylim(0,max(counts)*1.2+1);axes[1,0].set_title('C  超过2.2ms的次数（每例约5000间隔）',loc='left')
    axes[1,0].set_ylabel('次数')
    cpu=[r['reader_cpu_percent'] for r in results];axes[1,1].bar(labels,cpu,color=plt.get_cmap('tab10').colors[:len(labels)])
    for i,n in enumerate(cpu):axes[1,1].text(i,n+.6,f'{n:.1f}%',ha='center')
    axes[1,1].set_ylim(0,max(cpu)*1.15+5);axes[1,1].set_title('D  读取线程CPU占用（含首帧等待期）',loc='left')
    axes[1,1].set_ylabel('单个逻辑CPU百分比')
    for ax in axes[1]:ax.tick_params(axis='x',labelrotation=12)
    fig.suptitle('EGO4P0 · 读取等待与调度对照 · 0.02倍速 / 每例0.2s仿真\n按存档实际策略统计；单次短测不能代替长期实时保证',fontsize=14)
    fig.savefig(out/'jitter_comparison.png',dpi=170);fig.savefig(out/'jitter_comparison.pdf');plt.close(fig)
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
