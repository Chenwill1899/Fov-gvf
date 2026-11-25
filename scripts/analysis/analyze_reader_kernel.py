#!/usr/bin/env python3
"""Offline CLOCK_MONOTONIC correlation of archived C++ reads and perf traces.

No privileges, simulation, process control, or live system changes. Input files
are preserved. PID/CPU and timing parameters come from the archived run.
"""
import argparse
import bisect
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

import numpy as np

EVENT = re.compile(r'^\s*(.*?)\s+(-?\d+)\s+\[(\d+)\]\s+(\d+)\.(\d+):\s+(\w+:\w+):\s+(.*)$')
SWITCH = re.compile(r'prev_comm=(.*?) prev_pid=(\d+) prev_prio=(\d+) prev_state=(.*?) ==> next_comm=(.*?) next_pid=(\d+) next_prio=(\d+)')
WAKE = re.compile(r'comm=(.*?) pid=(\d+) prio=(\d+) target_cpu=(\d+)')


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def distribution(values):
    return dict(count=len(values), p50=float(np.percentile(values, 50)),
                p99=float(np.percentile(values, 99)), p999=float(np.percentile(values, 99.9)),
                max=float(np.max(values)))


def overlap(start, end, a, b):
    return max(0, min(end, b) - max(start, a))


def union_duration(spans, start, end):
    intervals=sorted((max(start,a), min(end,b)) for a,b in spans if overlap(start,end,a,b))
    total=0;right=start
    for a,b in intervals:
        total+=max(0,b-max(a,right));right=max(right,b)
    return total


def parse_trace(path):
    events=[]
    for line in path.read_text().splitlines():
        match=EVENT.fullmatch(line)
        if not match:
            raise ValueError(f'unrecognized perf line in {path}: {line[:160]}')
        comm,tid,cpu,sec,fraction,event,args=match.groups()
        # Preserve all nine decimal places; never round wall timestamps via float.
        stamp=int(sec)*1_000_000_000+int(fraction.ljust(9,'0'))
        events.append((stamp, comm, int(tid), int(cpu), event, args))
    if not events or any(a[0]>b[0] for a,b in zip(events,events[1:])):
        raise ValueError(f'empty or unsorted trace: {path}')
    return events


def audit_perf(run, name, out):
    raw=run/(name+'.perf.data')
    # --show-lost-events ensures loss records are not silently omitted.
    result=subprocess.run(['/usr/bin/perf','script','--ns','--show-lost-events','-i',str(raw)],
                          text=True,capture_output=True,check=True)
    debug=subprocess.run(['/usr/bin/perf','script','-d','-i',str(raw)],
                         text=True,capture_output=True,check=True)
    (out/(run.name+'_'+name+'_audit.log')).write_text(result.stderr+debug.stderr)
    if re.search(r'\bLOST\b|PERF_RECORD_LOST',result.stdout) or re.search(r'\blost\b',result.stderr,re.I):
        raise ValueError(f'perf reports lost records: {raw}')
    if not re.search(r'Misordered timestamps: 0\b',debug.stderr):
        raise ValueError(f'perf ordering check did not pass: {raw}')
    archived=(run/(name+'.trace.txt')).read_text()
    # Root exported kernel function names; an ordinary-user re-export shows
    # their addresses. Only normalize that symbol field, retaining event args.
    def normalize(value):
        return re.sub(r'function=\S+', 'function=<kernel-symbol>', value)
    if normalize(result.stdout)!=normalize(archived):
        raise ValueError(f'perf output differs from archived trace: {raw}')
    return dict(returncode=result.returncode, lost_records_reported=False,
                misordered_timestamps=0, exported_text_matches_except_kernel_symbols=True,
                sha256=hashlib.sha256(raw.read_bytes()).hexdigest())


def analyze(run, out):
    meta=json.loads((run/'reader_meta.json').read_text())
    proof=json.loads((run/'privileged_run.json').read_text())
    validation=json.loads((run/'validation.json').read_text())
    if not validation['passed'] or not proof['passed']:
        raise ValueError(f'original run validation failed: {run}')
    if (meta['actual_scheduler_policy'],meta['actual_priority']) != (proof['recorded_actual_policy'],proof['recorded_actual_priority']):
        raise ValueError('actual scheduler metadata mismatch')
    checks={name:audit_perf(run,name,out) for name in ('kernel_cpu5','kernel_wakeup')}
    cpu=parse_trace(run/'kernel_cpu5.trace.txt');wake_events=parse_trace(run/'kernel_wakeup.trace.txt')
    pid=proof['reader_pid_and_tid'];core=proof['cpu'];spin=meta['spin_us']*1000
    rows=read_csv(run/'reads.csv');t=[int(row['wall_monotonic_ns']) for row in rows]
    for name,events in (('cpu',cpu),('wake',wake_events)):
        if not events[0][0]<=t[0]<=t[-1]<=events[-1][0]:
            raise ValueError(f'{name} trace does not cover the whole reading window')
    switches=[];irq_stack=[];irq_spans=[]
    for stamp,comm,tid,cp,event,args in cpu:
        if cp!=core:raise ValueError('wrong CPU in per-CPU trace')
        if event=='sched:sched_switch':
            m=SWITCH.fullmatch(args)
            if not m:raise ValueError(args)
            pc,pp,prio,state,nc,npid,nprio=m.groups()
            if switches and switches[-1]['next_pid']!=int(pp):
                raise ValueError('discontinuous sched_switch PID chain')
            switches.append(dict(t=stamp,prev_comm=pc,prev_pid=int(pp),prev_state=state,
                                 next_comm=nc,next_pid=int(npid)))
        elif event.startswith('irq:'):
            kind=event.split(':')[1]
            key=args.split()[0]
            if kind.endswith('_entry'):
                irq_stack.append((stamp,kind[:-6],key))
            elif kind.endswith('_exit'):
                match=(kind[:-5],key)
                if not irq_stack or irq_stack[-1][1:]!=match:
                    raise ValueError('unbalanced IRQ/softirq entries')
                start,_,_=irq_stack.pop();irq_spans.append((start,stamp))
    if irq_stack:raise ValueError('unterminated IRQ event')
    wake=[]
    for stamp,comm,tid,cp,event,args in wake_events:
        m=WAKE.fullmatch(args)
        if event!='sched:sched_wakeup' or not m:raise ValueError(args)
        if int(m[4])!=core:raise ValueError('wrong wake target CPU')
        if int(m[2])==pid:wake.append(stamp)
    switch_in=[event['t'] for event in switches if event['next_pid']==pid]
    spans=[(a['t'],b['t'],a['next_comm'],a['next_pid']) for a,b in zip(switches,switches[1:])]
    times=[span[0] for span in spans]
    occupancy=collections.Counter()
    for start,end,comm,tid in spans:occupancy[(comm,tid)]+=overlap(t[0],t[-1],start,end)
    matches=[];outliers=[]
    for index,row in enumerate(rows):
        now=t[index];scheduled=int(row['scheduled_ns']);early=scheduled-spin
        wi=bisect.bisect_right(wake,now)-1
        if wi<0:raise ValueError('no matching wake')
        w=wake[wi];si=bisect.bisect_left(switch_in,w)
        if si==len(switch_in) or switch_in[si]>now or (index and w<=t[index-1]):
            raise ValueError('could not uniquely correlate wake -> dispatch -> read')
        dispatch=switch_in[si]
        if not early<=w<=dispatch<=now:raise ValueError('invalid wake/dispatch ordering')
        interval=now-t[index-1] if index else None
        info=dict(read_index=int(row['read_index']),wall_monotonic_ns=now,scheduled_ns=scheduled,
                  wake_ns=w,dispatch_ns=dispatch,interval_ms=interval/1e6 if interval is not None else None,
                  lateness_us=(now-scheduled)/1000,timer_wake_delay_us=(w-early)/1000,
                  runqueue_delay_us=(dispatch-w)/1000,dispatch_to_read_us=(now-dispatch)/1000,
                  read_us=float(row['read_ms'])*1000)
        matches.append(info)
        if interval is not None and interval>2_200_000:
            contenders=collections.Counter()
            for a,b,comm,tid in spans[max(0,bisect.bisect_right(times,w)-1):]:
                if a>=dispatch:break
                duration=overlap(w,dispatch,a,b)
                if duration:contenders[(comm,tid)]+=duration
            dominant=max(contenders,key=contenders.get)
            info=info.copy()
            info.update(dominant_comm=dominant[0],dominant_tid=dominant[1],
                        dominant_scheduled_us=contenders[dominant]/1000,
                        irq_softirq_union_us=union_duration(irq_spans,w,dispatch)/1000,
                        scheduled_contexts=[dict(comm=k[0],tid=k[1],us=v/1000) for k,v in contenders.items()])
            outliers.append(info)
    frames=read_csv(run/'frames.csv');received=read_csv(run/'received.csv')
    first=int(frames[0]['image_stamp_ns']);last=int(frames[-1]['image_stamp_ns'])
    step=round(1e9/json.loads((run/'manifest.json').read_text())['camera_hz'])
    expected=set(range(first,last+1,step))
    stamps=[{int(row['image_stamp_ns']) for row in received if int(row['camera'])==cam} for cam in range(4)]
    missing=[sorted(expected-view) for view in stamps]
    missing_groups=sorted(set().union(*map(set,missing)))
    summary=dict(label=run.name,run=str(run),reader_tid=pid,actual_policy=meta['actual_scheduler_policy'],
                 actual_priority=meta['actual_priority'],spin_us=meta['spin_us'],
                 intervals=len(t)-1,over_2_2ms=len(outliers),below_2_2ms=sum(b-a<2_200_000 for a,b in zip(t,t[1:])),
                 interval_ms=distribution(np.diff(np.array(t,dtype=np.int64))/1e6),
                 runqueue_delay_us=distribution([x['runqueue_delay_us'] for x in matches]),
                 timer_wake_delay_us=distribution([x['timer_wake_delay_us'] for x in matches]),
                 reader_cpu_percent=100*meta['reader_thread_cpu_ns']/meta['reader_run_wall_ns'],
                 skipped=meta['reads'] and sum(int(row['skipped_slots']) for row in rows),
                 involuntary_switches=meta['reader_involuntary_switches'],minor_faults=meta['reader_minor_faults'],major_faults=meta['reader_major_faults'],
                 complete_groups=len(frames),expected_groups_in_first_last_complete_stamp_window=len(expected),
                 missing_camera_stamps_ns=missing,missing_group_stamps_ns=missing_groups,
                 perf_audit=checks,capture_exit_codes=proof['capture_exit_codes'],
                 trace_covers_all_reads=True,sched_switch_chain_continuous=True,irq_entries_balanced=True,
                 matched_wakes=len(matches),
                 perf_scheduled_ns_in_valid_window=sum(value for (comm,tid),value in occupancy.items() if comm=='perf'),
                 competing_context_counts=dict(collections.Counter(row['dominant_comm'] for row in outliers)),
                 outliers=outliers)
    with (out/(run.name+'_wake_dispatch.csv')).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(matches[0]));writer.writeheader();writer.writerows(matches)
    (out/(run.name+'_kernel_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
    return summary,matches,spans


def plot(results, out):
    os.environ.setdefault('MPLCONFIGDIR',str(out/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
                         'axes.unicode_minus':False,'axes.grid':True,'grid.alpha':.25,'font.size':10})
    fig,axes=plt.subplots(2,2,figsize=(15,9),constrained_layout=True)
    base=next(result for result in results if result[0]['actual_priority']==0)
    summary,base_matches,spans=base;bad=summary['outliers'];x=np.arange(len(bad))
    axes[0,0].bar(x,[r['runqueue_delay_us']/1000 for r in bad],color='#c36a42')
    axes[0,0].set_xticks(x,[str(r['read_index']) for r in bad]);axes[0,0].set_xlabel('普通调度：超2.2ms的读取序号')
    axes[0,0].set_ylabel('等待CPU / ms');axes[0,0].set_title(f'A  已唤醒 → 实际执行：{len(bad)}次超2.2ms间隔',loc='left')
    for result,matches,_ in results:
        delay=np.sort([r['runqueue_delay_us'] for r in matches])
        tail=(len(delay)-np.arange(len(delay)))/len(delay)
        label='普通调度' if result['actual_priority']==0 else 'FIFO10'
        axes[0,1].step(delay,tail,where='post',label=label,lw=1.2)
    axes[0,1].set_xscale('symlog',linthresh=10);axes[0,1].set_yscale('log');axes[0,1].legend()
    axes[0,1].set_xlabel('唤醒至执行 / μs（10μs后为对数刻度）');axes[0,1].set_ylabel('达到或超过该延迟的比例')
    axes[0,1].set_title('B  全部有效读取的调度等待长尾',loc='left')
    worst=max(bad or base_matches[1:],key=lambda r:r['interval_ms']);target=worst['scheduled_ns']
    lo=worst['wake_ns']-100_000;hi=worst['wall_monotonic_ns']+10_000
    contexts=[]
    for a,b,comm,tid in spans:
        if overlap(lo,hi,a,b):contexts.append((max(a,lo),min(b,hi),comm,tid))
    names=list(dict.fromkeys(comm for a,b,comm,tid in contexts))
    colors={'ChatGPT':'#d48646','migration/5':'#ad87be','ego4p0_2ms':'#4b9c8a','swapper/5':'#bdbdbd'}
    for a,b,comm,tid in contexts:
        axes[1,0].broken_barh([((a-target)/1e6,(b-a)/1e6)],(names.index(comm)-.3,.6),facecolors=colors.get(comm,'#7396bb'))
    for stamp,name,color in [(worst['wake_ns'],'已唤醒','#c84b40'),(target,'计划读取','#486aab'),(worst['wall_monotonic_ns'],'实际读取','#222222')]:
        axes[1,0].axvline((stamp-target)/1e6,ls='--',label=name,color=color)
    axes[1,0].set_yticks(range(len(names)),names);axes[1,0].legend(fontsize=8,loc='lower right')
    axes[1,0].set_xlabel('相对计划读取时刻 / ms');axes[1,0].set_title(f'C  最长间隔{worst["interval_ms"]:.6f}ms的CPU5时间线',loc='left')
    fifo=next(result[0] for result in results if result[0]['actual_priority']>0)
    missing=fifo['missing_camera_stamps_ns'];stamps=fifo['missing_group_stamps_ns']
    for cam,values in enumerate(missing):
        axes[1,1].scatter([stamps.index(value) for value in values],[cam]*len(values),marker='x',s=65,color='#c84b40')
    axes[1,1].set_yticks(range(4),['Front','Left','Back','Right']);axes[1,1].set_xticks(range(len(stamps)),[str(value//1_000_000) for value in stamps])
    axes[1,1].set_xlabel('缺帧图像采集时间戳（分类等距） / ms');axes[1,1].set_ylim(-.5,3.5)
    axes[1,1].set_title(f'D  FIFO轮{len(stamps)}组未齐：相机接收需单独验证',loc='left')
    fig.suptitle('EGO4P0 · 内核调度与C++读取逐次对时 · CLOCK_MONOTONIC\n0.02倍速 / 每例0.2s仿真；本图说明本次记录，不能外推长期保证',fontsize=14)
    fig.savefig(out/'kernel_correlation.png',dpi=170);fig.savefig(out/'kernel_correlation.pdf');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();session=args.session.resolve();out=(args.output or session/'analysis').resolve()
    out.mkdir(parents=True,exist_ok=True)
    results=[analyze(session/name,out) for name in ('normal_200us','fifo10_200us')]
    (out/'kernel_comparison.json').write_text(json.dumps([r[0] for r in results],indent=2)+'\n')
    plot(results,out)
    print(json.dumps([dict(label=r[0]['label'],over_2_2ms=r[0]['over_2_2ms'],
                           interval_ms=r[0]['interval_ms'],runqueue_delay_us=r[0]['runqueue_delay_us']) for r in results],indent=2))


if __name__=='__main__':main()
