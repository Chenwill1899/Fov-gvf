#!/usr/bin/env python3
"""Offline pacing verification and comparison of two archived timing runs."""
import argparse
import csv
import json
import os
from pathlib import Path
import numpy as np


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--baseline', type=Path, required=True)
    args = parser.parse_args()
    run, old = args.run.resolve(), args.baseline.resolve()
    stage = json.loads((run/'stage_timing_summary.json').read_text())
    meta = json.loads((run/'reader_meta.json').read_text())
    manifest = json.loads((run/'manifest.json').read_text())
    sim = json.loads((run/'simulation.json').read_text())
    read = rows(run/'reads.csv')
    start, end = int(read[0]['wall_monotonic_ns'])/1e9, int(read[-1]['wall_monotonic_ns'])/1e9
    updates = rows(run/'simulator_updates.csv')
    requested_scale = manifest['time_scale']
    assert sim['target_time_scale'] == requested_scale
    assert np.isclose(sim['actual_physics_dt'], 1/manifest['camera_hz'])
    assert np.isclose(sim['actual_render_dt'], 1/manifest['camera_hz'])
    assert all(np.isclose(float(u['sim_end_s'])-float(u['sim_start_s']),
                          sim['nominal_step_s'], atol=1e-9) for u in updates)
    measured = [u for u in updates if start <= float(u['start_wall_s'])
                and float(u['pacing_end_wall_s']) <= end]
    assert measured
    budget = sim['target_wall_step_s']
    assert np.isclose(budget, sim['nominal_step_s']/requested_scale)
    paced = np.array([float(u['pacing_end_wall_s'])-float(u['start_wall_s']) for u in measured])
    compute = np.array([float(u['end_wall_s'])-float(u['start_wall_s']) for u in measured])
    assert np.all(paced >= budget-1e-6), 'pacing allowed a step faster than requested'
    with (run/'pacing_samples.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['frame','wall_elapsed_s','sim_end_s','update_ms','paced_step_ms','wait_and_overhead_ms','over_budget'])
        writer.writerows((u['frame'],float(u['start_wall_s'])-start,u['sim_end_s'],c*1000,p*1000,(p-c)*1000,int(c>budget))
                         for u,c,p in zip(measured,compute,paced))
    def values(path):
        r = json.loads((path/'reader_summary.json').read_text())
        t = json.loads((path/'stage_timing_summary.json').read_text())
        return dict(sim_seconds=r['measured_sim_duration_s'],wall_seconds=r['measured_wall_duration_s'],
            actual_time_scale=r['measured_sim_duration_s']/r['measured_wall_duration_s'],
            target_time_scale=t.get('target_time_scale',1.),reads=r['reads'],complete_groups=r['complete_sets'],
            repeated_reads=r['repeated_reads'],duplicate_or_late_images=r['late_or_duplicate_images'],
            incomplete_groups_discarded=r['partial_discarded'],group_wall_hz=r['producer_sets_per_wall_s'],
            group_sim_hz=r['producer_sets_per_sim_s'],read_mean_ms=r['read_ms']['mean'],
            interval_mean_ms=r['interval_ms']['mean'],interval_max_ms=r['interval_ms']['max'],
            skipped_periods=r['skipped_periods'],deadline_misses=r['deadline_misses'],
            app_mean_ms=t['stages']['app_update']['mean_ms'],app_max_ms=t['stages']['app_update']['max_ms'],
            render_mean_ms=t['stages']['render_chain']['mean_ms'])
    previous, current = values(old), values(run)
    with (run/'pacing_comparison.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['metric','previous_profiled_run','slow_profiled_run'])
        writer.writerows((key,previous[key],current[key]) for key in previous)
    result=dict(passed=True,baseline=str(old),run=str(run),previous=previous,current=current,
        expected_wall_s=meta['measured_sim_duration_s']/requested_scale,
        actual_wall_s=end-start,actual_time_scale=meta['measured_sim_duration_s']/(end-start),
        complete_paced_steps=len(measured),steps_update_over_budget=int(np.sum(compute>budget)),
        min_paced_step_ms=float(np.min(paced)*1000),mean_paced_step_ms=float(np.mean(paced)*1000),
        max_paced_step_ms=float(np.max(paced)*1000),all_sim_steps_match_requested_dt=True,
        all_paced_steps_at_least_requested_wall_period=True,
        note='Both are single profiled runs. Pacing changed; this is not evidence of long-term or hardware latency guarantees.')
    (run/'pacing_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    os.environ.setdefault('MPLCONFIGDIR',str(run/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
                         'axes.unicode_minus':False,'axes.grid':True,'grid.alpha':.25})
    fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
    x=np.array([(int(r['wall_monotonic_ns'])/1e9-start) for r in read])
    y=np.array([(int(r['clock_ns'])-meta['start_clock_ns'])/1e9 for r in read])
    axes[0].plot(x,y,label='实际 ROS 仿真时钟')
    axes[0].plot(x,x*requested_scale,'--',label=f'目标 {requested_scale:g}×')
    axes[0].set_ylabel('仿真推进时间 / s');axes[0].legend();axes[0].set_title('A  仿真推进速度核验',loc='left')
    t=np.array([float(u['start_wall_s'])-start for u in measured])
    axes[1].plot(t,paced*1000,label='每步总墙钟（含节拍等待）',lw=1)
    axes[1].plot(t,compute*1000,label='app.update 实际计算墙钟',lw=1)
    axes[1].plot(t,(paced-compute)*1000,label='节拍等待与少量统计开销',lw=1)
    axes[1].axhline(budget*1000,ls='--',color='red',label=f'{budget*1000:g} ms 现实预算')
    axes[1].set_ylabel('每步墙钟时间 / ms');axes[1].legend(loc='upper right',fontsize=9)
    axes[1].set_title(f'B  仿真步长{sim["nominal_step_s"]*1000:g}ms不变；每步现实节拍{budget*1000:g}ms',loc='left')
    for ax in axes:ax.set_xlim(0,end-start);ax.set_xlabel('首个成功读取后的现实时间 / s')
    fig.suptitle(f'EGO4P0 · 仿真目标 {requested_scale:g}× · 实测 {result["actual_time_scale"]:.5f}×')
    fig.savefig(run/'simulation_speed.png',dpi=170);fig.savefig(run/'simulation_speed.pdf');plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
