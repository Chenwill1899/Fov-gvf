#!/usr/bin/env python3
"""Offline CPU trace/monotonic CSV analysis; never runs in the control loop.

Usage: PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/analysis/plot_depth_stage_timing.py RUN_DIR
Trace scopes are inclusive wall durations. Nested scopes/worker scopes must not
be added together or interpreted as GPU kernel time or full DDS latency.
"""
import argparse
import csv
import gzip
import hashlib
import json
import os
from collections import defaultdict
from pathlib import Path

import numpy as np


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    out = parser.parse_args().run_dir.resolve()
    os.environ.setdefault('MPLCONFIGDIR', str(out/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    cjk_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(cjk_path)
    cjk_name = font_manager.FontProperties(fname=cjk_path).get_name()
    plt.rcParams.update({'font.family': cjk_name, 'axes.unicode_minus': False,
                         'font.size': 10, 'axes.grid': True, 'grid.alpha': .22})
    trace = json.load(gzip.open(out/'cpu_trace.json.gz', 'rt'))
    if isinstance(trace, dict):
        trace = trace['traceEvents']
    reads, updates = read_csv(out/'reads.csv'), read_csv(out/'simulator_updates.csv')
    frames, received = read_csv(out/'frames.csv'), read_csv(out/'received.csv')
    meta = json.loads((out/'reader_meta.json').read_text())
    simulation = json.loads((out/'simulation.json').read_text())
    scale = simulation.get('target_time_scale', 1.)
    wall_budget_ms = simulation.get('target_wall_step_s', simulation['nominal_step_s']/scale)*1000
    manifest = json.loads((out/'manifest.json').read_text())
    assert manifest.get('time_scale', 1.) == scale
    assert np.isclose(simulation['actual_physics_dt'], simulation['nominal_step_s'])
    assert np.isclose(simulation['actual_render_dt'], simulation['nominal_step_s'])
    roots = sorted((e for e in trace if e.get('ph') == 'X' and
                    e['name'] == 'EGO4P0_APP_UPDATE (Python)'), key=lambda e: e['ts'])
    assert len(roots) == len(updates), 'missing trace update markers'
    offsets = np.array([float(u['start_wall_s'])*1e6-e['ts'] for u, e in zip(updates, roots)])
    # Fit relative rate as well as offset: the Kit trace and CLOCK_MONOTONIC
    # need not have exactly the same clock rate over a multi-second capture.
    trace_marks = np.array([e['ts'] for e in roots])
    wall_marks = np.array([float(u['start_wall_s'])*1e6 for u in updates])
    trace_center, wall_center = np.median(trace_marks), np.median(wall_marks)
    rate, intercept = np.polyfit(trace_marks-trace_center, wall_marks-wall_center, 1)
    offset = float(wall_center+intercept-rate*trace_center)
    alignment_errors = wall_marks-(rate*trace_marks+offset)
    assert abs(rate-1) < .001, 'trace/monotonic fitted rate mismatch >0.1%'
    assert float(np.max(np.abs(alignment_errors))) < 100, 'trace alignment residual >100us'
    for event in trace:
        if event.get('ph') == 'X':
            event['ts'] = rate*event['ts']+offset
            event['dur'] *= rate
    first_ns, last_ns = int(reads[0]['wall_monotonic_ns']), int(reads[-1]['wall_monotonic_ns'])
    origin, finish = first_ns/1e9, last_ns/1e9
    begin, end = first_ns/1000, last_ns/1000
    main_tid = roots[0]['tid']
    complete_roots = [(i, r) for i, r in enumerate(roots)
                      if begin <= r['ts'] and r['ts']+r['dur'] <= end]
    names = {
        'app_update': 'EGO4P0_APP_UPDATE (Python)',
        'render_chain': 'UsdContext hydraRender',
        'physics_update': 'PhysXUpdate - update of PhysX',
        'omnigraph_update': 'ComputeGraph::update',
        'render_product_prims': 'RtxHydraEngine::updateRenderProductPrims',
        'render_settings_sync': 'IRenderSettingsContext Usd sync',
        'sensor_partition_render': 'performMultiTickRender::renderSensorPartition',
        'geometry_streaming_wait_budget': 'Wait for budget',
    }
    by_name = defaultdict(list)
    for event in trace:
        if event.get('ph') == 'X':
            by_name[event['name']].append(event)
    series = {}
    # Sum occurrences of the same named scope within each *complete* app update.
    # Different named parent/child scopes are kept separate throughout.
    for key, name in names.items():
        points = []
        for index, root in complete_roots:
            spans = [e for e in by_name[name] if e['tid'] == main_tid and
                     root['ts'] <= e['ts'] and e['ts']+e['dur'] <= root['ts']+root['dur']+.01]
            if spans:
                points.append((float(updates[index]['start_wall_s'])-origin,
                               sum(e['dur'] for e in spans)/1000))
        series[key] = points
    series['image_publish_call'] = [(e['ts']/1e6-origin, e['dur']/1000)
        for e in by_name['[IsaacSim] image publisher publish']
        if begin <= e['ts'] and e['ts']+e['dur'] <= end]
    selected_frames = [f for f in frames if first_ns <= int(f['published_ns']) <= last_ns]
    series['receiver_complete_copy'] = [((int(f['received_ns'])-first_ns)/1e9,
        (int(f['published_ns'])-int(f['received_ns']))/1e6) for f in selected_frames]
    arrivals = defaultdict(dict)
    for message in received:
        stamp, camera = int(message['image_stamp_ns']), int(message['camera'])
        arrivals[stamp].setdefault(camera, int(message['wall_monotonic_ns']))
    series['four_camera_arrival_spread'] = []
    for frame in selected_frames:
        ts = arrivals[int(frame['image_stamp_ns'])]
        assert len(ts) == 4 and max(ts.values()) <= int(frame['published_ns'])
        series['four_camera_arrival_spread'].append(((int(frame['published_ns'])-first_ns)/1e9,
                                                     (max(ts.values())-min(ts.values()))/1e6))
    for column in ('read_ms', 'work_ms', 'interval_ms', 'wake_lateness_ms', 'buffer_age_ms'):
        series[column] = [((int(r['wall_monotonic_ns'])-first_ns)/1e9, float(r[column]))
                          for r in (reads[1:] if column == 'interval_ms' else reads)]
    series['complete_group_wall_gap'] = [((int(b['published_ns'])-first_ns)/1e9,
        (int(b['published_ns'])-int(a['published_ns']))/1e6)
        for a, b in zip(selected_frames, selected_frames[1:])]
    # Independent pre/post physics callback wall span; not pure GPU execution.
    pre, pairs = None, []
    for e in read_csv(out/'physics_events.csv'):
        stamp = int(e['wall_monotonic_ns'])
        if e['event'] == 'pre':
            assert pre is None, 'nested/unmatched pre-physics event'
            pre = stamp
        else:
            assert pre is not None and stamp >= pre
            if first_ns <= pre and stamp <= last_ns:
                pairs.append(((pre-first_ns)/1e9, (stamp-pre)/1e6))
            pre = None
    assert pre is None
    series['physics_pre_post_callbacks'] = pairs
    summary = {}
    for name, points in series.items():
        values = np.array([p[1] for p in points])
        assert len(values) and np.isfinite(values).all() and (values >= 0).all(), name
        summary[name] = dict(count=len(values), mean_ms=float(np.mean(values)),
            p50_ms=float(np.median(values)), p95_ms=float(np.percentile(values,95)),
            p99_ms=float(np.percentile(values,99)), max_ms=float(np.max(values)))
    def save_csv(name, fields, rows):
        with (out/name).open('w', newline='') as stream:
            writer = csv.writer(stream); writer.writerow(fields); writer.writerows(rows)
    ranking = []
    for scope, events in by_name.items():
        for on_main in (True, False):
            values = [e['dur']/1000 for e in events if (e['tid'] == main_tid) == on_main
                      and begin <= e['ts'] and e['ts']+e['dur'] <= end]
            if values:
                ranking.append((scope, 'main' if on_main else 'worker', len(values),
                                sum(values), float(np.mean(values)), max(values)))
    save_csv('cpu_scope_ranking.csv',
             ['scope','thread_kind','count','inclusive_total_ms','mean_ms','max_ms'],
             sorted(ranking, key=lambda row: -row[3]))
    save_csv('stage_timing_curves.csv', ['stage','wall_elapsed_s','duration_ms'],
             ((name, x, y) for name, points in series.items() for x, y in points))
    save_csv('stage_timing_summary.csv', ['stage',*next(iter(summary.values())).keys()],
             ((name,*value.values()) for name,value in summary.items()))
    unique_stamps = [{int(r['image_stamp_ns']) for r in received if int(r['camera']) == i}
                     for i in range(4)]
    common_stamps = set.intersection(*unique_stamps)
    published_stamps = {int(f['image_stamp_ns']) for f in frames}
    assert published_stamps <= common_stamps, 'published stamp missing a received camera'
    missing_complete_stamps = common_stamps-published_stamps
    stamp_diagnosis = dict(unique_stamps_per_camera=list(map(len,unique_stamps)),
        duplicate_stamps_per_camera=[sum(int(r['camera']) == i for r in received)-len(unique_stamps[i])
                                    for i in range(4)],
        common_stamp_count=len(common_stamps), published_groups=len(published_stamps),
        fully_received_groups_missing_from_buffer=len(missing_complete_stamps))
    result = dict(stamp_diagnosis=stamp_diagnosis, run_dir=str(out), measured_sim_s=meta['measured_sim_duration_s'],
        measured_wall_s=finish-origin, target_time_scale=scale,
        actual_sim_per_wall=meta['measured_sim_duration_s']/(finish-origin),
        camera_time_domain='simulation', reader_time_domain='wall', target_wall_step_ms=wall_budget_ms, complete_app_updates=len(complete_roots),
        trace_clock_alignment_range_us=float(np.ptp(alignment_errors)),
        trace_clock_raw_offset_range_us=float(np.ptp(offsets)),trace_clock_fitted_rate=float(rate),
        trace_clock_alignment_max_deviation_us=float(np.max(np.abs(alignment_errors))),
        timing_scope='inclusive CPU wall spans; excludes startup before first successful read; nested scopes overlap; no pure GPU or complete DDS latency',
        profiling_overhead='CPU tracing and compression enabled; diagnostic sample, not uninstrumented performance',
        stages=summary)
    (out/'stage_timing_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    fig, axes = plt.subplots(3,2,figsize=(15,11),sharex=True,constrained_layout=True)
    colors = plt.get_cmap('tab10')
    def panel(ax, title, entries, unit='ms', target_ms=None):
        for i,(key,label) in enumerate(entries):
            points = np.array(series[key])
            ax.plot(points[:,0],points[:,1]*(1000 if unit == 'μs' else 1),
                    lw=.95,alpha=.9,label=label,color=colors(i),marker='.',ms=2)
        if target_ms is not None:
            ax.axhline(target_ms,color='#d62728',ls='--',lw=1,label=f'{target_ms:g} ms 现实预算')
        ax.set_title(title,loc='left',fontweight='bold');ax.set_ylabel('耗时 / '+unit)
        ax.legend(loc='upper right',fontsize=8,framealpha=.9)
        ax.set_xlim(0,finish-origin)
    panel(axes[0,0], 'A  Isaac 主循环及分项（CPU 墙钟）',[
        ('app_update','app.update 总耗时'),('render_chain','渲染链路（含准备/等待）'),
        ('physics_update','PhysX 更新'),('omnigraph_update','OmniGraph 更新')],target_ms=wall_budget_ms)
    panel(axes[0,1], 'B  渲染链路内部（与 A 存在嵌套）',[
        ('render_product_prims','四路渲染产品 Prim 更新'),
        ('render_settings_sync','渲染设置 USD 同步'),('sensor_partition_render','传感器分区渲染')],target_ms=wall_budget_ms)
    panel(axes[1,0], 'C  ROS 发布与 C++ 接收',[
        ('image_publish_call','单图 publish 调用（CPU）'),
        ('receiver_complete_copy','组齐回调→四图拷贝→发布缓冲区')],unit='μs')
    panel(axes[1,1], 'D  C++ 控制线程读取（引用 + 四像素探针）',[
        ('read_ms','读取'),('work_ms','读取周期内工作')],unit='μs')
    panel(axes[2,0], 'E  控制线程调度',[
        ('interval_ms','相邻读取间隔'),('wake_lateness_ms','唤醒迟到')],target_ms=meta['requested_period_ns']/1e6)
    panel(axes[2,1], 'F  同步组到达情况（墙钟，含等待）',[
        ('complete_group_wall_gap','相邻完整组间隔'),
        ('four_camera_arrival_spread','同时间戳四路回调到达跨度')],target_ms=wall_budget_ms)
    for ax, unit in ((axes[0,0], 'ms'), (axes[1,0], 'μs')):
        ax.set_yscale('log')
        ax.set_ylabel('耗时 / '+unit+'（对数）')
    for ax in axes[-1]:ax.set_xlabel('首个成功读取后的现实时间 / s')
    fig.suptitle(f'EGO4P0 · RTX 5070 · 仿真目标 {scale:g}× · 相机 {manifest["camera_hz"]} Hz（仿真）/ 读取 {manifest["read_hz"]:g} Hz（现实）\n'
        f'单次 {meta["measured_sim_duration_s"]:.3f} s 仿真；读取窗口 {finish-origin:.3f} s 墙钟；已开启 CPU 性能追踪',fontsize=14)
    fig.savefig(out/'stage_timing_curves.png',dpi=170)
    fig.savefig(out/'stage_timing_curves.pdf')
    plt.close(fig)
    checks = dict(passed=True, app_marker_count_matches=len(roots),complete_updates=len(complete_roots),
        trace_alignment_range_us=float(np.ptp(alignment_errors)),physics_pre_post_balanced=True,
        all_complete_groups_have_four_receives=True,all_stage_values_finite_nonnegative=True,
        all_complete_received_stamps_published=not missing_complete_stamps,
        source_at_analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'stage_timing_validation.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
