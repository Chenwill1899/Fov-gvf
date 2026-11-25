#!/usr/bin/env python3
"""Summarize offline C++ timings; blocked kernels are diagnostic estimates."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np


COARSE = [
    ('repair', '四路异常补值'), ('fused_pixel_ns', '逐像素变换、角度及归格'),
    ('prefix_coverage_ns', '前缀和及覆盖统计'), ('scatter_ns', '整理来源列表'),
    ('tiles_ns', '区域最小值摘要'), ('other', '清零及计时边界等'),
]
FINE = [
    ('transform_ns', '有效深度筛选及坐标变换'), ('hypot_ns', 'hypot(x, y)'),
    ('atan2_alpha_ns', '方位角 atan2(y, x)'), ('atan2_beta_ns', '俯仰角 atan2(z, hypot)'),
    ('bin_stage_ns', '角网格归类及来源暂存'),
]
MODES = ('original', 'fused_diagnostic', 'blocked_diagnostic')


def statistics(values):
    values = np.asarray(values, dtype=float) / 1e6
    return dict(count=len(values), mean_ms=float(values.mean()),
                p50_ms=float(np.percentile(values, 50)), p99_ms=float(np.percentile(values, 99)),
                min_ms=float(values.min()), max_ms=float(values.max()))


def read_rows(directory):
    rows = list(csv.DictReader((directory / 'stage_timings.csv').open()))
    for row in rows:
        for key in list(row):
            if key.endswith('_ns') or key == 'iteration':
                row[key] = int(row[key])
        row['repair'] = sum(row[f'repair_{camera}_ns'] for camera in ('front', 'left', 'back', 'right'))
        row['other'] = row['reset_ns'] + row['residual_ns']
        if row['mode'] == 'fused_diagnostic':
            assert sum(row[key] for key, _ in COARSE) == row['total_ns']
        if row['mode'] == 'blocked_diagnostic':
            assert (row['repair'] + row['other'] + row['prefix_coverage_ns'] + row['scatter_ns']
                    + row['tiles_ns'] + sum(row[key] for key, _ in FINE)) == row['total_ns']
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    out = args.directory.resolve()
    data = {case: read_rows(out / f'profile_{case}') for case in ('first', 'last')}
    pooled = [row for rows in data.values() for row in rows]

    def summarize(rows):
        return {mode: {key: statistics([row[key] for row in rows if row['mode'] == mode])
                       for key in rows[0] if key.endswith('_ns') or key in ('repair', 'other')}
                for mode in MODES}

    summary = dict(scope='offline cached snapshots on CPU5 SCHED_OTHER; no Isaac/GPU/render/DDS measurement',
                   pooled=summarize(pooled), by_snapshot={case: summarize(rows) for case, rows in data.items()},
                   caution='Fused diagnostic mirrors original loop. Blocked diagnostic changes memory access and loop layout; its stages cannot be added to the original 9ms.')
    means = {mode: summary['pooled'][mode]['total_ns']['mean_ms'] for mode in MODES}
    summary['fused_vs_original_percent'] = 100 * (means['fused_diagnostic'] / means['original'] - 1)
    summary['blocked_vs_original_percent'] = 100 * (means['blocked_diagnostic'] / means['original'] - 1)
    summary['blocked_vs_fused_percent'] = 100 * (means['blocked_diagnostic'] / means['fused_diagnostic'] - 1)
    summary['coarse'] = [dict(key=key, label=label, mean_ms=summary['pooled']['fused_diagnostic'][key]['mean_ms'],
                             percent=100*summary['pooled']['fused_diagnostic'][key]['mean_ms']/means['fused_diagnostic'])
                         for key, label in COARSE]
    summary['blocked_inner_kernels'] = [dict(key=key, label=label, mean_ms=summary['pooled']['blocked_diagnostic'][key]['mean_ms'],
                                           percent_of_blocked_total=100*summary['pooled']['blocked_diagnostic'][key]['mean_ms']/means['blocked_diagnostic'])
                                       for key, label in FINE]
    summary['atan2_pair_blocked_mean_ms'] = sum(summary['pooled']['blocked_diagnostic'][key]['mean_ms'] for key in ('atan2_alpha_ns', 'atan2_beta_ns'))
    summary['baseline_standalone'] = {case: json.loads((out / f'baseline_{case}' / 'benchmark.json').read_text()) for case in data}
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    with (out / 'stage_summary.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['method', 'stage', 'mean_ms', 'percent_of_same_method_total'])
        for row in summary['coarse']:
            writer.writerow(['fused_diagnostic', row['label'], row['mean_ms'], row['percent']])
        for row in summary['blocked_inner_kernels']:
            writer.writerow(['blocked_diagnostic', row['label'], row['mean_ms'], row['percent_of_blocked_total']])

    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False, 'font.size': 10})
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.7), gridspec_kw={'width_ratios': [1.12, 1]})
    for ax, rows, title, color in [
        (axes[0], summary['coarse'], f"整组分项：融合诊断 {means['fused_diagnostic']:.3f} ms", '#397ba5'),
        (axes[1], summary['blocked_inner_kernels'], f"分块诊断内部（整组 {means['blocked_diagnostic']:.3f} ms）", '#d1813c'),
    ]:
        values = [row['mean_ms'] for row in rows]
        ax.barh(range(len(rows)), values, color=color)
        ax.set_yticks(range(len(rows)), [row['label'] for row in rows])
        ax.invert_yaxis()
        ax.set_xlim(0, max(values)*1.3)
        for i, value in enumerate(values):
            ax.text(value + max(values)*.02, i, f'{value:.3f} ms', va='center', fontsize=10)
        ax.set_xlabel('每组平均耗时 / ms')
        ax.set_title(title, pad=14)
        ax.grid(axis='x', alpha=.2)
        ax.set_axisbelow(True)
    fig.suptitle('四路 320×240 深度：9 ms 花在哪里', fontsize=15, y=.98)
    fig.text(.5, .065, f"原实现 {means['original']:.3f} ms；融合诊断差异 {summary['fused_vs_original_percent']:+.2f}%；首末各200次，CPU5普通调度。", ha='center')
    fig.text(.5, .02, '右图拆开了循环，改变了缓存及指令执行；它解释计算量级，不能直接拼回左图。未运行新仿真。', ha='center', color='#555555')
    fig.tight_layout(rect=(0, .11, 1, .94), w_pad=2)
    for suffix in ('png', 'pdf'):
        fig.savefig(out / f'stage_breakdown.{suffix}', dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, sharey='row')
    for column, (case, rows) in enumerate(data.items()):
        modes = {mode: sorted((row for row in rows if row['mode'] == mode), key=lambda row: row['iteration']) for mode in MODES}
        for mode, label, color in [('original', '原实现总耗时', '#397ba5'), ('fused_diagnostic', '融合诊断总耗时', '#aa557f'), ('blocked_diagnostic', '分块诊断总耗时', '#d1813c')]:
            axes[0, column].plot([row['iteration'] for row in modes[mode]], [row['total_ns']/1e6 for row in modes[mode]], lw=.85, label=label, color=color)
        axes[0, column].set_title('首组存档' if case == 'first' else '末组存档')
        for key, label in COARSE[1:5]:
            axes[1, column].plot([row['iteration'] for row in modes['fused_diagnostic']], [row[key]/1e6 for row in modes['fused_diagnostic']], lw=.85, label=label)
        for key, label in FINE:
            axes[2, column].plot([row['iteration'] for row in modes['blocked_diagnostic']], [row[key]/1e6 for row in modes['blocked_diagnostic']], lw=.85, label=label)
        axes[2, column].set_xlabel('缓存数据重复构建次数（不是新相机帧）')
        for ax in axes[:, column]:
            ax.grid(alpha=.2)
    for i, label in enumerate(('整组耗时 / ms', '融合循环分项 / ms', '分块内部量级 / ms')):
        axes[i, 0].set_ylabel(label)
        axes[i, 1].legend(loc='upper right', fontsize=7)
    fig.suptitle('逐组计时曲线：三种模式按轮次轮换，测量期间不并行编译或测试', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .97))
    for suffix in ('png', 'pdf'):
        fig.savefig(out / f'timing_curves.{suffix}', dpi=180)
    plt.close(fig)
    print(json.dumps({key: summary[key] for key in ('fused_vs_original_percent', 'blocked_vs_original_percent', 'atan2_pair_blocked_mean_ms')}, indent=2))


if __name__ == '__main__':
    main()
