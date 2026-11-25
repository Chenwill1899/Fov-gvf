#!/usr/bin/env python3
"""Summarize staged offline block-index optimization; never used by control."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

LABELS = {'v1_pixel': '原逐像素索引', 'block3_serial_exact': '3×3＋旋转视线（串行）',
          'block3_parallel_exact': '四线程＋精确角度', 'block3_parallel_boundary': '四线程＋角边界判定'}


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), mean_ms=float(a.mean()), p50_ms=float(np.percentile(a, 50)),
                p99_ms=float(np.percentile(a, 99)), max_ms=float(a.max()), above_2ms=int((a > 2).sum()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    out = args.directory.resolve()
    execution = json.loads((out / 'merged_execution.json').read_text())
    data = {key: [] for key in LABELS}
    for run in execution['runs']:
        assert run['returncode'] == 0
        for row in csv.DictReader((Path(run['directory']) / 'build_timings.csv').open()):
            data[run['label']].append({**{k: float(v) for k, v in row.items()}, 'repeat': run['repeat'], 'snapshot': run['snapshot']})
    summary = dict(scope='600 forced offline cache rebuilds per variant, 3 rounds x 2 archived static snapshots x 100; not online 2ms/FIFO verification',
                   coordinator_cpu=0, camera_worker_cpus=[1, 2, 3, 4], reader_reserved_cpus=[5, 11],
                   stages={key: {col: stats([r[col] for r in rows]) for col in rows[0] if col.endswith('_ms')}
                           for key, rows in data.items()})
    summary['speedup_mean'] = summary['stages']['v1_pixel']['all_input_ms']['mean_ms'] / summary['stages']['block3_parallel_boundary']['all_input_ms']['mean_ms']
    final = next(run for run in execution['runs'] if run['label'] == 'block3_parallel_boundary')
    meta = json.loads((Path(final['directory']) / 'sample_observation_index.json').read_text())
    summary['observations'] = {key: meta[key] for key in ('blocks_per_camera', 'source_count', 'usable_blocks', 'observed_cells', 'unobserved_cells', 'fast_angle_observations', 'exact_angle_observations')}
    summary['fast_fraction'] = sum(meta['fast_angle_observations']) / meta['source_count']
    # Keep initial, slower boundary results visible as development evidence.
    initial = json.loads((out / 'execution.json').read_text())
    summary['initial_two_dispatch_means_ms'] = {}
    for key in LABELS:
        values = [float(row['all_input_ms']) for run in initial['runs'] if run['label'] == key
                  for row in csv.DictReader((Path(run['directory']) / 'build_timings.csv').open())]
        summary['initial_two_dispatch_means_ms'][key] = float(np.mean(values))
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    with (out / 'comparison.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['variant', 'count', 'mean_ms', 'p50_ms', 'p99_ms', 'max_ms', 'above_2ms'])
        for key in LABELS:
            s = summary['stages'][key]['all_input_ms']
            writer.writerow([key] + [s[k] for k in ('count', 'mean_ms', 'p50_ms', 'p99_ms', 'max_ms', 'above_2ms')])
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'font.size': 10, 'axes.unicode_minus': False})
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.3), gridspec_kw={'width_ratios': [1.4, 1]})
    means = [summary['stages'][key]['all_input_ms']['mean_ms'] for key in LABELS]
    colors = ['#8094a3', '#648eaa', '#5b9d94', '#d08a3d']
    axes[0].barh(range(4), means, color=colors)
    axes[0].set_yticks(range(4), LABELS.values());axes[0].invert_yaxis()
    for i, v in enumerate(means):
        axes[0].text(v+.06, i, f'{v:.3f} ms', va='center')
    axes[0].set_xlim(0, max(means)*1.22);axes[0].set_xlabel('每组完整输入处理平均耗时 / ms');axes[0].grid(axis='x', alpha=.2)
    final_rows = data['block3_parallel_boundary']
    coarse = [np.mean([r[k] for r in final_rows]) for k in ('repair_ms', 'camera_work_ms', 'merge_ms')]
    axes[1].bar(range(3), coarse, color=['#648eaa', '#5b9d94', '#d08a3d'])
    axes[1].set_xticks(range(3), ['四路补值', '并行工作墙钟\n压缩＋方向＋原图摘要', '统一归并'])
    for i, v in enumerate(coarse):
        axes[1].text(i, v+.005, f'{v:.3f}', ha='center')
    axes[1].set_ylim(0, max(coarse)*1.25);axes[1].set_ylabel('每组平均耗时 / ms');axes[1].grid(axis='y', alpha=.2)
    fig.suptitle('3×3代表观测优化：完整原图与区域最小值查询继续保留', fontsize=14)
    fig.text(.5, .035, '每项600次缓存重建；协调线程CPU0，工作线程CPU1～4；未运行新仿真/FIFO，均值不代表严格2ms保证。', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .07, 1, .94))
    for suffix in ('png', 'pdf'):
        fig.savefig(out / f'optimization_comparison.{suffix}', dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for key, color in zip(LABELS, colors):
        axes[0].plot([r['all_input_ms'] for r in data[key]], lw=.8, label=LABELS[key], color=color)
    for key, label in [('repair_ms', '补值'), ('camera_work_ms', '并行工作墙钟'), ('merge_ms', '统一归并')]:
        axes[1].plot([r[key] for r in final_rows], lw=.8, label=label)
    for ax in axes:
        ax.axhline(2, color='#b95050', ls='--', lw=.8, label='2ms参考线');ax.set_ylabel('耗时 / ms');ax.grid(alpha=.2);ax.legend(fontsize=8, loc='upper right')
        for split in range(100, 600, 100):
            ax.axvline(split, color='#bbbbbb', lw=.6, ls=':')
    axes[1].set_xlabel('离线重复重建编号：每100次分隔一轮/一个存档样本')
    axes[0].set_title('整组处理：各模式分别运行后按样本顺序对齐，非同一墙钟时刻')
    axes[1].set_title('最终四线程角边界模式：互不重叠的墙钟阶段')
    fig.tight_layout()
    for suffix in ('png', 'pdf'):
        fig.savefig(out / f'optimization_curves.{suffix}', dpi=180)
    plt.close(fig)
    print(json.dumps({'mean_speedup': summary['speedup_mean'], 'fast_fraction': summary['fast_fraction'],
                      'final_all_input': summary['stages']['block3_parallel_boundary']['all_input_ms']}, indent=2))


if __name__ == '__main__':
    main()
