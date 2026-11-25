#!/usr/bin/env python3
"""Offline repair/scan comparison plus optional completed FIFO session plots."""
import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
from verify_depth_preprocessing import verify_repair, verify_rays
from verify_direction_queries import verify_index

LABELS = {'serial_repair_old': '原串行补值方案', 'parallel_repair_separate': '四路并行补值',
          'parallel_repair_fused': '并行补值＋合并扫描'}


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), mean_ms=float(a.mean()), p50_ms=float(np.percentile(a, 50)),
                p99_ms=float(np.percentile(a, 99)), p999_ms=float(np.percentile(a, 99.9)),
                max_ms=float(a.max()), above_2ms=int((a > 2).sum()), above_2_2ms=int((a > 2.2).sum()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--online-session', type=Path)
    args = parser.parse_args()
    out = args.directory.resolve()
    execution = json.loads((out / 'execution.json').read_text())
    data = {key: [] for key in LABELS}
    for run in execution['runs']:
        assert run['returncode'] == 0
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader((Path(run['directory']) / 'build_timings.csv').open())]
        data[run['label']].extend(rows)
    summary = dict(scope='offline forced rebuilds, 3 rounds x 2 static archived images x 100; separate from FIFO read intervals',
                   stages={key: {column: stats([row[column] for row in rows]) for column in rows[0] if column.endswith('_ms')}
                           for key, rows in data.items()})
    values = [summary['stages'][key]['all_input_ms']['mean_ms'] for key in LABELS]
    summary['relative_to_same_session_baseline'] = dict(speedup=values[0]/values[2], reduction_percent=100*(1-values[2]/values[0]))
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'font.size': 10, 'axes.unicode_minus': False})
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={'width_ratios': [1, 1.2]})
    axes[0].bar(range(3), values, color=['#8094a3', '#5b9d94', '#d08a3d'])
    axes[0].set_xticks(range(3), ['原方案', '并行补值', '并行补值\n＋合并扫描'])
    for i, value in enumerate(values):
        axes[0].text(i, value+.01, f'{value:.3f} ms', ha='center')
    axes[0].set_ylim(0, max(values)*1.2);axes[0].set_ylabel('整组输入处理平均耗时 / ms');axes[0].grid(axis='y', alpha=.2)
    for key, color in zip(LABELS, ['#8094a3', '#5b9d94', '#d08a3d']):
        axes[1].plot([row['all_input_ms'] for row in data[key]], lw=.8, label=LABELS[key], color=color)
    axes[1].axhline(2, ls='--', color='#b95050', lw=.8, label='2ms参考线')
    axes[1].set_xlabel('缓存重建编号：每100次一个存档/轮次');axes[1].set_ylabel('完整输入处理 / ms');axes[1].legend(fontsize=8);axes[1].grid(alpha=.2)
    fig.suptitle('补值与扫描优化：原始邻域规则及所有深度/来源/区域结果保持', fontsize=14)
    fig.text(.5, .025, '每方案600次，CPU0协调／CPU1～4工作；补值核时间包含在并行工作墙钟内，不能重复相加。', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .06, 1, .94))
    for suffix in ('png', 'pdf'):
        fig.savefig(out / f'parallel_repair_comparison.{suffix}', dpi=180)
    plt.close(fig)

    if args.online_session:
        session = args.online_session.resolve()
        result = json.loads((session / 'session_result.json').read_text())
        assert result['passed'] and len(result['cases']) == 3
        summary['online_session'] = str(session)
        summary['online_scope'] = 'completed live FIFO runs; image/ELF/source audits repeated after run; separate from offline rebuild timings'
        summary['online_runs'] = []
        fig, axes = plt.subplots(3, 3, figsize=(14, 9))
        age_fig, age_axes = plt.subplots(2, 3, figsize=(14, 6.8))
        root = Path(__file__).resolve().parents[2]
        frozen_identity = json.loads((out/'build_identity.json').read_text())
        frozen_reader_hash = hashlib.sha256((out/'depth_reader.executed').read_bytes()).hexdigest()
        for col, case in enumerate(result['cases']):
            directory = session / case['label']
            meta = json.loads((directory / 'reader_meta.json').read_text())
            assert meta['actual_scheduler_policy'] == 1 and meta['actual_priority'] == 10
            assert meta['spin_us'] == 200 and meta['affinity_result'] == 0 and meta['direction_depth_queries']['worker_cpus'] == [1, 2, 3, 4]
            assert meta['direction_depth_queries']['worker_affinity_errors'] == [0]*4
            assert meta['direction_depth_queries']['worker_scheduler_errors'] == [0]*4
            assert all(thread['policy'] == (1 if thread['tid'] == case['reader_pid_and_tid'] else 0)
                       and thread['priority'] == (10 if thread['tid'] == case['reader_pid_and_tid'] else 0)
                       for thread in case['thread_policies_after_gate'])
            manifest = json.loads((directory/'manifest.json').read_text())
            identity = json.loads((directory/'build_identity.json').read_text())
            assert identity == frozen_identity
            assert all(hashlib.sha256((root/name).read_bytes()).hexdigest() == sha
                       for name, sha in manifest['source_sha256'].items())
            assert all(manifest['source_sha256'][name] == sha for name, sha in identity.items())
            assert hashlib.sha256((directory/'depth_reader.executed').read_bytes()).hexdigest() == frozen_reader_hash
            assert all(hashlib.sha256(Path(name).read_bytes()).hexdigest() == sha == frozen_reader_hash
                       for name, sha in manifest['installed_module_sha256'].items())
            sim = json.loads((directory/'simulation.json').read_text())
            assert hashlib.sha256(Path(manifest['scene']).read_bytes()).hexdigest() == sim['scene_sha256_after']
            reads = list(csv.DictReader((directory / 'reads.csv').open()))
            frames = list(csv.DictReader((directory / 'frames.csv').open()))
            received = list(csv.DictReader((directory/'received.csv').open()))
            stamps = defaultdict(set)
            for row in received:
                stamps[int(row['image_stamp_ns'])].add(int(row['camera']))
            frame_stamps = [int(row['image_stamp_ns']) for row in frames]
            assert len(set(frame_stamps)) == len(frames) == meta['direction_depth_queries']['index_builds']
            assert all(len(stamps[stamp]) == 4 for stamp in frame_stamps)
            period_ns = round(1e9/manifest['camera_hz'])
            expected_stamps = set(range(frame_stamps[0], frame_stamps[-1]+1, period_ns))
            missing_groups = [dict(stamp_ns=stamp, received_cameras=sorted(stamps[stamp]),
                                   absent_cameras=sorted(set(range(4))-stamps[stamp]))
                              for stamp in sorted(expected_stamps-set(frame_stamps))]
            cal = json.loads((directory/'simulation.sensors.json').read_text())
            width, height = cal['render_width'], cal['render_height']
            independent = dict(rays=verify_rays(directory, cal, width, height), frames={})
            for prefix, frame in [('first', frames[0]), ('last', frames[-1])]:
                repair, filled = verify_repair(directory/f'{prefix}_depth.bin', directory/f'{prefix}_filled_depth.bin', width, height)
                index = verify_index(directory, prefix, cal, filled, width, height, int(frame['image_stamp_ns']), int(frame['sequence']))
                independent['frames'][prefix] = dict(repair=repair, index=index)
            wall = np.asarray([int(row['wall_monotonic_ns']) for row in reads], dtype=np.int64)
            intervals = np.diff(wall)/1e6
            assert np.allclose(intervals, [float(row['interval_ms']) for row in reads[1:]], rtol=1e-9, atol=1e-9)
            assert all(int(b['scheduled_ns'])-int(a['scheduled_ns']) == meta['requested_period_ns']*(1+int(b['skipped_slots']))
                       for a,b in zip(reads, reads[1:]))
            x = (wall[1:]-wall[0])/1e9
            work = [float(row['camera_work_ms'])+float(row['index_merge_ms']) for row in frames]
            publish = [(int(row['published_ns'])-int(row['received_ns']))/1e6 for row in frames]
            online = dict(label=case['label'], directory=str(directory), interval=stats(intervals),
                          read_operation=stats([float(row['read_ms']) for row in reads]),
                          group_pipeline=stats(work), received_to_published=stats(publish),
                          repair_max=stats([float(row['repair_max_ms']) for row in frames]),
                          camera_work=stats([float(row['camera_work_ms']) for row in frames]),
                          index_merge=stats([float(row['index_merge_ms']) for row in frames]),
                          buffer_age_wall=stats([float(row['buffer_age_ms']) for row in reads]),
                          depth_age_sim=stats([float(row['depth_age_ms']) for row in reads]),
                          repeated_reads=sum(int(row['new_frame']) == 0 for row in reads),
                          expected_groups_in_image_stamp_window=len(expected_stamps), missing_groups=missing_groups,
                          missing_group_camera_numbering='zero based: 0 front, 1 left, 2 back, 3 right',
                          source_and_elf_match_offline_and_current=True, scene_sha256_verified=True,
                          independent_data_verification=independent,
                          complete_sets=len(frames), partial_discarded=meta['partial_discarded'],
                          skipped_periods=sum(int(row['skipped_slots']) for row in reads),
                          data_validation_passed=json.loads((directory/'validation.json').read_text())['passed'])
            summary['online_runs'].append(online)
            axes[0, col].plot(x, intervals, lw=.65);axes[0, col].axhline(2.2, color='#b95050', ls='--', lw=.8)
            axes[0, col].set_title(f"FIFO第{col+1}轮：>2.2ms {online['interval']['above_2_2ms']}/{len(intervals)}")
            axes[0, col].set_xlabel('读取墙钟 / s')
            axes[1, col].plot(work, lw=.8, label='补值→归并');axes[1, col].plot(publish, lw=.8, label='完整组收到→发布');axes[1, col].legend(fontsize=8)
            axes[1, col].set_xlabel('新完整组编号')
            axes[2, col].plot([float(row['repair_max_ms']) for row in frames], lw=.8, label='四路补值核最大值')
            axes[2, col].plot([float(row['camera_work_ms']) for row in frames], lw=.8, label='并行工作墙钟');axes[2, col].plot([float(row['index_merge_ms']) for row in frames], lw=.8, label='统一归并');axes[2, col].legend(fontsize=7)
            axes[2, col].set_xlabel('新完整组编号（补值核与工作墙钟重叠）')
            age_axes[0,col].plot((wall-wall[0])/1e9, [float(row['buffer_age_ms']) for row in reads], lw=.75)
            age_axes[0,col].set_title(f'第{col+1}轮：完整组 {len(frames)}/{len(expected_stamps)}')
            age_axes[1,col].plot((wall-wall[0])/1e9, [float(row['depth_age_ms']) for row in reads], lw=.75)
            age_axes[1,col].set_xlabel('读取墙钟 / s')
        for row, label in enumerate(('读取间隔 / ms', '接收端整组耗时 / ms', '各阶段耗时 / ms')):
            axes[row, 0].set_ylabel(label)
        for ax in axes.ravel():
            ax.grid(alpha=.2)
        fig.suptitle('在线FIFO10＋200μs：相机500仿真Hz，0.02倍速，每轮0.2s仿真', fontsize=14)
        fig.tight_layout(rect=(0, 0, 1, .96))
        for suffix in ('png', 'pdf'):
            fig.savefig(out / f'online_fifo_verification.{suffix}', dpi=180)
        plt.close(fig)
        age_axes[0,0].set_ylabel('已发布帧墙钟年龄 / ms')
        age_axes[1,0].set_ylabel('图像相对/clock年龄 / 仿真ms')
        for ax in age_axes.ravel():
            ax.grid(alpha=.2)
        age_fig.suptitle('在线数据年龄：0.02倍速下2ms仿真采集间隔约对应100ms墙钟', fontsize=14)
        age_fig.tight_layout(rect=(0, 0, 1, .95))
        for suffix in ('png', 'pdf'):
            age_fig.savefig(out / f'online_fifo_data_age.{suffix}', dpi=180)
        plt.close(age_fig)
        summary['online_aggregate'] = dict(intervals=sum(r['interval']['count'] for r in summary['online_runs']),
            over_2_2ms=sum(r['interval']['above_2_2ms'] for r in summary['online_runs']),
            skipped_periods=sum(r['skipped_periods'] for r in summary['online_runs']),
            complete_sets=sum(r['complete_sets'] for r in summary['online_runs']),
            expected_sets=sum(r['expected_groups_in_image_stamp_window'] for r in summary['online_runs']),
            all_intervals_at_most_2_2ms=all(r['interval']['above_2_2ms']==0 for r in summary['online_runs']),
            strict_every_interval_at_most_2ms=all(r['interval']['above_2ms']==0 for r in summary['online_runs']))
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary['relative_to_same_session_baseline'], indent=2))
    if args.online_session:
        print(json.dumps(summary['online_aggregate'], indent=2))


if __name__ == '__main__':
    main()
