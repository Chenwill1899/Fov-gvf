#!/usr/bin/env python3
"""Render all frozen results, including regressions, without selecting best runs."""
import argparse
import json
import re
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('label');a=p.parse_args()
root=Path(__file__).resolve().parent
manual=json.loads((root/(a.label+'_analysis.json')).read_text())
acceptance=json.loads((root/(a.label+'_acceptance.json')).read_text())
goalroot=root.parent/'navigation_benchmark_20260929'
goal=json.loads((goalroot/('omni_'+a.label+'_analysis.json')).read_text())
integrity=json.loads((root/(a.label+'_integrity.json')).read_text())
lines=['# EGO1P3 冻结对照完整数据', '',
       '标签：`'+a.label+'`。本页由逐轮CSV、独立扫掠审计和性能日志生成；探索轮及失败旧冻结组没有混入。', '',
       '**严格预设判据：'+('通过' if acceptance['all_predeclared_checks_pass'] else '未全部通过')+'。**', '',
       '未通过项：'+('、'.join('`'+k+'`' for k,v in acceptance['checks'].items() if not v) or '无')+'。', '',
       '24轮元数据、完整来源目录、冻结源码/二进制一致性：'+('通过' if integrity['integrity_pass'] else '未通过')+'。', '',
       '安全核心明确修改：'+('联合包含证明最大细分深度5→7，原1024预算及包含关系不变；其余受保护文件逐SHA未改。' if integrity.get('intentional_proof_changes') else '无；受保护文件均与P2一致。'), '',
       '## 人工输入等效轨迹', '',
       '表中为各轮指标的均值 ± 样本标准差；恢复工况仅各1轮，用“—”标识无重复估计。',
       '方向误差仅统计有输入且实测速率>0.05m/s的时刻；速度向量RMS包含所有有输入时刻，避免靠停车降低角误差。',
       '有输入停车按速率<0.05m/s统计，包含开始加速和反向起步。沿输入进展为实测速度在归一化输入上的时间积分。',
       'jerk由整个轨迹的实测速率差分得到；必要避障本身也会产生角误差，不能单独按跟随误差判断好坏。', '',
       '| 工况/版本 | 轮数 | 方向误差° | 向量RMS m/s | 沿输入进展m | 停车s | 最长停车s | jerk RMS m/s³ |',
       '|---|---:|---:|---:|---:|---:|---:|---:|']
keys=['moving_angle_mean_deg','active_vector_error_rms_mps','active_projected_progress_m','stop_s','longest_stop_s','jerk_rms_mps3']
def avgstd(values):
    if not values: return '无成功轮'
    return f'{np.mean(values):.3f} ± {np.std(values,ddof=1):.3f}' if len(values)>1 else f'{values[0]:.3f}（—）'
for case in ['sweep','spin','recovery']:
 for algorithm in ['ego1p2','ego1p3']:
  runs=[r for r in manual['runs'] if r['case']==case and r['algorithm']==algorithm]
  lines.append('| '+case+'/'+algorithm+' | '+str(len(runs))+' | '+' | '.join(avgstd([r[k] for r in runs]) for k in keys)+' |')
lines += ['', '## 计算性能和独立安全审计', '',
          '计算均值和p95列分别为各轮日志统计量的平均值，峰值列为所有轮次的最大值；不把“p95均值”说成合并样本p95。',
          '计算截止列为CSV中有输入的COMPUTE_DEADLINE累计时长，区别于重复状态日志的行数。所有运行均关闭RViz。', '',
          '| 工况/版本 | 计算均值ms | 计算p95均值ms | 最大计算ms | 计算截止总时长s | 最小扫掠净距m | 扫掠重叠/保护总数 |',
          '|---|---:|---:|---:|---:|---:|---:|']
for case in ['sweep','spin','recovery']:
 for algorithm in ['ego1p2','ego1p3']:
  runs=[r for r in manual['runs'] if r['case']==case and r['algorithm']==algorithm]
  audits=[json.loads((root/(r['name']+'_audit.json')).read_text()) for r in runs]
  deadline=sum(x['controller_status_seconds_with_input'].get('COMPUTE_DEADLINE',0) for x in audits)
  compute=[np.mean([r['compute_ms'][k] for r in runs]) for k in ['mean','p95']]+[max(r['compute_ms']['max'] for r in runs)]
  clearance=min(r['safety']['minimum_swept_sphere_clearance_m'] for r in runs)
  overlap=sum(r['safety']['swept_sphere_overlap_segments'] for r in runs);blocks=sum(r['safety']['external_collision_blocks'] for r in runs)
  lines.append(f'| {case}/{algorithm} | '+ ' | '.join(f'{v:.3f}' for v in compute)+f' | {deadline:.3f} | {clearance:.3f} | {overlap}/{blocks} |')
lines += ['', '## 固定目标五轮交错对照', '',
          '共同起点(-64,-11.2,2.8)、终点(64,11.2,2.8)，原240秒超时协议；机体半径.48m、速度2m/s、实际加减速度1.2m/s²。',
          '墙钟到达时间取协议计时，不含启动/等待共享锁；完整进程时长另存于原始JSON。', '',
          '| 版本 | 安全到达 | 成功轮仿真到达s | 含失败惩罚耗时s | 成功轮墙钟到达s | 停车s | jerk RMS m/s³ |',
          '|---|---:|---:|---:|---:|---:|---:|']
for algorithm in ['ego1p2','ego1p3']:
 runs=[r for r in goal['trials'] if r['algorithm']==algorithm]
 success=[r for r in runs if r['safe_arrival']]
 values=[avgstd([r['simulation_elapsed_s'] for r in success]),f"{goal['groups'][algorithm]['timeout_penalized_sim_s']:.3f}",avgstd([r['wall_elapsed_s'] for r in success]),avgstd([r['stopped_with_input_s'] for r in runs]),avgstd([r['jerk_rms_mps3'] for r in runs])]
 lines.append(f'| {algorithm} | '+str(len(success))+'/'+str(len(runs))+' | '+' | '.join(values)+' |')
lines += ['', '| 逐轮 | 结束状态 | 结束仿真s | 停车s | 最长停车s | jerk RMS | 最小扫掠净距m | 安全到达 |',
          '|---|---|---:|---:|---:|---:|---:|---|']
for run in sorted(goal['trials'],key=lambda r:(int(r['run_id'].split('_')[-1]),r['algorithm'])):
 lines.append('| '+run['run_id']+' | '+run['status']+' | '+' | '.join(f'{run[k]:.3f}' for k in ['simulation_elapsed_s','stopped_with_input_s','longest_stop_s','jerk_rms_mps3','minimum_swept_clearance_m'])+' | '+('通过' if run['safe_arrival'] else '未通过')+' |')
lines += ['', '| 固定目标/版本 | 计算均值ms | 计算p95均值ms | 最大计算ms | 计算截止总时长s |', '|---|---:|---:|---:|---:|']
for algorithm in ['ego1p2','ego1p3']:
 runs=[r for r in goal['trials'] if r['algorithm']==algorithm]
 computes=[]
 for run in runs:
  matches=re.findall(r'avoid_ms\(mean/p50/p95/max\)=([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+)',(goalroot/(run['run_id']+'.log')).read_text())
  computes.append(list(map(float,matches[-1])))
 lines.append(f'| {algorithm} | {np.mean([c[0] for c in computes]):.3f} | {np.mean([c[2] for c in computes]):.3f} | {max(c[3] for c in computes):.3f} | {sum(r["status_seconds"].get("COMPUTE_DEADLINE",0) for r in runs):.3f} |')
lines += ['', '## 人工轨迹逐轮数据', '',
          '| 逐轮 | 方向误差° | 向量RMS m/s | 沿输入进展m | 停车s | 最长停车s | jerk RMS |',
          '|---|---:|---:|---:|---:|---:|---:|']
for r in manual['runs']:
 lines.append('| '+r['name']+' | '+' | '.join(f'{r[k]:.3f}' for k in keys)+' |')
lines += ['', '## 证据和边界', '',
          '- [机器判据]('+a.label+'_acceptance.json)、[源码/二进制/基线一致性]('+a.label+'_integrity.json)、[人工轨迹全部分析]('+a.label+'_analysis.json)。',
          '- [固定目标全部分析](../navigation_benchmark_20260929/omni_'+a.label+'_analysis.json)、[ROS探针](ros_omni_probe_'+integrity['frozen_version']+'.json)、[核心测试](ctest_'+integrity['frozen_version']+'.log)。',
          '- [六项指标对照图]('+a.label+'_comparison.png)、[首对轨迹示意]('+a.label+'_trajectories.png)。轨迹示意仅用于理解，结论使用全部重复。',
          '- 独立审计采用包围整段位移的.48m机体球对原占据体素和地面校核；未放宽安全包络或执行动力学。',
          '- 全部24轮是静态Cloud脚本/固定目标试验；实际摇杆操作、GUI/RViz下性能、跨场景和真机均未验收。',
          '- 三次重复只能说明这批实测结果，不构成统计显著性或全域最优证明。严格不退步判据逐项照实保留。', '']
(root/(a.label+'_results.md')).write_text('\n'.join(lines))
print(root/(a.label+'_results.md'))
