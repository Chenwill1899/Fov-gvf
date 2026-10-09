# 新驱动结果收集适配器

`collect_adaptive.py` 只读 `run_adaptive_ablation.py` 产生的清单、`_process.json`、实际 runtime/parameters JSON、逐轨 analysis、CSV、性能文件和日志；在**新目录**生成 `supplement.json`、`before_analysis.json`、`after_analysis.json`，可同时调用已登记的 `assess.py` 生成 `assessment.json`。不修改 policy、运行驱动、生产代码或旧证据，不启动测试、构建或 Isaac。

完整目标探索的一条命令：

```bash
python3 EGO1P5/performance/adaptive_trials_20261007/assessment/collect_adaptive.py \
  --manifest EGO1P5/performance/adaptive_trials_20261007/closed_loop/p5_goal_polish_explore_20261007/manifest.json \
  --freeze EGO1P5/performance/adaptive_trials_20261007/candidates_bc \
  --plan EGO1P5/performance/adaptive_trials_20261007/preregistration/goal_polish.json \
  --stage explore \
  --output-dir EGO1P5/performance/adaptive_trials_20261007/assessment/goal_polish_explore_collected
```

手柄换为相应 closed_loop manifest 和 `preregistration/manual_continuous.json`。多个已完成场景/阶段通过 `--manifest` 追加；目标 analysis 自动从 result_path 推导，手柄自动找同目录 `<LABEL>_analysis.json`。可用 `--analysis` 显式列出全部文件。确认阶段包含探索与确认所有 manifest，并切换 `--stage confirm`。`--before-freeze` 可单独指定旧冻结；不同冻结不能自动继承当前 BC 的安全契约，需另提供经审查的 `--contracts`。

输出目录已存在时拒绝覆盖。未完成 manifest 默认拒绝；`--allow-incomplete` 仅用于诊断快照，会把整个集合标记证据不足，不能保留候选。实际输出的 decision 才是评估结论，进程成功解析数据不等于优化通过。

## 实际核验与摘要含义

- 源码压缩包逐文件 SHA256 与 `source_sha256.json` 对照；二进制在冻结、驱动 manifest、process、实际 runtime/parameters 以及文件本体间一致。
- actual runtime 的 `launch_sha256` 核对 cloud launch；实际安装目录选中的 cloud 或 navigation benchmark launch 文件也与驱动冻结 SHA 核对。目标 parameters 中 actual effective 的 trial 参数必须等于捕获环境的0/3或0/1；common_source_sha256 也与驱动冻结匹配。
- 驱动 `file_sha256` 中每个 runtime 文件当前内容均核对；源清单、归档与实际二进制是有来源的冻结配套产物，适配器没有重新构建以证明源码至二进制的可复现映射。
- **边界**：手柄 runtime 只记录二进制和 cloud launch SHA，没有实际 ROS 参数 dump；手柄 trial 参数依据相同冻结 launch 和驱动捕获环境推导，不能声称已独立观测实际参数。目标 runtime 的 launch SHA 是 cloud 参数源；benchmark 包装器的安装文件 SHA 是运行后的文件核验，不是包装器运行时的独立 `__file__` 记录。这些局限写入每轨 collection audit。
- `variant_sha256` = canonical JSON（冻结源码清单摘要、源码归档摘要、控制器二进制 SHA、六开关、两项 trial 选项的显式/默认值）的 SHA256。
- `conditions_sha256` 包含同一场景/trace/protocol、未被实验的环境、完整 runtime 文件冻结、六开关和目标 effective 参数；只排除两项被实验选项、运行ID与输出路径。实际安全半径/动力学等非实验参数变化会令配对条件不一致。
- 顺序来自**取得仿真锁之后**的 `[launch]: All log files can be found below .../YYYY-MM-DD-HH-MM-SS-microsecond-...` 时间；不能用获取锁之前的 PERF Started、driver START 或文件名排序替代。多个 manifest 合并后按实际 launch 时间赋唯一序号。
- 清单与 `_process.json` 不一致、孤立 process、缺逐轨分析、重复 ID、源/二进制/launch 漂移都阻止 freeze_verified。未提供的分析或运行仍无法由工具凭空发现，应与完整驱动记录核对。

## 已审查的有限安全负例

`contracts_bc_reviewed.json` 仅绑定本轮 `candidates_bc/source_sha256.json` 与控制器二进制，引用 `contract_evidence/` 中保留的原日志和冻结源码副本。

| 契约 | 数目 | 具体执行断言 |
| --- | ---: | --- |
| unknown_rejection | 2 | `paper_guidance_check` 的 `unobserved near field accepted` 和 `unknown inside body footprint accepted`，均是未满足拒绝才报错的负例 |
| expired_depth_rejection | 2 | `paper_guidance_check` 的 `stale observation accepted` 与 `observed_space_check` 的 `expired history authorized motion` |
| stale_command_rejection | 1 | `test_timed_sample_repeats_without_refreshing_receipt_or_sequence` 中接收时刻10.0、选择时刻10.101时目标归零，超过0.1s期限 |

C++ 两个测试在本轮原始 CTest 日志明确 Passed；死手 helper 本轮也 Passed，保留的 XML 核对具体 pytest case，整个 helper 39项通过但**不将39冒充39个过期命令负例**。这些共享安全组件的测试通过，不能当作每个 candidate 开关组合分别完成的端到端传感器故障注入，更不能证明全部真实未知、遮挡或时间故障。

原始 CTest 30/32（两个 Python 项因 NumPy 用户环境失败）、PYTHONNOUSERSITE 后2/2重试、平台两次导入环境失败和最终15/15日志全部保留。它们分别记录环境问题、组件测试与重试结果，不改写为首次全通过。

## 本次收集与工具测试

- 适配器新增9项 pytest 边界测试：选项/条件摘要分离、实际安装 launch 漂移、process与manifest不一致、未完成记录、缺分析、post-lock时间、归档失配、无关输出路径与重复运行。
- `adapter_partial_goal_snapshot/` 是目标仍在运行时显式 `--allow-incomplete` 的5轨快照，仅检验解析和冻结字段，不是最终B验收。
- `manual_continuous_explore_collected/` 为已完成C jitter 3对探索的正式收集。工具整体 INSUFFICIENT 是因为预登记 spin/sweep 尚未执行；已测主要角误差9.2001777097→9.3419463069°，实际改善门槛未达到且仅1/3对向好。可以据此提前停止无收益候选，不能声称整个预登记协议或未跑场景已完成。

源/测试文件哈希、已执行验证及本轮收集链接见 `adapter_validation.json`。
