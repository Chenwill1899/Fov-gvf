# 几何原始批次收集

`collect_replay.py` 将 `candidate_geometry/v*/replay_ablation.json` 和可选 `confirmation_ablation.json` 转成 `assess_replay.py` 所需的逐批数据，保留原始文件及哈希、全部失败、执行顺序和额外 pooled 统计；不运行重放二进制，不改变 policy。

```bash
python3 EGO1P5/performance/adaptive_trials_20261007/assessment/collect_replay.py \
  --explore EGO1P5/performance/adaptive_trials_20261007/candidate_geometry/v4/replay_ablation.json \
  --candidate-id geometry_v4 \
  --plan EGO1P5/performance/adaptive_trials_20261007/assessment/geometry_v4_plan.json \
  --output-dir EGO1P5/performance/adaptive_trials_20261007/assessment/geometry_v4_explore_collected
```

输出目录必须是新目录；上述正式结果已存在，不可覆盖。确认只在预设收益满足后运行，再添加 `--confirm` 并使用新输出目录。`--candidate-source` 和 `--baseline-source` 可指定来源文件，否则使用同级 candidate `paper_guidance.cpp` 与本轮 baseline 源清单。

每个独立样本是一批固定37帧的 P95。原驱动每帧 A/B（或B/A）相邻执行，适配器的 sequence 是该批相应版本的**首次实际执行行索引**；同时保留该批 first/last 索引，明确两个版本是在批内逐帧交错，不虚构先整批A、再整批B。5批×37帧的 pooled P95另报，不把185帧当185个独立重复。

适配器逐项核对原始 returncode=0、全部 stdout 精确一致和生产者 pair 标记一致；解析全部12个输出字段，连续 command/prefix/angle 值必须有限，8个离散字段独立作有顺序的决定摘要。37帧缺失、重复、文件/二进制漂移、次序错误或非有限值均使数值/等价检查失败。原始最大耗时、CPU指标、分组 pooled 汇总保留。源码文件与二进制通过冻结产物关联，适配器不重编译来证明源码至二进制映射。

没有 `--plan` 时可生成事后 run-ID 映射以分析已存在的失败，但 registration_evidence 留空，不能获准保留。v3 使用这种明确标识的事后映射；v4计划于2026-10-07 15:20:43 UTC登记，当时 v4 measurements 尚不存在，且该时刻早于原始 started_unix，正式评估无预登记缺失项。

实际结果：

| 候选 | 每批P95均值，旧→新 | 独立批次 | pooled P95，旧→新 | 原policy结论 |
| --- | --- | --- | --- | --- |
| v3 | 230.099598→307.031901 ms（退化33.4343%） | 5对全部变差；185帧对输出exact | 288.452789→456.855126 ms（退化58.3812%） | REJECT |
| v4 | 229.973069→223.101526 ms（改善2.9880%） | 5对全部向好；185帧对输出exact | 288.084052→279.718833 ms（改善约2.904%） | NO_PRACTICAL_IMPROVEMENT |

v4虽5/5向好、单侧符号p=.03125，仍不足预登记10%的实际收益，停止且不运行确认。两者都未用于生产、未获得新的整机 guardrail 验收。精确数值以各自 `*_explore_collected/{assessment,collection_audit}.json` 为准。

新增9项收集边界测试验证批次数、逐帧顺序、真实输出不一致、相同NaN不能通过、缺帧、哈希变化、阶段分离及事后计划不能接纳。最终全套评估工具64/64通过，见 `final_pytest.log`。不扩大实验数据集。
