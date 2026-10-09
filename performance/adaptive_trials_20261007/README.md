# 本轮消融证据索引

结论以项目根 `ADAPTIVE_OPTIMIZATION.md` 和完成后生成的 `completion_summary.json` 为准。这里的源码、二进制和报告是实验归档，不能把目录存在或某个单项收益视为生产已采用。

- `baseline/`：本轮开始前159份源文件、控制器、重放器及核心静态库的冻结副本；含P3/P4保护清单。
- `candidates_bc/`：B目标切弯、C连续球面评分的162份冻结源与构建产物，完整行为实验共同使用该二进制，通过单变量开关消融。
- `preregistration/`：B/C探索和新确认对的初始方案、原始协议快照与哈希。协议不因结果差而改动。
- `closed_loop/`：本轮12条Isaac运行的日志、运行环境、实际执行文件校验、轨迹、全回调耗时和机体扫掠审计。目标CSV和结果另在项目 `performance/navigation_benchmark_20260929/`，使用新的 `p5_goal_polish_explore_20261007_*` 标签。
- `candidate_goal/`：B细化实现回归、独立审查、方向跳变与超时失败分析；第三条候选240秒超时原样保留。
- `candidate_manual/`：C独立实现、关闭开关等价性、合成角边界测试及真实微扰失败结果。
- `candidate_geometry/`：A的各版独立源、差分、计时与失败结果。所有几何性能实验使用相同37个输入、5批交错重复，不能只选16个卡停快照报告。
- `assessment/`：固定多指标政策、独立评估器、边界回归、数据采集器、正式机器判定；后续确认只有在探索门槛通过后才运行。
- `behavior_rollback.json`：B/C共7份生产源恢复、2份候选专用源归档移出的逐项证明。

复现B/C时必须使用与该次实验匹配的冻结源码、launch和二进制；单独把旧控制器路径指给已回退launch不能激活候选。当前 `tools/run_adaptive_ablation.py` 会拒绝缺失的实验参数，避免静默比较两份基线；原执行脚本也在冻结源码中。恢复、构建和新跑实验仍须遵守AGENTS与工作日志要求，且不得覆盖现有运行ID。

B的driver因到达失败退出1，`manifest.complete=false`是失败记录，不是尚在运行。C只完成微扰探索，因主收益失败提前停止；spin/sweep和确认组未运行。不得把C工具的INSUFFICIENT改写成全协议通过。
