# 本轮候选保留协议与独立评估器

本目录仅新增离线工具、预登记模板、工具边界测试及兼容性读数，不修改控制器、旧实验工具或历史结果。主代理负责将变更和测试结果记入 EGO1P5/WORK_LOG.md 与 HISTORY/CHANGELOG.md。

## 先登记，再执行

1. 复制 `plan.template.json` 为候选的正式计划，填全部真实 run ID，预先固定目标/手柄场景、唯一主要收益、运行顺序及 3 对探索、5 对确认。每对使用相同输入/环境；依次 AB、BA、AB，确认阶段另起 AB、BA、AB、BA、AB。不同场景不得当成重复混在一起。每个注册场景都要满足样本数。
2. 把正式计划的文件 SHA256、`policy_sha256`、候选源码/二进制/参数及输入冻结摘要记入外部预登记记录。`registration_evidence` 指向该记录。模板中的占位符、0 个负例和 `freeze_verified=false` 必须在有真实证据后填写。工具能哈希记录以追踪来源，不能凭文件存在证明它确实早于运行；预登记时间和冻结链由运行清单与主审计负责核验。
3. 只运行探索阶段并用 `--stage explore` 分析。主要指标达到实际改善门槛、至少 2/3 配对方向正确、所有安全和性能守门项满足，才返回 `EXPLORE_PROMISING`。这只允许进入确认，不能作为保留/默认开启依据。
4. 冻结同一候选，完成独立 5 对确认，再用 `--stage confirm`。不能复用探索轨迹充数，不能只挑最快一次或失败后删样本。任何修改了源码、参数或输入的候选重新登记。确认未通过不得在看见 p 值后不断追加样本；需要额外样本时另立预登记计划并保留原失败。
5. 最终只在所有硬门和非劣检查满足、主要指标有足够实际收益及独立确认时返回 `CONFIRM_RETAIN`。未通过时保留数据，回退/关闭候选由主工作流执行；工具不会自动改生产代码。

`sequence` 是全候选实际执行的唯一、递增整数，不能从汇总文件排序臆造；清单必须记录真实顺序。工具检查注册顺序、AB/BA交替、同一轮配对先后、重复 ID、探索/确认分离、同一场景跨轮冻结一致。被选分析文件中的未配对运行（包括失败）会保留并阻止接纳。CLI 筛选只用于同一汇总中区分候选与基线；筛选条件写入输出，使用者仍须核对完整运行清单，工具无法发现未提供的文件。

## 判据（开始运行前固定）

每个指标的允许退化取 `max(绝对容差, 基线均值 × 相对容差)`；最小实际改善同理。正的 oriented paired change 一律表示变差，进展/路径效率会自动反向。均值超过容差即退化；确认阶段均值虽没超过、但单侧95%配对 t 上界仍超过容差，返回证据不足。容差允许小幅测量波动，并不要求每个采样点改善。

| 指标 | 允许均值退化：绝对 / 相对 | 作为唯一主要指标时的最低改善：绝对 / 相对 |
| --- | --- | --- |
| 目标到达时间 | 0.20 s / 1% | 0.30 s / 1% |
| 路径效率（固定起终点直线距离 / 实际路程） | 0.01 / 0% | 0.005 / 0% |
| 有输入停车时间 | 0.05 s / 10% | 0.10 s / 10% |
| 最长一次停车 | 0.05 s / 10% | 0.05 s / 10% |
| 实际速度 jerk RMS | 0.02 m/s³ / 5% | 0.05 m/s³ / 10% |
| 完整控制回调 P95 | 0.20 ms / 10% | 0.20 ms / 10% |
| 手柄移动方向平均误差 | 0.25° / 3% | 0.50° / 5% |
| 手柄移动方向 P95 误差 | 1° / 5% | 1° / 5% |
| 沿输入方向累计进展 | 0.10 m / 1% | 0.20 m / 1% |
| 手柄速度矢量误差 RMS | 0.005 m/s / 3% | 0.01 m/s / 5% |
| 松手到执行目标归零 P95 | 1/60 s / 5% | 1/60 s / 10% |
| 松手到实际停止 P95 | 0.05 s / 5% | 0.05 s / 10% |

额外目标尾部守门：候选最慢轮不得比基线最慢轮多 `max(1 s, 基线最慢轮 × 3%)`。到达时间标准差和配对变化标准差完整报告，**标准差不单独用作否决门**，避免基线标准差接近零时比例失效。确认的唯一主要指标还要求单侧精确配对符号检验 `p≤0.05`；平局不删、不算改善。恰好5对时，需要5对都向好才能达到最小 p=1/32，实际改善门槛仍另行要求。这使一次很快的轮次不能抵消多轮无收益。

绝对容差照顾 60 Hz 量化、短停车和接近零的基线；相对容差照顾不同路线尺度。1%且0.3s的到达收益、10%的 jerk/计算收益、5%且0.5°的角误差收益用于阻止把微小偶然差异包装成优化。阈值是本轮预登记的工程决策，不是普适安全标准。t 区间假定独立配对误差近似正常分布，小样本不提供分布无关保证；符号检验也依赖配对独立，模拟帧不是独立重复。报告可用于限定场景的工程取舍，不是实飞或全场景优越性证明。

## 安全与完整性硬门

- 目标每轮 ARRIVED；所有目标/手柄运行完整机体扫掠重叠 0、最小扫掠净空非负、外部保护介入 0。
- 外层进程退出 0，实际日志中所有非零子进程退出（包括关闭阶段 -11）均不能消失。rosout 无效上下文告警与 COMPUTE_DEADLINE 次数单独保留，不将“未在60Hz CSV里采到”理解成日志中未发生。
- 同一 `variant_sha256` 必须关联未知拒绝、过期深度拒绝、过期命令拒绝三项负例契约：非零样本数、unsafe=0、passed=true、实际证据文件。静态无碰撞轨迹不能自动证明这些契约；契约通过也只覆盖该机制测试，不证明真实未知/遮挡/动态事件全部被覆盖。
- 每轨 `freeze_verified=true` 及 SHA256 元数据需要独立运行清单核实。`variant_sha256` 应包含源码、二进制、被实验的配置；`conditions_sha256` 应包含场景、输入、动力学、安全参数、执行源与所有未被实验的条件。前后条件摘要相同，前后 variant 摘要可不同。
- 手柄每轨至少有完整观测的松手阶段，执行目标归零最慢不超过0.05 s。从 CSV 中 q 的模 >0.05 转为 ≤0.05 计为松手；目标阈值1e-6 m/s、实际停止阈值0.05 m/s。要求该松手区间剩余数据持续低于阈值，至少两个采样点。日志结束或再次输入前尚未稳定停止，记录右删失并判证据不足，不能当零延迟。此计量从被记录的意图变化起算，不包含手柄硬件/操作系统到意图发布的延迟。
- 输入不足、NaN/Inf、缺负例、缺完整回调 P95、缺松手段、缺实际顺序均不能作为“通过”。缺失与安全失败分别报告。

## 使用

工具仅依赖 Python 标准库；边界测试使用 pytest。`--before` 和 `--after` 各可指定多个任意位置的既有 analysis JSON，接受目标 `trials` 与手柄 `runs`；汇总均值不能代替逐轨数据。目标路径效率从 `protocol.start/goal` 和 `distance_travelled_m` 计算，固定到达半径由条件摘要保证一致，早停可令该比值略大于1。

```bash
python3 EGO1P5/performance/adaptive_trials_20261007/assessment/assess.py \
  --stage explore \
  --before /absolute/before_goal_analysis.json /absolute/manual_analysis.json \
  --after /absolute/after_goal_analysis.json /absolute/manual_analysis.json \
  --before-filter 'before' --after-filter 'after' \
  --plan /absolute/registered_plan.json \
  --supplement /absolute/supplement.json \
  --output /absolute/candidate_explore_assessment.json
```

确认时换 `--stage confirm`，提供全部探索和确认文件，并使用**新输出文件名**。工具拒绝覆盖旧输出。注册证据、负例证据和补充文件的相对路径都相对 supplement 所在目录解析；没提供 supplement 时相对 plan 所在目录解析。模板默认两个场景，可预登记其他目标或 `manual:spin`、`manual:sweep`；唯一主要指标例如在 `manual:jitter` 中填写 `"primary":"jerk_rms_mps3"`，其他场景不填 primary。

`runs[run_id]` 的补充字段见 `supplement.template.json`。`csv`、`performance`、`log` 可以明确指定；未指定时会查找该 analysis 文件同目录下 `<run_id>.csv`、`<run_id>_performance.md`、`<run_id>.log`。目标轨迹实际在导航基准目录、性能日志在本轮 closed_loop 目录时必须指定对应路径。每个输入和实际使用的补充证据都保存 SHA256。

**计算口径**：`compute_p95_ms` 从性能文件的 `Control callback to publish mean / P50 / P95 / max` 读取；旧 analysis 的 `compute_ms.p95` 只记为 `core_compute_p95_ms`，不能替代完整回调P95。两者都不是端到端相机到实际速度响应的时延。每轨 P95 先独立计算，再做配对；不能把不同长度轨迹的全部帧混合后冒充更多独立样本。

**退出码**：正式否决或证据不足为2；可读出的描述性报告、探索可继续或确认成功为0。因此调用者必须读取 `decision` 字段，不能将进程退出0解释为优化通过。输入格式错误同样非零退出，stderr 应由调用者存档，原文件不变。

不提供 `--plan` 时只生成 `DESCRIPTIVE_ONLY`，安全失败仍显示 `REJECT`。`historical_goal_descriptive.json` 与 `historical_jitter_descriptive.json` 是新工具兼容旧数据的只读检查，不是本轮候选实验：旧目标时间71.3222→70.7556 s、路径效率.957874→.964010，目标完整回调P95缺失；旧手柄完整回调P95 8.4277→7.7613 ms、松手目标归零0→0 s、物理制动P95 .6→.6 s。缺预登记/同冻结负例资料，未获接纳。

## 几何重放机制专用入口

`assess_replay.py` 使用独立 `replay_policy.json`、`replay_plan.template.json`、`replay_input.template.json`。每个 run 是对同一完整固定快照组的一次独立计时批次；3+5 指批次数，不是快照数、模拟帧数或飞行次数。冻结 fixture 摘要、每批离散认证/安全决定的 SHA256 和决定数量必须相同，数值有效性/等价检查显式通过。摘要字段由重放生产者给出，评估器不能从摘要倒推出覆盖内容；生产者必须把负例和所有安全决定放入摘要，连续数值的容差检查另存证据。

```bash
python3 EGO1P5/performance/adaptive_trials_20261007/assessment/assess_replay.py \
  --stage confirm --before old_replay_batches.json --after new_replay_batches.json \
  --plan replay_registered_plan.json --output replay_confirmation.json
```

主指标为各批 replay P95 的配对变化，最低均值收益10%，确认需精确符号检验p≤.05，所有失败/未配对批次保留。最大耗时也报告。机制确认仅返回 `MECHANISM_CONFIRM_RETAIN`，不证明整机到达/操控收益。

几何候选的导航协议可设 `"scope":"guardrails_only"` 并不填任何 primary。此时3对探索最多返回 `GUARDRAILS_EXPLORE_OK`，5对独立确认所有指标非劣才返回 `GUARDRAILS_CONFIRMED`。这一结论只表示保护条件满足；最终保留还必须由主审计将同一冻结几何候选的 `MECHANISM_CONFIRM_RETAIN` 证据绑定，不能单独据此宣称优化。行为候选省略 scope，仍要求唯一导航/手柄主要收益。

## 实际验证与边界

本目录测试运行：`python3 -m pytest -q EGO1P5/performance/adaptive_trials_20261007/assessment`。覆盖安全失败被均值隐藏、未配对失败、复用轨迹、统计不足、冻结漂移、无负例、异常数值、顺序不一致、计算口径混淆、无松手/右删失/反弹、关闭阶段崩溃、几何决定改变与不一致收益。最新结果见 `validation.json` 和 `pytest.txt`。

未由本子任务运行 C++ 主构建或新的 Isaac 轨迹；不验证新的候选确实更优。静态模拟、既有负例和微基准不能覆盖相机标定、真实动态障碍、多机器人通信可靠性或人类主观体验。
