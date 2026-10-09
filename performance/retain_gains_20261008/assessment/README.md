# 本轮无最小收益幅度的独立评估

本目录只新增2026-10-08的工具与证据，不修改2026-10-07冻结政策、失败记录或旧评估工具。`policy.json`落实用户的新标准：可复现正收益没有最小百分比；每项必须独立消融，明确退化不得由其他组收益抵消。八批顺序及bootstrap方法在几何计时前由`../geometry/plan.json`固定；本目录实现完成在数据产生之后，不能称作另一份事前统计预注册。

几何按完整批次配对，37输入是同批观测，不虚增独立样本。对baseline→v2、baseline→v4、v2→combined、v4→combined分别判读；baseline→combined只作额外说明。所有组的wall/CPU mean/P95/max及逐批std、原值、方向、描述性95%区间、pooled结果均保留。区间未调多重比较。区间同侧且至少75%批次同向才标记可复现方向；跨零或方向不足标记未分辨波动。无旧10%/1%门槛，也不强加固定五对确认。

`collect_geometry.py`仅读取现有文件并重算原始统计、配对和方向；bootstrap区间来自已冻结并经审查的`geometry/analyze_ablation.py`，保留它的SHA来源。主入口构建复验使用独立输出文件；同几何源SHA的算术差分测试可关联首轮证据，不能混用两个重放二进制的时延。输出拒绝覆盖。

```bash
cd /home/starry/isaac-data/EGO1P5
PYTHONNOUSERSITE=1 python3 performance/retain_gains_20261008/assessment/collect_geometry.py \
  --directory performance/retain_gains_20261008/geometry \
  --output performance/retain_gains_20261008/assessment/geometry_assessment.json
```

真实最小方案：只有一个组件通过几何时，baseline/该组件在goal、jitter、spin各三对，共18轨；两项和组合均有各自收益时，四变体各场景三次，共36轨。遵循新driver事前顺序，核验实际获得锁后ROS启动次序。三轮四变体交错不是完整四轮Latin平衡，文档和输出须明确。sweep只在扫向能力有额外疑点时追加；失败不能替换成成功样本。没有自动第五对确认要求；存在尚无法分辨的实质负向信号才追加针对性对照。

`collect_tasks.py`支持任意指定同格式analysis，以及自动发现driver默认分析文件。对三个场景分别比较四个必要增量，几何收益可以构成运行收益，真实到达时间/jerk无需同时改善。完整回调时延来自performance.md中的“Control callback to publish”，不冒用核心compute。松手指标为每段release在下一次意图前保持至少两个CSV采样的稳定目标归零/物理制动时间的最大值；无松手或右删失不能记为零。路径效率只用于ARRIVED轨迹。

每轨保留CSV/performance/log路径与SHA、原analysis、进程元数据、实际runtime、variant SHA、conditions SHA及真实序列。variant绑定冻结源归档与控制器；conditions绑定场景、physics相关启动源、输入、六关、callback和goal有效参数。manual没有实际ROS参数dump，不能把“launch+环境一致”写成直接测得全部参数。goal wrapper仅有driver前冻结/后核验installed文件，runtime直接记录的是cloud参数源SHA。生命周期非零子进程退出（包括结束后-11）及COMPUTE_DEADLINE都保留，不等价于已观测碰撞。

三个配对只有27种经验bootstrap重采样，工具穷举计算，避免在Isaac期间做无谓CPU采样。三对同向不是总体非劣证明；混合/缺失指标显式列出，不能写成全面优胜。工具会输出硬门失败、证据不完整及逐指标可复现退化，最终采用须结合几何等价和选定安全负例证据。未观测可复现退化不等于证明所有指标等价。

```bash
PYTHONNOUSERSITE=1 python3 performance/retain_gains_20261008/assessment/collect_tasks.py \
  --manifest performance/retain_gains_20261008/closed_loop/GOAL_LABEL/manifest.json \
             performance/retain_gains_20261008/closed_loop/JITTER_LABEL/manifest.json \
             performance/retain_gains_20261008/closed_loop/SPIN_LABEL/manifest.json \
  --freeze baseline=performance/retain_gains_20261008/baseline \
           v2=performance/retain_gains_20261008/controllers/v2 \
           v4=performance/retain_gains_20261008/controllers/v4 \
           combined=performance/retain_gains_20261008/controllers/combined \
  --contracts performance/retain_gains_20261008/assessment/contracts_reviewed.json \
  --output performance/retain_gains_20261008/assessment/tasks_assessment.json
```

`build_contract_inventory.py`只从已经执行的本轮四库能力测试与明确helper案例冻结登记，不运行测试。选定unknown两个、depth/history expiry两个、stale receipt一个断言；共同Python测试通过和四库C++通过分开解释。`contracts_reviewed.json`缺失时工具状态保持UNKNOWN，任何总体通过数都不能伪装成真实未知事件全覆盖。

工具边界测试入口（不启动基准或Isaac）：

```bash
PYTHONNOUSERSITE=1 python3 -m pytest -q performance/retain_gains_20261008/assessment
```

公共WORK_LOG/HISTORY与生产采用由root统一整合，本目录不并行修改公共记录。
