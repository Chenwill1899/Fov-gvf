# 受限保留报告草稿（新确认结果待填）

**状态：原 36 轨整机消融和编译期隔离验证已完成；新 6 条 goal 确认及 1 条默认 spin 验证尚未在本草稿结案。不得据此写成最终保留完成。** 本文件只汇编已有证据，没有重新编译、计时或运行实验。最终结论由主线程结合新确认结果填写。

## 标准与原 36 轨结论

本轮按用户新标准取消 10% 最小收益门槛：任意幅度的可复现收益可考虑保留，但每项优化及组合增量都须单独消融，有可复现能力退化即放弃该采用范围。局部计算收益不能抵消实际跟随、平滑性或到达退化。

原设计为 goal、jitter、spin 三类输入 × baseline/v2/v4/combined 四变体 × 三次重复，共 36 轨。原场景、物理、六项开关全关及 callback 执行源保持一致；36 次实际控制器身份均核验。[任务评估](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/assessment/tasks_assessment.json)及[独立复核](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/assessment/tasks_review.json)明确判定 `REPRODUCIBLE_TASK_REGRESSION`。

| 必要逐项 / 边际对比 | goal 的可复现退化 | jitter 的可复现退化 | spin 的可复现退化 |
|---|---|---|---|
| baseline → v2 | jerk RMS、P95 | 向量误差、进展 | 未观察到一致退化，仍有未分辨波动 |
| baseline → v4 | 未观察到一致退化，见下一节全部指标 | 角误差 P95、向量误差、进展、计算 P95 | 平均/P95角误差、向量误差、进展、jerk RMS、计算 P95/最大值 |
| v2 → combined（增添 v4） | 未观察到一致退化 | jerk RMS | 平均角误差、向量误差、jerk RMS |
| v4 → combined（增添 v2） | jerk RMS | jerk RMS | 未观察到一致退化，仍有未分辨波动 |

额外 baseline → combined 对照亦有 goal jerk RMS、jitter 角误差 P95、spin 平均角误差/向量误差/进展/计算最大值退化。因此 **v2、纯 v4、组合均不能全局采用**。例如 v4 spin 的 jerk RMS 均值 2.8232 → 4.0730 m/s³（约 +44.27%，3/3 变差），计算最大值均值 34.593 → 287.283 ms，其中一次为 618.326 ms；第三次额外停车仅一次出现，不能写成 3/3 长停。

这与离线重放计算收益不冲突：同 37 输入的主构建 8 批中，完整输出全同，v4 每批 P95 均值改善约 3.04%（8/8）；实际异步闭环并不保证各版收到相同时间的观测。当前尚未单独测出上述整机退化的因果机制，不能把调度时序或缓存开销写成已证实根因。离线逐项收益、全部负点估计及区间见[阶段报告](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/IMPLEMENTATION_AND_REPLAY.md)；其“36 轨仍进行中”是写作时的历史状态，本草稿补充原 36 轨已完成的结论。

## 纯 v4 goal：原三对收益及全部不利点

原目标组 baseline 与 v4 均 3/3 到达。jerk RMS 的三对为 1.6062/1.5832/1.6274 → 1.5616/1.5652/1.5813 m/s³，3/3 改善，均值下降约 2.26%。但到达时间、路径效率、计算最大值的均值均有不利点估计，不能省略。

下表“收益”统一正数为更好；区间为三对样本的描述性精确经验配对 bootstrap 95% 区间，未作多重比较校正。路径效率收益取 after-before，其余取 before-after。方向列为改善/变差/相同次数。

| 指标 | baseline → v4 均值 | 绝对收益 [95%区间] | 方向 |
|---|---:|---:|---:|
| 到达时间 / s | 69.2278 → 69.2389 | -0.0111 [-0.0392, +0.0131] | 1/1/1 |
| 路径效率（越大越好） | 0.975063 → 0.974823 | -0.000240 [-0.000790, +0.000219] | 2/1/0 |
| 停车总时间 / s | 0.2333 → 0.2333 | +0.0000 [+0.0000, +0.0000] | 0/0/3 |
| 最长停车 / s | 0.2333 → 0.2333 | +0.0000 [+0.0000, +0.0000] | 0/0/3 |
| jerk RMS / m/s³ | 1.6056 → 1.5694 | +0.0363 [+0.0238, +0.0458] | 3/0/0 |
| jerk P95 / m/s³ | 3.1179 → 3.0712 | +0.0467 [-0.0582, +0.1434] | 2/1/0 |
| 计算 P95 / ms | 8.0993 → 7.9483 | +0.1510 [+0.0147, +0.3037] | 2/1/0 |
| 计算最大值 / ms | 40.4737 → 41.3317 | -0.8580 [-9.6131, +5.9290] | 2/1/0 |

具体不利样本也保留：第 2 对到达慢 0.0500 s、路径效率下降 0.00101354；第 3 对 jerk P95 增加 0.09375 m/s³、计算 P95 增加 0.0100 ms；第 1 对计算最大值增加 13.590 ms。jerk P95 和计算 P95 的均值虽更好，也不能写成三对一致改善。三对全同方向的单侧符号检验 p=0.125；这些是小样本工程信号，不是总体非劣或普遍收益证明。

**只在固定目标模式尝试 v4 是看完原 36 轨后形成的新假设。** 原三对不能冒充提前设定的独立确认，因此另冻结[新确认计划](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/mode_isolation/confirmation_plan.json)，新旧数据分开报告。

## 隔离实现与默认部署范围

同一份 [paper_guidance.cpp](/home/starry/isaac-data/EGO1P5/src/pc_gvf/src/paper_guidance.cpp:64) 通过 `PC_GVF_GOAL_TUBE_REUSE` 编译两份内核。goal 版仅在递归联合认证且 tube>8 时按原逆序预计算 `b-a` 和平方长度，保留原投影除法、clamp、norm、异常回退、证据顺序、两球联合、7层和1024预算；**没有 v2 根盒筛选**。人工版预处理后为原认证代码，无新增运行期模式分支或缓存分配。

[CMake](/home/starry/isaac-data/EGO1P5/src/pc_gvf/CMakeLists.txt:62) 生成 baseline 与 goal 两套 core/controller/replay；宏为 goal core 私有，每个可执行文件只链接对应 core。未新增 PaperConfig 字段、未改变 ABI 或快照格式；旧 M 快照仍可被两份 replay 读取，但必须用 manifest/SHA 标明实际内核。

主 Shell 的 P5 固定目标入口按真实最终 `use_fixed_goal=True` 默认选择 `depth_angular_controller_goal`；Cloud 人工/手柄入口默认保持 `depth_angular_controller`。显式 `FOV_GVF_CONTROLLER_EXECUTABLE` 仍优先供消融，并记录实际执行文件身份。**旧 `depth_angular_demo.launch.py` 虽为固定目标模式，仍明确保留 baseline，是本次采用范围之外的例外。** 这项路由实现已部署供确认，不等于最终收益验收已完成。

三种安装后路由检查通过：默认人工→baseline、P5默认固定目标→goal、固定目标显式baseline覆盖→baseline；默认人工 runtime manifest 也记录实际路径及SHA。[路由证据](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/mode_isolation/launch_routes_v2/summary.json)仅验证启动描述构造和身份，不冒充 Isaac 执行结果。

## 六份产物身份与已有验证

下列安装产物 SHA 均与此前实际测试的 baseline / 纯 v4 完全一致，覆盖两份 controller、两份 replay 和两份静态库；不是仅“源码近似相同”。[身份清单](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/mode_isolation/binary_identity.json)和[部署冻结](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/mode_isolation/deployment/binary_sha256.json)保存完整路径。

| 部署产物 | 对照 | SHA-256 | 完全一致 |
|---|---|---|---|
| manual / depth_angular_controller | baseline | `4b82e8d3c1381b8d9b25af5510cf1d11dfaafb3f99125ffffeecffea64506c15` | 是 |
| manual / paper_replay | baseline | `b518bb7461b93cb12ecf4bb02ea0f2eb9f84870125a47b1ee8c8d060fe6d9683` | 是 |
| manual / libpc_gvf_depth_angular_core.a | baseline | `8bd00e8cee30082c4e28fabfb56d43f247de5d865da9a3ea76ff7942bdde5723` | 是 |
| goal / depth_angular_controller | 已测纯 v4 | `2569161496747e8fa5e10cb3dfd51233dfd8c34b33aa68d7c2d4f8a766fd1047` | 是 |
| goal / paper_replay | 已测纯 v4 | `5fbf5493fee3102a0dbd14449e2c6a94c6705504581670507daca4c246dfe3c8` | 是 |
| goal / libpc_gvf_depth_angular_core.a | 已测纯 v4 | `6ccc2734d82c9b1698195c42a153cc0dc0cdad272bf046f3f68da1f8ab43412a` | 是 |

- 隔离主构建后的核心回归 **36/36**，含新增 goal 库对应检查：[CTest日志](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/ctest_mode_isolation.log)。
- 平台检查为本轮前次 **15/15**：[日志](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/platform_candidates.log)；未把它写成隔离后重新执行。
- 评估工具 **47/47**、运动审计工具 **18/18**：[评估日志](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/assessment/tool_tests_final.log)、[运动工具日志](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/assessment/motion_limits_tool_tests.log)。
- 原四版冻结库各 **5/5** 能力检查；未知、过期深度、过期指令拒绝的选定负例通过。几何谓词百万级差分、主构建 296 个四变体案例/888 项全输出等价证据见[实现与重放报告](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/IMPLEMENTATION_AND_REPLAY.md)。这些是具体回归范围，不证明任意环境。
- 原 **36/36** 轨完整机体扫掠重叠 **0**、外部保护介入 **0**，12条 goal 全到达，实际控制器身份36/36核验，无评估硬失败或完整性缺项；参见 tasks_review。原人工轨迹没有直接ROS参数dump，依据冻结launch/环境及实际二进制身份复核。
- 原 **36/36** 运动约束审计通过：[motion_limits_assessment.json](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/assessment/motion_limits_assessment.json)。覆盖全部行及相邻有效区间，含起步/松手/停车；速度≤2 m/s、垂直速度≤1 m/s、实际加速度≤1.2 m/s²及执行目标速度约束（浮点容差1e-6）。该审计不设jerk或目标变化率上限，不能抵消上述操控退化。
- 关闭阶段各版各有9处 `rosout invalid context` 文本；`COMPUTE_DEADLINE` 文本出现数 baseline/v2/v4/combined 为0/3/9/9，各版非零子进程退出为空。这是日志出现次数，不能冒充独立控制周期数或无警告运行。

原六项修后实现仍保留，主算法开关默认均0；callback样本绑定保持默认启用。详见[保留清单](/home/starry/isaac-data/EGO1P5/performance/retain_gains_20261008/inventory/README.md)。本轮几何隔离没有把动态、多机或盲区能力扩大为已验证实飞结论，P3/P4不在修改范围。

## 新确认阶段（待主线程填入）

计划固定为新三对 goal：baseline→v4、v4→baseline、baseline→v4，共6轨；另1轨默认人工 spin 验证真实默认路由。输入、安全/运动门槛与无最小幅度规则不变；若仍有不利信号则放弃本受限假设，不再以重放收益抵消。

| 待填内容 | 本草稿状态 |
|---|---|
| 新6条goal逐轨身份、到达、安全与运动审计 | 待最终汇总 |
| 新三对全部指标、方向/区间及原三对分开列示 | 待最终汇总 |
| 默认spin实际baseline SHA、扫掠/外部保护、运动与生命周期 | 待最终汇总 |
| goal受限保留或回退决定、最终默认启动命令 | **尚未定案，不预写采用成功** |

本草稿仅新增此文件；主线程负责最终总报告及 `WORK_LOG.md`、`HISTORY/CHANGELOG.md` 同任务记录。新确认日志即使已有部分结果，也须完整审计后再替换本节。
