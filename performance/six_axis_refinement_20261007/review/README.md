# 独立代码审查：数值保守性、重放与人工意图

本审查只读取生产代码，修改均由对应实现者进行；审查目录内保存独立小型探针、固定输入及源码快照。未运行主构建、Isaac或密集性能实验。

## 已确认问题

1. **[P1] 共享占据包络预测时长因大时间戳消去而低估。** 审查时 `time = now + end - earliest` 与精确检验 `(now - stamp) + end` 运算顺序不同。`now=stamp=1e9,end=.04,radius=.58` 时精确边界为`.7304`，包络边界`.730399884796`；查询x`.73039994239807138`由精确检查拒绝、加速检查放行。最初直接生产源调用输出见`before_time_fix.log`。实现者已改同序计算，修后快照探针见`intermediate_review.log`，case0已一致拒绝。
2. **[P1] 共享未来时间戳阈值同序问题。** 原包络比较`latest > now + .02`与精确比较`now - stamp < -.02`在浮点边界不等价。`now=1,stamp=1.02`实际age为`-.020000000000000018`，精确检查拒绝，远处包络却可能跳过。实现者已改为`now - latest < -.02`，case1修后已一致拒绝。`pre_fix_reconstructed.log`保存两例修前输出；该对照源码明确标注从修后快照仅复原两条已审查表达式，不冒充未修改的原始冻结。修后快照文件为`shared_review_snapshot.cpp`。
3. **[P1] 球面缓存块级排除在大世界坐标处漏撤销。** `center=Rᵀ*((lower+upper)*.5-origin)`先求世界坐标均值再减origin，padding仅按局部center/half量级，无法覆盖大世界坐标均值的舍入误差。`origin.x=1e9`，两个原始相机点x为`-2`和`-1.9999998807907104`、z同为`2.0424262886619036`，旧90°相机深度全10。精确`observedEnvelope`判定上端点在旧自由空间中，逐点撤销=true，块级撤销=false。见`spherical_roundoff.log`、`spherical_roundoff_probe.cpp`和`spherical_review_snapshot.cpp`。本例当前Cloud坐标约±64不会触发，但接口未限制世界原点，属于块排除的实际保守性缺口。已通知实现者先逐端点减origin再计算包络，或将世界坐标舍入界计入padding，并补边界回归。生产修复及复验由实现者/主任务记录。
4. **[P2] 双级意图平滑可小幅突破原角滞后硬上限。** 更新前检查anchor/raw≤.018rad不能限制更新后的anchor，因为中间stage有自己的滞后。实际header对固定203步输入产生`.018039772220365173`rad误差，超出现有测试`.018001`阈值；见`operator_lag.log`、`operator_lag_angles.txt`和`operator_lag_probe.cpp`。已建议输出后再次检查角差，超限同时重置anchor/stage到原方向。本问题是操作响应契约退化，不是已发现碰撞风险。

## 未发现阻断问题的范围

- **增量残差容限**：相对修正rhs的容限变为绝对目标除以rhs.norm，仍对最终`A*x-b`重新计算真实残差且要求归一化≤1e-7；冷解回退恢复1e-9容限。按本机Eigen头文件，`compute()`会初始化info，因此无需修正迭代时检查solver.info不读未初始化值。建议的rhs.norm<1回归已由主任务补充；该情况绝对目标使用max(1,norm)，不能字面声称完全等于所有冷解rhs的相对阈值，但未改变既有最终有效性契约。
- **identity SE(3)猜测复制**：要求相同尺寸、spacing、offset、内参、位姿，且旧potential/mask长度匹配；只复制有限旧自由像素作为猜测，当前mask/Dirichlet和几何证明仍独立重建。未发现旧自由空间被直接授权的路径。
- **M格式容量锁**：新字段追加在L已有field_position之后，M才读取；L及更早版本保持旧布局且容量锁默认负无穷。NaN/+inf显式拒绝，负无穷作为“无锁”有效。新增正向容量锁往返用例覆盖直接/重放均拒绝。建议补M非法字段/截断回归；21+16历史快照正式兼容复验由主任务执行。
- **动态关联索引**：单轴索引只缩小候选集合，精确三维距离和原索引tie-break保留；正常受界输入下搜索guard扩大区间，没有发现漏选路径。共享包络在上述时间算式修复后，最早stamp、最大绝对速度、最大半径、3m/s误差和加速度界组成原所有轨迹包络的超集。
- **⑤松手/大转向**：零输入重置anchor、stage、previous、delta、coherent和oscillating；大于.018rad及三次一致转向同时设置anchor/stage为原方向，幅值原样恢复。立即松手、大转向路径没有发现新增延迟；第4项仅涉及小角反转时最终输出角差。

## 证据说明

`shared_review_snapshot.cpp`在写入时实现者已完成两处共享修复，因此`intermediate_review.log`实际上验证两例均已修复；保留原文件名避免覆盖已经生成的数据。`shared_pre_fix_reconstruction.cpp`清晰标注只复原两条旧表达式，用于同输入消融。

`spherical_review_snapshot.cpp`和`operator_intent_review_snapshot.hpp`是确认问题时的生产实现副本。review探针不纳入主构建，也不替代对应实现者的正式回归和全量验收。
