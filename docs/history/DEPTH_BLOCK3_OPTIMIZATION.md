# 3×3代表观测、旋转视线与四线程优化

2026-11-03：已按用户要求接入C++接收端。最终离线整组平均 **0.645539ms**，同机对照原逐像素实现 **8.935458ms**，约快 **13.84倍**；P99为0.862309ms。600次仍有1次超过2ms，最大2.215624ms，因此未宣称严格逐次2ms通过。本次没有启动新Isaac或FIFO仿真。

## 当前数据与接口

每路对**补值后的全分辨率D_filled**按图内不重叠3×3分块，仅将块内有限正深度以double累加、求均值后存为float32。每个有效块生成一个代表观测；全异常块记录`valid_pixels=0`、内部均值NaN，不进入方向网格，CSV均值留空。

块范围为原图半开矩形`[u0,u1)×[v0,v1)`。宽/高不足3的边缘块保留；中心坐标是`((u0+u1-1)/2,(v0+v1-1)/2)`，因此偶数宽/高的边缘中心可能为半像素。320×240每路107×80＝8560块，四路最大34240块；右边缘2×3块不会丢弃。原始307200像素、D_filled和原始二维b/d表都继续保留。

当前存档样本中每组实际有效代表33511个：

| 相机 | 有效代表 | 快速角边界判格 | 精确角度回退 |
|---|---:|---:|---:|
| Front | 8544 | 8464 | 80 |
| Left | 8391 | 8313 | 78 |
| Back | 8045 | 7965 | 80 |
| Right | 8531 | 8451 | 80 |
| 合计 | **33511** | **33193（99.051%）** | **318** |

729块全异常，几何排除0。补值后有限正深度像素仍为298795个；这个数是原图数值统计，不再等于方向来源数。方向格有观测22373格、无观测42427格；块代表只占其自身中心表面方向，不把整个原图块对应的角区域填成已观测。

`DepthObservationView::observation(cell,k)`现在返回相机编号、中心`u/v`（double）、块均值`optical_z`、原图`u0/v0/u1/v1`、块内可用/总像素数、同步组时间戳和组序号。仍保留同格所有相机/块来源，不跨相机比较Z取最近，也不跨盲缝插值。

**单像素与原图区域查询的含义保持**：`pixel`读取全分辨率D_filled；`region`在原图坐标范围内求已有有限正深度的精确最小值，16×16摘要及边界块精确扫描仍在。查询不改用3×3均值。原始D也不被改写。均值代表与块最小值是不同数值：新增测试中块均值9m，而同块原图区域查询仍返回最小值1m。

## 实现与线程

主要代码：[构建与查询](../../src/new_controller/include/new_controller/depth_observations.hpp)、[预计算角边界](../../src/new_controller/include/new_controller/depth_direction_boundaries.hpp)、[常驻相机工作线程](../../src/new_controller/include/new_controller/depth_camera_workers.hpp)、[存储](../../src/new_controller/include/new_controller/depth_observation_storage.hpp)。

初始化为每个块中心计算`b=K^-1[u,v,1]`、`q=R_BC*b`及q方向的参考角格。K/T、图像尺寸、角分辨率和构建设置不变时，`configure`直接复用表及线程，不重新生成。每帧仅用块均值Z执行`P_B=Z*q+t_BC`。内参/外参改变会使配置编号变化，旧索引不能被同组复用误认作新索引。

角格边界的sin/cos在初始化计算。通常先验证预计算的参考格，再检查相邻两格；方向因相机平移/深度发生较大变化时才对边界二分搜索。用相对叉积保护阈值`1e-10`识别近边界情况，回退到原`atan2`及`layout.cell`规则。接缝、极点、极大/极小数值也有精确回退；没有采用近似atan2或改变角分辨率。

四路各有一条常驻普通调度工作线程，每组只唤醒一次。每路独立完成块均值、代表方向及**全分辨率区域摘要**，写自己的预分配数组；全部完成后，接收端一次归并格计数/前缀和/来源列表。顺序固定为相机编号再块编号，不因线程完成顺序改变。稳态无逐像素锁/原子操作、无每帧创建线程或申请图像/索引内存；只有组级分发/完成同步。

线程CPU从进程允许集合中选择，并明确剔除读取CPU及其完整SMT同胞集合，优先选择不同物理核心。本机四路工作线程为CPU1、2、3、4，保留CPU5/11给读取核心；工作线程显式`SCHED_OTHER`，不提升为FIFO。可用CPU不足4个或亲和性/调度设置失败时拒绝启动，避免静默占用读取核心。

原严格四路同步器已拒绝完成时间戳之前的重复/迟到图像。构建端另以“组序号＋时间戳＋配置编号＋ready”检查同组索引，重复调用直接复用；2ms读取线程仍只引用已经完成的保护槽，不调用构建，也不等待相机工作线程。原/补值图、块均值和CSR来源位于同一受保护帧，完成所有工作后才发布。

## 离线对照与计时

原始数据为历史`privileged_20261102_181620/fifo10_200us`静止场景的首末图。每种模式三轮、首末各100次，共600次，预热每轮每图10次；各模式分开串行运行并轮换次序。缓存数据重复重建，不是600组新相机图。

CPU为Ryzen 5 9600X，GCC11.4 Release/-O3/-fno-fast-math。协调线程固定CPU0，工作线程CPU1～4、普通调度；所有本任务构建、CTest及Sanitizer在计时前结束。没有全机CPU/SMT/中断/频率隔离，桌面负载仍存在。运行命令、源码身份及ELF SHA见[最终执行清单](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/merged_execution.json)。原逐像素基线使用修改前冻结的真实可执行文件。

| 模式 | 平均/ms | P99/ms | 最大/ms | >2ms/600 |
|---|---:|---:|---:|---:|
| 原逐像素索引 | 8.935458 | 9.988872 | 13.582840 | 600 |
| 3×3＋旋转视线，串行精确角度 | 1.608284 | 1.927170 | 2.538163 | 5 |
| 3×3＋旋转视线，四线程精确角度 | 0.688890 | 1.028129 | 3.070283 | 4 |
| **四线程＋角边界判定（当前）** | **0.645539** | **0.862309** | **2.215624** | **1** |

整组范围包含全分辨率异常补值、块压缩、方向整理和全分辨率区域摘要；不含初始ROS消息复制、渲染/DDS/同步等待、启动LUT/线程创建、后续查询或导出。均值改善同时包含用户授权的代表观测降采样、预计算、更多CPU并行和角边界优化，不能归为相同逐像素算法的纯加速。

![各级优化对照](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/optimization_comparison.png)

最终互不重叠墙钟阶段平均：补值0.186744ms、四路并行工作/同步等待0.366140ms、统一归并0.092569ms，余下约0.000086ms为入口检查/计时边界。并行工作包含各路压缩、方向与全分辨率摘要，不能把四路耗时相加当墙钟。

| 单路核平均/ms（彼此并行） | Front | Left | Back | Right |
|---|---:|---:|---:|---:|
| 块均值压缩 | 0.045081 | 0.044134 | 0.045533 | 0.050007 |
| Z*q+t、判格和来源暂存 | 0.181629 | 0.156799 | 0.172526 | 0.183910 |
| 全分辨率区域摘要 | 0.102172 | 0.091627 | 0.102609 | 0.083464 |

![逐次耗时与阶段曲线](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/optimization_curves.png)

本次1次>2ms指**接收端完整重建耗时**，不是一次2ms读取线程唤醒间隔。没有同时内核trace，不能逐次断言该长尾由哪个进程/中断造成；也不将此次短离线样本当在线2ms上界。

开发对照均保留：初版两阶段唤醒/归一化边界方式整组平均0.736394ms，慢于其精确角度对照0.661615ms；简化叉积和相邻格检查后，两阶段版本0.709289ms、精确角度0.698149ms。最后将每路压缩/方向/摘要合成一次任务，得到上表当前结果。前两轮`execution.json`、`final_execution.json`及全部CSV没有被覆盖。

## 核对与输出格式

- 最新4包Release构建成功，CTest **4/4**；现有零稳态分配、4000直接扫描ROI对照及1000保护槽并发组检查继续通过。
- 新`depth_blocks_check`核对 **1,007,285组**随机/极值/角边界方向，快判与精确结果全部相同；测试输入包含多个角分辨率、接缝、极点、跨多个格的参考格偏移；近边界和极小尺度精确回退均实际执行。
- 新测试覆盖边缘2×1/2×3块、NaN/Inf/零/负值混合及全异常块、half-pixel中心、skew内参、旋转/平移、标定复用及失效、1000次同组复用、CPU/SMT排除、串行精确与并行快速结果一致。仅允许CPU5/11时，工作线程启动按预期拒绝（退出2），未占用保留核心。ASan＋UBSan通过；关闭LSan以适应已知工具ptrace限制，不声称泄漏检测通过。
- 最终24个输出目录都通过独立NumPy验证：全部块的边界/中心/均值/可用数、全部方向来源、覆盖格、1200区域摘要与25种查询；浮点边界格差异0。
- 所有新模式与冻结v1的**原始图、补值图、b/d、区域摘要CSV和查询示例CSV逐字节相同**。新模式之间的完整有序CSR、覆盖与块来源记录逐字节相同；与v1的方向来源单元不同是本次明确更改的3×3代表观测语义。

新索引JSON版本为`ego4p0_direction_queries_v2_blocks`，来源码为`camera*blocks_per_camera+block_v*block_columns+block_u`；新增`*_block_observations.csv`导出全部块的原图范围、中心、均值、计数和组身份。不能用旧`camera*pixels+pixel`解码v2来源。独立校验脚本兼容旧v1和新v2；历史v1复制循环profiling工具已加版本拒绝，旧冻结二进制和头文件包继续保留。

新计时列为`camera_work_ms`、`index_merge_ms`，分别表示四路并行工作墙钟和后续统一归并；各相机`compression_ns/projection_ns/region_camera_ns`是重叠的单路核计时。分析脚本按版本区分，不能沿用旧方向/区域串行时间相加。

[统计JSON](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/summary.json) · [对照CSV](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/comparison.csv) · [全部独立核对](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/independent_comparison.json) · [首组逐次计时](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/merged_r1_first_block3_parallel_boundary/build_timings.csv) · [块来源示例](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/merged_r1_first_block3_parallel_boundary/sample_block_observations.csv) · [对照图PDF](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/optimization_comparison.pdf) · [曲线PDF](../../../EGO4P0/docs/performance/depth_block3_20261103_133230/optimization_curves.pdf)

复现构建/离线最终模式（输出目录必须新建；计时期间不并行编译或测试）：

```bash
cd /home/starry/isaac-data
EGO4P0_BUILD_TESTING=ON bash EGO4P0/scripts/build_isaac_ros_workspace.sh
ctest --test-dir EGO4P0/docs/performance/runtime/build/new_controller --output-on-failure
taskset -c 0-4,6-10 \
  EGO4P0/docs/performance/runtime/build/new_controller/depth_observations_benchmark \
  EGO4P0/docs/performance/reader_jitter_20261102_165205/privileged_20261102_181620/fifo10_200us \
  first /tmp/ego4p0_block3_new_result 1 1 3 1 1
PYTHONNOUSERSITE=1 /usr/bin/python3 -B EGO4P0/scripts/verify_direction_queries.py \
  /tmp/ego4p0_block3_new_result
```

离线基准专门清除`ready`，强制每次重建，避免把同组复用计成构建加速；生产接收与2ms读取没有这一强制清除。最终三参数分别为`block_edge/parallel/boundary_lookup`，串行精确对照为`3 0 0`，四线程精确对照为`3 1 0`。

相机500仿真Hz、RTX5070、.02倍/.2s、现实2ms读取和选定FIFO10＋200μs方案保持。新版本在线数据年龄、相机完整组率及读取线程间隔尚未重测；没有新增导航、点云/地图、未知策略、自由域或运动输出。
