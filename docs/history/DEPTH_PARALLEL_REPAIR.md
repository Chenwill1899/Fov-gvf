# 四路并行补值与块/区域摘要合并扫描

2026-11-03：两项优化已接入C++接收端，离线整组平均 **0.411755ms**。与本轮冻结旧方案0.678919ms相比减少39.35%；5/5 CTest、ASan/UBSan及18份独立数据核对通过。

**2026-11-03 16:07 +0800：本版在线FIFO三轮验证已完成。** 用户已在本机终端认证并运行固定入口，原始CSV复算合计 **0/15020个读取间隔超过2.2ms，漏周期0**，最长2.057619ms。同步完整组为100/99/100（合计299/300）；第二轮缺少0.200s前向相机接收，不能把读取节拍通过等同相机完整性全部通过。在线整组处理平均1.289/1.736/1.662ms，高于离线缓存测试；详见下方独立在线统计。此前14:09非交互sudo失败记录保留。

## 代码和处理顺序

生产接收端完成四路原图复制后调用`FrameObservationBuilder::process(frame,rays)`，每路常驻普通线程处理自己的原图与输出缓冲：

```text
同步完整组 → 预分配raw帧槽
                 ↓ 每路同一条常驻线程
             原始3×3异常补值
                 ↓
             块均值＋16×16区域摘要的一次图像扫描
                 ↓
             Z*q+t → 角格判定 → 独立来源数组
                 ↓ 四路全部完成
             接收线程统一归并 → 发布完整保护槽
                 ↓
             2ms读取线程引用/查询已完成索引
```

补值调用原`fill_invalid_depth`，只使用该相机**原始D**的3×3图内有限正邻居，保持原有效/未解像素位模式；不读取刚补好的邻居，不跨相机取值，不平滑原有效像素。每路原始与补值缓冲仍各自预分配、互不重叠，所有处理完成前不发布。

默认启用合并扫描，并保留分开扫描配置作为同源码对照。对完整落入一个16×16区域摘要的3×3块，扫描时同时累积double深度和、有效数及局部最小值，随后一次更新摘要。跨摘要边界的块使用预计算的行/列摘要编号，逐个有效像素更新其实际所属摘要；无循环内整数除法、无像素锁/原子或临时图像分配。

每个块内部仍按原来的v/u顺序累加，因此合并扫描不改变块均值舍入顺序。全分辨率区域最小值及可用数保持，边缘不足3×3和不足16×16的块都保留。D/D_filled、33511个有效代表、本组来源与网格、像素/ROI查询语义均与前版相同，无新导航/自由域/点云/地图输出。

`build(frame,rays)`继续接受已经补好的图，供独立数值测试；生产入口使用`process`。同组复用还区分原始处理/已补值模式，避免测试或调用切换时错误复用。K/T及配置不变仍只生成一次旋转视线/角边界表，四路工作线程仍为CPU1～4、SCHED_OTHER，剔除读取CPU5及SMT11。

主要源码：[补值与扫描任务](../../src/new_controller/include/new_controller/depth_observations.hpp)、[接收端调用](../../src/new_controller/src/depth_reader.cpp)、[新增并行补值与边界测试](../../src/new_controller/test/depth_worker_preprocess_check.cpp)。

## 离线对照结果

每方案三轮×首末两份历史静止图×100次，合计600次；各轮预热10次，按轮次轮换方案顺序。协调线程CPU0、工作CPU1～4、普通调度，GCC11.4/-O3/-fno-fast-math。所有本任务编译/CTest/ASan先于计时结束，没有全机频率/SMT/中断/桌面负载隔离。强制重建缓存图，不是600组新相机帧。

| 方案 | 平均/ms | P99/ms | 最大/ms | >2ms/600 |
|---|---:|---:|---:|---:|
| 前版：串行补值，四线程分开扫描 | 0.678919 | 1.049212 | 2.209486 | 1 |
| 四路并行补值，分开扫描 | 0.465646 | 1.140034 | 3.135072 | 3 |
| **并行补值＋合并扫描（当前）** | **0.411755** | **0.596049** | **2.112952** | **1** |

补值四路并行的核最大经过时间平均约0.052553ms（Front0.032992、Left0.045906、Back0.051831、Right0.032930ms）。它包含在本组并行工作墙钟内，**不能再与整组/并行工作时间相加**。当前并行工作平均0.324781ms、归并0.086900ms。

合并扫描相对并行分开扫描，再降低平均约11.57%。单路合并扫描平均Front0.079102、Left0.080737、Back0.077527、Right0.077932ms；包含块均值和区域摘要全部成本。原分开扫描中两项分别计时，不能把新`region_camera_ns=0`解释成摘要没有成本。

用户估算0.506ms以旧0.646ms为基数；本轮同场景基线0.679ms，实际并行补值0.466ms、再合并0.412ms。实际收益还受数据局部性和普通调度负载影响，未做缓存/指令级因果归因。最大2.113ms说明离线重建仍有长尾，也不代表在线读取间隔。

![补值与扫描优化对照及曲线](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/parallel_repair_comparison.png)

## 验证与计时格式

- 4包Release构建成功，仅保留已有Python字节编译禁用提示；CTest5/5通过。
- 新增12种图像尺寸、3×3/16×16交界与图边、混合NaN/±Inf/零/负值/极大极小正数、未解NaN载荷、原始邻域补值、同组复用和零稳态分配核对。4800随机ROI与原图直接扫描一致。
- 原约100万组角边界/精确方向、4000ROI、1000保护槽并发组和原补值规则检查继续通过。
- 新增测试ASan＋UBSan通过；因既知ptrace限制关闭LSan，不声称泄漏检测通过。
- 最终18份目录逐个独立NumPy核对通过；原始/补值图、b/d、所有块均值/来源范围、有序CSR/覆盖、区域摘要与查询示例CSV，全部逐字节等于冻结前版。

帧CSV使用`repair_max_ms`替代串行`preprocess_ms`，表示四路补值核经过时间的最大值。`camera_work_ms`现在包含并行补值、扫描、方向与组级同步；后面才是`index_merge_ms`。`compression_ns`在合并模式下包含区域摘要，JSON通过`fused_block_region_scan`和`compression_timing_scope`明确说明。旧数据分析按`depth_preprocessing.location`分支兼容，不修改旧结果。

[全部统计JSON](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/summary.json) · [独立逐字节核对](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/independent_comparison.json) · [实际命令/可执行身份](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/execution.json) · [首组逐次计时](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/r1_first_parallel_repair_fused/build_timings.csv) · [图表PDF](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/parallel_repair_comparison.pdf)

## 在线FIFO三轮实测

实际会话：[optimized_20261103_155735](../../../EGO4P0/docs/performance/reader_jitter_20261102_165205/optimized_20261103_155735/session_result.json)。每轮0.2秒仿真，读取窗口约10.014秒现实时间；RTX5070/GPU0、cuda:0、相机500仿真Hz、0.02倍速，读取2ms现实周期。仿真/ROS为starry普通用户；管理员监督器只临时提高已核验reader进程的RT限额。读取主线程CPU5，实际SCHED_FIFO/10、200μs短自旋；其余所有reader线程SCHED_OTHER/0，四相机工作CPU1～4且亲和设置成功。无profiler/perf重型追踪或永久系统修改。

| 轮次 | 读取间隔数 | >2.2ms | P99.9/ms | 最长间隔/ms | 漏周期 | 完整同步组 |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 5006 | 0 | 2.000101 | 2.040451 | 0 | 100/100 |
| 2 | 5007 | 0 | 2.013108 | 2.017607 | 0 | 99/100 |
| 3 | 5007 | 0 | 2.000101 | 2.057619 | 0 | 100/100 |

三轮读取间隔均满足本次2.2ms阈值，严格每次不超过2.000ms仍未满足，也不据这三次短测承诺长期上界。读取本身包含最新保护槽引用和示例查询，平均分别约0.900/1.412/0.890μs。重复读取4907/4909/4908次，符合0.02倍速下约100ms现实才有一组新图的节拍；2ms读取不会制造新相机帧。

整组处理定义为`camera_work_ms + index_merge_ms`，每组只处理一次。它包括四路补值、合并扫描、方向及归并，不含原图复制、LUT启动或相机渲染/DDS；`received_to_published`从完整组接收回调计到保护槽发布，包含原图复制等接收端开销，仍不含此前四路传输/同步等待。

| 轮次 | 整组平均/ms | 整组P99/ms | 整组最大/ms | 整组>2ms | 完整组接收→发布平均/ms |
|---|---:|---:|---:|---:|---:|
| 1 | 1.288974 | 4.007223 | 4.942936 | 16/100 | 1.338702 |
| 2 | 1.735914 | 4.185987 | 5.032367 | 35/99 | 1.787867 |
| 3 | 1.661569 | 4.297117 | 4.367881 | 30/100 | 1.710391 |

在线整组共81/299次超过2ms，长尾主要位于`camera_work_ms`这一计时区间。该区间包含普通工作线程的调度/唤醒/等待和计算，现有粗粒度记录不能继续区分内核调度、缓存和实际计算的各自占比，也不能把在线退化直接归因于FIFO。离线0.412ms不能用作本次在线处理成绩；当前新图现实周期约100ms，已完成组的接收端处理均在该周期内完成。

| 轮次 | 四路补值核最大值的平均/ms | 并行工作平均/ms | 归并平均/ms | 已发布帧最大墙钟年龄/ms | 图像最大仿真年龄/ms |
|---|---:|---:|---:|---:|---:|
| 1 | 0.135609 | 1.109152 | 0.179823 | 103.341486 | 4 |
| 2 | 0.127216 | 1.565791 | 0.170123 | 199.300562 | 6 |
| 3 | 0.131881 | 1.490642 | 0.170927 | 103.830630 | 4 |

补值核时间与并行工作时间重叠，不重复相加。墙钟年龄按当前读取时刻减保护槽发布时刻计算；仿真年龄按`/clock`减图像采集时间戳计算，两者不可混用。

第二轮：预期图像时间戳从0.014到0.212s、步长0.002s，缺少0.200s一组。`received.csv`在200000000ns只有Left/Back/Right（零基相机1/2/3），Front（零基相机0）没有接收记录；前后0.198/0.202s均四路齐全。同步器清理1个不完整组，没有将三路发布成完整组；没有覆盖未读组、重复/迟到图像或读取漏周期。已有记录将问题定位到该前向帧进入接收同步器之前，尚不能区分仿真发布与DDS传输环节，不能断言是渲染或处理超时造成。末尾0.214s的未齐消息属于窗口结束尾部，不计入上述100组。缺组时继续使用上一完整组，墙钟帧年龄最大199.301ms，仿真图像年龄最大6ms；第一/三轮墙钟最大约103ms。

本次重新核对了三轮源码清单、9项构建身份、实际运行ELF与本轮离线冻结ELF的SHA256及场景SHA；六份首末组的原始邻域补值、b/d表、全部块均值/来源/覆盖、1200个区域摘要及查询示例逐份独立NumPy核对通过。CSV间隔、绝对调度槽和同步时间戳复算通过，索引构建次数恰等于完整组数。原在线会话的CSV、manifest及validation保持原样；本次没有重新编译、运行CTest/ASan或追加仿真。

![在线读取与整组处理曲线](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/online_fifo_verification.png)

![在线数据年龄曲线](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/online_fifo_data_age.png)

[完整复算与数据核对](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/summary.json) · [在线耗时PDF](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/online_fifo_verification.pdf) · [数据年龄PDF](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/online_fifo_data_age.pdf)。本次完成输入流水线在线验证，运动控制/调和场尚未实现，无运动指令。

## 在线FIFO复现入口

本次用户在本机终端实际执行的固定命令（今后仅按用户授权复现）：

```bash
sudo /usr/bin/python3 -I /home/starry/isaac-data/EGO4P0/scripts/run_reader_privileged_check.py --mode optimized --repeat 3
```

每轮0.2秒仿真、0.02倍、相机500仿真Hz、5070、现实每2ms读取；只有已核验读取主线程FIFO10，四路工作/ROS/Isaac保持普通用户及普通调度。关闭重型profiler/perf，不改sysctl/IRQ/governor或永久权限。密码只在用户自己的Ubuntu终端输入。

自动监督器保存了实际实时策略、其它线程策略、首末图与来源、读取间隔/跳周期/数据年龄/组完整性。此前14:09非交互尝试退出1、需要密码的记录仍保留在[online_supervisor.log](../../../EGO4P0/docs/performance/depth_parallel_repair_20261103_140256/online_supervisor.log)及online_attempt.json；15:57起的用户认证后三轮成功会话是本版在线结果，二者按时间区分。

本次图表/复算命令：

```bash
MPLCONFIGDIR=/tmp/ego4p0-mpl-profile PYTHONNOUSERSITE=1 /usr/bin/python3 -B /home/starry/isaac-data/EGO4P0/scripts/analyze_parallel_repair.py /home/starry/isaac-data/EGO4P0/docs/performance/depth_parallel_repair_20261103_140256 --online-session /home/starry/isaac-data/EGO4P0/docs/performance/reader_jitter_20261102_165205/optimized_20261103_155735
```
