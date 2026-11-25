# 整组约9ms的耗时位置

2026-11-03 13:16 +0800，离线定位完成。主要开销在四路逐像素的机体方向计算与角网格归类；其中两次`atan2`是分块诊断中最大的计算项。生产读取代码、FIFO10＋200μs方案和仿真配置均未修改。

## 计时范围与测量方法

此前“约9ms”实际包括**第二步补值＋第三步方向索引＋第四步区域摘要**。从内存中的四路原始深度开始，到本组可查询索引完成；不包含文件读取、GPU渲染、ROS/DDS传输、同步等待、最初消息复制、初始化b/d表、后续查询或日志导出。

输入来自原`privileged_20261102_181620/fifo10_200us`真实静止仿真会话的首组和末组，分别为序号1/95、仿真时间14/212ms。每组4×320×240＝307200像素，补值后298795个有限正深度进入方向索引，几何排除0；网格1°×1°、64800格，22966格有观测。它们是同一静止场景的两个存档样本，不是本次采集的新相机帧。

CPU为本机AMD Ryzen 5 9600X，固定CPU5、普通调度`SCHED_OTHER/0`。GCC11.4，`-O3 -DNDEBUG -fno-fast-math -std=gnu++17`，与原离线基准的优化级别相同。先完成编译和CTest，再串行运行所有计时；没有并行编译、Sanitizer或本任务的其它测试，但没有隔离桌面负载、SMT、中断或锁定CPU频率。

新增独立C++诊断程序[depth_observations_profile.cpp](../../src/new_controller/test/depth_observations_profile.cpp)，三种模式每轮轮换顺序，各预热10次、正式200次，首末合计每模式400次：

1. **原实现**：直接调用现有`FrameObservationBuilder::build`，保留原融合循环，计时口径与旧基准一致，并细分四路补值。
2. **融合诊断**：复制原数学表达式、逐像素顺序和分支，只在大循环边界计时，以区分方向投影、前缀和、来源整理、区域摘要。没有逐像素时间戳。编译器代码布局仍可能不同，必须用第1种模式作对照。
3. **分块诊断**：每512像素将坐标变换、`hypot`、方位角`atan2`、俯仰角`atan2`、归格分别计时。仅用固定小块临时向量，随后复用，不在帧中保存/导出三维点。循环拆分、临时读写、缓存及编译器优化都发生变化，所以内部数值仅是该分块方式的耗时，**不能直接加回原来的9ms**。

初始化与预分配、逐轮结果核对、CSV和图像导出全部在计时区间外。每一轮三种模式都与原实现核对完整来源顺序、格偏移、覆盖位、计数、组身份和所有区域摘要，而非只比较总来源数。

## 原实现与诊断对照

| 实现与条件 | 首组平均/ms | 末组平均/ms | 合并平均/ms | 合并P99/ms | 合并最大/ms |
|---|---:|---:|---:|---:|---:|
| 原实现，轮换测量 | 9.251472 | 9.218480 | **9.234976** | 10.453929 | 12.478098 |
| 融合诊断，大循环分段 | 9.348511 | 9.306871 | **9.327691** | 10.533523 | 12.291929 |
| 分块诊断，内部数学拆分 | 8.280649 | 8.255586 | **8.268118** | 9.593154 | 12.518848 |

融合诊断相对原实现均值增加1.004%，适合定位大循环比例；此差异同时包括插桩、代码布局和测量波动，不能全部叫作时钟开销。分块诊断比原实现快10.470%，但它目前只是独立诊断路径，未接入接收线程，不能写成线上已优化。三者最大值均超过12ms，本次不作逐次上界保证。

另用**原封不动的旧`depth_observations_benchmark`可执行文件**先后各测100次，首组9.651177ms、末组9.252188ms；这两轮不与上表的轮换样本混合。原记录的9.081360/9.085636ms保留，说明普通调度、CPU频率和负载下均值会变化，不能把旧9ms当常量。具体顺序与可执行SHA见[execution.json](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/execution.json)。

原实现本轮平均墙钟9.234976ms、线程CPU时间9.234488ms；两者量级接近，支持这段平均时间主要消耗在CPU执行。CPU时钟和墙钟边界并不完全重合，不能据差值精确归因单次抢占。

## 大循环耗时及代码位置

以下均为融合诊断400次的平均，可相加得到9.327691ms；比例使用同一模式的总时间作分母。

| 位置 | 每组平均/ms | 占比 | 对应生产源码 |
|---|---:|---:|---|
| 四路异常补值 | 0.191035 | 2.05% | `depth_preprocess.hpp:19`，复制原图并仅修补异常像素 |
| 有效值筛选、Z*b、R/t变换、角度计算、格计数与来源暂存 | **8.494656** | **91.07%** | `depth_observations.hpp:36`～56 |
| 格前缀和、每相机覆盖格数统计 | 0.050419 | 0.54% | `depth_observations.hpp:59`～64 |
| 按格整理连续来源列表（CSR散写） | 0.223291 | 2.39% | `depth_observations.hpp:65` |
| 四路16×16区域最小深度/可用像素数摘要 | 0.365285 | 3.92% | `depth_observations.hpp:68`～80 |
| 清零、边界计时与剩余薄层 | 0.003004 | 0.03% | 清空偏移/覆盖、计时调用和模式入口等 |
| **合计** | **9.327691** | **100%** | |

四路补值各自平均：Front0.040350ms、Left0.055439ms、Back0.057257ms、Right0.037988ms。这些都包含相应图像的复制、有效性扫描和异常邻域均值，未拆成纯`memcpy`或纯邻域核耗时。

源码入口：[方向计算与索引](../../src/new_controller/include/new_controller/depth_observations.hpp)、[异常补值](../../src/new_controller/include/new_controller/depth_preprocess.hpp)。

![大循环与内部数学耗时](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/stage_breakdown.png)

## 8.49ms逐像素循环的内部量级

这里使用分块诊断，整组总时间8.268118ms，**不是上一表的9.327691ms**。各项包含本段循环、临时数据读写和边界计时，不等于数学函数自身指令的纯耗时。

| 内部计算 | 分块方式下每组平均/ms | 对应生产位置 |
|---|---:|---|
| 检查深度、Z*b、R_BC/t_BC、有限性及零向量检查、暂存 | 1.185079 | `depth_observations.hpp:39`～50 |
| 水平长度`hypot(x,y)` | 0.827070 | 第52行中的`hypot` |
| 方位角`atan2(y,x)`及极点分支 | **2.366156** | 第51行 |
| 俯仰角`atan2(z,hypot)` | **2.365639** | 第52行中的`atan2` |
| 角单元计算、格计数、来源暂存 | 0.736331 | 第53～56行 |

两段`atan2`合计**4.731795ms**，占该分块整组57.23%；再计`hypot`为5.558865ms、67.23%。每组298795个可用像素，几何排除0；本数据没有垂直极点，因此需要约597590次`atan2`和298795次`hypot`。主要计算热点已落到生产源码第51、52行。补值、格前缀和及来源散写各自都远小于这一量级。

分块拆开后逐像素部分合计7.480276ms，低于融合循环8.494656ms；不能把这1.014380ms直接标成某一种“缓存开销”，本次未做PMU/指令级归因，也没有将分块诊断替换生产实现。

## 逐次曲线、核对与复现

![首末各200次的耗时曲线](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/timing_curves.png)

- 每模式正式400次，共1200次；包含预热的1260次C++结果逐项等于原实现。
- 首末分别通过独立NumPy校验：全部298795来源、22966有格/41834空格、原始补值规则及b/d表、1200块摘要和25种查询；浮点边界格差异0。
- 现有CTest3/3通过；执行在计时之前。此次仅独立诊断程序编译，无警告；生产源码与原构建身份7项SHA全部一致，无需重建线上读取程序。新Python分析脚本实际运行并生成PNG/PDF，图像已目视核对。
- 本轮未启动Isaac、未执行新的FIFO测量，未测GPU渲染/ROS传输，也未改变相机500仿真Hz、现实2ms读取、0.02倍/.2s或运动输出。

[完整统计JSON](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/summary.json) · [分项CSV](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/stage_summary.csv) · [首组逐次原始计时](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/profile_first/stage_timings.csv) · [末组逐次原始计时](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/profile_last/stage_timings.csv) · [分项图PDF](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/stage_breakdown.pdf) · [曲线PDF](../../../EGO4P0/docs/performance/depth_index_profile_20261103_131114/timing_curves.pdf)

独立编译并复现（输出目录必须是新目录，编译与测试结束后再测量）：

```bash
cd /home/starry/isaac-data
/usr/bin/c++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -fno-fast-math \
  -std=gnu++17 -I EGO4P0/src/new_controller/include \
  EGO4P0/src/new_controller/test/depth_observations_profile.cpp \
  -lyaml-cpp -o /tmp/ego4p0_depth_observations_profile
taskset -c 5 /tmp/ego4p0_depth_observations_profile \
  EGO4P0/docs/performance/reader_jitter_20261102_165205/privileged_20261102_181620/fifo10_200us \
  first /tmp/ego4p0_profile_first_new 200
PYTHONNOUSERSITE=1 /usr/bin/python3 -B EGO4P0/scripts/verify_direction_queries.py \
  /tmp/ego4p0_profile_first_new
```

末组将`first`换为`last`并使用另一新目录。本次绘图命令：

```bash
MPLCONFIGDIR=/tmp/ego4p0-mpl-profile PYTHONNOUSERSITE=1 /usr/bin/python3 -B \
  EGO4P0/scripts/analyze_depth_index_profile.py \
  EGO4P0/docs/performance/depth_index_profile_20261103_131114
```

以上均为缓存样本的离线重算。约9ms发生在**普通接收线程**处理新完整组时；现实2ms线程复用已发布完整索引。此次定位不能替代后续新版本在线数据年龄、相机完整组率或严格2ms期限验证。
