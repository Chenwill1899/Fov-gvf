# 0.1倍速仿真：重新计时与绘图

2026-11-02当前默认改为0.02倍速、0.2秒仿真：[最新结果与图表](DEPTH_SLOW_002X.md)。以下保留本报告当时配置与数据。


2026-11-02 16:25 +0800。本次只实际运行一次：本机RTX5070渲染/GPU物理cuda:0、原Cloud场景、四路320×240、0.5秒仿真窗口，启用与上次相同的CPU性能追踪。运行目录：[`depth_cpp_500hz_reference_20261102_161834_280404`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404)。

## 当前配置与结果

本报告运行时默认`--time-scale 0.1`，保留仿真物理/渲染步长0.002s、相机500Hz（仿真时间），按每步至少0.020s现实时间调度。每步耗时不足20ms时睡眠补足，超时如实保留，不通过快速追赶压缩后续步骤。C++独立读取线程保持每2ms现实时间取最新完整组，因此一组新图通常会被多次读取。

**0.500秒仿真实测耗费5.084秒读取墙钟，实际速度0.098348倍。** 理想0.1倍速对应5.000秒；睡眠唤醒和一次长尾等使实际略慢。这里读取窗口从首个成功读取开始计，适配器从播放到停机的总墙钟为5.935秒，还包含播放预热和收尾；不把这一总时长误当作0.5秒验证窗口。

249个完整测量步骤中，248次`app.update`处于20ms预算内，1次超过预算，最大71.798ms。主循环平均9.952ms，仍有几何流送预算等待的长尾。减速增加了每步可用现实时间，没有把一次渲染/计算本身按比例加速。

本次2543次读取、243组完整同步新图、2300次重复读取；同步组为47.770现实Hz / 485.944仿真Hz，接近0.1倍速下50现实Hz目标。四路时间戳交集243全部发布到缓冲，重复时间戳0；7个不完整组清理、1组待齐。读取间隔均值2.000000ms，最长3.652902ms，跳过周期0、完成期限超时0。**没有达到每次读取间隔均不超过2ms，也没有达到所有相机新图均完整到达。**

## 六类耗时曲线

![0.1倍速分项耗时曲线](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_curves.png)

沿用前次六个面板：主循环分项、渲染内部分项、ROS发布/接收、C++引用读取、读取调度、同步组到达。横轴统一为成功读取窗口的现实时间0～5.084秒；C、D单位为μs，其余为ms；A、C为对数轴。仿真相关面板红线改为20ms现实预算，读取调度面板仍为2ms现实预算。原始全部样本保留，没有平滑或去掉长尾。

以下统计单位全部为ms，父子区间和异步线程存在重叠，不能将各行直接相加。主循环分项按249次完整更新统计；窗口内接收拷贝242组，整次运行243组（第一组发布早于首个读取）。

| 范围 | 样本数 | 均值ms | P95 ms | 最大ms |
|---|---:|---:|---:|---:|
| 仿真主循环 | 249 | 9.952195 | 11.443556 | 71.798308 |
| 四路渲染链路 | 249 | 5.785243 | 6.285369 | 66.707673 |
| ↳ 渲染产品Prim更新 | 249 | 3.224857 | 3.571616 | 4.050370 |
| 　↳ 四路设置USD同步合计 | 249 | 3.006692 | 3.334809 | 3.765272 |
| ↳ 传感器分区渲染 | 249 | 1.714293 | 1.932453 | 4.245069 |
| PhysX更新 | 249 | 0.702652 | 1.069340 | 1.647185 |
| OmniGraph更新 | 249 | 0.999693 | 1.151109 | 1.822852 |
| 单图ROS publish调用 | 998 | 0.050959 | 0.071433 | 2.113068 |
| C++组齐回调到四图拷贝/缓冲发布 | 242 | 0.042119 | 0.058125 | 0.111294 |
| C++引用读取与四像素探针 | 2543 | 0.000183 | 0.000361 | 0.000801 |
| C++读取线程周期内工作 | 2543 | 0.000203 | 0.000381 | 0.000821 |
| 同组四路回调到达跨度 | 242 | 3.440649 | 3.835564 | 6.255351 |
| 相邻完整组墙钟间隔 | 241 | 20.952113 | 22.226530 | 71.482894 |
| 相邻读取墙钟间隔 | 2542 | 2.000000 | 2.000021 | 3.652902 |
| 读取唤醒迟到 | 2543 | 0.009090 | 0.000049 | 1.652924 |

这些是CPU墙钟区间；渲染包含准备/同步/等待，publish只是单图发布调用。纯GPU执行、GPU到CPU搬运及完整DDS端到端延迟仍未单独测得。读取只取引用并做四路像素探针，不代表完整未来控制算法。

## 实际速度与上次对照

![实际速度核验](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/simulation_speed.png)

速度图上半部直接使用C++收到的ROS时钟与单调墙钟；下半部区分每步总现实时间、实际app.update区间与节拍等待。256次适配器更新每次仿真增量均核对为2ms；249次完整节拍样本最小20.010ms、平均20.261ms、最大71.829ms。没有把2ms物理步长改成20ms。

| 指标 | 上次目标1倍速追踪 | 本次目标0.1倍速追踪 |
|---|---:|---:|
| 仿真窗口 | 0.500s | 0.500s |
| 读取墙钟窗口 | 2.540s | 5.084s |
| 实际仿真/现实速度比 | 0.19685 | 0.09835 |
| 完整同步组 | 188 | 243 |
| 重复/迟到图像时间戳 | 137 | 0 |
| 不完整组清理 | 58 | 7 |
| 同步组现实Hz | 74.037 | 47.770 |
| 同步组仿真Hz | 377.016 | 485.944 |
| 平均读取间隔 | 2.003155ms | 2.000000ms |
| 最长读取间隔 | 4.880964ms | 3.652902ms |
| 跳过读取周期 | 2 | 0 |

两次均为短测、均开启追踪，配置节拍发生有意变化；这是本次同步完整性与调度的观察，不能据此保证长期稳定或认为渲染性能得到根本提升。原始目标1倍速运行实际仅0.19685倍速，表中没有将它称为已经达到实时。上次原数据和图保持不改。

## 复现与数据

```bash
cd /home/starry/isaac-data/EGO4P0
# 复现本报告当时的0.1倍速、0.5秒窗口
bash scripts/run_depth_step1.sh --profile-stages --time-scale 0.1 --duration 0.5
# 替换为新运行目录后重新绘图
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/plot_depth_stage_timing.py docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/analyze_depth_pacing.py docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404 --baseline docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232
```

恢复旧节拍可传`--time-scale 1`；不限速模式需同时传`--pace unlimited --time-scale 1`。此处time-scale只控制适配器现实节拍，C++读取和ROS采集时间的定义不变。

- [六面板PNG](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_curves.png) / [PDF](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_curves.pdf)
- [速度核验PNG](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/simulation_speed.png) / [PDF](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/simulation_speed.pdf)
- [逐次耗时CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_curves.csv) / [耗时统计CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_summary.csv) / [完整JSON](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_summary.json)
- [前后对照CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/pacing_comparison.csv) / [每步节拍CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/pacing_samples.csv) / [节拍验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/pacing_validation.json)
- [原始Trace](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/cpu_trace.json.gz) / [适配器每次更新CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/simulator_updates.csv) / [读取CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/reads.csv)
- [运行时输入验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/validation.json) / [源归档及运行时ELF验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/runtime_archive_validation.json) / [计时验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404/stage_timing_validation.json)

运行时输入、源/实际ELF和场景核验通过，两个子进程退出0；实际dt、每步节拍、时间戳交集、六面板和速度图均核对。四个Python文件AST通过，本次C++未改，未重建或重跑CTest/231项基础回归/导航/实机。

初次绘图因旧固定时钟偏移极差111.673μs超过100μs阈值而停止，失败脚本和日志保留。离线分析改用匹配标记拟合时钟相对速率及偏移，拟合速率1.000017578、残差极差27.400μs、最大绝对残差17.785μs后通过；没有放宽100μs残差阈值或重新仿真挑选结果。此拟合仅用于对齐两个计时源，不能推断亚微秒跨进程延迟。运行后修改的只有离线绘图代码（新节拍分析脚本也在运行后创建），运行时适配器/C++与执行ELF保持，首版分析与最终分析源码分别存档并记录SHA。
