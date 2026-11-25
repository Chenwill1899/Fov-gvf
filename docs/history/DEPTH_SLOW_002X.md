# 0.02倍速、0.2秒仿真实测

当前读取等待默认已更新，见[抖动优化与管理员入口修复](READER_JITTER_OPTIMIZATION.md)。本文保留当时200μs短自旋的历史原数据。

2026-11-02 16:34 +0800。默认仿真速度0.1→0.02倍，官方测试入口默认验证窗口0.5→0.2秒仿真时间。本次实际只运行一次：本机RTX5070渲染GPU0、GPU物理cuda:0，原Cloud场景、四路320×240，开启CPU性能追踪；C++读取逻辑与执行ELF保持。运行目录：[`depth_cpp_500hz_reference_20261102_163051_546841`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841)。

## 结果

- 读取仿真时钟0.016→0.216s，恰好0.200s；读取现实窗口10.014000s，实际速度0.019972倍，接近目标0.02。理想对应10.000s，计时等待/调度使实际略慢。播放至停机总11.167358s另含预热、收尾，不与读取窗口混用。
- 四路相机仍500Hz仿真时间，物理/渲染步长仍2ms，每步最小现实节拍100ms，不追赶。100组完整同步图的99个相邻采集时间戳间隔全部为2ms；采集时间戳0.014→0.212s，第一组早于读取起点，保留原先首末组边界口径。组频率500.000仿真Hz / 9.9835现实Hz。
- 接收四路101/100/100/101张消息，四路共同时间戳100，全部进入缓冲；重复/迟到0，不完整组清理0，结束边界尚有1组未齐。完整组四路时间戳差0ns。这里验证的是本次已观测完整组的间隔和同步，不宣称任意时间窗口或长期都不缺帧。
- C++仍每2ms现实时间读取，5008次读取、100组新图、4908次重复；读取操作平均0.0001605ms、最大0.013805ms。相邻读取间隔平均2.000000ms、P99 2.000176ms、最大3.939725ms；跳过周期0、完成期限超时0，但仍不能称现实逐次严格2ms。最大唤醒迟到1.939761ms，调度抖动仍存在。
- 成功读取窗口内99个完整步骤，主循环平均11.732ms、最大15.401ms；没有更新超过100ms现实预算。包括节拍等待的每步现实时间最小100.012ms、平均100.059ms、最大100.079ms。本次仅99次测量步骤，不能据此断言之前约70ms长尾已消除。

当前已经观察到完整同步组按每2ms仿真时间连续更新；现实每2ms读取仍有调度抖动，降低仿真速度未建立严格实时保证。相机新图约每100ms现实时间产生一组，所以大多数2ms读取会重复最近图像。

## 更新后的曲线和统计表

![0.02倍速分项耗时](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_curves.png)

仿真面板的现实预算线为100ms，读取调度面板为2ms。横轴为首个成功读取后的现实时间，C/D单位为μs，其他为ms，A/C为对数轴。曲线保留长尾，统计均使用原始样本。嵌套父子区间和异步线程会重叠，不能将表中各项相加。以下表单位全部为ms；主循环99个完整更新，窗口内接收拷贝99组（总100组，首组在窗口前发布）。

| 计时范围 | 样本数 | 均值ms | P95 ms | 最大ms |
|---|---:|---:|---:|---:|
| 仿真主循环 | 99 | 11.732281 | 13.112778 | 15.401192 |
| 四路渲染链路 | 99 | 6.293104 | 6.722832 | 8.182776 |
| ↳ 渲染产品Prim更新 | 99 | 3.596762 | 3.960144 | 4.085477 |
| 　↳ 四路渲染设置USD同步合计 | 99 | 3.328043 | 3.667723 | 3.774059 |
| ↳ 传感器分区渲染 | 99 | 1.955124 | 2.046768 | 3.733817 |
| PhysX更新 | 99 | 0.913181 | 1.266192 | 2.595876 |
| OmniGraph更新 | 99 | 1.204351 | 1.368869 | 2.574437 |
| 单图ROS publish调用 | 398 | 0.059581 | 0.077304 | 0.980118 |
| C++组齐回调至四图拷贝/缓冲发布 | 99 | 0.052359 | 0.064528 | 0.104367 |
| C++引用读取与四像素探针 | 5008 | 0.000160 | 0.000311 | 0.013805 |
| 读取周期内工作 | 5008 | 0.000181 | 0.000331 | 0.013825 |
| 相邻读取现实间隔 | 5007 | 2.000000 | 2.000027 | 3.939725 |
| 同组四路回调到达跨度 | 99 | 3.499323 | 4.027892 | 5.022622 |
| 相邻完整组现实间隔 | 98 | 100.191742 | 101.295941 | 103.459834 |

![实际仿真速度](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/simulation_speed.png)

上图对照ROS仿真时钟与现实单调时钟，并区分每步计算与等待。物理/渲染步长读回0.002s；全部适配器步骤仿真增量核对2ms。CPU trace与单调时钟沿用仿射拟合，残差极差26.843μs、最大绝对残差15.075μs。所有耗时都是所标明的CPU墙钟区间，未单独测纯GPU、GPU搬运、完整DDS延迟或后续控制算法。

## 与0.1倍速短测的对照

| 指标 | 0.1倍速前次 | 0.02倍速本次 |
|---|---:|---:|
| 仿真窗口 | 0.5s | 0.2s |
| 读取现实窗口 | 5.084s | 10.014s |
| 实际速度比 | 0.098348 | 0.019972 |
| 完整同步组 | 243 | 100 |
| 新组仿真Hz | 485.944 | 500.000 |
| 新组现实Hz | 47.770 | 9.9835 |
| 重复/迟到图像时间戳 | 0 | 0 |
| 不完整组清理 | 7 | 0 |
| 读取间隔P99 | 2.255510ms | 2.000176ms |
| 读取间隔最大 | 3.652902ms | 3.939725ms |
| 跳过读取周期 | 0 | 0 |

两次仿真窗口长度不同，原始完整组数不能直接当作性能优劣，分别用新组频率和间隔解释。新组仿真Hz来自已接收相邻时间戳，不是现实频率。两次均开启追踪，属于短测观察，无长期稳定或硬件上限保证。

## 复现与数据

```bash
cd /home/starry/isaac-data/EGO4P0
# 当前默认就是0.02倍速、0.2秒仿真；显式参数便于复现
bash scripts/run_depth_step1.sh --profile-stages --time-scale 0.02 --duration 0.2
# 将下方路径换为上一步新生成目录
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/plot_depth_stage_timing.py docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/analyze_depth_pacing.py docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841 --baseline docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_161834_280404
```

- [六面板PNG](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_curves.png) / [PDF](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_curves.pdf)
- [速度图PNG](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/simulation_speed.png) / [PDF](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/simulation_speed.pdf)
- [逐次耗时CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_curves.csv) / [统计CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_summary.csv) / [全部统计JSON](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_summary.json)
- [节拍验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/pacing_validation.json) / [逐步节拍CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/pacing_samples.csv) / [对照CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/pacing_comparison.csv)
- [原输入验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/validation.json) / [归档/运行时源码/ELF与时间戳验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/runtime_archive_validation.json) / [计时验证](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/stage_timing_validation.json)
- [原始CPU Trace](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/cpu_trace.json.gz) / [读取CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/reads.csv) / [完整组CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_163051_546841/frames.csv)

两进程退出0；输入、源归档/运行时源码/实际ELF/场景SHA、物理与渲染dt、100ms节拍及100组时间戳间隔核验通过，四Python AST和两张图中文目视检查通过。修改两个Python启动/适配文件的默认参数，并将离线速度图标题改为按实际预算动态生成；Python未参与控制算法。C++由入口显式传入--duration 0.2，本次C++未改也未重建、未重跑CTest/231基础回归、导航或实机；无新依赖、运动指令、后台续跑。速度图初版标题沿用20ms，目视检查后改为动态100ms并重绘；运行当时的分析脚本与修订后实际分析脚本分别存档，最终运行时源码/ELF与原始数据保持。没有仿真或脚本执行失败，旧运行/图表保持原样。
