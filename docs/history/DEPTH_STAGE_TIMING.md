# 四路深度相机分项计时与瓶颈定位

2026-11-02当前默认改为0.02倍速、0.2秒仿真：[最新结果与图表](DEPTH_SLOW_002X.md)。以下保留本报告当时配置与数据。


2026-11-02当前默认已改为0.1倍速：[新曲线、统计表与速度核验](DEPTH_SLOW_01X.md)。相机仍按仿真时间500Hz，C++仍按现实时间每2ms读取；以下保留旧节拍原始实测。


2026-11-02 10:29 +0800。按用户要求只做一次本机 RTX 5070 / cuda:0 实际仿真，保持相机目标500Hz、C++每2ms读取、四路320×240、固定机体，验证窗口0.5秒仿真时间。数据目录：[`depth_cpp_500hz_reference_20261102_101958_359232`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232)。

## 结论

本次测到的主要瓶颈是 **Isaac 四路相机的渲染准备、同步及等待链路**。读取窗口内249个完整主循环中，app.update平均10.091ms，渲染链路平均6.197ms，约占主循环总墙钟的61.4%。其中渲染产品Prim更新平均3.254ms，内部四路渲染设置USD同步合计平均3.048ms，单这一部分已超过500Hz对应的2ms预算。

PhysX更新平均0.641ms，OmniGraph更新平均0.961ms；C++组齐回调到拷贝四张图并标记缓冲区可用平均0.047074ms；读取平均0.0002416ms（0.242μs）。当前读取模块本身不是限制新图频率的主要阶段。读取仅获取受保护引用并做四像素探针，不代表完整图像处理或未来控制算法耗时。

这是当前场景/配置的测量结论，不能推成Isaac固定最高频率或5070硬件上限。**尚未分离纯GPU执行时间、GPU到CPU搬运时间和完整DDS端到端延迟。** 渲染区间是包含准备、提交及等待的CPU墙钟；ROS publish区间只是发布调用，不能当作相机采集到接收的完整延迟。

## 曲线和分项统计

![单次运行分项耗时曲线](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_curves.png)

横轴统一为首个成功C++读取之后的现实时间0～2.540s。A、C面板采用对数纵轴；C、D面板单位是μs，其余为ms。红虚线是2ms目标。曲线包含所有采样点，没有平滑或删除长尾。渲染内部各项存在父子嵌套，**不得把表中所有行相加**；异步发布线程与主线程也会重叠。

下表单位全部为ms。主循环及其分项统一使用完整落在读取窗口内的249次更新，每次同名区间若出现多次先合计；独立回调/发布/读取按自身事件计数。首个完整图发布早于首个读取，因此窗口内接收拷贝是187组，整次运行188组。

| 计时范围 | 样本数 | 均值ms | P95 ms | 最大ms |
|---|---:|---:|---:|---:|
| Isaac app.update | 249 | 10.090733 | 11.310079 | 70.560749 |
| 四路渲染链路 | 249 | 6.197117 | 6.371169 | 66.125197 |
| ↳ 渲染产品 Prim 更新 | 249 | 3.254195 | 3.664193 | 4.442395 |
| 　↳ 四路渲染设置 USD 同步合计 | 249 | 3.048069 | 3.436690 | 4.216882 |
| ↳ 传感器分区渲染 | 249 | 1.654184 | 1.910901 | 7.614079 |
| PhysX 更新 | 249 | 0.640546 | 0.907803 | 1.701919 |
| OmniGraph 更新 | 249 | 0.960625 | 1.084717 | 1.806014 |
| 单图 ROS publish 调用 | 994 | 0.052121 | 0.080247 | 1.600890 |
| C++ 组齐回调至四图拷贝/缓冲发布 | 187 | 0.047074 | 0.068186 | 0.092984 |
| C++ 引用读取与四像素探针 | 1269 | 0.000242 | 0.000450 | 0.000861 |
| 独立物理 pre/post 回调区间 | 250 | 0.485877 | 0.751284 | 1.539437 |
| 四路回调到达跨度（含等待） | 187 | 3.710319 | 6.609235 | 9.996526 |
| 相邻完整组到达间隔（含等待） | 186 | 13.530901 | 23.764678 | 83.958536 |
| 相邻读取间隔 | 1268 | 2.003155 | 2.000021 | 4.880964 |
| 读取唤醒迟到 | 1269 | 0.016569 | 0.000058 | 2.881007 |

三次主循环长尾为70.561、70.239、69.241ms，其内部`GeometryStreaming : postSync`中的`Wait for budget`区间分别约60.094、60.090、60.095ms。这进一步将这些长尾定位到几何流送预算等待；没有把事件名字解释成已证明的显存不足或纯GPU瓶颈。

## 数据读取与同步情况

- 仿真时钟0.016→0.516s，正好0.500s；C++读取墙钟窗口2.540000s。读取1269次，新完整组188，重复1081；读取平均499.213Hz，完整组实际74.037墙钟Hz / 377.016仿真Hz。
- 相邻读取间隔平均2.003155ms、P99 2.497045ms、最大4.880964ms；跳过2个周期、本槽完成期限超时2次。严格每2ms完成和现实每2ms一组新图均未达到。
- 接收到的四路消息数为246、247、245、245；唯一时间戳分别245、206、201、194，重复时间戳1、41、44、51。四路共同时间戳交集188，缓冲区发布也为188，**没有已经完整接收却被同步器漏掉的组**。58个不完整组被清理，末尾还有1个不完整组；这些数据只定位到观测链路，未把不完整组直接称为DDS丢包。
- 完整组内四路采集时间戳差0ns，但同组四路ROS回调的现实到达跨度平均3.710ms，最大9.997ms。时间戳同步与同时到达是两个不同指标，该跨度包含发布、传输、排队和回调调度等待。
- 这次开启了CPU性能追踪、压缩写盘和物理事件回调；它们会带来观测开销。因此本次74.037Hz是诊断样本，原未开启追踪测试的101.341Hz保留在[500Hz报告](DEPTH_500HZ.md)，两次不能视作同条件优化对比，也不能用本次短测宣称长期稳定。

## 计时方法与原始数据

新增可选`--profile-stages`，默认关闭。使用本机已有`carb.profiler-cpu.plugin`保存Chrome Trace，在每个`app.update`外加匹配标记；记录物理pre/post事件的`time.monotonic_ns()`。控制读取、ROS接收、缓冲发布沿用原C++计时和预分配存储，本任务没有修改C++算法、安装二进制、相机分辨率、场景文件或GPU选择。

Trace共256个app.update标记，与适配器CSV逐一匹配。分析仅取成功读取窗口内249个完整更新，排除了启动和跨窗口边界；其中启动期最大约635ms的整次更新不放入稳定窗口统计。Trace与进程单调时钟通过匹配标记对齐，偏移极差28.113μs、最大偏离中位数25.490μs。该对齐足够用于毫秒阶段定位，不能用它推断亚微秒跨进程延迟；C++读取亚微秒耗时是同线程两次计时器差值，与跨进程对齐无关。

- [六面板PNG](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_curves.png) / [PDF](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_curves.pdf)
- [逐次各类耗时CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_curves.csv) / [统计CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_summary.csv) / [统计及口径JSON](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_summary.json)
- [完整CPU原始Trace，gzip JSON](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/cpu_trace.json.gz) / [全部CPU区间排名，包含嵌套](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/cpu_scope_ranking.csv)
- [读取CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/reads.csv) / [相机接收CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/received.csv) / [物理事件CSV](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/physics_events.csv)
- [原输入及源码核验](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/validation.json) / [计时核验](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232/stage_timing_validation.json)

复现：

```bash
cd /home/starry/isaac-data/EGO4P0
bash scripts/run_depth_step1.sh --profile-stages --time-scale 1 --duration 0.5
# 将下行路径替换为上一步输出的新运行目录
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/plot_depth_stage_timing.py docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_101958_359232
```

原始输入核验与离线独立复算通过：时间网格、四路完整时间戳交集、首末深度/标定/状态、执行ELF与归档源码、场景SHA及两进程退出0均核对；新增分析核对256标记对应、物理pre/post配对、有限非负耗时和188组共同时间戳全部发布。三个Python文件AST检查通过。C++未改，本次未重新构建或运行CTest/231项基础回归。初次离线Matplotlib读取用户NumPy2失败，使用项目既有`PYTHONNOUSERSITE=1`环境后成功；首次绘图字体名未被Matplotlib发现，改为显式加载本机Noto CJK字体后重绘并目视核验中文完整。没有安装新依赖、发送运动指令或启动后台续跑。
