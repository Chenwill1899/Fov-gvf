# 相机目标500Hz、C++每2ms读取

2026-11-02当前默认改为0.02倍速、0.2秒仿真：[最新结果与图表](DEPTH_SLOW_002X.md)。以下保留本报告当时配置与数据。


2026-11-02当前默认已改为0.1倍速：[新曲线、统计表与速度核验](DEPTH_SLOW_01X.md)。相机仍按仿真时间500Hz，C++仍按现实时间每2ms读取；以下保留旧节拍原始实测。


2026-11-02新增：[一次0.5秒分项计时、瓶颈定位与曲线](DEPTH_STAGE_TIMING.md)。追踪样本中渲染链路平均6.197ms，其中渲染产品更新3.254ms；下表仍为2026-11-01未开启追踪的原数据。

当前四路相机的目标更新频率均为500Hz，独立C++读取线程也改为500Hz（每2ms读取一次最新完整组）。默认仍使用本机RTX5070：渲染GPU0、GPU物理cuda:0；验证窗口仍为0.5秒仿真时间。

```bash
cd /home/starry/isaac-data/EGO4P0
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_depth_step1.sh
```

## 本次修改

- `src/new_controller/src/depth_reader.cpp`：绝对调度周期1,000,000→2,000,000ns；计时存储按500Hz预分配；线程名ego4p0_2ms；元数据明确记录requested_period_ns/read_hz。
- `scripts/run_depth_step1.py`：默认相机90→500Hz，manifest记录read_hz=500、read_period_ms=2。默认sensor-mode为rate，四个实际`omni:sensor:tickRate`都读回500。
- `scripts/isaac/run_depth_input_simulation.py`：直接调用时默认相机也为500Hz。物理、渲染、时间线目标步长都为0.002s。320×240、相机位姿和原Cloud场景保持。
- `scripts/analyze_depth_cpp.py`：从元数据读取周期，调度网格、漏周期、期限判断和统计阈值均使用2ms；新输出使用skipped_periods、read_over_period等通用字段，避免旧skipped_1ms_slots名称混淆。旧元数据缺省按原1ms解释，不改写旧结果。

接收线程、严格时间戳同步、预分配三缓冲及引用生命周期保持原实现。正在使用的读槽不会被写入覆盖，下一次取帧才释放旧槽；没有新完整组时继续引用最近完整组，并统计重复读取。当前输出仍为D1～D4、K1～K4、TBC1～TBC4、采集时间、光轴深度类型、v_B/R_WB。不做时间补偿，无运动指令发布。

## 0.5秒本机5070实测

证据：`docs/performance/depth_step1/depth_cpp_500hz_reference_20261101_184453_352017`。

| 指标 | 实测 |
|---|---:|
| 目标读取 / 相机频率 | 500 / 500Hz |
| 四路实际传感器tickRate配置 | 500、500、500、500 |
| 实际仿真窗口 / 墙钟读取窗口 | 0.500000 / 2.326000s |
| 实际读取 / 新同步组 / 重复读取 | 1164 / 236 / 928 |
| 读取墙钟频率 | 499.999998Hz |
| 相邻读取间隔均值 / P99 / 最大 | 2.000000 / 2.667700 / 3.542909ms |
| 引用读取耗时均值 / P99 / 最大 | 0.0002462 / 0.0005510 / 0.0007110ms |
| 跳过2ms周期 / 超过本周期完成期限 | 0 / 0 |
| 相邻间隔超过2ms的次数 | 699 |
| 同步相机新帧墙钟 / 仿真Hz | 101.341 / 473.790 |
| 四路最大时间差 | 0ns |
| 深度年龄均值 / 最大（仿真时间） | 4.488 / 8.000ms |
| 缓冲发布后年龄均值 / 最大（墙钟） | 5.894 / 68.458ms |

配置和短测读取功能通过。读取平均500Hz，但最长间隔3.543ms，所以不能宣称每一相邻读取间隔都严格不超过2ms。无漏周期表示本次唤醒/读取在下一固定槽到期前完成，不等于零调度抖动。

相机配置500Hz未达到实际墙钟500张新图/秒：本次完整同步组仅约101.34Hz；0.5s仿真读取窗口消耗约2.326s墙钟。读取频率与新图频率分开统计，重复读取没有改写成新采集。这里只改变目标频率，没有降低相机像素数或伪造帧。

引用读取耗时只计受所有权保护的最新完整帧/状态引用获取和四路像素探针，不计完整复制/渲染/ROS接收/日志/后续算法。接收复制和整个循环耗时另有分项。数据年龄的仿真/墙钟定义沿用原记录。

原始数据：[`reader_summary.json`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261101_184453_352017/reader_summary.json)、[`reads.csv`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261101_184453_352017/reads.csv)、[`frames.csv`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261101_184453_352017/frames.csv)、[`validation.json`](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261101_184453_352017/validation.json)。首末NPZ包含完整四路数组与参数，manifest/实际ELF/源码归档保存本次身份。4包Release构建、C++ CTest1/1、编排AST检查通过；保存数据独立重算2ms固定网格、漏周期、四路采集时间、标定/数组和源场景SHA通过；两进程退出0。未重新运行原231基础回归或实机，也未进行导航。

旧1ms版本、GPU平台迁移和长时数据保留在[原报告](CPP_DEPTH_INPUT.md)，均按对应日期解释。当前默认值以本文和源文件为准。项目规则、checkpoint、WORK_LOG与HISTORY已同步，无后台续跑。
