> 当前默认已按新要求改为相机目标500Hz、C++每2ms读取。下文为18:36以前的1ms版本报告，保留历史结果；最新配置和实测见[500Hz说明](DEPTH_500HZ.md)。

# EGO4P0 C++第一步输入与本机5070运行

当前采用本机RTX 5070（GPU0）进行相机渲染和GPU物理仿真，物理设备`cuda:0`。默认仿真读取窗口已按最新要求从5s缩短为0.5s，默认相机目标90Hz。第一步接收、同步、状态转换、缓冲管理和1ms循环全部为C++；Python只用于Isaac适配、启动和离线统计。读取/ROS线程仍运行在CPU上，没有把它们写成CUDA计算。

## 运行

```bash
cd /home/starry/isaac-data/EGO4P0
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_depth_step1.sh
# 同样仅0.5s仿真时间，尝试1000Hz目标
bash scripts/run_depth_step1.sh --camera-hz 1000
```

0.5s从第一组完整输入就绪后的ROS `/clock`起算，不包括Isaac启动和暖机；启动本身仍需十余秒。固定机体位于原Cloud40m场景起点，四路仍为320×240/32FC1，场景文件不改。预检本机GPU0名称含5070，保存型号/UUID到`gpu_device.json`；若设备不符明确拒绝。`active_gpu=0`、`multi_gpu=false`、`SimulationManager.setup_simulation(device='cuda:0')`，运行结果再次记录实际物理设备。没有远程服务器调用。

## C++实现

- `src/new_controller/include/new_controller/depth_input.hpp`：`ControllerInputView`、`DepthFrame`、`BodyState`、K/T_BC标定、严格相同时间戳四路同步、SPSC三缓冲。
- `src/new_controller/src/depth_reader.cpp`：独立ROS接收线程与1ms控制读取线程。`new_controller`由ament_python改为ament_cmake/C++17，旧Python实现及旧build/install已保存并从活动包移出。
- `scripts/analyze_depth_cpp.py`：测试结束后分析CSV和二进制快照，生成NPZ、统计和独立校验。

ROS接收侧收到完整相同时间戳的四路后，将真实像素复制到预分配写槽并原子发布；标定不变时只进行一次完整校验，状态在接收侧转为v_B/R_WB。控制线程只借用最新完整帧及当前状态引用，整个周期由读槽持有；到下一次取帧时才归还旧读槽，写线程不能覆盖正在使用的槽。未使用的中间完整帧可被覆盖，不建立图像历史队列；最多4个未完成时间组也有界丢弃。首末图像只作为测试证据保存。

三完整图像槽约3.69MB；另预分配首帧证据和可选复制槽，图片空间固定复用。DDS接收和部分组占用额外有界内存。计时行也在开始前分配固定容量；超过容量明确失败，结束后统一写盘。引用生命周期截止到同一读线程下一次`latest()`调用，不可跨周期保留指针，也不可增加第二个并发读者而不更改所有权协议。

读取线程使用`CLOCK_MONOTONIC + clock_nanosleep(TIMER_ABSTIME)`按固定网格调度；晚于下一槽时跳过过期槽，逐行保存跳过数，不补造读取。默认本次进程把CPU5留给读取，排除其SMT兄弟CPU11供渲染/ROS使用；只改变本次进程亲和性。到期前200us短自旋，其余时间睡眠。没有更改系统内核/CPU调速器；一次尝试SCHED_FIFO优先级5返回EPERM，实际仍SCHED_OTHER。默认不再请求无权限优先级。

输出为四路D、K、T_BC、原始采集t、光轴Z米制类型，以及当前v_B、R_WB及状态时间。光学系右/下/前，机体系前/左/上，`p_B=T_BC*p_C`，`v_B=R_WB.T*v_W`。四路时间相同后直接使用，状态时间另记，不做时间补偿。NaN/Inf/非正深度原样保留，不当作已认证自由空间。

## 最新本机5070、0.5秒结果

默认90Hz证据：`docs/performance/depth_step1/depth_cpp_90hz_reference_20261101_183104_292042`；1000Hz目标证据：`docs/performance/depth_step1/depth_cpp_1000hz_reference_20261101_183324_380804`。两次进程均退出0，源场景SHA未变，第一步数据验证通过。

| 指标 | 默认90Hz短测 | 1000Hz目标短测 |
|---|---:|---:|
| 实际仿真读取时长s | 0.500000 | 0.500000 |
| 对应墙钟读取时长s | 0.500000 | 4.593000 |
| 实际读取 / 新完整组 | 500 / 45 | 4580 / 462 |
| 引用读取均值 / P99 / 最大ms | 0.0001988/0.0004411/0.011912 | 0.0001833/0.0004310/0.013946 |
| 相邻读取间隔均值 / P99 / 最大ms | 1.002004/1.124322/2.059300 | 1.003057/1.495418/3.572378 |
| 明确跳过1ms周期 | 1 | 14 |
| 同步新图组墙钟Hz | 89.614 | 100.546 |
| 同步新图组仿真Hz | 90.000 | 922.000 |
| 重复读取 | 455 | 4118 |
| 深度年龄均值 / 最大ms（仿真时间） | 13.733/22.222 | 2.199/4.000 |
| 缓冲发布→读取年龄均值 / 最大ms（墙钟） | 5.626/12.153 | 6.603/77.132 |

`read_ms`在reference模式只计受所有权保护的帧/状态引用获取和四路实际像素探针；不含完整图像复制、ROS反序列化、接收线程的状态转换或后续控制算法。`--read-mode copy`在此基础上复制四张图到预分配空间。标定校验、接收复制发布耗时和循环工作耗时另有统计。引用模式的极短耗时不可当成完整算法或相机曝光到控制端耗时。

这两次短测不支持长期严格实时保证。当前每次严格1ms及墙钟1000Hz新图均未通过；配置1000Hz不等于实际达到。`depth_age_ms`是ROS仿真时间，`buffer_age_ms`仅指完整缓冲发布后的墙钟年龄，两者不可互换。

每个目录含`reads.csv`（每次耗时/间隔/迟到/漏槽/时间/年龄）、`received.csv`（每路实际收到的时间戳）、`frames.csv`（完整组发布/覆盖记录）、`reader_summary.json`、`first_input.npz`、`last_input.npz`、实际ELF及SHA/源码归档。输入校验独立重算调度槽、间隔、四路接收时间戳交集、首末数组/K/T/状态和场景身份。

当前数据：[reader_summary.json](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_90hz_reference_20261101_183104_292042/reader_summary.json)、[validation.json](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_90hz_reference_20261101_183104_292042/validation.json)、[reader_summary.json](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_1000hz_reference_20261101_183324_380804/reader_summary.json)。

## 优化过程保留结果

在最新0.5s要求之前，已完成同场景Python基线、C++复制/引用模式、60/90/120/240/1000Hz探索，以及两次30墙钟秒负载检查。这些运行都在本机5070渲染；当时物理设备仍为CPU，不能将其标成新的GPU物理成绩。新要求后只进行了上述两次0.5s验证。

Python基线`depth_step1_20261101_175241_549233`完整读取均值0.139582ms，最终C++完整复制60Hz均值0.018018ms，约7.75倍；优化同时包含预分配、提前校验、接收与统计分离，不能把收益全归给语言。固定机体保存数据中，四路有效掩码、有限深度像素、K和T_BC与Python基线逐项相同（差0），比较JSON保存在HISTORY本任务保存点。

| 测试 | 相机目标Hz | 仿真/墙钟s | 完整组墙钟Hz | 读取均值ms | 间隔P99/最大ms | 跳过1ms周期 |
|---|---:|---|---:|---:|---|---:|
| final_cpu_physics_reference1000 | 1000 | 5.000/43.299 | 107.80 | 0.0001703 | 1.262/7.022 | 102 |
| current_local5070_gpu_physics_1000hz_0.5s | 1000 | 0.500/4.593 | 100.55 | 0.0001833 | 1.495/3.572 | 14 |
| final_cpu_physics_reference120 | 120 | 5.000/5.376 | 73.24 | 0.0001670 | 1.257/3.238 | 9 |
| final_cpu_physics_copy60 | 60 | 5.000/5.091 | 58.50 | 0.0180176 | 1.123/3.262 | 9 |
| final_cpu_physics_reference90 | 90 | 5.000/5.173 | 85.58 | 0.0001662 | 1.178/2.598 | 9 |
| 30s_depth_load_cpu_physics | 90 | 29.356/30.000 | 87.29 | 0.0001561 | 1.170/3.925 | 52 |
| 30s_viewport_load_cpu_physics | 90 | 29.300/30.000 | 74.63 | 0.0001651 | 1.166/4.072 | 33 |
| current_local5070_gpu_physics_0.5s | 90 | 0.500/0.500 | 89.61 | 0.0001988 | 1.124/2.059 | 1 |


完整14次C++记录（含开发阶段）见[all_cpp_runs.csv](../../../EGO4P0/docs/performance/cpp_depth_migration_20261101/all_cpp_runs.csv)。较早1000Hz目标优化前同步组约67.36墙钟Hz，关闭无用headless视口、限制渲染线程并采用随渲染帧触发后约107.23Hz；相机320×240不变。30秒额外原1280×720视口负载下出现33个跳过周期、最长读取间隔4.072ms，进一步表明不能用0.5s或5s平均值宣称长期严格1ms。

## 构建和检查

4个ROS包Release构建成功；C++ CTest 1/1通过，包含三缓冲读槽保护、最新帧替换、20万次并发发布、乱序四路严格同步、有界部分组、非零机体速度转换及非法姿态拒绝。缓冲/同步核心的ASan/UBSan压力检查通过（关闭LeakSanitizer，未对整个ROS进程做同类检测）；首次LeakSanitizer因沙箱ptrace限制报错，未声称泄漏检查通过。首次CMake配置因旧`starry@localhost`维护者邮箱不符合ament校验失败，改为有效本地域占位后成功，失败日志保留。原231项基础回归未重跑，其源码未改。没有导航、飞行避障或实机验收。

新算法C++要求已写入docs/AGENTS.md。原Python包及旧build/install、旧说明和当前代码均在`/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_cpp_depth_20261101_175238`保存。当前没有后台仿真或自动续跑。

实现参考：[ROS2实时内存与线程设计](https://design.ros2.org/articles/realtime_background.html)、[Linux绝对时间等待](https://man7.org/linux/man-pages/man2/clock_nanosleep.2.html)、[Isaac相机发布频率](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_publish_rate.html)。本机还核对了安装版本的RenderingManager、SimulationManager和ROS2CameraHelper源码；auto模式tickRate=0表示随渲染帧触发，物理/渲染/时间线设置共同决定名义相机周期。
