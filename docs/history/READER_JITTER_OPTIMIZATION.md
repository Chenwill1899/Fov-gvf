2026-11-02 18:32 +0800 更新：[FIFO10实测和内核逐次定位](../../../EGO4P3/docs/READER_FIFO_RESULTS.md)已完成；FIFO轮0/5008次>2.2ms、最长2.010857ms，但95/100完整相机组，严格2.000ms和长期保证未通过。优化的无重型追踪入口已准备，三轮复验尚未运行。

# 管理员测试启动修复与普通调度等待优化

2026-11-02 17:46 +0800。用户提供的管理员测试在`normal_200us`等待读取线程就绪时超时，只有第一例失败记录，FIFO例和perf录制均未开始。**这批失败记录没有可用于判断21次迟到具体抢占者的内核调度或IRQ数据。** 原失败目录与日志完整保留：[原会话](../../../EGO4P0/docs/performance/reader_jitter_20261102_165205/privileged_20261102_171624)。

## 本轮结果

修复授权入口漏传桌面会话变量的问题，并完成普通用户启动、C++等待优化及授权门握手测试。新默认为500μs短自旋、读取线程timer slack 1ns、锁定当前接收进程已映射内存；0.02倍速、0.2秒仿真、本机RTX5070、四路320×240、500仿真Hz相机及每2ms现实读取保持。

同场景普通调度基线有19/5008个间隔>2.2ms；500μs候选/复验/授权门测试分别2、7、14次，仍有4.648ms最坏间隔和漏周期。CPU占用约从单个逻辑CPU的7.3%升到24.6%～24.7%。这是一组改善多数间隔但仍有长尾的候选，**严格现实逐次2ms和最坏延迟改善均未通过**。1000μs自旋没有更好结果，却接近49.5%单核占用，故未设为默认。

## 启动失败与修复

原Kit日志停在`Waiting for RtPso async group async compilation`，随后等待GPU驱动编译主RTX管线；始终未产生simulator_ready，C++和perf均未启动。旧监督脚本`setpriv --reset-env`清除了DISPLAY、XAUTHORITY、XDG_RUNTIME_DIR，日志也显示无法打开默认显示。单凭日志不能断言只有这一种原因，着色器缓存冷暖等也可能影响启动。

本轮做了三个普通用户启动检查：

1. 同样清理环境并启用no-new-privs，外层45秒看门狗触发；约42秒才进入读取器，终止时没有收到完整帧，保留为**未完成**，不称通过。目录`depth_cpp_500hz_reference_20261102_172310_700014`。
2. 保留桌面环境和no-new-privs，完成0.2秒仿真，98组/1跳周期；目录`depth_cpp_500hz_reference_20261102_172513_627113`。
3. 清理环境后仅补回修复脚本所用的三项白名单变量，完成0.2秒仿真，96组/0跳周期；目录`depth_cpp_500hz_reference_20261102_172614_006577`。

修复入口在降权后用`env`只补回本地DISPLAY及本用户拥有的认证/运行目录；root监督程序仍使用固定清洁环境，不执行用户项目代码为root。等待期间每10秒显示当前启动阶段；外层让原编排器先报告180秒启动超时和清理，监督上限220秒，并在错误记录保存末尾Kit日志。没有通过盲目增加仿真时长代替修复。

## C++改动与测量

- 保持2ms绝对时间网格与超时跳槽规则；仅在尚未到预唤醒时刻时调用绝对睡眠，随后用CPU pause指令短自旋。
- 增加可恢复的`--timer-slack-ns`（0表示保持继承值）和当前映射内存锁定选项；实际值和errno写入元数据，本轮优化例读回timer slack=1ns、锁定错误码0。定时器松弛是调用线程的属性，[Linux说明](https://man7.org/linux/man-pages/man2/PR_SET_TIMERSLACK.2const.html)；当前映射内存锁定用于减少换页风险，[mlock说明](https://man7.org/linux/man-pages/man2/mlock.2.html)。它们不等于实时调度保证。
- 可配置自旋0～1000μs；选择500μs以限制CPU代价。未启用持续纯自旋，默认FIFO仍为0。
- 增加读取线程CPU时间、主动/非主动上下文切换与缺页计数；计数在循环外采集，CPU占用覆盖首帧等待及有效读取的整个读取线程循环。
- 统计补P99.9、>2.2ms次数，仍从原始单调时钟重新计算，不把跳过周期或重复图像改成正常采集。

![普通调度候选对照](../../../EGO4P0/docs/performance/reader_jitter_repair_20261102_172308/jitter_comparison.png)

| 例 | >2.2ms次数/总间隔 | P99 ms | P99.9 ms | 最大ms | 跳周期 | 完整组 | 读取单核CPU |
|---|---:|---:|---:|---:|---:|---:|---:|
| 旧等待 | 19/5008 | 2.000997 | 2.908768 | 3.917493 | 0 | 98 | 7.3% |
| 500μs候选 | 2/5008 | 2.000047 | 2.016182 | 2.564290 | 0 | 99 | 24.6% |
| 1000μs候选 | 2/5008 | 2.000037 | 2.022059 | 2.647861 | 0 | 96 | 49.5% |
| 500μs复验 | 7/5005 | 2.000736 | 2.218328 | 4.648172 | 1 | 98 | 24.7% |
| 500μs授权门测试 | 14/5007 | 2.011331 | 2.816516 | 4.629674 | 1 | 98 | 24.6% |

各例都是0.02倍/.2s、相同相机像素和CPU5，均开启同样的Isaac CPU profiler，使用SCHED_OTHER，FIFO并未生效。候选组合同时涉及pause等待、timer slack和内存锁定，本组不把收益分摊到其中某一项。500μs授权门测试仅验证循环前握手，仍为普通调度，不冒充FIFO测试。图保留全部长尾，不能只挑首次候选2次作为稳定成绩。完整组96～99也说明本轮短测有未齐组；未宣称相机链路全部达标。

## 修复后的管理员方案

读取器增加仅在计时循环前使用的有界授权门：读取进程就绪后，root监督者验证PID为本次编排的直属子进程、UID为starry、可执行路径为本项目depth_reader，随后只对该读取进程设置RLIMIT_RTPRIO10并释放授权门。Isaac与编排器的RT限额保持0，不把实时权限传给渲染器。原C++读取线程再自行设置FIFO10，按实际策略/优先级记录和复核；ROS接收线程保持普通策略。监督器和perf捕获进程在启动普通编排后避开CPU5/SMT11，结束恢复监督器原亲和，避免捕获线程自身干扰读取核心。

授权门已以普通用户、RT限额0完成端到端测试，验证文件为[gate_smoke_validation.json](../../../EGO4P0/docs/performance/reader_jitter_repair_20261102_172308/gate_smoke_validation.json)。**管理员资源限额提升、FIFO10和perf录制尚未重跑，不能声称已通过。**

在用户自己的Ubuntu桌面终端重新运行原命令即可：

```bash
sudo /usr/bin/python3 -I /home/starry/isaac-data/EGO4P0/scripts/run_reader_privileged_check.py
```

脚本仍固定两例普通200μs和FIFO10/200μs，显式关闭新timer slack/内存锁定以隔离优先级影响；每例0.02倍/.2s、最多15秒内核记录。只记录CPU5调度/IRQ/softirq/hrtimer，以及所有CPU上目标CPU5的唤醒事件。仅局部资源限额、线程策略及有界perf事件生效，进程结束即消失；未修改全局sysctl、IRQ掩码、governor、内核或/etc，也不收取聊天密码。

当前会话仍不能复用用户终端中的sudo认证，`sudo -n`返回需要密码；这是OS认证边界，不是自动审批拒绝。管理员密码仍应只在用户终端交给sudo。

## 数据与复现

- [全部对照CSV](../../../EGO4P0/docs/performance/reader_jitter_repair_20261102_172308/jitter_comparison.csv) / [JSON](../../../EGO4P0/docs/performance/reader_jitter_repair_20261102_172308/jitter_comparison.json) / [图PDF](../../../EGO4P0/docs/performance/reader_jitter_repair_20261102_172308/jitter_comparison.pdf)
- 最终默认复验：[运行数据](../../../EGO4P0/docs/performance/depth_step1/depth_cpp_500hz_reference_20261102_173836_169004/reader_summary.json)；授权门普通用户检查：[运行数据](../../../EGO4P0/docs/performance/reader_jitter_repair_20261102_172308/gate_smoke_run/reader_summary.json)。
- 原根权限失败：[privileged_run.json](../../../EGO4P0/docs/performance/reader_jitter_20261102_165205/privileged_20261102_171624/normal_200us/privileged_run.json)、[Kit日志](../../../EGO4P0/docs/performance/reader_jitter_20261102_165205/privileged_20261102_171624/normal_200us/kit.log)。

```bash
cd /home/starry/isaac-data/EGO4P0
# 新普通用户默认：500μs短自旋、1ns timer slack、锁定当前映射
bash scripts/run_depth_step1.sh --profile-stages
# 按旧等待参数做普通调度对照
bash scripts/run_depth_step1.sh --profile-stages --spin-us 200 --timer-slack-ns 0 --no-lock-memory
```

本轮两次4包Release构建成功，最终CTest1/1（含既有三缓冲与同步检查）通过；Python AST、白名单环境端到端检查、实际输入/时间网格/源码及ELF/场景验证、普通调度授权门测试和比较图目视核对完成。原231基础回归、实机、导航、修复后管理员分支未执行。所有失败和较差候选均保留，当前只有输入模块，未新增运动输出或后台续跑。
