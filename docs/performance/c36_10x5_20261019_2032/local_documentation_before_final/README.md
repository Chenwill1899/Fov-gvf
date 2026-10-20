# EGO2P6

更新：2026-10-19 20:42。c36局部避障算法；本机Ubuntu22.04/ROS Humble，使用已有Isaac Sim。根目录恢复五个主要文件夹：

| 目录 | 作用 |
|---|---|
| `src/` | C++避障算法、ROS节点/消息、输入桥接与显示、配置、必要回归和夹具；路线与协议在`src/pc_gvf/config/` |
| `scripts/` | 构建、手柄/目标/仿真运行入口；`manual_profile.env`保存当前参数，`environment.sh`设置本机环境 |
| `scenes/` | Cloud和平地场景、占据文件；地图身份保持 |
| `tools/` | 目标实验驱动、独立轨迹/安全审计、证据汇总及其回归 |
| `performance/` | 运行生成物和实验结果；`runtime/install/`是编译后的程序，`runtime/build/`是可重建中间文件，其余为缓存/日志或各任务结果 |

`.git/`是隐藏的版本记录，保留提交与分支。Markdown文件说明算法、参数、状态和修改历史；`WORK_LOG.md`按工作区要求持续记录。原`config`、`profiles`、`deployment`、`runtime`四个顶层目录已合并，不再保留同内容兼容目录或重复脚本。

## 运行

```bash
cd /home/starry/isaac-data/EGO2P6
# 手柄，Isaac与RViz
bash scripts/run_manual.sh
# R07目标意图，沿用手柄避障策略
EGO2P6_GOAL_ROUTE=r07 EGO2P6_GOAL_HEADLESS=0 EGO2P6_GOAL_RVIZ=true bash scripts/run_goal.sh "r07_$(date +%Y%m%d_%H%M%S)"
```

其它路线：`r01`至`r10`、`route300`。关闭Isaac窗口或Ctrl+C退出。右杆平移、左杆升降/偏航，四轴回中0.5秒解锁。ROS域42、共用导航锁及单发布者检查保持。

## 构建与输出

```bash
# 默认只构建运行程序，安装到performance/runtime/install
bash scripts/build_isaac_ros_workspace.sh
# 需要算法回归时
EGO2P6_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh
source scripts/environment.sh
source /opt/ros/humble/setup.bash
source performance/runtime/install/setup.bash
ctest --test-dir performance/runtime/build/pc_gvf --output-on-failure
```

手柄输出：`performance/runtime/manual_runs/`；目标输出：`performance/p6_speed_20261019/local_ubuntu/`。完整CSV与审计保留；大型二进制重放默认关闭，`EGO2P6_RECORD_REPLAYS=1`按需开启。图形缓存按需再生；删除整个performance/runtime后需重新构建，运行配置源文件仍在src/scripts。

保存profile：水平请求10m/s、垂向1、depth20/H.5、a16/tau.10/延迟.20；物理.48、硬.595、重启.615、可靠锚点.635m；普通舒适.08/恢复0；探索.3/垂向.2、预算3.5m/12活跃秒/5m³。四水平深度，无垂直相机，无在线全图真值。

P5 c36来源原始21次20到达，用户指定排除范围及补跑后正式20/20；正式计算均值1.648ms、最大1668.216ms，仍有低效恢复和长尾。稳定1ms、任意路线、真人长期操纵未验收。本次仅调整本机结构，不改算法参数、不同步服务器。其它任务的5090实验资料和状态保留在performance/WORK_LOG中，不由本次目录合并重新认定。

[算法流程](ACTIVE_NAVIGATION.md) · [当前状态](EGO2P6_VERSION.md) · [手柄参数](MANUAL_CONTROL_PROFILE.md) · [精简记录](CLEANUP_REPORT.md) · [工作日志](WORK_LOG.md)
