# EGO2P6

2026-10-19 22:26 — **5090 c36十路线各5次正式实验完成：47/50安全到达，3次普通失败。** 原始52次飞行，顶界斜角围困2次按用户既定范围复核排除并同路补跑，另2次无飞行初始化挂起留原失败重试。普通失败：R05/原始第3次 ABORTED_EXPLORATION_STALL 115.183s；R02/原始第5次 TIMEOUT 600.000s；R06/原始第7次 TIMEOUT 600.000s。全程避障逐次均值再平均2.630ms，正式最坏1456.541ms；原始含排除均值2.996ms/最坏2155.939ms。无画面/无RViz/无视频、同一源码与安装/参数冻结；所有原轨迹与审计留服务器，本机保留简单表和紧凑证据。本轮没有优化算法或重建；不是稳定1ms/任意路线验收。[逐次数据](performance/c36_10x5_20261019_2032/formal_results.csv) · [本轮汇总](performance/c36_10x5_20261019_2032/RESULTS.md)。

目录更新：2026-10-20。c36局部避障算法；本机Ubuntu22.04/ROS Humble，使用已有Isaac Sim。运行与源码分为三个主要文件夹，说明与规则集中在docs/：

| 目录 | 作用 |
|---|---|
| `docs/` | 项目说明、维护规则、版本进度、工作日志与精简记录；docs/performance/子目录保存实验记录与运行产物，从README.md开始阅读 |
| `src/` | C++避障算法、ROS节点/消息、输入桥接与显示、配置、必要回归和夹具；路线与协议在`src/pc_gvf/config/`，实验驱动/审计/汇总工具在`src/pc_gvf/tools/` |
| `scripts/` | 构建、手柄/目标/仿真运行入口；`manual_profile.env`保存当前参数，`environment.sh`设置本机环境 |
| `scenes/` | Cloud和平地场景、占据文件；地图身份保持 |

`.git/`是隐藏的版本记录，保留提交与分支。说明文件集中在`docs/`，`docs/WORK_LOG.md`按工作区要求持续记录；根目录`AGENTS.md`仅为完整规则的自动发现入口。本文代码与命令中的相对路径均以项目根目录为基准。原`config`、`profiles`、`deployment`、`runtime`四个顶层目录已合并，不再保留同内容兼容目录或重复脚本。

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
# 默认只构建运行程序，安装到docs/performance/runtime/install
bash scripts/build_isaac_ros_workspace.sh
# 需要算法回归时
EGO2P6_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh
source scripts/environment.sh
source /opt/ros/humble/setup.bash
source docs/performance/runtime/install/setup.bash
ctest --test-dir docs/performance/runtime/build/pc_gvf --output-on-failure
```

手柄输出：`docs/performance/runtime/manual_runs/`；目标输出：`docs/performance/p6_speed_20261019/local_ubuntu/`。完整CSV与审计保留；大型二进制重放默认关闭，`EGO2P6_RECORD_REPLAYS=1`按需开启。图形缓存按需再生；删除整个docs/performance/runtime后需重新构建，运行配置源文件仍在src/scripts。

保存profile：水平请求10m/s、垂向1、depth20/H.5、a16/tau.10/延迟.20；物理.48、硬.595、重启.615、可靠锚点.635m；普通舒适.08/恢复0；探索.3/垂向.2、预算3.5m/12活跃秒/5m³。四水平深度，无垂直相机，无在线全图真值。

P5 c36来源原始21次20到达，用户指定排除范围及补跑后正式20/20；正式计算均值1.648ms、最大1668.216ms，仍有低效恢复和长尾。稳定1ms、任意路线、真人长期操纵未验收。本次仅调整本机结构，不改算法参数、不同步服务器。其它任务的5090实验资料和状态保留在docs/performance和docs/WORK_LOG.md中，不由本次目录合并重新认定。

[算法流程](ACTIVE_NAVIGATION.md) · [当前状态](EGO2P6_VERSION.md) · [手柄参数](MANUAL_CONTROL_PROFILE.md) · [精简记录](CLEANUP_REPORT.md) · [工作日志](WORK_LOG.md)

## GitHub保存范围

当前发布分支为`EGO2P6`，仓库为`Chenwill1899/Fov-gvf`。源码、必要测试夹具、场景、项目说明与`docs/performance/`中的紧凑实验报告纳入版本记录；`docs/performance/runtime/`中的编译安装、日志和缓存不上传。新检出后先运行`bash scripts/build_isaac_ros_workspace.sh`重建，再使用上方原启动命令；本机ROS Humble和Isaac Sim需另行安装。实验报告中的冻结路径对应其生成时环境。
