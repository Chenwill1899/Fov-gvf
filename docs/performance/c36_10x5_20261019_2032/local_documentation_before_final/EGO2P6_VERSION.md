# EGO2P6 当前状态

2026-10-19。算法来源EGO2P5 c36；本机已精简历史数据，并继续合并为performance/scenes/scripts/src/tools五个主要目录。入口见README，完整历史见WORK_LOG与HISTORY。

## 算法

四路水平深度和真实位姿形成保守自由证据及原生障碍约束；优先生成深度方向候选，必要时使用角域调和场/流盒后备。从当前实际状态检查延迟、速度响应、加速度、完整机体运动与完全制动扫掠，最终发布一条世界系速度。

不足时可尝试已认证足迹返回/侧移、辅助升降或独立预算的低速未知探索。未知探索不写自由证明，松杆/换锚点不补额度。几何查询精确复用与保守整曲线粗筛、短时提名重证、停滞恢复和边界滞回保留。

主要源码：src/pc_gvf/src中的depth_angular_controller_node.cpp、paper_guidance.cpp、active_navigation.cpp、omni_depth.cpp和paper_replay.cpp；接口位于同包include/pc_gvf。

## 当前参数

| 项目 | 值 |
|---|---|
| 保存profile水平请求/垂向 | 10 / 1m/s；未加载profile的主脚本仍为旧默认6m/s |
| depth / horizon | 20m / .5s |
| 加速度/响应/延迟 | 16m/s² / .10s / .20s |
| 物理/硬/重启/可靠锚点半径 | .48 / .595 / .615 / .635m |
| 普通/恢复附加舒适 | .08 / 0m |
| 探索速度/垂向 | .3 / .2m/s |
| 累计未知预算 | 3.5m / 12活跃秒 / 5m³ |
| 历史/提名 | 16份、1s采样；.3s/.75m短时提名，每次重新认证 |
| 工作域 | X[-80,399.8]、Y[-60,59.8]、Z[0,19.8] |
| 感知/开关 | 四水平320×240深度，无垂直相机/在线全图真值；六附加模块仅深度不确定性开，速度平滑关 |

## 入口与数据位置

手柄scripts/run_manual.sh；目标scripts/run_goal.sh；参数scripts/manual_profile.env；环境scripts/environment.sh；路线与基础协议src/pc_gvf/config。构建和安装统一在performance/runtime，历史或其它实验结果保留各自performance子目录。默认不录二进制重放，需要时EGO2P6_RECORD_REPLAYS=1；测试构建EGO2P6_BUILD_TESTING=ON。

P5/P4和服务器不随本机目录调整修改。原服务器部署工具归档在本次HISTORY保存点，服务器独立副本和另一个任务正在记录的实验按其自身结果解释。

## 验证边界

上轮本机精简已完成3包构建、63核心/174工具平台、8参数、入口构造与安装验证；本轮迁移新安装前缀的验证单列于[CLEANUP_REPORT.md](CLEANUP_REPORT.md)末尾。本轮没有启动新的ROS节点/Isaac或导航飞行。

P5 c36来源原始21次20到达，按用户顶/底斜角围困排除口径并补跑后正式20/20；正式计算均值1.648ms、原始1.937ms、最大1668.216ms，最慢227.6仿真秒。原基线14/20与正式20/20纳入范围不同。稳定1ms、无停顿、任意路线和真人长期手柄未验收，局部恢复仍可能低效。

[运行说明](README.md) · [算法流程](ACTIVE_NAVIGATION.md) · [工作日志](WORK_LOG.md)
