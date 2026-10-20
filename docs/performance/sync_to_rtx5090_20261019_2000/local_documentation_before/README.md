# EGO2P6：从 EGO2P5 c36 保存版本建立的本机项目

更新：2026-10-19 19:41。本目录为 `/home/starry/isaac-data/EGO2P6`，继承 c36 避障算法，只做项目路径、运行身份和独立安装适配。EGO2P5 原目录及全部录像保留。

已复制来源中除录像外的全部内容：97,009 个普通文件、5,419 个目录、118 个链接；137 个录像文件未复制。代码、配置、场景、Git、日志、图片、CSV/JSON、原始二进制重放、旧构建与缓存均保留。P6 内不可变历史数据做内容去重，P5/P6 没有共享普通文件 inode；原始时间戳与去重差异逐条记账。

本次 P6 独立构建3包、核心63/63、工具/平台174/174通过；源码/安装参数8组、RViz开关4组与显示选项6组、实际R07 Shell命令截获核验通过。6份冻结状态在 P5/P6 手柄/goal 重放器上共12对决定全等。**本次没有启动新仿真或真人手柄飞行。**

来源 P5 c36 原始21次运行20次安全到达，按用户顶/底界斜角围困排除范围剔除1次顶界工况并补跑后，R01/R06/R07/R09正式各5/5。正式平均避障1.648ms、原始1.937ms，最大1668.216ms，最慢227.6仿真秒；仍有低效恢复和长尾，稳定1ms、无停顿及任意路线可靠性未通过。这是来源成绩，不能写成 P6 新飞行验收。

## 当前运行

手柄（Isaac和RViz，右杆平移、左杆升降/偏航，四轴回中0.5秒解锁）：

```bash
cd /home/starry/isaac-data/EGO2P6
bash deployment/local_ubuntu/run_manual.sh
```

R07自动目标意图、仍使用手柄避障策略，Isaac和RViz同时显示：

```bash
cd /home/starry/isaac-data/EGO2P6
EGO2P6_GOAL_ROUTE=r07 EGO2P6_GOAL_HEADLESS=0 EGO2P6_GOAL_RVIZ=true EGO5090_MANUAL_POLICY=1 bash deployment/local_ubuntu/run_goal.sh "p6_r07_view_$(date +%Y%m%d_%H%M%S)"
```

300米默认路线将 `EGO2P6_GOAL_ROUTE=r07` 改为 `EGO2P6_GOAL_ROUTE=route300`；R01/R06/R09分别使用r01/r06/r09。新目标实验输出到 `performance/p6_speed_20261019/local_ubuntu`，手柄日志在 `runtime/local_ubuntu/manual_runs`。关闭Isaac窗口或Ctrl+C退出；脚本共用导航锁，勿同时运行多个版本。

需要重建时：`bash scripts/build_isaac_ros_workspace.sh`。活动安装为本目录 `runtime/local_ubuntu/install`。复制的 P5 本机 runtime 完整归档在 `runtime/imported_local_ubuntu_from_ego2p5`；更早服务器 runtime 保留来源身份。历史脚本和旧构建仅作来源记录，日常使用上述 P6 入口。

## 阅读入口

- [当前算法、参数、证据和未完成事项](EGO2P6_VERSION.md)
- [避障算法构成与执行流程](ACTIVE_NAVIGATION.md)
- [手柄参数与运行方式](MANUAL_CONTROL_PROFILE.md)
- [本次复制和独立验证报告](performance/copy_from_ego2p5_20261019/RESULTS.md)
- [继承的P5完整优化报告](performance/iterative_recovery_20261019_1425/RESULTS.md)，其中录像链接仍指向保留的P5原件
- [机器状态](CURRENT_CHECKPOINT.json)、[工作日志](WORK_LOG.md)、[目录修改规则](AGENTS.md)

`EGO2P5_*` 及更早文件保留原版说明，performance内的路径、SHA与验收身份不批量替换。复制时磁盘余量约2.4GiB，后续大量新重放/录像会继续占空间；本次没有删除来源数据。
