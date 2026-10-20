# 最新c36同步5090完成

更新时间：2026-10-19 20:10。目标 `ubuntu5090@192.168.31.112:/home/ubuntu5090/dhm/EGO2P6`，新运行环境 `runtime/rtx5090_c36`，Ubuntu24.04/ROS Jazzy/Intel i9-14900K/RTX5090，使用既有Isaac Python3.12。原服务器 `/home/ubuntu5090/dhm/EGO2P4` 保持为candidate24来源。

本次同步的是本机P6的c36，等价于用户选中的P5最新版本。来源P5 c36原始21次20安全到达；按用户顶/底界斜角围困排除范围剔除1次顶界工况并补跑后，R01/R06/R07/R09各5/5，正式20/20。原始失败、排除理由和来源路径不改写。**这些是本机来源成绩，本次5090没有新增导航飞行或真人手柄测试。稳定1ms/任意路线仍未验收。**

## 同步与适配

采用独立新目录，先备份旧P4活动源码、配置、说明和四个实际安装ELF：245项，归档5,087,189字节，SHA256 `e8a5b4ecab1eed742bd357ecca9f282fa4986b6d864f9b355e1dd63d5b67683e`，保存在 `/home/ubuntu5090/dhm/deployment_backups/EGO2P4_before_c36_20261019_2000`。最终245项SHA仍与旧服务器一致。

首次导出295项、219,127,487逻辑字节；含活动src/scripts/tools/profiles、全部当前scenes、根文档、本机来源正式结果/排除摘要与六个真实重放状态。测试所需旧归档地图及颜色依赖另行补齐。大量历史原始轨迹/录像、旧runtime/Git/缓存未传；它们继续保存在本机P5/P6和原服务器P4。这是可运行版本导出，非全部历史镜像。

70份C++/头文件/消息与本机P6逐字节一致；当前地图及占据SHA保持 `cfcc22694241eb2ca5e506060c136a2f9fe5b3b5e2efd76cf0d44a1da26ed1ca` / `e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8`。六项活动适配仅build/main shell、profile、benchmark launch、sim4默认路径以及旧测速CLI帮助百分号转义；没有算法或数值变化。全部原件另存 `deployment/rtx5090_c36/before_platform_adaptation`，变更SHA见[平台清单](remote_delivery/platform_adaptation.json)。

保存10m/s请求上限、垂向1、a16/tau.10、delay.20、depth20/H.5、缓存.3/.75、history16/1s、四水平相机、速度平滑关和全部硬安全/风险预算。默认不绑核；新入口兼容历史P4锁，保留共用锁/ROS42/单发布者检查。

## 5090实际验证

| 检查 | 结果 |
|---|---|
| 独立Jazzy构建 | 3包成功 |
| 核心CTest | 63/63 |
| 工具与平台pytest | 174/174 |
| 源/安装有效参数 | 8组通过 |
| RViz构造/显示选项 | 4/6通过，Node.execute为0 |
| 六个真实状态×手柄/goal重放 | 12对完整JSON与本机全等 |
| Python AST/Bash语法 | 88文件通过 |
| 三CLI帮助 | 全部通过 |
| 真正R07 Shell入口 | 最终exec拦截，协议/手柄策略/安装路径/双画面开关通过 |
| 四实际安装ELF | SHA已记录、动态库全部解析到可用依赖 |
| 新导航/真人手柄测试 | 0/未执行 |

[构建](remote_validation/build.log) · [最终核心回归](remote_validation/ctest_final.log) · [最终工具回归](remote_validation/pytest_final.log) · [参数](remote_validation/launch_checks/launch_parameters_validation.json) · [重放](remote_validation/replay_validation.json) · [真实入口](remote_validation/real_goal_entry_validation.json) · [部署核验/ELF](remote_validation/deployment_verification.json) · [完成摘要](completion_summary.json)

初次CTest61/63缺归档地图；补四文件后仍61/63缺派生颜色层，补齐完整原依赖后63/63。均未改认证源码或哈希常量，日志保留为 `ctest.log` 和 `ctest_with_fixtures.log`。首次扩展CLI验证发现旧speed_benchmark帮助字符串95%未转义触发argparse TypeError，原件/初检日志保留，仅服务器帮助转义95%%后3项通过。普通BatchMode首次认证失败后复用既有授权SSH控制连接；SFTP取目录/.失败后用归档流取回日志。失败没有省略或改写。

## 在5090运行

在5090图形桌面终端中执行，手柄接在5090：

```bash
cd /home/ubuntu5090/dhm/EGO2P6
bash deployment/rtx5090_c36/run_manual.sh
```

R07约300m路线＋Isaac＋RViz：

```bash
cd /home/ubuntu5090/dhm/EGO2P6
EGO2P6_GOAL_ROUTE=r07 EGO2P6_GOAL_HEADLESS=0 EGO2P6_GOAL_RVIZ=true bash deployment/rtx5090_c36/run_goal.sh "r07_c36_$(date +%Y%m%d_%H%M%S)"
```

r07可换r01/r06/r09，route300为旧直线300m起终点。新结果写 `performance/p6_speed_20261019/rtx5090_c36`。回退只需回到旧P4使用其原入口；本任务没有替换旧P4或安排后台飞行。

## 同期本机目录精简边界

20:12交付后复核本机原始活动树时，旧分析工具已被另一项20:11明确记录的本机P6精简任务删除，部分入口/测试夹具路径正迁移，因此原完整树相等检查在首个缺失文件处未通过。没有恢复或覆盖该任务的改动；逐项差异另存local_source_delivery_recheck.json，当前生产C++/头/消息29项仍与本次冻结源全等。5090保持本任务20:00冻结、20:12交付验证通过的c36，后续目录精简不在本次同步范围；远端验收与备份不受影响。见[交付边界](DELIVERY_SCOPE.json)。
