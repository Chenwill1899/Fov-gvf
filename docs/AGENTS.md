# EGO2P6 开发要求

本文件由根目录AGENTS.md引入，适用于整个EGO2P6项目。

- 用户2026-10-19要求精简本机P6，只保留关键运行/算法与必要验证。该明确要求取代早期“本目录完整保存全部非录像数据”默认范围；P5/P4及服务器不随本次精简修改。旧失败原件留P5，不能把清理解释为实验通过。
- 修改前读工作区HISTORY/README.md与RECORDING_RULES.md，遵守R-001；同步本项目docs/WORK_LOG.md与HISTORY/CHANGELOG.md。当前入口均在docs/：README.md、EGO2P6_VERSION.md、ACTIVE_NAVIGATION.md、MANUAL_CONTROL_PROFILE.md、CURRENT_CHECKPOINT.json。
- 活动本机构建/运行：scripts/build_isaac_ros_workspace.sh、scripts/run_manual.sh和run_goal.sh；docs/performance/runtime/install。默认BUILD_TESTING=OFF，EGO2P6_BUILD_TESTING=ON启用回归。静态依赖配置放src/pc_gvf/config和scripts，docs/performance/runtime只放可重建运行产物。
- 大型二进制诊断快照默认关闭，EGO2P6_RECORD_REPLAYS=1显式开启。保留必要运行日志、轨迹/全量样本和审计，不自动删除新失败现场。普通构建/缓存可再生，人工清理影响恢复时仍记账。
- 不就地改写来自历史的硬链接；需编辑时用新inode替换。旧数据恢复来源P5与HISTORY/snapshots/EGO2P6_cleanup_*清单/小源码备份；不重新复制30GB冗余到本机P6。
- 本机Ubuntu22.04/ROS Humble/Isaac；ROS域42、/tmp/fov_gvf_user_navigation.lock和单发布者检查保持，不并行控制同一仿真，不改全局调度。默认不绑5090的0..15核。服务器部署工具已归档到/home/starry/isaac-data/HISTORY/snapshots/EGO2P6_structure_20261019_204101/rtx5090_c36，未获明确授权不自动同步本机后续清理。
- 继承c36：保存profile请求10m/s、垂向1、depth20/H.5、缓存.3s/.75m、历史16份/1s、a16/tau.10、模型延迟.20s；手柄use_fixed_goal=false，未source profile旧默认6m/s。物理.48、硬.595、重启.615、可靠锚点.635m；普通舒适.08m、恢复0。
- 探索速度.3/垂向.2，累计预算3.5m/12活跃秒/5m³；换锚点/跨周期/松杆不补债。未知探索不写自由证明；真实障碍、制动、完整工作域为硬约束。
- 四路水平深度，无垂直相机，无在线全图真值，原地图SHA不变。六附加模块仅深度不确定性开，普通历史和短时提名保留，速度平滑关。
- 来源c36原始21次20到达，按用户排除顶/底斜角围困并补跑后正式20/20，原始失败保留P5。正式均值1.648ms、最大1668.216ms、最慢227.6s；稳定1ms、无停顿、任意路线及真人长期手柄未验收。普通卡停/超时/预算/计算问题继续纳入分析。

- 实验驱动、独立审计、汇总脚本及其回归统一在src/pc_gvf/tools/，项目根目录不再保留tools/。自动路线仍从scripts/run_goal.sh启动。

- 当前GitHub发布使用Chenwill1899/Fov-gvf的EGO2P6分支；docs/performance紧凑报告应版本化，docs/performance/runtime生成物保持忽略。用户本次明确授权在新分支上传验证后删除旧远端ego1p0，dev及其它引用保持。
