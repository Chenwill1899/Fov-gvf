# EGO2P6 开发要求

- 用户2026-10-19要求保存当前EGO2P5 c36，并把除录像外全部内容复制为本目录。后续本目录的修改只在P6进行，P5/P4/P3及服务器保存为来源，不自动改写或提交推送。
- 修改前阅读工作区HISTORY/README.md和RECORDING_RULES.md，遵守R-001；同步WORK_LOG.md和HISTORY/CHANGELOG.md。当前状态入口README.md、EGO2P6_VERSION.md、ACTIVE_NAVIGATION.md、MANUAL_CONTROL_PROFILE.md、CURRENT_CHECKPOINT.json。
- EGO2P5_*、更早版本文档及performance内原始记录保留来源身份，不批量替换历史路径、算法名称、SHA、验收结论。来源录像137文件按用户要求未复制，原件仍在P5；排除清单保存在performance/copy_from_ego2p5_20261019。
- 原样复制包括Git、日志、图片、CSV/JSON、二进制重放、缓存/构建和场景。为容纳全部非录像数据，P6内部相同不可变历史数据共用新inode；P5/P6没有共享普通文件inode。禁止修改继承performance、runtime归档及旧备份；确需写历史文件须先独立复制并记录，不能就地改变内部硬链接指向的多份记录。各来源原时间戳保留在清单，内部去重时间戳差异另账。
- 来源runtime/local_ubuntu完整移入runtime/imported_local_ubuntu_from_ego2p5；P6活动runtime/local_ubuntu独立新建/构建。服务器runtime与deployment/rtx5090保持来源证据，不默认source或执行。主构建/运行入口仍scripts/build_isaac_ros_workspace.sh和deployment/local_ubuntu/run_manual.sh、run_goal.sh。
- 本机Ubuntu22.04/ROS Humble/Isaac，ROS域42、共用/tmp/fov_gvf_user_navigation.lock和单发布者检查保持；不并行控制同一仿真，不改全局调度，默认不绑5090的0..15核。
- 继承c36算法：保存profile请求10m/s、垂向1，depth20/H.5、缓存.3s/.75m、历史16份/1s、a16/tau.10、模型延迟.20s、world/floor0与三主动策略开。未source profile旧入口默认6m/s。手柄use_fixed_goal=false，不凭二进制名判断策略。
- 物理半径.48、硬包络.595、重启.615、可靠锚点.635m；正常探测附加舒适.08m、恢复附加舒适0。风险速度.3/垂向.2，预算3.5m/12活跃秒/5m³；移动锚点/跨周期/松杆不补债。未知探索不写自由证明，真实障碍/制动/工作域为硬约束。
- 四路水平深度相机，无垂直相机，无在线全图真值；地图/占据SHA继承不改。六个附加模块仅深度不确定性开，其余动态、原增量场、球面记忆、输入辅助、共享占据关；普通历史和c36短时提名仍按保存策略工作，速度平滑关。
- 来源P5 c36原始21次20到达，按用户排除顶/底界斜角围困范围剔除1次顶界工况后正式20/20，R01/R06/R07/R09各5/5；源63核心通过，正式均值1.648ms、原始1.937ms、最大1668.216ms、最慢227.6s。全任务73启动/72飞行/9未到达保留；不是P6新飞行。稳定1ms、无停顿、任意路线及真人手柄长期操纵未验收。
- 用户明确到顶/到底进入斜角卡死不属于本次局部避障目标；仅经轨迹/录像复核的对应工况另账剔除并补新实验，原始失败状态不改写、不删除；普通卡停/超时/预算或计算问题仍纳入分析。
- 本次复制不自动启动导航；所有构建/回归/参数与离线重放结果要按实际完成情况另记，不把来源成绩写成P6新验收。
