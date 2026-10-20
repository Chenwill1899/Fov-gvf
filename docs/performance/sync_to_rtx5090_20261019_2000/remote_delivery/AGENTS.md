# EGO2P6 RTX5090部署要求

- 用户2026-10-19明确授权同步最新c36到本服务器。活动根/home/ubuntu5090/dhm/EGO2P6，运行环境deployment/rtx5090_c36/environment.sh与runtime/rtx5090_c36；旧EGO2P4保留来源。
- 修改前阅读deployment/rtx5090_c36/SOURCE_RECORDING_RULES.md和本项目WORK_LOG.md；按R-001更新本项目WORK_LOG和本机/home/starry/isaac-data/HISTORY/CHANGELOG.md。平台修改前原件在deployment/rtx5090_c36/before_platform_adaptation，不修改来源性能记录或SHA。
- 本机来源算法要求见before_platform_adaptation/AGENTS.md。这里仅做Jazzy、Python3.12、运行目录适配；所有核心C++/头文件与本机P6一致，不改数值参数，不自动升级依赖、提交或推送。
- 默认不绑核，可显式设置EGO2P6_CONTROLLER_CPU_AFFINITY；ROS域42、/tmp/fov_gvf_user_navigation.lock、历史P4锁和单发布者检查保持；不并行控制同一仿真。
- 来源20/21与正式20/20不是5090新飞行；顶/底斜角排除只按用户明确范围、复核证据另账并补跑，原始失败保留。稳定1ms/任意路线/真人长期操纵未验收。
- 当前导出含活动源码/脚本/工具/地图/profile、文档、精选原始来源摘要和六个真实重放状态；历史大批量原始轨迹/录像/旧构建仍在本机P5/P6，不能声称服务器是全量历史副本。
