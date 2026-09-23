# EGO1P1 当前版本

- 后续算法开发主线，2026-09-23 从 EGO1P0 完整复制；本地分支 ego1p1，保留来源未提交修改，未推送远程。
- 本轮精简为两个 Shell 主入口及其运行/回归依赖；当前使用方法以 README.md 为准。
- 保留 C++ 深度角域算法、四路90°/320×240相机、XY避障/Z独立、北通BFM/XInput、4倍Cloud、平地、RViz与性能记录。算法/速度/安全参数未调整。
- 手柄模块现位于 scripts/isaac/beitong_joystick.py；独立 KEYBOARD/MISSANDKEYBOARD 启动入口已移出活动工程。
- 未接入的 pc_gvf_core、备用场景、场景生成器、研究代理框架、旧说明和项目内构建缓存归档于 `/home/starry/isaac-data/备份/EGO1P1_before_cleanup_20260923_194915`。完整快照、SHA-256 清单和移除清单均在该目录。
- src 中的测试、Python 数值基准/合成演示与 tools 探针用于后续算法回归，因此保留；它们不参与默认 Isaac 控制。
- 构建：/tmp/fov_gvf_ego1p1_isaac_{build,install,log}；ROS日志：/tmp/fov_gvf_ego1p1_isaac_ros_log。共用原域42和单实例锁。
- WORK_LOG/performance 中继承的历史结果不代表本版本新验收；本轮实际验证另记 WORK_LOG。
