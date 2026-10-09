# 2026-09-24 论文实现验收证据

正式结论见 ../../PAPER_ACCEPTANCE.md；完整导航未通过。

- build_final.log、numeric_final.log：最终构建和数值检查。
- ctest_final.log、platform_final.log：最终完整回归；CTest中的闭环到达失败不可忽略。
- closed_loop_final.csv / .log：最终八场景10秒闭环原始数据；外部相机数学夹具，无ESDF保护。
- ros_strict_final.json、ros_strict_controller.log：最终真实ROS节点的严格拒绝运动探针。
- isaac_strict_verified.log、isaac_strict_performance.md：最终机载四水平相机严格停车试验。
- intermediate_*：实施中的失败证据；包括NumPy依赖冲突与严格域下闭环失败。
- exploratory_*以及isaac_cloud.log、isaac_performance.md：**严格观测规则完成前的探索结果**；
  其中运动、零碰撞、12/12等结论不可用于当前严格版本验收。

Isaac日志中的系统rclpy导入失败后使用内部Humble，以及退出rosout context invalid告警均保留。
验收不使用该告警推断运动成功；以进程退出、控制状态、位移和ESDF介入分别判断。
