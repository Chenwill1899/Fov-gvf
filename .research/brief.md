# Fov-gvf 研究任务书

## 问题与目标
在现有 ROS 2 Jazzy 框架上做纯深度图流场 GVF，目标计算耗时 < 1 ms。
这是用户研究目标，尚无性能达标证据。允许实验推翻候选方法，不为达标牺牲安全语义。

## 本次范围
用户授权搭建 HiAgent agent 框架和证据盘点。本次不修改算法，不启动性能实验、GPU 工作或实机工作。
后续执行根据实际用户指令更新授权；不重复确认已有授权。

## 现有基础
- Python 主路径：src/pc_gvf/pc_gvf/depth_angular_core.py，NumPy/SciPy 角度域 guidance。
- ROS 2 适配：depth_angular_controller.py，深度图、CameraInfo、里程计和意图到 PositionCommand。
- C++ 候选基础：src/pc_gvf_core/，Eigen/OpenMP 笛卡尔流场与深度网格投影；不等同于已部署的 Python 角度域链路。
- ROS2 深度仿真、FOV 可视化与四旋翼 mesh 已有代码；场景真值只供生成观测与评价，不得进入待测导航算法。
- src/*/test 和 log/ 是既有测试/日志库存，未作为本轮可复现实验重新验证。

## 待冻结的输入与指标
“纯深度图”暂解释为环境观测只来自深度图；标定/位姿为辅助输入。但是否允许临时点集、局部网格和跨帧记忆需明确。
Python 当前会构造临时点集，不能宣称已经满足“全程不生成点云”的更严格定义。
1 ms 的硬件、线程数、输入尺寸、统计口径、计时范围均未指定。
建议分别记录纯场求解、预处理到速度指令、ROS 端到端；建议报告 p50/p95/p99/max 和超过 1 ms 比例。建议不是已确认验收条件。
单次快样本、均值或降低输入难度均不能代替完整验收。

## 基准与最低证据要求
保留现有 Python 行为基线；C++ 路线仅为候选，先确认功能语义再比较。
冻结数据集/种子、输入尺寸、噪声/无效深度处理、空场/密集障碍/FOV边缘等场景。
记录源码哈希、编译优化、依赖版本、CPU/线程/频率条件、warmup、逐帧原始耗时和错误状态。
测量之外同时记录碰撞/安全停止、成功率、指令有效性及 field_valid 等质量指标。
必须区分仿真渲染、可视化与算法耗时；不得只保留快速或成功帧。
现有 solve_ms 在仿真中包含 render_depth，真实适配器 render_depth 只返回已接收深度，二者计时边界不等价。
当前 CMake 缓存的 build type 为空；不将既有 C++ 产物默认为 Release 性能基线。

## Agent 分工
Governor 负责授权、任务图与恢复；Experiment Architect 冻结指标和最小实验；
Implementation Engineer 维护基准与候选实现；Runner 保存原始运行记录；
Evidence Auditor 独立判定证据是否支持结论；Adversarial Reviewer 用于必要的阶段审查。
其余文献/论文角色已安装，暂不创建没有当前需求的任务。
一次只调度必要角色；图的原子 claim 防止重复运行，失败 Run 用新 ID 和新输出路径。

## 资源与恢复
本轮只完成 BOOT 库存，不预授权后续 Run。CPU/GPU 时长与模型 token 用量未知，不声称自动硬件限额已执行。
原生角色配置已安装，是否被当前宿主自动载入未知；可用 collaboration 工具按角色文件显式派发。
恢复入口为 $hiagent；controller resume/validate 检查当前尝试、输入哈希及授权，不重跑已结束 Run。
