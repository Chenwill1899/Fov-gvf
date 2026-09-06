# BOOT-001：代码与证据盘点 v1

## 当前研究问题
如何在深度图提供环境观测的前提下生成可用的流场 GVF，并在约定硬件、输入规模和计时口径下将计算耗时压到严格 < 1 ms？
本轮为框架安装与库存记录，没有进行性能测量或作出科学 verdict。

## 已检查的实现
1. src/pc_gvf/pc_gvf/depth_angular_core.py 的 compute_guidance 使用 NumPy/SciPy，读取深度、反投影临时点集、估算碰撞锥自由距离、计算角度场并进行速度/rollout 限制。已存在基础，不应从零重建。
2. 同文件 t0 位于 scene.render_depth 之前，solve_ms 结束于输出 command_w 之后。仿真场景 render_depth 会生成图像；depth_angular_controller.py 中 LiveDepthScene.render_depth 仅返回接收的深度。当前 solve_ms 不能不加区分地跨这两条路径比较。
3. src/pc_gvf_core 是独立 C++ 库：Eigen、可选 OpenMP，包含 2D/3D 流场以及 projectDepthFrameToGrid。该接口头文件说明直接深度投影、不构造点集/体素图/距离变换；这只是实现接口约定，不是性能或安全证明。
4. ROS2 当前 demo 启动 Python controller；C++ 库存在不代表已有高性能 C++ ROS 控制链路。
5. build/pc_gvf_core/CMakeCache.txt 中 CMAKE_BUILD_TYPE 为空，不能视为已固定 Release 的可比基准。
6. src 下存在 Python/C++ 回归检查，log 下存在 colcon 历史日志，README/MIGRATION 声称过迁移验证；本轮未重跑，不将其转录为新实验结论。
7. 当前项目目录未检测到 Git 仓库。本轮用文件哈希锚定库存；不主动 git init，不将 install/ 的副本当源代码。

## 已有证据和缺口
- 可追溯事实：下表列出的源码与配置快照；既有仿真、测试、可视化入口。
- 未验证旧结果：迁移文档中的 build/test 描述、历史日志与上一轮对话中的集成测试结果未注册为本轮 Run。
- 缺口：没有本轮冻结的数据、硬件配置、统一计时契约、原始逐帧性能记录和独立审计。
- 不能支持：已达 1 ms、最坏延迟有界、实机安全、相比其他方法更优等结论。

## 最小后续计划（均 proposed，未自动启动）
SPEC-LATENCY-001：明确纯深度图边界及 <1 ms 验收，设计最小基准。
IMPL-BASELINE-001：按冻结契约建立基准入口，保留当前行为和输入；不先假定必须重写 C++。
RUN-BASELINE-001：执行有界基准，保存环境、输入哈希、逐帧原始耗时、质量指标和完整退出状态。
AUDIT-BASELINE-001：由独立 Evidence Auditor 重算指标，给出范围受限的结论；然后才提出针对热点的优化。
下一步规范整理不需要 GPU；人工/模型时间未知。基准运行成本要在契约中确定，本轮无实验用量。

## 恢复和模型路由
项目已装 9 个角色文件；只按需要调用。并发上限为 4。
当前 collaboration 工具支持 astra/sol/terra/luna/5.5；模板 fast_code 指向 spark，而当前 collaboration 不支持它。
因此本项目计划实现/运行均使用 balanced_code (terra)，实验设计与审计使用 deep (sol)；未来派发仍须核对宿主。
BOOT 由当前 Governor 直接盘点，claim 的模型是路由建议，不代表实际切换；runtime_model 保持 null，没有伪造遥测。
安装本地配置不等于当前会话已自动加载原生角色；可按角色文件与 worker-protocol 显式派发。

## 库存快照（SHA-256）
- README.md: 0d54e6f3690313c0be6c8c188accea7a2c33d183d6d5a020c21b9fe7f45999f8
- MIGRATION.md: 16c90501f78afd1020bb85b5d1dc6207a420ed48b1ce4b5f29c633ae72116513
- build_ros2.sh: 3173a4dae43fdebf9e925c1f0c46f0aa4c2a6e96b66f29a1c936f8f7731376a6
- src/pc_gvf/pc_gvf/depth_angular_core.py: e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5
- src/pc_gvf/pc_gvf/depth_angular_controller.py: f2766607ae6c280ceb95f3ae15c89d7e81d3ce1b105731a8ee9ca566e1611f38
- src/pc_gvf_core/CMakeLists.txt: 3d29e53e1efb049cd170338e7d18c1f55a6059ad74734e2749a743046b7c70cc
- src/pc_gvf_core/include/fluid/depth_grid_observation.h: 51397b1ca30472c677f455afba5f4dc6f0895ecbc6be2bca5d6ac5db375a9f60
- build/pc_gvf_core/CMakeCache.txt: 6fb723da49b558ec2701e2c56ec90f71a96c461ca0fe3f113ee5933ede496118
