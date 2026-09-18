# USER 键盘/手柄避障工程进展与文件说明

> 工程位置：`/home/starry/isaac-data/EGO1P0`
> 整理日期：2026-09-10
> 文档用途：帮助用户快速理解当前进展、目录结构、文件命名含义、运行链路以及真正需要修改的文件。

## 1. 当前工程是什么

USER 以当前 `user_ego` 为基础，保留其避障、显示和性能统计能力，当前主入口默认采用
北通 Mode 2 手柄自由驾驶，并保留显式键盘兼容模式：

- 场景：默认使用4倍 `ego_swarm_cloud` 障碍场；`200×200 m` 无障碍平地完整保留，
  可用 `FOV_GVF_SCENE_MODE=flat` 显式启动；
- 出生位姿：平地为 `(0.0, 0.0, 1.5)`，只用于初始化；
- 起点/终点：均已从 USD 和控制流程移除；
- 期望水平速度：`2.0 m/s`；
- 控制器：C++ 深度角域 FOV-GVF 控制器；
- 输入：Isaac Sim 前/左/后/右四路深度图、相机内参和里程计；
- 控制方式：默认把手柄机体系输入转换为世界坐标意图；兼容键盘模式直接给出世界坐标方向；
  两者均按期望XY方向选择深度视角做水平避障，Z独立直控，不使用全局三维地图规划；
- 仿真保护：平地模式限制机体不得穿过 `z=0`；Cloud 模式仍由 `occupancy.bin`
  生成的 ESDF 在 Isaac Sim 内部做碰撞保护；
- 在线地图：由深度传感器逐帧累计，仅用于 RViz 观察，不进入控制回路；
- 视图：Isaac Sim 支持第一/第三人称切换，RViz 默认使用第三人称跟随。

当前控制数据链路为：

```text
Isaac Sim 键盘或北通手柄 + 四路水平深度 / 相机内参 / 里程计
                  |
                  v
       /human_intent（带时间戳）
                  |
                  v
 C++ depth_angular_controller（desired XY选视角、只修正XY）
                  |
                  v
        /position_cmd（期望速度）
                  |
                  v
      Python position_cmd_to_twist
                  |
                  v
         /sim/cmd_vel -> Isaac Sim
                  |
                  v
             无人机运动
```

显示数据链路为：

```text
深度图 -> 当前青色 radar_points
      -> radar 坐标点 + /sim/odom -> world 坐标体素累计
      -> 紫色 observed_map（只显示）

/sim/odom + /position_cmd -> 无人机模型、轨迹、原始/平滑命令箭头
```

## 2. 当前完成情况

### 已完成

1. Isaac Sim 场景、无人机、四台水平深度相机、ROS 2 Bridge 和里程计已经联通。
2. 原先 Python 深度角域运行路线已迁移为 C++ 控制器；Python 数值实现仅保留为测试基准。
3. 无人机默认由北通 Mode 2 四轴自由驾驶，Pitch/Roll/Throttle 意图经实时深度图局部
   避障后执行；四杆回中时由输入端和最终执行器双层 dead-man 立即停车。显式键盘模式
   仍保留 `W/S/A/D/R/F` 映射。
4. 当前速度已设置为 `2.0 m/s`。
5. 深度掩码具有时域滞回：新障碍立即生效，连续 3 帧确认消失后才释放。
6. 连续调和梯度导引、目标保持、warm start、加速度限制和 jerk 限制已经加入，异常箭头抖动已明显消除。
7. RViz 能显示无人机、轨迹、命令箭头、深度图、角域导引、当前青色点云和历史紫色地图。
8. 红色 `occupancy.bin` 场景真值地图 ROS/RViz 显示链路已经移除。
9. 在线历史地图按体素去重，至少重复命中 2 帧后确认；无人机移动后旧障碍保持在 world 坐标原位置。
10. Isaac Sim 主视口黑白噪点已通过 `MinimalRendering / Texture Diffuse` 工程渲染模式处理。
11. Isaac Sim 光照已调整为保留材质颜色、暗部可见且不灰雾的配置。
12. Isaac Sim 第一/第三人称相机可切换；RViz 默认启用无人机第三人称跟随视图。
13. 启动脚本使用 `ROS_DOMAIN_ID=42`、单实例锁和 `/position_cmd` 单发布者检查，避免重复节点互相干扰。
14. 四台90°、`320×240`相机以前/左/后/右拼接覆盖水平360°；W/S/A/D及四个对角方向统一按
    `atan2(desired_y, desired_x)`选择视角，避障算法不读取具体按键。
15. XY与Z已解耦：水平组合键单独归一化，FOV-GVF只修改XY，只按升降时不产生XY；
    最终命令再与直接Z合成。

### 尚未完成或不在当前控制链中的内容

- `pc_gvf_core` 内存在 C++ 二维/三维流体势流求解代码，但当前 USER 默认导航并未链接或调用该包。
- 当前避障仍是“水平深度角域直接导航”，不是三维体素规划；上下方向不存在完整感知
  与绕障能力，只有仿真ESDF/AABB末端紧急停止保护。
- `/pc_gvf/observed_map` 目前只可视化，不参与路径规划或碰撞决策。
- 在线地图尚未生成单独的安全膨胀版本 `/pc_gvf/observed_map_inflated`。
- 手动控制尚需实机按键复验，尤其是组合键、窗口失焦、Space 停止和碰撞保护后继续驾驶。

## 3. 顶层目录说明

| 文件夹 | 名称含义 | 主要作用 | 是否属于当前运行链路 |
|---|---|---|---|
| `src/` | source，源代码 | ROS 2 消息、控制器、算法库、平台适配与测试 | 是 |
| `scripts/` | scripts，脚本 | 构建、启动、场景生成、Blender/Isaac 转换和仿真主程序 | 是 |
| `scenes/` | scenes，场景 | USD、Blender、地图图片、场景参数和 ESDF 输入资产 | 是，当前默认使用 `flat_ground/` |
| `KEYBOARD/` | standalone input test | 不启动 ROS 和避障、使用 BEITONG Mode 2 手柄直接移动无人机；目录名保留兼容 | 独立测试链路 |
| `MISSANDKEYBOARD/` | manual input + safety | 使用同一北通映射发布人工意图，并接入完整 FOV-GVF、RViz、性能记录和末端碰撞保护 | 手柄避障组合链路 |
| `tools/` | tools，工具 | 回归基线生成、ROS/C++ 探针和在线地图验证工具 | 否，验证时使用 |
| `.agents/` | agent configuration | 本地研究代理规则与技能说明 | 否 |
| `.codex/` | Codex configuration | Codex 代理配置 | 否 |
| `.research/` | research state | 研究任务计划、检查点和收件记录 | 否 |
| `.research-system/` | research system | 研究工作流脚本、模板、schema 和角色配置 | 否 |
| `__pycache__/` | Python bytecode cache | Python 自动生成的字节码缓存 | 否，可重新生成 |
| `.pytest_cache/` | pytest cache | 测试框架自动生成的缓存 | 否，可重新生成 |

本副本不在工程目录内保留正式 `build/`、`install/`、`log/`。当前 Humble 构建产物放在：

```text
/tmp/fov_gvf_ego1p0_isaac_build
/tmp/fov_gvf_ego1p0_isaac_install
/tmp/fov_gvf_ego1p0_isaac_log
/tmp/fov_gvf_isaac_ros_log
```

这样命名包含 `user` 和 `isaac`，用于避免与 `user_ego` 自动版及其他 ROS 工作区混用。

## 4. `src/`：ROS 2 与算法源代码

### 4.1 `src/pc_gvf/`：当前主控制包

`pc_gvf` 是项目沿用的包名。`GVF` 表示 Guiding Vector Field（导引向量场）；`pc` 是项目历史命名的一部分，通常用于指向点云/感知侧导航。该包当前负责深度角域算法和 ROS 控制节点。

| 文件或文件夹 | 命名原因 | 对应作用 |
|---|---|---|
| `CMakeLists.txt` | CMake 标准工程文件名 | 编译 C++ 核心库、`depth_angular_controller` 节点和 C++ 回归测试 |
| `package.xml` | ROS 包标准清单名 | 声明包名、版本、依赖和构建类型 |
| `setup.py` | Python 包标准安装文件 | 安装保留的 Python 基准实现和演示入口 |
| `setup.cfg` | setuptools 配置名 | 指定 Python 可执行程序安装位置 |
| `resource/pc_gvf` | ament 资源索引文件以包名命名 | 让 ROS 2 能通过 ament index 找到该包 |

#### `include/pc_gvf/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `depth_angular_core.hpp` | depth=深度图，angular=角域网格，core=不依赖 ROS 的核心 | 声明深度反投影、安全角域、路径/调和导引、速度组合、时域掩码等 C++ 数据结构和函数 |

#### `src/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `depth_angular_core.cpp` | 与同名 `.hpp` 配对的实现文件 | 实现 C++ 深度角域导航算法，是当前避障数学逻辑的主要文件 |
| `depth_angular_controller_node.cpp` | controller_node 表示“把算法包装成 ROS 节点” | 订阅 `/sim/odom`、深度图和 CameraInfo，调用 core，发布 `/position_cmd`、状态和角域可视化；同时实现速度、加速度、jerk、紧急停车和目标到达逻辑 |

如果要修改当前避障算法，优先查看以上两个 C++ 文件。一般规则是：

- 数学算法、像素/角域处理：改 `depth_angular_core.cpp/.hpp`；
- ROS 话题、参数、时间同步、输出平滑：改 `depth_angular_controller_node.cpp`；
- 当前 USER 参数值：优先改 `launch/isaac_cloud_navigation.launch.py`，不要直接改 C++ 默认值。

#### `launch/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `isaac_cloud_navigation.launch.py` | Isaac Cloud 场景的导航总 launch | 当前默认 ROS 侧入口；启动 C++ 控制器、命令桥、导航显示、在线地图显示和 RViz，并集中保存当前运行参数 |
| `depth_angular_demo.launch.py` | 深度角域算法的独立 demo | 启动合成场景验证链路，不是 USER Cloud 默认入口 |

#### `pc_gvf/` Python 模块

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `depth_angular_core.py` | C++ core 的 Python 前身/数值参考 | 保留为行为基准和测试 oracle，不是当前实机控制路径 |
| `depth_angular_demo_sim.py` | demo_sim 表示演示用运动学仿真 | 发布合成深度、里程计和障碍物，供独立 demo 测试 |
| `__init__.py` | Python 包标准文件 | 标记目录为 Python 包 |

#### `scripts/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `depth_angular_core` | 与 Python 模块入口同名 | ROS/setuptools 兼容启动包装器 |
| `depth_angular_demo_sim` | 与演示节点同名 | ROS/setuptools 兼容启动包装器 |

#### `test/`

`*_check.cpp` 表示单一 C++ 行为契约检查，`test_*.py` 表示 pytest 测试。

| 文件 | 测试内容 |
|---|---|
| `camera_geometry_check.cpp` | 相机内参和角域几何 |
| `depth_geometry_check.cpp` | 深度像素反投影与几何关系 |
| `angular_field_check.cpp` | 角域安全场和导引场 |
| `guidance_composition_check.cpp` | 目标方向与避障导引的组合 |
| `motion_safety_check.cpp` | 速度、rollout 和安全停止行为 |
| `horizontal_360_check.cpp` | 八个水平目标方向、四视角选择、绕障偏转和零Z泄漏 |
| `fixture_reader.hpp` | 为 C++ 测试读取固定行为样本 |
| `test_depth_angular_core.py` | Python core 单元测试 |
| `test_python_behavior_baseline.py` | 固定 Python 行为基线，防止 C++ 迁移改变功能 |
| `test_manual_control_math.py` | 15种必测输入、XY/Z独立归一化和分量安全保护 |
| `fixtures/python_behavior_v1/` | Python behavior version 1 | 保存迁移时冻结的输入/输出样本 |

`fixtures` 中的文件按场景现象命名，例如：

- `empty_start.fixture`：起点前方为空；
- `single_pillar_near.fixture`：近处单柱；
- `narrow_gate_near.fixture`：近处窄门；
- `overhead_bar_near.fixture`：上方横梁；
- `goal_behind_camera.fixture`：目标位于相机后方；
- `all_zero_depth.fixture` / `all_nan_depth.fixture`：异常深度输入；
- `history_and_lateral_velocity.fixture`：检查历史状态与侧向速度。

### 4.2 `src/pc_gvf_msgs/`：自定义消息包

包名后缀 `msgs` 是 ROS 的常见命名方式，表示只定义消息接口。

| 文件 | 对应作用 |
|---|---|
| `msg/PositionCommand.msg` | 定义位置、速度、加速度、jerk、yaw、增益和轨迹状态；当前主要使用其中速度字段 |
| `CMakeLists.txt` | 调用 `rosidl_generate_interfaces` 生成 C++/Python 消息代码 |
| `package.xml` | 声明 `geometry_msgs`、`std_msgs` 和 rosidl 依赖 |

`PositionCommand` 的命名表示“位置控制接口使用的完整轨迹指令”，即使当前 USER 主要输出速度，也沿用该消息以兼容原控制链路。

### 4.3 `src/pc_gvf_platforms/`：平台适配与可视化包

`platforms` 表示算法与具体机器人、仿真器、输入设备和显示系统之间的适配层。这里的 Python 不负责核心避障决策。

#### `pc_gvf_platforms/` Python 模块

| 文件 | 命名原因 | 对应作用 | 当前默认是否使用 |
|---|---|---|---|
| `command_bridge.py` | bridge=桥接两种消息接口 | 把 `/position_cmd` 转成 Isaac Sim 接收的 `/sim/cmd_vel`；支持 holonomic/differential 模式 | 是 |
| `navigation_visualizer.py` | navigation + visualizer | 发布无人机模型、飞行轨迹、原始橙色命令箭头和仅显示用平滑紫色箭头 | 是 |
| `observed_map_visualizer.py` | observed map=实际观测地图 | 深度反投影生成当前青色点云，将点转换到 world 后进行历史体素累计，并发布紫色地图和 `world -> radar` TF | 是 |
| `kinematic_sim.py` | kinematic simulation | 独立 ROS 运动学小仿真器，供非 Isaac 平台演示 | 否 |
| `joy_to_intent.py` | joystick to navigation intent | 通用ROS Joy消息适配器 | 否；当前默认由Isaac内的北通Linux joystick读取器直接发布 |
| `intent_trace_replay.py` | replay=重放预录意图 | 按 YAML 重放人类输入，用于确定性测试 | 否 |
| `__init__.py` | Python 包标准文件 | 标记 Python 包 |

#### `config/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `isaac_cloud_navigation.rviz` | 与当前 Isaac Cloud launch 同名 | 当前 RViz 布局；包含无人机、点云、地图、轨迹、箭头和 `ThirdPersonFollower` 默认视图 |
| `depth_demo.rviz` | 深度角域 demo 配置 | 合成深度演示的 RViz 布局 |
| `ground_demo.rviz` | ground platform demo | 地面机器人演示布局 |
| `forward_stop.yaml` | “前进后停止”的输入轨迹 | 为 `intent_trace_replay.py` 提供确定性输入 |

#### `launch/`

| 文件 | 对应作用 |
|---|---|
| `ground_platform.launch.py` | 启动差速地面平台适配、运动学仿真和 ground RViz，不用于当前无人机 Cloud 任务 |

#### `meshes/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `hummingbird.mesh` | Hummingbird 四旋翼模型名 | RViz 中显示无人机外形 |
| `README.md` | mesh 说明 | 记录模型来源/用途 |

#### `test/`

| 文件 | 对应作用 |
|---|---|
| `test_observed_map_visualizer.py` | 检查深度反投影、radar→world 变换、体素命中和静止重复帧不无限增长 |
| `test_platform_math.py` | 检查命令桥、摇杆映射等平台数学逻辑 |
| `test_robot_visualization.py` | 检查无人机模型、轨迹和箭头显示消息 |

`setup.py` 注册 `position_cmd_to_twist`、`navigation_visualizer`、`observed_map_visualizer` 等 ROS 可执行入口；`resource/pc_gvf_platforms` 和 `setup.cfg` 分别负责 ament 索引和安装位置。

### 4.4 `src/pc_gvf_core/`：二维/三维流体势流研究库

该包名中的 `core` 表示与 ROS 话题解耦的 C++ 数学库。它目前能独立构建和测试，但没有被 `pc_gvf` 当前控制节点链接，因此不是 USER 默认控制路线。

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `include/fluid/fluid_solver_2d.h` | 二维 fluid solver | 声明二维调和流场、障碍连通分量和粗细网格接口 |
| `src/fluid_solver_2d.cpp` | 对应二维实现 | 计算二维障碍绕流调和场 |
| `include/fluid/fluid_solver_3d.h` | 三维 fluid solver | 声明三维体素、Neumann 障碍边界、势函数和矩阵自由 CG 求解接口 |
| `src/fluid_solver_3d.cpp` | 对应三维实现 | 实现三维势流压力投影、连通性处理和 OpenMP 求解 |
| `include/fluid/depth_grid_observation.h` | depth→grid observation | 声明深度帧投影到三维体素栅格的接口 |
| `src/depth_grid_observation.cpp` | 对应实现 | 将深度、相机位姿和膨胀半径转换为 free/occupied/unknown 体素状态 |
| `include/fluid/fluid_guidance.h` | guidance=将场转成运动导引 | 声明二维/三维求解器的上层调度、粗细层和输出接口 |
| `src/fluid_guidance.cpp` | 对应实现 | 组织 2D/3D 网格、求解计时、导引与速度输出 |
| `test/coarse_boundary_check.cpp` | 检查粗细场边界耦合 |
| `test/depth_guidance_integration_check.cpp` | 检查深度投影到导引输出的组合链路 |
| `test/depth_grid_observation_check.cpp` | 检查三维观测栅格生成 |

这个文件夹对应未来“第三条 C++ 三维链路”的研究基础，但将其接入当前控制器前仍需完成 ROS 节点集成、实时性能、未知空间策略和闭环安全验证。

## 5. `scripts/`：构建、启动和场景处理

### 5.1 当前必须使用的脚本

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `build_isaac_ros_workspace.sh` | build + Isaac ROS workspace | 使用 ROS 2 Humble 编译 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf`，产物隔离在 `/tmp/fov_gvf_ego1p0_isaac_*` |
| `run_isaac_fov_gvf_navigation.sh` | run + Isaac + FOV-GVF navigation | 当前一键启动入口；检查单实例和话题发布者，启动 ROS launch，再运行 Isaac 主程序 |
| `prepare_flat_ground_navigation.sh` | prepare + flat ground | 生成当前默认的无障碍平地手动场景，出生点为 `(0,0,1.5)` |
| `prepare_ego_swarm_cloud_navigation.sh` | prepare + Cloud 场景名 | 生成保留的4倍障碍场景，加入无人机和相机但不创建起终点 |

### 5.2 `scripts/isaac/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `run_fov_gvf_navigation.py` | 当前 Isaac 运行主程序 | 打开 USD、设置渲染和灯光、创建/连接 ROS 2 OmniGraph、发布深度与里程计、接收速度、更新无人机、执行 ESDF 安全保护、管理第一/第三人称相机 |
| `create_flat_ground_scene.py` | create + flat ground | 创建 `200×200 m`、`z=0` 的无障碍基础平地 USD |
| `prepare_uav_navigation_scene.py` | prepare UAV navigation scene | 把普通 USD 包装成带 `/World/Robot`、相机和手动出生位姿的场景 |
| `prepare_blender_cloud_usd.py` | Blender USD→Isaac USD | 修正从 Blender 导出的 USD，使其适合 Isaac 使用 |
| `generate_cloud_pillar_scene.py` | 生成 cloud pillar 场景 | 程序化创建连续云柱候选场景 |
| `generate_crack_maze_scene.py` | 生成 crack maze | 程序化创建裂隙迷宫场景 |
| `generate_isolated_cloud_scene.py` | 生成 isolated clouds | 程序化创建相互分离的云团障碍 |
| `view_cloud_pillar_scene.py` | view 对应场景 | 只打开并观察 cloud pillar，不启动导航 |
| `view_crack_maze_scene.py` | view 对应场景 | 只打开并观察 crack maze |
| `view_isolated_cloud_scene.py` | view 对应场景 | 只打开并观察 isolated clouds |
| `view_blender_cloud_scene.py` | view Blender 转换场景 | 检查 Blender→USD→Isaac 转换结果 |

`generate_*` 表示“生成资产”，`prepare_*` 表示“修改已有资产使其可用于导航”，`view_*` 表示“只观察资产”，`run_*` 表示“执行完整任务”。

### 5.3 `scripts/blender/`

| 文件 | 对应作用 |
|---|---|
| `generate_isolated_cloud_scene.py` | 必须由 Blender Python 执行，生成可编辑 `.blend` 独立云团场景 |
| `export_scene_to_usd.py` | 从当前 Blender 场景导出 Z-up、米制 USD |

### 5.4 其他场景包装脚本

| 文件 | 对应作用 |
|---|---|
| `export_blender_cloud_to_isaac.sh` | 串联 Blender 导出、Isaac 修正和 UAV 导航场景准备 |
| `run_blender_isolated_cloud_scene.sh` | 生成并打开 Blender 独立云团场景 |
| `run_cloud_pillar_scene.sh` | 生成并查看 cloud pillar 场景 |
| `run_crack_maze_scene.sh` | 生成并查看 crack maze 场景 |
| `run_isolated_cloud_scene.sh` | 生成并查看 isolated cloud 场景 |
| `prepare_ego_swarm_navigation.sh` | 处理旧 `ego_swarm_blender` 场景；不是当前 Cloud 默认场景 |

## 6. `scenes/`：场景资产与命名规则

### 6.1 当前默认：`scenes/flat_ground/`

| 文件 | 对应作用 |
|---|---|
| `flat_ground.usd` | 仅含 `200×200 m` 平地的基础场景 |
| `flat_ground_navigation.usd` | 加入 Robot、深度相机、第三人称相机和手动出生位姿的默认运行场景 |

### 6.2 保留的障碍场景：`scenes/ego_swarm_cloud/`

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `ego_swarm_cloud_navigation.usd` | 来源为 ego_swarm_cloud，后缀 navigation 表示已加入运行对象 | 保留的4倍 Cloud 场景；用 `FOV_GVF_SCENE_MODE=cloud` 选择，不含起终点 Marker |
| `occupancy.bin` | occupancy=占据状态，bin=二进制 | Isaac 主程序读取后生成 ESDF，只做仿真碰撞保护；不会发布为 RViz 真值地图，也不进入 C++ 深度控制器 |

### 6.3 其他场景目录

| 文件夹 | 名称含义 | 对应作用 |
|---|---|---|
| `ego_swarm/` | 较早的 EGO-Swarm Blender 场景 | 保存 `ego_swarm_navigation.usd`，不是当前默认 |
| `blender_isolated_clouds/` | Blender 可编辑的独立云团 | 保存 `.blend`、Blender 原始 USD、Isaac 修正版 USD、导航版 USD、预览和纹理 |
| `cloud_pillars/` | 连续云柱障碍 | 保存选择后的 seed42 场景、参数 JSON、俯视图和候选场景 |
| `crack_maze/` | 裂隙迷宫 | 保存 seed42 USD、参数 JSON 和地图图像 |
| `isolated_clouds/` | 相互分离的云状障碍 | 保存选中场景、候选种子和接触表 |

常见文件名模式：

```text
<scene>_seed42.usd       固定随机种子 42 生成的可复现场景
<scene>_seed42.json      该场景的尺寸、障碍和随机参数
<scene>_seed42_map.png   俯视地图/检查图
*_candidates/            多组种子候选，供人工选择
*_contact_sheet.png      多候选缩略图拼图
*_blender.usd            Blender 直接导出的原始 USD
*_isaac.usd              经 Isaac 兼容处理后的 USD
*_navigation.usd         已加入无人机和相机的运行版 USD
*.blend1                  Blender 自动备份文件
```

`seed` 出现在名称中是为了确保随机障碍场景可以精确复现，而不是每次生成不同地图。

## 7. `tools/`：验证与迁移辅助工具

| 文件 | 命名原因 | 对应作用 |
|---|---|---|
| `generate_python_baseline.py` | 生成 Python baseline | 将旧 Python 算法行为冻结成 fixtures，供 C++ 一致性验收 |
| `ros_cpp_state_probe.py` | probe=探针 | 检查 C++ ROS 节点在给定输入下的状态和输出 |
| `ros_cpp_closed_loop_probe.py` | closed loop probe | 检查 C++ 节点与仿真闭环运行 |
| `ros_observed_map_probe.py` | observed map probe | 检查实时点云、累计地图、frame_id、点数和静止/移动后的行为 |

这些工具不在正常启动链中，只有调试或验收时才运行。

## 8. 根目录文档和配置文件

| 文件 | 名称含义与作用 | 当前参考优先级 |
|---|---|---|
| `DONG.md` | 本文档；面向用户的当前工程总览 | 最高，配合实际代码 |
| `USER_MANUAL_CONTROL_GUIDE.md` | USER 手柄主控、键盘兼容模式与安全机制 | 高，以 launch 和运行脚本为准 |
| `WORK_LOG.md` | 每次修改、原因和验证结果的时间记录 | 高 |
| `README.md` | 原 ROS 2 工程通用说明 | 中；仍写 Jazzy，而当前 Isaac 链路实际使用 Humble |
| `FOV_GVF_PRINCIPLES_RECORD.md` | FOV-GVF 数学原理与代码核对记录 | 原理参考 |
| `DIRECT_DEPTH_3D_NAVIGATION_HANDOFF.md` | 深度图直达三维导航的换账号交接思路 | 后续路线参考 |
| `CPP_3D_VOXEL_FLOW_PLAN.md` | C++ 三维体素势流实施计划 | 规划文档，未等于当前已接入 |
| `CPP_MIGRATION_PLAN.md` | Python→C++ 分阶段迁移方案 | 历史实施计划 |
| `MIGRATION.md` | ROS 1→ROS 2 文件映射 | 历史迁移参考 |
| `ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md` | 当前 Cloud 场景建立与导航记录 | 场景参考 |
| `ISAAC_FOV_GVF_NAVIGATION.md` | Isaac + FOV-GVF 总体接入记录 | 场景/集成参考 |
| `BLENDER_ISOLATED_CLOUD_SCENE.md` | Blender 独立云团场景说明 | 备用场景参考 |
| `ISAAC_CLOUD_SCENE.md` | Cloud pillar 场景说明 | 备用场景参考 |
| `ISAAC_CRACK_MAZE_SCENE.md` | 裂隙迷宫说明 | 备用场景参考 |
| `ISAAC_ISOLATED_CLOUD_SCENE.md` | 独立云团 Isaac 场景说明 | 备用场景参考 |
| `AGENTS.md` | 项目内代理协作与日志规则 | 开发流程配置，不参与运行 |
| `build_ros2.sh` | 原通用 Jazzy 构建脚本 | 不建议用于当前 USER Isaac Humble 链路 |

## 9. 隐藏研究目录

这些目录不影响 ROS/Isaac 运行：

- `.agents/skills/`：保存 claim/evidence、科研写作、实验契约等代理技能规则；
- `.codex/agents/`：保存 Codex 本地代理角色配置；
- `.research/checkpoints/`：研究流程检查点；
- `.research/inbox/`：研究任务输入；
- `.research/plans/`：研究计划；
- `.research-system/agents/`：研究角色定义；
- `.research-system/config/`：研究系统配置；
- `.research-system/schemas/`：研究状态数据格式；
- `.research-system/templates/`：研究报告模板；
- `.research-system/tools/`：研究工作流控制脚本。

命名中的点号表示隐藏目录，目的是把开发/研究元数据与机器人运行文件分开。

## 10. 当前关键参数在哪里修改

### 导航、速度与安全参数

文件：`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`

```text
speed / max_speed                 水平速度，当前 2.0 m/s
max_vertical_speed                最大垂直速度，当前 1.0 m/s
horizontal_360_enabled            四视角水平避障，当前 true
obstacle_clear_frames             障碍释放确认帧数，当前 3
body_radius                       机体半径，当前 0.25 m
safety_margin                     导航安全边距，当前 0.20 m
rollout_margin                    前向预测额外边距，当前 0.10 m
planning_horizon                  预测时域，当前 3.0 s
command_accel_limit               普通加速度限制，当前 1.2 m/s²
speed_brake_accel                 制动加速度，当前 2.5 m/s²
command_jerk_limit                jerk 限制，当前 8.0 m/s³
continuous_harmonic_guidance      连续调和梯度导引，当前 true
safe_goal_hold_time               安全目标最短保持，当前 0.35 s
human_intent_topic                人工意图话题，当前 /human_intent
human_intent_timeout              意图超时，当前 0.25 s
fixed_forward_intent              当前 false
use_fixed_goal / stop_at_goal     当前均为 false
```

### 在线点云地图参数

同一 launch 文件中的 `observed_map_visualizer` 参数：

```text
depth_stride=4                    每隔 4 个像素采样，降低 RViz/建图负载
minimum_depth=0.3 m
maximum_depth=15.0 m
map_resolution=0.08 m             内部历史地图体素
minimum_hits=2                    至少跨 2 帧命中后确认
map_publish_rate=1.0 Hz
publication_resolution=0.25 m     只影响 RViz 发布密度
maximum_publish_points=160000
camera_offset=[0.22,0,0.02]       base 到 radar 固定外参
```

### RViz 显示与第三人称视角

文件：`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`

```text
ThirdPersonFollower Target Frame=radar
Distance=8 m
Pitch=0.42 rad
Yaw=3.14159 rad
Observed obstacle map 颜色=紫色
Radar depth point cloud 颜色=青色
RViz Frame Rate=60 FPS（目标上限，实际取决于 GPU 与显示负载）
Angular GVF 发布=15 Hz
无人机/命令 Marker 发布上限=60 Hz
完整 Path 发布=10 Hz，移动至少 0.10 m 才增加一个轨迹点
```

为提高第三人称跟随的连贯性，RViz 默认不再重复绘制 Odometry 历史坐标轴；无人机
Mesh、两根命令箭头和绿色实际路径仍保留。两层点云禁止逐点选择并将显示队列限制为
1，避免旧点云排队造成画面滞后。上述调整只影响可视化，不改变控制器使用的深度图、
`48×36` 规划网格或 50 Hz 控制输出。

### Isaac Sim 光照、渲染和相机

文件：`scripts/isaac/run_fov_gvf_navigation.py`

```text
ISAAC_MINIMAL_SHADING_MODE=2       Texture Diffuse
ISAAC_SUN_INTENSITY=800
ISAAC_DOME_INTENSITY=0
ISAAC_AMBIENT_LIGHT_INTENSITY=0.05
ISAAC_VIEW_EYE / ISAAC_VIEW_TARGET 初始总览相机
```

Isaac 主视口快捷键：

- `W/S`：世界 `+X/-X`；`A/D`：世界 `+Y/-Y`；`R/F`：世界 `+Z/-Z`；
- `Space`：立即停止；
- `1`：无人机第一人称；
- `3`：无人机第三人称；
- `V`：在第一/第三人称间切换。

## 11. 启动方法

只测试 BEITONG Mode 2 手柄和直接运动，不经过 ROS、深度控制或避障：

```bash
cd /home/starry/isaac-data/EGO1P0/KEYBOARD
bash run_joystick_only.sh
```

使用同一手柄映射和完整深度避障，默认运行4倍Cloud障碍场景：

```bash
cd /home/starry/isaac-data/EGO1P0/MISSANDKEYBOARD
bash run_joystick_avoidance.sh
```

完整 USER 控制链（默认北通手柄）：

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_isaac_fov_gvf_navigation.sh
```

需要使用旧键盘映射时：

```bash
ISAAC_MANUAL_INPUT_MODE=keyboard bash scripts/run_isaac_fov_gvf_navigation.sh
```

正常启动顺序是：

1. `build_isaac_ros_workspace.sh` 将 ROS 包编译并安装到 `/tmp` 独立目录；
2. `run_isaac_fov_gvf_navigation.sh` 检查旧进程和 `/position_cmd` 发布者；
3. ROS launch 启动 C++ 控制器、桥接、显示节点和 RViz；
4. `run_fov_gvf_navigation.py` 打开 USD 并启动 Isaac Sim；
5. Isaac 发布前向兼容话题 `/sim/depth/*`、新增 `/sim/depth_{left,back,right}/*`
   以及 `/sim/odom`；
6. C++ 控制器开始发布 `/position_cmd`；
7. 桥接节点输出 `/sim/cmd_vel`，Isaac 更新无人机位置。

## 12. 最重要的文件速查

只想快速修改时，先看这些文件：

1. `KEYBOARD/keyboard_only.py`：直接读取 Linux joystick 并执行 Mode 2 机体系控制；文件名
   为兼容旧入口而保留，不包含 ROS/避障。
2. `KEYBOARD/run_joystick_only.sh`：独立手柄测试主入口；`run_keyboard_only.sh` 仅为兼容
   包装入口。
3. `MISSANDKEYBOARD/beitong_joystick.py` 与 `run_joystick_avoidance.sh`：北通 Mode 2
   输入和完整避障组合入口。
4. `scripts/run_isaac_fov_gvf_navigation.sh`：完整控制链一键启动和进程隔离。
5. `scripts/isaac/run_fov_gvf_navigation.py`：Isaac Sim、相机、光照、ROS Bridge、运动和 ESDF。
6. `src/pc_gvf/launch/isaac_cloud_navigation.launch.py`：所有完整链路 ROS 节点和主要参数。
7. `src/pc_gvf/src/depth_angular_controller_node.cpp`：当前 C++ 控制节点、输出平滑和安全状态。
8. `src/pc_gvf/src/depth_angular_core.cpp`：当前深度角域避障算法。
9. `src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`：PositionCommand→Twist。
10. `src/pc_gvf_platforms/pc_gvf_platforms/observed_map_visualizer.py`：实时点云和历史地图。
11. `src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`：RViz 显示与第三人称视图。

## 13. 修改时的边界提醒

- 不要把 `occupancy.bin` 接入 C++ 导航；当前设计要求导航只使用实时深度。
- 不要删除 Isaac 主程序中的 `occupancy.bin/ESDF`，它仍承担仿真安全保护。
- 不要把 `/pc_gvf/observed_map` 误当作控制地图；它目前只是历史可视化。
- 修改 launch、RViz、Python 包后要重新运行 `build_isaac_ros_workspace.sh`，否则 `/tmp` install 中仍可能是旧文件。
- 避免同时启动多个副本；正常入口会检查 `/position_cmd` 只有一个发布者。
- `pc_gvf_core` 的 3D 势流文件存在不代表第三条路线已经成为当前控制器；是否被 `CMakeLists.txt` 链接和是否被运行节点调用才是判断依据。
- 根目录 `README.md` 的 Jazzy 命令属于通用旧说明；当前 USER + Isaac Sim 应使用 Humble 专用脚本。

## 14. 每次运行的性能指标

正常使用 `scripts/run_isaac_fov_gvf_navigation.sh` 时，脚本会生成统一运行 ID。
C++ 控制器和命令桥每 5 秒在终端输出一次 `[PERF CTRL]`、`[PERF BRIDGE]`
摘要，退出时再输出 `FINAL`；Isaac 退出时输出 `[PERF ISAAC FINAL]`。

所有结果按运行 ID 追加到 `performance/PERFORMANCE_METRICS.md`，包括：

- 避障核心 `computeGuidance()` 的 mean/P50/P95/max 墙钟耗时；
- 控制定时回调数、命令帧数、实际导引计算帧数；
- ROS 仿真时间控制 FPS、活跃运行段墙钟 FPS和相邻控制间隔；
- 最新深度帧时间戳到控制发布的延迟；
- 整个控制回调到 `/position_cmd` 发布，以及 DDS publish 调用耗时；
- `/position_cmd` 到桥接节点接收、Twist 转换和 `/sim/cmd_vel` 发布耗时；
- Isaac 60 Hz OmniGraph 实际采样帧数、仿真/墙钟 FPS和命令变化间隔。

其中 ROS 时间指标与仿真时间对齐，steady-clock 指标表示主机真实耗时。Isaac 的
`ROS2SubscribeTwist` 没有消息时间戳，因此桥接发布到 OmniGraph 取用之间不能在不
改变控制接口的情况下精确逐消息配对；文档会如实记录桥接侧可测延迟和 Isaac 的
实际采样周期，不把两者伪装成精确端到端数字。
