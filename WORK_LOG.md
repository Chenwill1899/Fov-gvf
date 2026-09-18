# Fov-GVF 分支工作日志

> 分支：`feature/local-modifications`  
> 基线分支：`dev`  
> 基线提交：`804f26e2e631d0571f5e1cc5088100481df1b16d`  
> 建立日期：2026-09-06

本文档用于记录 `feature/local-modifications` 分支的功能基线和后续变更。凡是对本分支源码、配置、脚本或文档的修改，都应在文末“修改记录”中追加一条记录。

---

## 一、当前项目已经完成的功能

### 1. ROS 1 到 ROS 2 的独立迁移

- 项目已从旧 ROS 1/catkin 工作区迁移为 ROS 2/ament 工作区。
- 当前仓库可以独立构建，不依赖旧 ROS 1 仓库中的文件。
- 上游文档以 ROS 2 Jazzy 为目标；本机已额外验证 ROS 2 Humble 可以完成构建、测试和演示运行。

### 2. 四个 ROS 2 功能包

| 功能包 | 当前职责 |
|---|---|
| `pc_gvf_msgs` | 定义统一的 `PositionCommand` 位置、速度、加速度和轨迹状态消息 |
| `pc_gvf_core` | 提供二维/三维流体场求解、深度栅格投影和笛卡尔空间导引算法 |
| `pc_gvf` | 提供深度视场角域 GVF 算法、ROS 2 控制器和合成深度演示 |
| `pc_gvf_platforms` | 提供平台适配、运动学仿真、手柄意图、轨迹回放和 RViz 可视化 |

### 3. 深度视场感知

- 接收 `sensor_msgs/msg/Image` 深度图和 `sensor_msgs/msg/CameraInfo` 相机内参。
- 支持 `32FC1` 米制深度图。
- 支持 `16UC1` 毫米制深度图。
- 将输入深度图缩放为角域规划网格，默认大小为 `48 × 36`。
- 对无效值、负深度和超出量程的数据进行清理和截断。
- 将深度像素反投影为空间障碍物点，用于安全距离和碰撞判断。

### 4. FOV/角域 GVF 局部避障

- 根据机器人尺寸和安全余量计算碰撞锥自由距离。
- 在相机视场中生成不可通行角域掩码。
- 根据目标方向、历史指令方向和障碍物分布选择安全目标。
- 在安全角域内求解调和势场/导引向量场。
- 对指令方向进行角速度限制，降低方向突变。
- 根据可用净空、制动距离和预测碰撞结果限制速度。
- 在视场不足、深度过期或导引无效时输出安全停止命令。

### 5. ROS 2 闭环控制器

控制器接收：

- 里程计；
- 深度图；
- 相机参数；
- 人工或自动导航意图。

控制器输出：

- `pc_gvf_msgs/msg/PositionCommand`；
- 当前导航状态；
- 视场、障碍点、参考方向、目标方向、命令方向和角域 GVF 可视化标记。

已实现的运行状态包括：

- `WAITING_ODOMETRY`：等待里程计；
- `WAITING_DEPTH`：等待深度图；
- `STALE_DEPTH`：深度数据超时；
- `STALE_INTENT`：导航意图超时；
- `NAVIGATING`：正在导航；
- `DEGRADED`：导引场退化；
- `GOAL_REACHED`：到达目标。

### 6. 自包含合成深度闭环演示

- 内置运动学无人机模型，无需 Gazebo、PX4 或 Isaac Sim 即可运行算法闭环。
- 合成发布里程计、深度图、相机参数和前进意图。
- 接收控制器速度命令并更新机器人位置。
- 对实际运动线段执行碰撞检查。
- 默认从约 `(0.0, 0.0, 1.2)` 飞向 `(7.2, 0.0, 1.2)`。

当前内置场景：

| 场景 | 用途 |
|---|---|
| `empty` | 无障碍直线导航基准 |
| `single_pillar` | 中央立柱绕障 |
| `offset_box` | 偏置箱体避障 |
| `center_sphere` | 中央球形障碍避障 |
| `overhead_bar` | 上方横杆和高度方向安全性 |
| `diagonal_gap` | 斜向间隙通过能力 |
| `narrow_gate` | 狭窄通道通过和安全停止能力 |

### 7. RViz 可视化

默认演示可以显示：

- 测试障碍物几何体；
- 无人机模型和当前姿态；
- 速度指令箭头；
- 已执行路径；
- 里程计；
- 前向深度图；
- 相机视场边界；
- 深度障碍点；
- 不安全角域；
- 参考射线、目标射线和命令射线；
- 角域 GVF 向量。

### 8. 地面平台适配

- 提供 `PositionCommand` 到 `geometry_msgs/msg/Twist` 的转换桥。
- 支持全向和差速运动学仿真模式。
- 提供地面机器人 RViz 配置。
- 对输出线速度和角速度进行限制。

### 9. 人工意图和确定性回放

- 支持将 ROS 2 `Joy` 手柄输入转换为 `TwistStamped` 导航意图。
- 支持死区、方向映射、最大速度和手柄超时保护。
- 支持从 YAML 文件回放确定性意图轨迹。
- 自带 `forward_stop.yaml` 前进—停止测试轨迹。

### 10. 测试与性能统计

- 包含二维/三维粗细网格边界回归测试。
- 包含深度投影和笛卡尔导引集成测试。
- 包含角域核心算法、深度解码、平台数学和可视化测试。
- 离线仿真会输出成功状态、碰撞状态、最终距离、最小净空、路径长度、速度、加速度和抖动指标。
- 算法内部记录 `solve_ms`，并输出计算耗时 P50、P95 和 P99。

### 11. 当前本机验证结果

验证环境：

- Ubuntu 22.04；
- ROS 2 Humble；
- Python 3.10；
- 使用 `/usr/bin/python3`；
- 使用 `PYTHONNOUSERSITE=1` 隔离用户目录中的 NumPy 2.x。

验证结果：

- 四个 ROS 2 包构建成功；
- C++ 与 Python 合计 13 项测试通过；
- `single_pillar` 图形和无界面演示均成功进入 `NAVIGATING`；
- 演示最终进入 `GOAL_REACHED`，未报告碰撞；
- 深度输入实测约 `8.9 Hz`；
- `PositionCommand` 输出稳定在约 `50 Hz`；
- `single_pillar` 离线算法耗时约为 P50 `1.09 ms`、P95 `8.29 ms`、P99 `8.92 ms`。

### 12. 已知限制和待处理事项

- 上游 `build_ros2.sh` 固定使用 `/opt/ros/jazzy`，不能直接用于本机 Humble 环境。
- 用户 Python 目录中的 NumPy 2.2.6 与 Ubuntu SciPy 1.8.0 不兼容；运行本项目时必须使用 `PYTHONNOUSERSITE=1`，或者建立独立兼容环境。
- `rosdep` 无法解析两个 Python 包中的 `ament_pytest` 测试依赖名称，但当前本机已有测试工具，实际测试可以运行。
- 运行时偶尔出现 `sequence size exceeds remaining buffer` DDS 反序列化警告；当前数据流仍能继续，但需要在后续修改中定位和消除。
- 当前默认飞行演示使用合成深度相机和运动学模型，并非真实相机或完整飞行动力学。
- 当前仓库尚未提供与 Isaac Sim、PX4 或真实无人机直接连接的专用适配器。
- 仓库没有给出明确的论文题名、DOI、BibTeX 或作者对应关系；相关论文仍需进一步确认。

---

## 二、后续修改记录规范

每次修改都在本节最上方追加记录，建议使用以下格式：

```text
### YYYY-MM-DD — 修改标题

- 目的：为什么修改。
- 涉及文件：修改了哪些文件。
- 修改内容：具体做了什么。
- 验证：执行了什么测试，结果如何。
- 遗留问题：尚未完成或需要注意的内容；没有则写“无”。
```

---

## 三、修改记录

### 2026-09-17 — 四视角水平360°避障与XY/Z独立控制

- 目的：将原前向约90°局部避障扩展到W/S/A/D和四个对角方向，同时保证升降输入不
  进入水平规划、水平避障不改写Z。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、
  `src/pc_gvf/src/depth_angular_core.cpp`、`depth_angular_controller_node.cpp`、launch、
  CMake及新增两项测试；`scripts/isaac/prepare_uav_navigation_scene.py`、
  `run_fov_gvf_navigation.py`、新增 `manual_control_math.py`；平地/Cloud导航USD及相关
  用户、版本、场景说明；同步更新 `HISTORY/CHANGELOG.md`。
- 修改内容：保留前向相机路径和 `/sim/depth/*`，新增左/后/右三台120°相机与话题，
  四视角覆盖水平360°；控制器用期望XY角选择最近相机。核心新增水平中线规划模式，
  仍使用原碰撞锥、调和导引、制动和rollout。键盘XY单独归一化，Z独立限幅；控制器
  拆分 `desired_xy/vz`，只规划XY后重新组合。纯Z直接发布且XY为零。ESDF/AABB末端
  保护改为分量检查，可在水平候选碰撞时保留安全Z，但不冒充三维规划。
- 验证：Python语法通过；Humble四包构建成功；`pc_gvf` 9/9测试通过。新增Python测试
  表覆盖用户要求的15种输入及Z保护，新C++测试覆盖八个水平方向、目标障碍偏转和零Z
  泄漏；四相机USD朝向检查为 `+X/+Y/-X/-Y`。首次沙箱内Isaac联动受ROS套接字权限
  限制失败；主机环境复跑成功创建四路160×120深度视角，123帧零输入保持
  `(0,0,1.5)`且碰撞阻断0。未注入物理键，真实动态15项仍待现场验收。
- 遗留问题：水平360°不提供上下感知或三维绕障；RViz在线点云继续只显示前向兼容
  话题。需要用户在Cloud障碍场实际操作15组输入完成最终动态验收。
- 载入修正：用户首次运行反馈场景未正常出现。检查13:01和13:02两次Kit及性能日志，
  确认USD均成功打开，但主入口默认值仍为历史键盘测试用 `flat`，实际载入的是零障碍
  `flat_ground_navigation.usd`，不是Cloud载入失败。已将完整主入口默认场景改为
  `cloud`，平地仅在显式设置 `FOV_GVF_SCENE_MODE=flat` 时使用，并同步修正文档。

### 2026-09-10 — 新增 USER-EGO 用户工程总览 DONG.md

- 目的：面向用户统一整理当前 USER-EGO 完成进展、运行链路、全部主要目录、文件命名原因、职责边界和参数修改位置。
- 涉及文件：新增 `DONG.md`，更新 `WORK_LOG.md`。
- 具体内容：按当前源码和 launch 实际状态区分主运行链、辅助工具、备用场景、历史迁移文件和未接入的 `pc_gvf_core` 三维势流研究库；记录当前自动导航、2 m/s、在线地图、ESDF、Isaac/RViz 第三人称和渲染配置。在线地图参数采用当前 launch 实际值 `map_resolution=0.08`、`publication_resolution=0.15`，并明确旧文档数值可能滞后。
- 验证：完成目录文件盘点、ROS 包构建项和入口注册核对；后续执行 Markdown 路径与关键名称检查。

### 2026-09-10 — RViz 无人机第三人称跟随视图

- 目的：让 RViz 像 Isaac Sim 的 ThirdPersonCamera 一样，从无人机后上方持续跟随观察。
- 修改：将 `isaac_cloud_navigation.rviz` 默认 View Controller 改为 `rviz_default_plugins/ThirdPersonFollower`，目标帧使用现有 `world -> radar` 动态 TF；默认 `Distance=8 m`、`Pitch=0.42 rad`、`Yaw=3.14159 rad`。原来的全局 Orbit 视图保留为 Saved View `Navigation overview`。
- 影响范围：只改变 RViz 相机显示，不改变 C++ 导航控制、深度输入、在线地图、Isaac Sim 运动或 ESDF 保护。

### 2026-09-10 — 提亮 Isaac Sim 主视口暗部

- 目的：解决 USER-EGO Cloud 场景在 Isaac Sim 主视口中暗处偏暗、第三人称观察细节不够清楚的问题，不改变 ROS 深度输入、导航控制和 ESDF 碰撞保护。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：复查本机 Isaac Sim 6.0 的 `omni.rtx.settings.core` 后确认其实际枚举为 `1 = Constant Diffuse`、`2 = Texture Diffuse`、`3 = Diffuse/Glossy/Emission`，与 `SimulationApp` 中遗留的参数说明不一致；此前使用模式 `3` 才是截图中黑色材质大面积不可见的直接原因。现将默认 `ISAAC_MINIMAL_SHADING_MODE` 修正为 `1`，设置中性浅灰蓝恒定颜色 `ISAAC_MINIMAL_CONSTANT_COLOR=0.60,0.65,0.70`。经过可见主视口逐组抓帧标定，最终采用 `ISAAC_SUN_INTENSITY=500`、`ISAAC_DOME_INTENSITY=0`、`ISAAC_AMBIENT_LIGHT_INTENSITY=0.15`：方向光提供几何轮廓，弱全局环境光抬起阴影，避免原 `3500/2400/2.0` 组合造成过曝；当载入的 USD 场景没有灯时，仍创建运行时灯光节点。
- 验证：Python 语法和 Bash 语法检查通过；ROS 2、C++ 控制器、Isaac Sim、深度相机、ESDF 和第三人称视口均完成短时联动启动。最终主视口抓帧 `/tmp/fov_gvf_constant_contoured.png` 的 `mean_rgb=161.21`，暗部可见且保留障碍轮廓；测试按设定超时正常退出，未改变导航控制参数。
- 遗留问题：默认 Constant Diffuse 更适合工程观察，但会弱化真实材质与光影层次；如需恢复材质颜色，可运行前设置 `ISAAC_MINIMAL_SHADING_MODE=2`。

- 后续对比：用户复验认为 Constant Diffuse 画面灰蒙、阴影不够黑。可见主视口对比测试后，将默认切回无随机采样噪点的 `2 / Texture Diffuse`，并将光照重新平衡为 `ISAAC_SUN_INTENSITY=800`、`ISAAC_DOME_INTENSITY=0`、`ISAAC_AMBIENT_LIGHT_INTENSITY=0.05`。最终抓帧 `/tmp/fov_gvf_texture_contrast.png` 的 `mean_rgb=72.30`；蓝色障碍、灰色地面和绿色无人机均可区分，阴影恢复为深色但没有大面积死黑，灰雾感消失。

### 2026-09-09 — 清除 Isaac Cloud 主视口黑白噪点

- 目的：修复 USER-EGO 运行时 Isaac Sim 第一/第三人称画面出现密集黑白及青色盐粒、难以观察无人机过程的问题，且不改变深度导航输入和控制算法。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：确认噪点只存在于 RGB 主视口，ROS 深度和控制链正常；先后复验 TAA、单 NVIDIA GPU、关闭 TV/film grain、色调映射抖动、随机采样光照、反射、AO 和间接漫反射，RaytracedLighting 截图均没有实质改善，因此没有将这些无效尝试单独当作修复。最终将正常观察默认改为 Isaac 官方 `MinimalRendering` 的 `Textured Diffuse` 模式，保留障碍物/地面/无人机材质色和几何遮挡，避开 156 万三角形体素 Cloud 在完整 RTX 光照下的持续像素噪声；固定 NVIDIA GPU 0 并关闭多 GPU，避免不受 RTX 支持的 AMD 核显参与；保留 `ISAAC_VIEW_RENDERER=RaytracedLighting` 作为显式画质诊断开关。抗锯齿只允许 `OFF/TAA/FXAA`，避免 DLSS/DLAA 破坏低分辨率度量深度标注。
- 验证：三次 GUI 短测逐项排除无效设置，最终短测同时显示 `[RENDER] mode=MinimalRendering`、深度相机 `320×240`、ESDF 正常加载、第三人称相机成功绑定，C++ 控制器从 `WAITING_DEPTH` 进入 `NAVIGATING` 并在 2 秒窗口内开始移动。相同第三人称截图的 3×3 局部中值残差均值由 `33.55` 降至 `0.16`，残差大于 40 的强噪声像素比例由 `26.67%` 降至 `0.09%`，唯一 RGB 颜色数由 `106783` 降为 `52`；人工查看确认盐粒噪声消失，障碍轮廓、地面和无人机可见。导航/ESDF/调和控制代码未修改。
- 遗留问题：默认工程模式不显示完整 RTX 阴影与反射；如用户主动切到 RaytracedLighting，当前超密体素 Mesh 的噪点可能重现，这是画质模式取舍，不影响默认导航。

### 2026-09-09 — USER-EGO 连续调和导引与运动平滑阶段 3–4

- 目的：继续阶段 1–2 的稳定化工作，消除离散像素选路、场刷新重锚、固定控制周期和一阶速度限幅造成的橙色命令箭头异常抖动，同时保持深度直达导航、固定终点、碰撞锥/rollout 安全限制和 ESDF 仿真保护结构。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/test/cpp/angular_field_check.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：阶段 3 在源像素邻域以加权最小二乘拟合调和势局部平面，用连续负梯度和 `2.5` 像素前视替代运行时离散路径第 3 点，遇到障碍时沿同一方向缩短前视；安全目标滞回权重增至 `0.40`，增加 `0.35 s` 最短保持且旧目标失效时立即重锚；节点保留世界系命令方向作为场刷新间的活动参考，调和 CG 使用上一有效势函数 warm start。阶段 4 将角速度限制改用实际 ROS 控制周期，加入显式命令加速度状态与 `8 m/s³` jerk 限制；速度恢复限制为 `1.0 m/s²`，预测制动允许 `2.5 m/s²`，两者非对称；只有导引射线进入约 `0.05 m` 接触距离才旁路平滑急停。首次实现误将所有 rollout 限速视为急停，实测出现 4 次在途单帧归零，随后拆分“预测性制动”和“真正急停”，并消除加速度上限切换及越过目标速度时的内部状态跳变。
- 验证：隔离 Isaac 安装重新构建成功；五项 C++ 核心测试全部通过，其中新增连续梯度无像素量化和 harmonic warm-start 一致性测试；Isaac Python 兼容环境中的 3 项既有 Python 基准测试通过。系统 Python 的两项 Python 测试仍因本机 NumPy 2.2.6 与 Ubuntu SciPy 二进制 ABI 不兼容而仅在收集期失败，与源码结果无关。最终完整 Cloud 场景从 `(-64,-11.2,2.8)` 出发，在 `t=70.35 s` 到达 `(63.274,11.862,4.926)`，无碰撞、无在途零速度脉冲；`/position_cmd` 记录 3517 帧、`50.02 Hz`，速度变化 P50/P95/最大值为 `0.00323/0.02144/0.08294 m/s`，加速度 P50/P95/最大值为 `0.1715/1.20/2.4883 m/s²`，方向角速度 P50/P95/最大值为 `0.0774/0.5980/1.2065 rad/s`，jerk P50/P95/最大值为 `0.928/8.000/8.000 m/s³`。相比阶段 1–2 的速度变化 P95 `0.040 m/s` 和 jerk P95 约 `60–70 m/s³`，异常控制抖动已消除；运行中累计约 `2.38 s` 的 `DEGRADED` 为调和场暂时无效时的连续限角回退，期间仍满足平滑约束，不属于箭头抖动。
- 遗留问题：当前实跑已满足抖动验收，无需继续用显示滤波掩盖控制问题。后续若要提高算法鲁棒性，可独立处理偶发 `DEGRADED`：只在源点所在自由连通域组装 Laplace 方程，并发布 solver 迭代数/残差诊断；这不是本阶段安全运行或平滑性的阻塞项。

### 2026-09-09 — USER-EGO 运行隔离与深度时域稳定阶段 1–2

- 目的：消除 RViz 原始控制箭头因重复控制器、低分辨率深度轮廓和单帧障碍释放造成的异常跳变，同时不改变既有调和场导航目标和安全限速结构。
- 涉及文件：`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/pc_gvf_platforms/navigation_visualizer.py`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、相关测试、`USER_EGO_GUIDE.md` 和重新生成的 Cloud 导航 USD。
- 修改内容：USER-EGO 默认隔离到 `ROS_DOMAIN_ID=42`，增加单实例文件锁、启动前零发布者检查、启动后单发布者验收和独立进程组清理；RViz 同时发布橙色原始命令箭头与紫色 0.20 秒显示专用低通箭头；深度相机垂直孔径由 `16` 改为 `18`，在保持 90 度水平视场时使 `fx=fy`（原 `160×120` 配置对应 80，升级到 `320×240` 后对应 160），输出分辨率由 `160×120` 提升为 `320×240`，Replicator 实时传感器渲染切换为 FXAA；C++ 角域规划掩码增加“新障碍立即占用、连续三张新深度帧为空才释放”的非对称滞回，并避免控制循环重复使用同一帧时错误累计；Python ROS 节点补充优雅处理外部关闭异常。
- 验证：Shell/Python 静态检查通过；三项 ROS 2 包构建成功；`pc_gvf_platforms` 6 项测试、5 项 C++ 核心测试及 Isaac Python 环境中的 3 项原有 Python 数值/行为基线全部通过。完整 Isaac Sim + RViz 实跑启动时确认 `/position_cmd` 发布者恰为 1，重复启动被锁拒绝；无人机在 `t=69.85 s` 从 `(-64,-11.2,2.8)` 到达 `(63.468,11.992,5.064)`，没有碰撞；控制遥测最大相邻速度变化 `0.04 m/s`、最大加速度 `1.20 m/s²`。相机内参强制修正告警消失；显式调用 Replicator FXAA 后 DLSS 低输入分辨率告警也在短测中消失。
- 遗留问题：当前阶段只稳定运行环境、感知输入和显示，不包含连续调和梯度、状态锚定参考或 jerk 限制；这些属于后续第三、四阶段。

### 2026-09-09 — 建立 user_ego 自动导航双视角副本

- 目的：保留 user 键盘版本不变，另建只负责自动起点到终点导航、并可切换 Robot 第一/第三人称相机的独立副本。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、副本中的路径与构建脚本、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：从提交 `d267feb` 重新复制到 `/home/starry/isaac-data/user_ego/Fov-gvf`；保持固定前进、固定终点和原 C++ 避障链路；增加绑定 Robot 的第三人称跟随相机及 `1/3/V` 视角切换；构建目录隔离为 `/tmp/fov_gvf_user_ego_isaac_*`。
- 验证：相关 Python 文件通过 `py_compile`，全部 Shell 脚本通过 `bash -n`；确认 launch 中 `fixed_forward_intent=True`、`use_fixed_goal=True`、`stop_at_goal=True`，并确认整个副本不存在 `/keyboard_intent`、运动键状态或 dead-man 代码。成功重新生成带两台 Robot 子相机的 4 倍 Cloud USD，三项 ROS 2 包在独立目录构建成功。Isaac Sim 与 RViz 实跑中默认启用 `/World/Robot/ThirdPersonCamera`，主视口平均 RGB `96.07`；无人机自动从 `(-64,-11.2,2.8)` 出发，经三维避障后在 `t=83.18 s` 到达 `(63.433,11.999,4.910)` 并进入 `GOAL_REACHED`，未触发碰撞保护。
- 遗留问题：无。

### 2026-09-09 — 接入 4 倍 EGO-Swarm Cloud 场景与 ESDF 三维碰撞保护

- 目的：在独立分支使用 `/home/starry/isaac-data/ego_swarm_cloud` 的 Perlin 三维云场景继续无人机避障任务，并按用户要求不用合并 Mesh 的 AABB、加入 RViz、将地图整体放大 4 倍。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/prepare_ego_swarm_cloud_navigation.sh`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、`src/pc_gvf_platforms/test/test_platform_math.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`scenes/ego_swarm_cloud/`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：环境 Mesh 从 `40×30×5 m` 统一放大为约 `160×120×20 m`，无人机不缩放；场景准备器记录 4 倍比例并把相机量程同步扩至 `40 m`。复制 `400×300×50` 的 `occupancy.bin`，在 Isaac Python 中通过欧氏距离变换建立 ESDF，分辨率随场景变为 `0.4 m`，候选位置使用三线性插值并按 `0.25 m` 机体半径拦截碰撞，不对单体 Cloud Mesh 使用 AABB。速度桥新增 `z` 速度与 `max_vz`，仿真运动学积分改为三维，控制器最大垂直速度设为 `1.0 m/s`。起终点为 `(-64,-11.2,2.8)`、`(64,11.2,4.8)`；RViz 默认启动，网格和观察距离扩大到大场景尺度。
- 验证：导航 USD 重新生成成功，OpenUSD 复核确认场景边界约为 `160×120×20 m`、包含 `/World/PhysicsScene`、Robot、DepthCamera 和 2 个静态碰撞 Mesh；运行时成功从 `occupancy.bin` 建立分辨率 `0.4 m` 的 ESDF。ROS 2 三包构建成功；`pc_gvf_platforms` 6 项测试全部通过（含新增垂直速度限幅），`pc_gvf` 5 项 C++ 测试通过。Isaac Sim 与 RViz 同时启动，RViz 使用 OpenGL 4.6，Isaac 主视口平均 RGB 为 `204.27`；无人机从 `(-64,-11.2,2.8)` 出发，经 `(-28.77,-2.97,3.05)`、`(-0.33,-4.02,3.13)`、`(28.10,2.58,1.87)`、`(57.14,9.21,3.57)` 完成三维避障，在 `t=68.52 s` 到达 `(63.331,11.927,4.734)` 并进入 `GOAL_REACHED`，未触发碰撞保护。Python/Shell 静态检查和 `git diff --check` 通过。
- 遗留问题：`pc_gvf` 的 2 项旧 Python 测试在系统 Python 环境收集阶段因 NumPy 2.2.6 与旧 SciPy 二进制不兼容而报错；这不是本次 ESDF 运行环境的问题，Isaac Python 内的 NumPy 2.5.2 与 SciPy 1.18.1 已实际完成 ESDF 构建和闭环运行。

### 2026-09-09 — 完成 EGO-Swarm Blender 场景运行接入

- 目的：补完上一项未完成的 `ego_swarm_blender` Isaac Sim 导航场景接入，并进行真实运行验证。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/prepare_ego_swarm_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm/ego_swarm_navigation.usd`、`WORK_LOG.md`。
- 修改内容：导航场景补充 `/World/PhysicsScene`；使用 501 个静态碰撞 Mesh、前向深度相机和 C++ 深度角域控制器执行约 60 m 横穿任务。首次终点为 `(30,-1,1.2)`；在该目标附近触发碰撞保护后，另准备了净空更大的 `(25,4,1.2)` 终点配置，但按用户决定不继续以到达终点作为此项接入验收条件。
- 验证：Python/Shell 语法与 `git diff --check` 通过；三项 ROS 包成功重建。Isaac Sim 实跑成功加载场景，识别 500 个空中障碍，视口平均 RGB `223.93`；控制器进入 `NAVIGATING` 并以接近 `2 m/s` 从 `(-30,0,1.2)` 飞至 `(25.772,-0.248,1.2)`，期间曾在局部无有效场时进入 `DEGRADED`，最终由 `COLLISION_GUARD` 安全拦停，没有穿越障碍。用户确认能正常运行即可，允许进入下一项。
- 遗留问题：在此超长、密集的柱体/圆环随机场景中，当前局部深度角域算法未保证到达远端目标；这是算法可达性问题，不是 Isaac/ROS 场景接入故障。

### 2026-09-09 — 保存 EGO-Swarm Blender 场景适配中间检查点

- 目的：在转向 `ego_swarm_cloud` 场景前，保存当前 `feature/ego-swarm-blender-navigation` 分支已完成的场景泛化工作，避免丢失并为新分支提供基础。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/prepare_ego_swarm_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm/ego_swarm_navigation.usd`、`WORK_LOG.md`。
- 修改内容：场景准备器支持通过命令行创建起终点；运行器的灯光和碰撞保护由旧场景固定路径/14 障碍泛化为按 USD 类型与飞行高度识别；默认场景切换为由 `/home/starry/isaac-data/ego_swarm_blender/ego_swarm.usd` 派生的导航 USD；设置 `(-30,0,1.2)` 至 `(30,-1,1.2)` 横穿任务和大场景观察相机。
- 验证：相关 Python、Shell 语法检查和 `git diff --check` 已通过；导航 USD 成功生成，OpenUSD 检查确认 501 个 Mesh/碰撞体、起点、终点、无人机和深度相机存在。
- 后续完成：场景准备器已补充 `/World/PhysicsScene`；导航 USD 重新生成、ROS 包重新构建和 Isaac 闭环实跑结果见同日后续完成记录。

### 2026-09-08 — 保存当前 Isaac C++ 深度导航基线

- 目的：在适配 `ego_swarm_blender` 新场景前保存当前可运行成果，确保后续场景迁移可以独立开发和安全回退。
- 涉及文件：`.gitignore`、当前工作区内尚未提交的 C++ 控制器、ROS launch、Isaac/Blender 脚本、场景、测试、方案文档及 `WORK_LOG.md`。
- 修改内容：把当前 `feature/cpp-3d-voxel-flow` 的项目成果整理为 Git 基线；将 `build_current_humble/`、`build_humble/`、`install_current_humble/`、`install_humble/`、`log_current_humble/`、`log_humble/` 明确列为可再生成构建产物并排除在提交外。
- 验证：提交前执行 `git diff --check`；具体提交号和新分支状态在完成 Git 操作后补充。
- 遗留问题：此基线仍使用 `scenes/blender_isolated_clouds` 的 seed 42 云团场景；`ego_swarm_blender` 场景适配将在新分支完成。

### 2026-09-08 — 将 Isaac 场景导航速度提高到 2 m/s

- 目的：把当前 C++ 深度角域控制器在 Isaac 云团场景中的目标移动速度由 `0.45 m/s` 提高到 `2.0 m/s`。
- 涉及文件：`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`ISAAC_FOV_GVF_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：将控制器的 `speed` 与 `max_speed` 同步设为 `2.0`，并将下游 `position_cmd_to_twist` 的 `max_vx`、`max_vy` 同步放宽至 `2.0`，避免适配器二次限速；保留控制器原有加速度限制、安全制动、深度 rollout 和避障降速逻辑。
- 验证：launch Python 语法检查和 `git diff --check` 通过；使用 `scripts/build_isaac_ros_workspace.sh` 将 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三包重新构建到 `/tmp/fov_gvf_isaac_install`，构建成功，并确认安装后的 launch 四个速度参数均为 `2.0`。随后以关闭 RViz 的方式完成 Isaac 图形闭环实跑：控制器进入 `NAVIGATING`，无人机从 `(-8,0,1.2)` 经场景上侧绕障，在 `t=29.72 s` 到达 `(7.711,0.074,1.200)`，未触发 `COLLISION_GUARD`。同一任务原 `0.45 m/s` 配置耗时约 `39.5 s`。一次未设置 `ROS_LOG_DIR` 的 `ros2 launch --show-args` 辅助检查因沙箱禁止写入 `~/.ros/log` 而失败，与 launch 内容无关；正式脚本使用 `/tmp/fov_gvf_isaac_ros_log` 并成功运行。
- 遗留问题：`2.0 m/s` 是目标速度与硬上限，不是强制恒速；本次障碍密集场景中安全制动和 depth rollout 将日志采样点的实际平移命令主动降至约 `0.45 m/s`，因此已经允许空旷区加速到 `2.0 m/s`，但不会为追求恒定 `2.0 m/s` 而绕过避障安全约束。

### 2026-09-08 — 修复 Isaac 黑视口并接入 RViz 同步观察

- 目的：解决闭环运行期间 Isaac Sim 主视口全黑、且一键脚本不打开 RViz 的可视化问题。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`ISAAC_FOV_GVF_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：最初在非 headless 模式下移动 `/OmniverseKit_Persp`，但用户复验确认 Isaac 主视口仍为黑屏；现改为创建独立 `/World/NavigationOverviewCamera`，等待主视口 ready，并在 ROS 深度 RenderProduct 创建完成后把活动视口显式绑定到该相机，防止传感器 RenderProduct 影响 UI 视图；增加实际主视口 RGB 截图和平均亮度诊断；观察点和目标点可分别通过 `ISAAC_VIEW_EYE`、`ISAAC_VIEW_TARGET` 调整。诊断阶段主视口与独立离屏截图均为 `mean_rgb=0.00`，进一步确认并非相机或视口绑定，而是 Blender USD 的 Sun/Dome 强度仅为 `0.55/1.0`、不适合 Isaac RTX；运行时现默认提升为 `3000/900`，并允许通过 `ISAAC_SUN_INTENSITY`、`ISAAC_DOME_INTENSITY` 覆盖。验证根因后移除临时的第二路离屏 RenderProduct，避免正式运行重复渲染。导航 launch 默认同时启动 `navigation_visualizer` 和使用仿真时间的 `rviz2`；Isaac 专用 RViz 配置显示无人机、命令、轨迹、里程计、实时深度图、FOV、深度点和角域 GVF；一键脚本支持用 `FOV_GVF_RVIZ=false` 显式关闭 RViz。
- 验证：第一版 Shell、Python 和 launch 静态检查及 `git diff --check` 均通过；`pc_gvf_platforms` 增量构建成功，新 RViz 配置已安装到 `/tmp/fov_gvf_isaac_install`，平台包 5 项测试全部通过。实际图形运行确认 `rviz2` 进程成功启动并使用 OpenGL 4.6，控制器进入 `NAVIGATING`；无人机经 `(0.14,-1.00)`、`(4.45,-1.96)`、`(6.15,-1.38)` 绕障后，在 `t=39.47 s` 到达 `(7.766,-0.185,1.200)` 并进入 `GOAL_REACHED`，未触发碰撞保护。用户随后确认 RViz 正常但 Isaac 仍黑；第二轮实跑确认活动视口已绑定 `/World/NavigationOverviewCamera`、分辨率为 `1280×720`，但主视口与独立离屏相机截图平均 RGB 均为 `0.00`，从而将根因收敛到可见光照。将 Sun/Dome 提升至 `3000/900` 后，主视口截图平均 RGB 恢复为 `97.88`、独立诊断相机为 `98.43`，截图可清楚看到地面、14 个障碍物和无人机；同次闭环在 `t=39.53 s` 到达 `(7.768,-0.183,1.200)`，未触发碰撞保护。移除临时离屏诊断后，Shell、Python 静态检查和 `git diff --check` 再次通过。
- 遗留问题：RViz 当前显示实时感知到的深度障碍点，不显示从 USD 读取的完整障碍 Mesh，以保证控制可视化与控制器实际观测一致。

### 2026-09-08 — 恢复 Blender 云团场景的首次 C++ 控制器闭环运行

- 目的：使用当前已部署的 C++ 深度角域控制器，让运动学四旋翼在 Blender 导入的 14 障碍场景中完成首次真实避障运行。
- 涉及文件：`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`WORK_LOG.md`。
- 修改内容：修正导航 launch 遗留的终点坐标方向错误，将 `goal_x` 从起点 `-8.0` 改为导航 USD 中 `/World/UAV_Goal` 的实际 X 坐标 `8.0`；首次实机运行确认深度和 CameraInfo 发布者存在且约为 136–137 Hz，但控制器因仿真时间消息与墙钟不一致持续报告 `STALE_DEPTH`，因此在 OmniGraph 中增加 `/clock` 发布并让控制器/速度适配器启用 `use_sim_time`；只在 Isaac 子进程中清除系统 ROS 2 Python 3.10 路径，并加载 Isaac Sim 内置 Humble bridge 库，避免污染其 Python 3.12 环境；加入机体 yaw、body/world Twist 正确互换和零速原地扫描；把控制器的历史命令与历史安全目标改为世界系保存并按当前相机姿态逐帧重投影；最终利用全向平台的独立航向自由度让 90 度前视相机持续朝向终点，而平移仍执行 GVF 指令，并将安全余量设为 `0.20 m`、速度设为 `0.45 m/s`。
- 验证：终点、Shell 和 Python 静态检查通过；ROS 2 三包干净构建成功，修改后的 `pc_gvf` 增量构建成功，隔离安装中的 C++ 控制器、速度适配器和修正后的 launch 均存在；`pc_gvf` 的 7 项 C++/Python 测试全部通过。前十一次运行依次定位并排除了时间基准、Isaac Python 环境、固定相机、历史像素坐标系、过宽视场退化及擦边安全余量问题；其中 120 度和 105 度视场均安全停在起点，未被误判为成功。最终第十二次运行从 `(-8.00,0.00,1.20)` 出发，经 `(-6.08,0.78)`、`(-3.93,0.87)`、`(-1.99,-0.19)`、`(0.07,-0.96)`、`(2.18,-1.56)`、`(4.38,-1.96)`、`(6.09,-1.40)` 绕过障碍，在 `t=39.60 s` 到达 `(7.765,-0.184,1.200)` 并进入 `GOAL_REACHED`；全过程未触发 `COLLISION_GUARD`，闭环验收通过。
- 遗留问题：当前成功结果针对 seed 42 的 14 障碍场景和运动学全向四旋翼；尚未验证其他随机种子，也尚未接入真实旋翼动力学或飞控。当前接管路线仍是 C++ 深度角域控制器，不代表 C++ 三维体素势流已经接管。

### 2026-09-08 — 将无人机和 FOV-GVF 导航适配到 Isaac Sim 场景

- 目的：把当前 Blender 云团环境扩展为可放置无人机、可发布深度/里程计并接收 FOV-GVF 速度命令的 Isaac Sim 导航场景，起点到终点采用定高绕障任务。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/build_isaac_ros_workspace.sh`、`scripts/export_blender_cloud_to_isaac.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`ISAAC_FOV_GVF_NAVIGATION.md`、`scenes/blender_isolated_clouds/isolated_clouds_seed42_navigation.usd`、`WORK_LOG.md`。
- 修改内容：在 USD 中加入 `/World/Robot` 四旋翼可视模型、前向 `/World/Robot/CameraMount/DepthCamera`、机体半径和导航元数据；新增 Isaac Sim ROS 2 OmniGraph 适配，发布 `/sim/odom`、`/sim/depth/image_raw`、`/sim/depth/camera_info`，订阅 `/sim/cmd_vel` 并按速度命令更新无人机；新增固定终点、定高、低速和安全余量参数的 ROS 2 launch；一键导出脚本现在自动生成环境 USD 和导航 USD；新增干净构建脚本。
- 验证：导航 USD 的 OpenUSD 检查确认起点 `(-8,0,1.2)`、终点 `(8,0,1.2)`、Robot、DepthCamera 和 14 个障碍均存在；ROS 2 三包工作区使用 `/tmp/fov_gvf_isaac_install` 干净构建成功。受限沙箱闭环尝试未完成：控制器和速度适配器启动，但 DDS 被 `getifaddrs/UDP socket Operation not permitted` 阻断，Isaac Sim 另报告 `NVML_ERROR_DRIVER_NOT_LOADED`，无人机安全停在起点；这属于运行环境限制，不能记作导航成功。非沙箱 GPU/ROS 运行仍需在具备 NVIDIA 驱动和 DDS 网络权限的环境中复验。
- 遗留问题：当前 Isaac 适配使用运动学四旋翼（速度接口），尚未接入真实旋翼动力学/飞控；未完成具备 GPU 和 DDS 权限环境中的最终 `GOAL_REACHED` 闭环验收。

### 2026-09-08 — 将 Blender 云团场景导入 Isaac Sim

- 目的：保留当前 Blender 母版，并将其材质、14 个障碍及起终点导出为可直接载入 Isaac Sim 的物理 USD。
- 涉及文件：`scripts/blender/export_scene_to_usd.py`、`scripts/isaac/prepare_blender_cloud_usd.py`、`scripts/isaac/view_blender_cloud_scene.py`、`scripts/export_blender_cloud_to_isaac.sh`、`BLENDER_ISOLATED_CLOUD_SCENE.md`、`scenes/blender_isolated_clouds/`、`WORK_LOG.md`。
- 修改内容：新增 Blender Z-up/米制 USD 导出脚本；新增 OpenUSD 后处理，为场景加入 PhysicsScene，为地面和每个云团 Mesh 加入精确静态 CollisionAPI/MeshCollisionAPI，并为 `UAV_Start`、`UAV_Goal` 写入显式角色属性；增加独立 Isaac Sim 查看器和可重复的一键导出命令。
- 验证：Blender USD 导出和 Isaac 后处理脚本通过语法检查；Blender 5.2.1 成功输出原始 USD，OpenUSD 复核确认 Z-up、1 米单位、14 个云团 Mesh、起终点均存在；后处理后 `PhysicsScene`、地面碰撞和全部 14 个精确静态 MeshCollisionAPI 均存在，起终点角色属性分别为 `uav_start`/`uav_goal`；Isaac Sim 6.0.1 在 RTX 5070 上成功 `open_stage returned True` 并打印场景就绪信息。后续导航验证发现 Blender 的可选 USD 前向轴转换会镜像已在 Z-up 世界中 authored 的 X 方向，已关闭 `convert_orientation` 并将 Isaac 导航目标与实际 `/World/UAV_Goal` 读取保持一致。
- 遗留问题：尚未把实际无人机、深度相机和 ROS 2 控制闭环加入该 USD。

### 2026-09-08 — 在 Blender 中实现独立云团柱体地图

- 目的：将独立不规则云团地图实现为 Blender 原生可编辑三维场景，并保留明确的无人机起点和终点。
- 涉及文件：`scripts/blender/generate_isolated_cloud_scene.py`、`scripts/run_blender_isolated_cloud_scene.sh`、`BLENDER_ISOLATED_CLOUD_SCENE.md`、`scenes/blender_isolated_clouds/`、`WORK_LOG.md`。
- 修改内容：通过 Blender Python API 生成白色地面、独立封闭云团柱体 Mesh、起终点彩色圆盘及位于 `z=1.2 m` 的 `UAV_Start`/`UAV_Goal` 空对象；使用障碍外接圆执行保守最小间距约束；保存 `.blend`、JSON manifest 和顶视 PNG，并增加一键生成后打开 Blender 的脚本。
- 验证：系统 Python 语法检查和 Shell `bash -n` 通过；使用已安装 Blender 5.2.1 LTS 后台成功生成并渲染 seed 42，输出 `.blend`、JSON 和 PNG，保守保证的最小障碍间隙为 `0.936 m`（请求 `0.90 m`）；重新打开 `.blend` 检查确认 `Cloud_Obstacles` 恰有 14 个对象、Ground 和 Overview_Camera 存在，`UAV_Start=(-8,0,1.2)`、`UAV_Goal=(8,0,1.2)`；`git diff --check` 通过。首次实测发现该 Blender 构建的 Eevee 枚举仍为 `BLENDER_EEVEE` 且空场景没有 World，修正后生成成功；后台运行仍打印 PulseAudio mainloop 权限提示和无法写用户缩略图缓存提示，但不影响 `.blend` 与预览输出。
- 遗留问题：尚未导入具体无人机模型，也未导出 USD 或在 Isaac Sim 中复核材质、碰撞与坐标层级。

### 2026-09-08 — 新增独立不规则云团场景生成器

- 目的：新增障碍物彼此不直接连接、每团形状不规则且团间留出无人机通行空隙的场景生成方式，保留原云状随机场生成器不变。
- 涉及文件：`scripts/isaac/generate_isolated_cloud_scene.py`、`scripts/isaac/view_isolated_cloud_scene.py`、`scripts/run_isolated_cloud_scene.sh`、`ISAAC_ISOLATED_CLOUD_SCENE.md`、`WORK_LOG.md`。
- 修改内容：使用带低频角向扰动的椭圆云团逐个放置；通过栅格膨胀约束保证云团间最小间距；只保护起终点圆盘而不人工挖出显眼主通道；验证黑色连通分量数量与云团数量相同、带安全净空的自由空间连通，并将黑色区域复用现有精确静态 Mesh 挤出为三维柱体。
- 验证：初始默认 16 团在保留人工主通道的尺寸和间隙约束下安全拒绝（只能放置 8 团）；中间参数成功生成 10 个分量，但呈上下两排，因此改为全场随机分布 14 团并仅保护起终点；最终使用欧氏距离场执行团间间隙约束和实际最小间隙复核。新增脚本通过 `py_compile` 和 `bash -n`，seed 12、27、42、68、103、205 均成功生成，每张恰有 14 个黑色连通分量、带净空自由区为 1 个连通分量，实测最小团间距为 `0.914–0.937 m`，均不小于请求的 `0.90 m`；默认 seed 42 的 OpenUSD 复核确认精确静态 CollisionAPI/MeshCollisionAPI 存在且 approximation 为 `none`；`git diff --check` 通过。
- 遗留问题：尚未与无人机、深度相机和 Fov-gvf 闭环合入同一启动脚本。

### 2026-09-08 — 新增全连通裂隙迷宫场景生成器

- 目的：保留原有白底黑色云状障碍生成方式，同时新增“从全黑障碍中挖出全连通白色裂隙道路”的 Isaac Sim 场景，包含分岔、死胡同和急转弯，且不产生独立自由空间。
- 涉及文件：`scripts/isaac/generate_crack_maze_scene.py`、`scripts/isaac/view_crack_maze_scene.py`、`scripts/run_crack_maze_scene.sh`、`ISAAC_CRACK_MAZE_SCENE.md`、`scenes/crack_maze/crack_maze_seed42.{usd,json}`、`scenes/crack_maze/crack_maze_seed42_map.png`、`WORK_LOG.md`。
- 修改内容：使用偏向长路径并随机回访旧节点的 Growing Tree 迷宫算法，在粗网格上建立覆盖全部路口且具有更多分岔的生成树，再将带路口扰动和逐边宽度变化的树边栅格化为白色道路；选择树直径端点作为起终点；强制验证全部白色区域只有一个连通分量，以及起终点在指定净空阈值下连通；将其余黑色区域拉伸为带精确静态 Mesh 碰撞的三维障碍，并增加独立生成、查看和一键启动入口。原云状场景脚本未修改。
- 验证：两个新增 Python 脚本通过 `py_compile`，新增 Shell 启动器通过 `bash -n`，`git diff --check` 通过；使用 Isaac Sim 自带 Python 成功生成 seed 42，并对 seed 1、7、103、999 做额外生成抽查；seed 42 的全部白色区域和按 `0.35 m` 净空收缩后的中心区均各为 1 个连通分量，具有 7 个分岔点、9 个末端，实际黑色障碍占比约 51.51%；OpenUSD 复核确认 default prim、PhysicsScene、MazeField、CollisionAPI、MeshCollisionAPI 均存在，碰撞近似为 `none`；Isaac Sim 6.0.1 在 RTX 5070 上成功打开新 Stage，打印 `open_stage returned True` 及三行 `[SCENE READY]`，查看器保持运行。
- 遗留问题：场景尚未与无人机、深度相机和 Fov-gvf 闭环合入同一启动脚本；尚未用动态刚体执行接触碰撞实测。

### 2026-09-08 — 新增 Isaac Sim 云状柱障碍场景生成器

- 目的：按用户提供的俯视参考图生成随机黑色障碍/白色通道，并将二维障碍区域拉伸为 Isaac Sim 中可碰撞的三维云状柱体。
- 涉及文件：`scripts/isaac/generate_cloud_pillar_scene.py`、`scripts/isaac/view_cloud_pillar_scene.py`、`scripts/run_cloud_pillar_scene.sh`、`ISAAC_CLOUD_SCENE.md`、`scenes/cloud_pillars/cloud_pillars_seed42.{usd,json}`、`scenes/cloud_pillars/cloud_pillars_seed42_map.png`、`WORK_LOG.md`。
- 修改内容：使用固定种子的多尺度高斯随机场生成平滑连通斑块；清出弯曲飞行走廊并用距离变换检查起终点宽通道连通性；将占据栅格合并为一个带精确静态三角网格碰撞体的黑色拉伸柱体；输出 USD、manifest 和二维占据图；增加显式载入 Stage、检查 CloudField 并设置固定斜俯视视角的 Isaac Sim 查看器和一条启动命令。
- 验证：生成器和查看器通过 `py_compile`，启动脚本通过 `bash -n`；使用 Isaac Sim 自带 Python/OpenUSD 成功生成并重新打开 seed 42 场景。默认实际障碍占比约 53.85%，验证通道净空阈值为 0.5225 m，网格包含 84884 个顶点和 21221 个四边形面；独立 OpenUSD 复核确认 default prim、PhysicsScene、CloudField、CollisionAPI 和精确 MeshCollisionAPI 均存在；Isaac Sim 6.0.1 在 RTX 5070 上打印 `open_stage returned True` 以及三行 `[SCENE READY]`，自动斜俯视查看器保持运行。
- 遗留问题：当前场景只包含障碍、地面、标记和灯光，尚未把 ARL Robot、深度相机和 Fov-gvf 控制闭环合入同一个启动脚本；尚未用动态刚体执行接触碰撞实测。

### 2026-09-07 — 记录深度图直驱三维势流导航的换账号交接方案

- 目的：把用户最新确定的“深度图直接导航、不建立持久三维地图/ESDF”的技术路线整理为可脱离原聊天记录独立阅读的交接文档，便于更换账号后继续实施。
- 涉及文件：`DIRECT_DEPTH_3D_NAVIGATION_HANDOFF.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增换账号交接文档，明确删除点云构造、跨帧融合、占据地图和 ESDF 环节，但保留短生命周期局部三维求解网格；
  - 记录目标数据流、选择理由、当前已经完成和尚未完成的真实状态、约 `8.2 ms` 的三维导引性能基线及第一轮 CPU 优化顺序；
  - 固化 `Hit / NoReturn / Invalid`、Unknown、`TrustedFree` 和 fail-closed 安全边界；
  - 记录与 FlowForge 点云/ESDF 路线的可复用部分和不能直接复制的边界；
  - 指定新账号从阶段 `3D-1` 接手，并列出阅读顺序、单步任务、退出条件、关键文件和脏工作树保护要求。
- 验证：仅新增/更新 Markdown 文档，未修改算法和运行配置；已检查文档内容与当前分支 `feature/cpp-3d-voxel-flow`、现有源码状态及已记录性能数据一致，未把未执行的测试写为通过。
- 遗留问题：阶段 `3D-1` 尚未开始；下一步先实现显式深度像素语义及其纯 C++ 测试，正式 `/position_cmd` 继续由当前 C++ 角域控制器独占。

### 2026-09-07 — 建立 C++ 三维体素势流路线分支与实施方案

- 目的：在已经完成 C++ 深度角域迁移的基础上，建立独立开发分支，核实现有三维体素/势流能力与缺口，并制定不影响当前稳定控制路径的分阶段切换方案。
- 涉及文件：`CPP_3D_VOXEL_FLOW_PLAN.md`、`WORK_LOG.md`；新分支为 `feature/cpp-3d-voxel-flow`。
- 修改内容：
  - 从当前 `feature/local-modifications` 工作状态建立 `feature/cpp-3d-voxel-flow`，完整保留尚未提交的迁移成果和用户已有构建目录，不删除、不重置任何现有修改；
  - 阅读并核对 `pc_gvf_core` 的 `DepthFrame3D`、深度栅格投影、三维有限体积 Poisson 系统、粗细场边界先验、矩阵无关 CG、三线性场采样、停滞横流锁存、积分流线和横向误差反馈实现；确认当前 `pc_gvf` 没有调用 `FluidGuidance::calcGuidance3D()`；
  - 对照论文截图和 `FOV_GVF_原理与CPU方案分析.md`，区分已有工程近似与论文条件：当前参考是积分折线最近点反馈，不是完整流盒坐标/Jacobian 伪逆；当前三维核心也没有最终连续运动和制动尾段认证；
  - 新增三维路线计划，明确采用局部滚动三维状态栅格而非持久全局地图，复用 `pc_gvf_core`，按纯 C++ 契约加固、流线/安全核心、非执行 ROS 影子、仿真可视化、闭环 CPU 验收、可回退切换六个实现阶段推进；
  - 明确零深度/无回波/无效值、近相机 `TrustedFree`、数据 freshness、垂向控制和未知空间是正式接管前的强制修正项；当前已验收的 C++ 角域控制器继续独占正式输出。
- 验证：
  - `git branch --show-current` 确认为 `feature/cpp-3d-voxel-flow`；
  - 使用独立 `/tmp/fov_gvf_3d_analysis_{build,install,log}` 前缀以 CMake Release 构建 `pc_gvf_core` 成功；
  - 现有 `depth_guidance_integration_check --benchmark` 各测 40 次：单线程粗细深度投影 P50/P95 约 `7.472/7.489 ms`，完整三维导引约 `8.256/8.268 ms`；四线程分别约 `7.464/7.476 ms` 与 `8.138/8.208 ms`；
  - 四线程收益很小，证实现阶段首要 CPU 瓶颈是逐体素深度包围框扫描；当前结果可支持 10 Hz 场更新加 50 Hz 缓存场跟踪的首版设计，但不能证明完整链路小于 1 ms。
- 遗留问题：本记录只完成分支、源码核对、性能基线和实施设计，尚未修改默认控制算法；下一步从阶段 3D-1 开始，先加固深度三态语义、近域证据和结构化失败结果，未经影子及闭环验收不得让三维路线发布 `/position_cmd`。

### 2026-09-07 — 阶段 5.2：清理影子迁移设施并完成实现收口

- 目的：在 Python 控制器已经删除且正式 C++ 路径通过闭环验收后，移除只为双实现并行验证而存在的影子构建、启动入口、诊断数据和在线对照工具，将运行结构收敛为单一正式 C++ 控制器。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`（重命名为 `src/pc_gvf/src/depth_angular_controller_node.cpp`）、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 将同时生成 active/shadow 两个目标的 CMake 循环改为只构建和安装 `depth_angular_controller`，删除 `PC_GVF_SHADOW_MODE` 编译定义及 `depth_angular_shadow` 安装目标；
  - 将共享源码正式命名为 `depth_angular_controller_node.cpp`，固定节点名、`cmd_topic` 参数和 `/pc_gvf/angular_field` 输出，删除影子命令/可视化话题分支及影子模式日志；
  - 删除迁移期 `~/diagnostics` JSON 发布器、逐帧汇总与求解计时；该接口只被双实现对照探针消费，不属于冻结公共控制契约，移除后也减少了每个控制周期的字符串格式化开销；
  - demo launch 删除 `shadow` 参数和条件影子进程，保留正式控制器、合成模拟器、平台可视化和 RViz 结构；删除已经失去第二实现输入源的 `tools/ros_shadow_parity_probe.py`；
  - 明确保留 Python 数值核心、场景/合成模拟器、黄金数据生成器、C++ 闭环与状态探针作为持续回归基础；保留 `pc_gvf_core` 独立二维/三维 C++ API，不把仍有独立功能的模块当作迁移垃圾删除。
- 验证：
  - 使用全新的 `/tmp/fov_gvf_stage52_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功，`depth_angular_controller_node.cpp` 在 `-Wall -Wextra -Wpedantic` 下编译、链接成功；全量测试汇总 18 项，0 错误、0 失败、0 跳过；
  - 干净安装的 `ros2 pkg executables pc_gvf` 只列出 `depth_angular_controller`、`depth_angular_core` 和 `depth_angular_demo_sim`，不存在 `depth_angular_shadow`；launch 参数只剩 `rviz` 与 `scenario`；
  - 安全状态探针再次完整观测 `WAITING_ODOMETRY → STALE_INTENT → ZERO_INTENT → WAITING_DEPTH → NAVIGATING → STALE_DEPTH → NAVIGATING → STALE_INTENT → INVALID_ODOMETRY → INVALID_GUIDANCE → GOAL_REACHED`，共收到 290 个命令样本，保护状态持续零命令、正常导航最大速度约 `0.9998 m/s`，消息契约检查无失败；
  - 从干净安装运行 `empty` 正式闭环：约 6.61 s 无碰撞到达目标，最终目标距离约 `0.183 m`，收到 330 个命令样本，稳定频率约 `50.000 Hz` 和 58 组角域可视化；命令与可视化话题均只有 `depth_angular_controller` 发布；
  - 运行代码和工具中不再存在 `PC_GVF_SHADOW_MODE`、`depth_angular_shadow`、影子 launch、迁移诊断或双实现探针引用；沙箱中的 Fast DDS UDP 权限警告仍由共享内存回退覆盖，不影响测试结果。
- 遗留问题：阶段 5 已完成。下一阶段应在正式 C++ 路径上逐项实现已记录的安全与 CPU 改进；其中完整深度保守处理、未知空间语义、最终连续运动/制动认证等属于行为变更，必须各自增加测试和验收，不能与迁移清理混合实施。

### 2026-09-07 — 阶段 5.1：删除已被 C++ 替代的 Python 控制器路径

- 目的：在阶段 4 完成公共接口和闭环验收后，移除已不再承担运行职责的 Python 控制器、回退入口及其专用测试，同时保留仍用于行为验收的 Python 数值 oracle 和合成场景基础设施。
- 涉及文件：`src/pc_gvf/pc_gvf/depth_angular_controller.py`、`src/pc_gvf/scripts/depth_angular_controller`、`src/pc_gvf/scripts/depth_angular_controller_py`、`src/pc_gvf/test/test_depth_visualization.py`、`src/pc_gvf/test/test_depth_angular_core.py`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`src/pc_gvf/setup.py`、`README.md`、`FOV_GVF_PRINCIPLES_RECORD.md`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 删除 Python ROS 控制器模块、旧公共包装器和阶段 4 使用的显式 Python 回退包装器；CMake 与 setuptools 均不再安装或声明这些入口；
  - demo launch 删除 `controller` 参数、`PythonExpression` 条件和 Python 回退节点，正式 `depth_angular_controller` C++ 节点改为无条件启动；
  - 删除只针对旧 Python 控制器 Marker 发布实现的测试，并从核心测试中移除两个直接构造旧控制器的用例；对应四元数和非法深度编码行为已经由 C++ 单元测试及阶段 4.3 ROS 状态探针覆盖；
  - README 删除临时回退说明，原理记录明确 Python 控制器仅是迁移前行为基准；保留 `depth_angular_core.py`、`depth_angular_demo_sim.py`、黄金数据生成器和场景/验收探针，它们不再构成可选生产控制路线；
  - 暂不删除 `pc_gvf_core`：其独立二维/三维 API 尚未由新的角域 C++ 控制器复现；暂不删除 C++ 影子入口和在线对照工具，统一留到阶段 5.2 核定和清理。
- 验证：
  - 使用全新的 `/tmp/fov_gvf_stage51_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；全量测试汇总 18 项，0 错误、0 失败、0 跳过；
  - 干净安装的 `ros2 pkg executables pc_gvf` 只列出 `depth_angular_controller`、`depth_angular_core`、`depth_angular_demo_sim` 和 `depth_angular_shadow`；安装树中不存在 `depth_angular_controller_py` 或 `depth_angular_controller.py`，排除了旧安装产物掩盖依赖的可能；
  - `ros2 launch ... --show-args` 确认旧 `controller` 选择参数已经消失；仅保留 `rviz`、`shadow` 和 `scenario`；
  - 从干净安装运行 `empty` 正式闭环：6.57 s 内无碰撞到达目标，最终目标距离约 `0.183 m`，收到 328 个命令样本，稳定频率约 `50.000 Hz`，并收到 57 组角域可视化；`/position_cmd` 与 `/pc_gvf/angular_field` 均只有 C++ `depth_angular_controller` 发布；
  - 排除文档、旧构建目录和缓存后全仓搜索，运行代码中不再引用 Python 控制器模块、包装器或 `controller:=python`；沙箱中的 Fast DDS UDP 权限警告仍存在，但共享内存通信完成闭环验收，不影响结果。
- 遗留问题：阶段 5.2 需要删除迁移期 C++ 影子可执行文件和只服务于双实现对照的工具，并把目前同时编译 active/shadow 的源文件整理为正式控制器命名；Python 数值 oracle、合成模拟器以及 `pc_gvf_core` 的去留需按其剩余独立功能分别决定，不能随影子设施机械删除。

### 2026-09-07 — 阶段 4.3：完成 C++ 七场景、频率与安全状态验收

- 目的：在删除 Python 控制器前，对已经取得正式接口的 C++ 路径执行最终运行验收，确认闭环无碰撞结果、控制频率、保护状态、消息契约和两种深度编码均满足冻结基准。
- 涉及文件：`src/pc_gvf/CMakeLists.txt`、`tools/ros_cpp_closed_loop_probe.py`、`tools/ros_cpp_state_probe.py`、`README.md`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 当调用方没有指定构建类型时，将 `pc_gvf` 默认配置为 CMake `Release`；未启用 `fast-math`，16 组 Python 数值黄金数据仍作为语义约束；该设置使完整 C++ 节点在含调和求解和 Marker 生成时满足 20 ms 控制周期；
  - 新增闭环验收探针，订阅正式里程计、命令、状态和角域 Marker，逐段使用场景几何检查带 `0.25 m` 机体半径的碰撞，记录最小机体表面净空、目标距离、稳定停车、发布频率和实际发布者；
  - 新增安全状态探针，以定时输入序列覆盖无里程计、无/零/过期意图、无/过期深度、正常导航、NaN 里程计、零范数四元数和到达目标，并分别使用 `32FC1` 米制及 `16UC1` 毫米制深度；
  - 安全状态探针同时检查保护状态持续零速度、正常导航存在非零速度，以及 `PositionCommand` 的坐标系、READY 标志和 Python 原先保留为零的加速度、jerk、yaw rate、增益和轨迹 ID 字段；
  - README 明确默认运行入口已经是 C++，并记录 `controller:=python` 临时回退方式。
- 验证：
  - 使用全新的 `/tmp/fov_gvf_stage43_clean_{build,install,log}` 前缀以默认 Release 配置从零构建四个 ROS 2 包成功；全量测试汇总 22 项，0 错误、0 失败、0 跳过，五个 C++ 黄金数据检查在 Release 下继续通过；
  - 七场景正式 ROS 闭环全部通过且 `/position_cmd`、`/pc_gvf/angular_field` 均只有 C++ 控制器发布：`empty`、`single_pillar`、`offset_box`、`center_sphere`、`diagonal_gap` 无碰撞到达目标，最终目标距离分别约为 `0.183/0.183/0.167/0.169/0.179 m`；
  - `overhead_bar` 与 `narrow_gate` 未发生机体碰撞并稳定降为零速度，分别在约 `(2.533, 0.052, 1.2)` 与 `(2.574, -2.319, 1.2)` 停车；七场景测得命令频率范围为 `50.001–50.007 Hz`；
  - 七场景最小机体表面净空均非负；障碍场景约为 `0.226/0.195/0.246/0.217/0.030/0.193 m`（依次对应立柱、偏置箱、球体、横杆、斜向间隙、窄门），验收探针未检测到膨胀几何相交；
  - 状态注入依次观测到 `WAITING_ODOMETRY → STALE_INTENT → ZERO_INTENT → WAITING_DEPTH → NAVIGATING → STALE_DEPTH → NAVIGATING → STALE_INTENT → INVALID_ODOMETRY → INVALID_GUIDANCE → GOAL_REACHED`；共收到 290 个命令样本，各保护状态均至少连续收到 3 个零速度命令，正常导航最大命令速度约 `0.9998 m/s`，消息字段检查全部通过；
  - 沙箱内 Fast DDS 仍报告 UDP socket 权限警告，但共享内存完成全部闭环和状态测试，不影响结果。
- 遗留问题：阶段 4 已通过，下一步可删除被替代的 Python 算法/控制器及临时回退入口；Python 合成场景、黄金数据生成器和测试仍是当前验收基础，暂不应一并删除；独立 `pc_gvf_core` 仍提供新 C++ 角域控制器尚未复现的二维/三维 API，不能仅因默认链路已切换就删除；`diagonal_gap` 的最小机体表面净空仅约 `0.030 m`，虽未碰撞且与迁移行为一致，但保守安全余量改进仍属于阶段 6。

### 2026-09-07 — 阶段 4.2：将公共控制器入口切换为 C++ 并保留 Python 回退

- 目的：在不删除 Python 行为基准的前提下，让已完成在线等价验收的 C++ 节点取得正式控制器入口和 ROS 公共输出，为下一阶段七场景闭环验收建立真实运行路径。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/scripts/depth_angular_controller_py`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 将同一 C++ ROS 节点源码通过编译定义生成 `depth_angular_controller` 和 `depth_angular_shadow` 两个 ELF 可执行文件；正式构建保持节点名 `depth_angular_controller`，声明既有 `cmd_topic` 参数并发布 `/position_cmd`、私有 `~/status`/`~/diagnostics` 和 `/pc_gvf/angular_field`，影子构建继续固定使用私有隔离输出；
  - 停止把原 Python 包装器安装为公共入口，新增 `depth_angular_controller_py` 显式回退入口；Python 模块和算法本体暂时保留，尚未删除；
  - demo launch 新增取值限定为 `cpp`/`python` 的 `controller` 参数，默认 `cpp`；`controller:=python` 会以原节点名启动回退入口，因此既有私有状态话题和外部 launch 调用仍兼容；
  - 扩展在线探针的 active-C++ 模式：将 Python 回退节点重命名并隔离其命令/可视化，把 C++ 接到正式话题，同时继续逐帧调用 Python oracle 校验诊断并检查命令、状态、Marker 和发布者归属；
  - 将控制定时器由墙钟改为节点 ROS 时钟，保持 Python `create_timer()` 在 `use_sim_time` 下的语义；将无效里程计位置的 `NaN/+Inf/-Inf` 替换值对齐 NumPy `nan_to_num` 的 `0/最大有限值/最小有限值`。
- 验证：
  - 增量安装后 `ros2 pkg executables pc_gvf` 同时列出 C++ 公共入口、Python 回退入口和 C++ 影子入口；`file` 确认 `depth_angular_controller` 为 x86-64 ELF，launch 参数检查确认 `controller` 默认 `cpp` 且只接受 `cpp`/`python`；
  - 使用全新的 `/tmp/fov_gvf_stage42_final_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；全量测试汇总 22 项，0 错误、0 失败、0 跳过；
  - 基于全新安装前缀运行正式接口在线对照，收到 134 组连续 C++ 诊断且全部匹配 Python oracle；双方状态均为 `NAVIGATING`，稳定命令误差和 7 类 Marker 最大几何误差均为 `0.0`；
  - ROS 图确认正式 `/position_cmd` 和 `/pc_gvf/angular_field` 均只有 C++ `depth_angular_controller` 发布，Python oracle 只发布到隔离话题；
  - 分别短时运行默认 `controller:=cpp` 和回退 `controller:=python` 的 `empty` 场景 launch，两条路径均进入 `NAVIGATING`；C++ 日志明确显示命令发布在 `/position_cmd`，Python 回退进程按预期由 `depth_angular_controller_py` 启动。
- 遗留问题：尚未完成 C++ 正式路径的七场景闭环结果、约 50 Hz 命令频率、深度/意图超时、无效里程计、零意图、到达目标和安全停止验收；Python 回退和原 Python 源码在这些检查全部通过前继续保留；短时 `timeout` 终止 Python 演示/回退进程时仍会出现既有 `KeyboardInterrupt` 退出栈，不属于运行期导航失败。

### 2026-09-07 — 阶段 4.1：移植并在线验收 C++ 角域可视化

- 目的：在切换正式控制权之前补齐 C++ 控制器的最后一个外部功能面——角域 `MarkerArray` 可视化，并继续以 Python 输出作为在线行为基准。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/package.xml`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - C++ 影子节点新增 `field_radius` 参数和瞬态本地 QoS 的私有 `~/angular_field` 发布者，保持正式 `/pc_gvf/angular_field` 仍由 Python 控制器拥有；
  - 移植 Python 可视化中的势场梯度填充、三像素采样、制动速度着色、时间脉冲透明度、箭杆/箭头、相机视锥、参考/目标/命令射线、禁行像素和深度障碍点投影，输出同样的 7 类 Marker；
  - 扩展 ROS 对照探针，将 Python `/pc_gvf/angular_field` 重映射到隔离测试话题，并订阅 C++ 私有可视化；检查命名空间集合、ID、类型、动作、坐标系、点/颜色数量、全部三维点和 RGB，同时检查两个可视化话题的发布者归属；
  - 首次在线对比发现 C++ `project` lambda 以 `auto` 返回 Eigen 延迟表达式，引用了已销毁的临时射线，表现为可视化 Y/Z 坐标接近零；改为显式返回 `Eigen::Vector3d` 后强制求值，几何恢复并与 Python 完全一致。
- 验证：
  - 增量编译、链接和安装成功；第一次对照准确捕获 Eigen 生命周期错误，修复后的增量在线对照收到 128 组诊断，命令误差和 Marker 最大几何误差均为 `0.0`；
  - 使用全新的 `/tmp/fov_gvf_stage41_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；全量测试汇总 22 项，0 错误、0 失败、0 跳过；
  - 基于全新安装前缀再次在线对照，收到 132 组连续诊断，Python 与 C++ 状态均为 `NAVIGATING`，稳定命令误差 `0.0`，7 类 Marker 最大几何误差 `0.0`；命令和可视化的四个隔离话题均只有预期节点发布，正式 `/position_cmd` 无 C++ 发布者；
  - 沙箱内 Fast DDS 仍报告 UDP socket 权限警告，但共享内存通信完成全部在线检查，不影响验收结果。
- 遗留问题：本阶段只补齐并验收可视化，正式 `depth_angular_controller` 入口、`/position_cmd`、`~/status` 和 `/pc_gvf/angular_field` 尚未交给 C++；下一步应建立可回退的入口切换，随后完成状态保护、频率和七场景闭环验收，Python 控制器此时仍不得删除。

### 2026-09-07 — 阶段 3：接入非执行 C++ ROS 影子控制器

- 目的：在 Python 继续独占正式控制输出的前提下，把已验收的 C++ 导引核心接入真实 ROS 2 消息和控制状态机，并用同一组不可变传感器输入完成在线 Python—C++ 对照。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/package.xml`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ `depth_angular_shadow` ROS 2 节点，复现 Python 控制器的参数约束、里程计/相机/深度/意图订阅、`32FC1` 与 `16UC1` 深度解码、最近邻角域缩放、相机内参缩放、数据时效检查、参考重锚、四元数变换、历史像素更新、垂向限幅、加速度限幅、状态字符串和 `PositionCommand` 字段；
  - 影子节点不声明正式 `cmd_topic` 参数，命令、状态和 JSON 诊断固定发布到私有 `/depth_angular_shadow/command`、`/status`、`/diagnostics`；诊断包含序号、深度版本、场有效性、四个关键像素、原始/限幅速度、自由距离摘要、禁行像素数、有限势值数量/总和和求解耗时；
  - demo launch 新增默认关闭的 `shadow:=false` 参数；设为 `true` 时同时启动影子节点并共享控制器输入参数，Python 节点仍是 `/position_cmd` 的唯一控制方；
  - 新增 ROS 对照探针，连续发布固定里程计、相机内参、中央近障深度和前进意图，逐帧以 Python `compute_guidance()` 重算 C++ 诊断，检查诊断序号、场状态、像素、原始命令和数值摘要，并比较两节点的最终状态、稳定速度及实际发布者归属。
- 验证：
  - C++ 影子节点在阶段 2.5 临时构建目录增量编译、链接和安装成功；launch Python 语法检查通过，`ros2 launch ... --show-args` 确认 `shadow` 默认值为 `false`；
  - 使用全新的 `/tmp/fov_gvf_stage3_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；`colcon test-result` 汇总 22 项测试，0 错误、0 失败、0 跳过，其中 `pc_gvf` 的五个 C++ 数值检查和三个 Python 注册测试均通过；
  - 基于全新安装前缀运行在线对照，收到 141 组连续 C++ 诊断，全部通过 Python oracle 检查；Python 与 C++ 最终状态均为 `NAVIGATING`，稳定速度向量误差为 `0.0`；
  - ROS 图检查确认 `/shadow_test/python_command` 仅由 `depth_angular_controller` 发布，`/depth_angular_shadow/command` 仅由 `depth_angular_shadow` 发布，正式 `/position_cmd` 在隔离测试中无发布者，因此影子节点没有取得执行权；沙箱禁止 UDP socket 时 Fast DDS 打印传输警告，但共享内存通信完成全部对照，未影响结果。
- 遗留问题：C++ 尚未接管原 `depth_angular_controller` 可执行入口、`/position_cmd`、角域 Marker 可视化和正式闭环；下一阶段需先补齐可视化与公共接口，再执行七场景、发布频率、数据超时和安全停止验收，验收前不得删除 Python 控制器或旧 C++ 独立功能。

### 2026-09-07 — 阶段 2.5：组合并验收完整 C++ 导引核心

- 目的：将阶段 2.1 至 2.4 已独立验收的 C++ 子模块组合为单一导引调用，从原始深度和机器人状态生成完整诊断结果及最终世界系速度，完成 ROS 无关核心的 Python—C++ 等价迁移闭环。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/guidance_composition_check.cpp`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ `AngularSolution` 和 `GuidanceResult`，保存深度、方向自由距离、规划掩码、调和势、参考/源/目标/命令像素、场有效性、返回目标和世界系速度命令；墙钟耗时继续不进入确定性结果；
  - 新增 C++ `computeGuidance()`，接收不可变深度图、相机、配置、位置、速度、目标、历史像素、相机姿态及可选参考线，串联深度反投影、碰撞锥、参考方向投影、安全目标、调和场、方向选择、参考修正、制动和滚动限速；
  - 保持 Python 的相机后方目标边界回退、FOV 一像素禁行边界、无安全目标回退、源目标角距小于 `0.4°` 的短路分支、无效调和场回退、参考修正重投影和最终速度生成顺序；
  - 新增 `guidance_composition_check`，直接从 16 组黄金输入调用完整 C++ 核心，对深度及 NaN 布局、逐像素自由距离/掩码/势场、四个诊断像素、场有效性、返回目标和最终三维速度执行端到端比较。
- 验证：
  - 阶段 2.4 构建目录增量编译成功，`guidance_composition_check` 通过全部 16 组完整 Python 基准，全套 CTest 8/8 通过；
  - 使用全新的 `/tmp/fov_gvf_stage25_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 8/8 通过：五个 C++ 检查和三个 Python 测试注册项全部通过；
  - 完整比较遵循迁移计划容差：掩码和有效性精确一致，自由距离为 `atol=1e-9`、`rtol=1e-9`，势场及命令为 `atol=1e-6`、`rtol=1e-6`，NaN 布局精确一致；
  - 本阶段未改变黄金数据，组合 SHA-256 保持 `ffe6a2d034b0311b00045e7aff2d7d244769da307b30968a806333259bf3ed95`，Python 算法源码 SHA-256 保持 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`。
- 遗留问题：正式 ROS 控制器仍为 Python，C++ 核心尚未订阅真实 ROS 消息或发布影子诊断；下一阶段需要建立非执行的 C++ ROS 影子节点，在相同传感器快照上比较两条链路，并确保仍只有 Python 控制器拥有命令输出权；既有安全语义缺口和 CPU 优化仍留到切换后阶段处理。

### 2026-09-07 — 阶段 2.4：移植路径、参考修正、制动与滚动限速

- 目的：完成 `compute_guidance()` 组合前的最后一组 ROS 无关子模块移植，将调和势转换为受限方向，并复现参考线收敛、制动和前向滚动速度认证行为。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/motion_safety_check.cpp`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ 双线性图像采样，保持越界或四点中存在非有限值时返回调用方默认值的语义；
  - 新增离散调和路径，保持源点裁剪/偶数舍入、8 邻域扫描顺序、已访问惩罚、目标距离微小破同值项和最多 500 步规则；
  - 新增相机射线角距离、像素空间角速率限制和最终命令方向选择，保持“先回到安全源点，否则沿调和路径前看两步”的分支；
  - 新增状态锚定参考线方向修正，保持横向误差、净空平滑门控、最大修正角和三维单位化语义；
  - 新增延迟加制动距离速度上限、加速度/一阶速度响应滚动、点云扫掠球碰撞、相机 FOV 检查和固定 10 次二分速度限制；
  - 黄金数据新增路径、历史点到安全源点角距、限角目标、直接选择命令、参考修正前命令、观测净空、参考修正方向、命令净空、制动速度、全速滚动判定和最终限速结果；
  - 新增 `motion_safety_check`，逐场景核对这些中间量，并补充双线性默认值、零/大净空制动、微小速度、参考门控、空点云和初始扫掠碰撞分支测试。
- 验证：
  - 生成器与 Python 基准回归通过 `py_compile`，Python 黄金数据回归 1 项通过；
  - 16 组黄金数据连续两次强制生成的组合 SHA-256 均为 `ffe6a2d034b0311b00045e7aff2d7d244769da307b30968a806333259bf3ed95`，算法源码 SHA-256 仍为 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`；
  - 在阶段 2.3 构建目录增量编译成功，新增 `motion_safety_check` 通过 16 组 Python 基准，全套 CTest 7/7 通过；
  - 使用全新的 `/tmp/fov_gvf_stage24_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 7/7 通过：四个 C++ 检查和三个 Python 测试注册项全部通过；
  - 二分限速的三个代表性分支均与 Python 一致：`center_sphere_near` 为 `0.49108990452468421 m/s`，`reference_line_offset` 为 `1.170703125 m/s`，`single_kept_near_pixel` 为 `0.36780645666149114 m/s`。
- 遗留问题：阶段 2.5 尚未把全部 C++ 子模块组合成单一 `computeGuidance()` 并直接比较完整诊断结果；正式运行入口仍为 Python；当前为等价迁移而保留“空障碍点立即判定滚动安全”和固定 10 次二分等既有语义，动态容器、逐点滚动和路径集合也尚未做部署 CPU 优化。

### 2026-09-07 — 阶段 2.3：移植安全目标、连通域与角域调和场

- 目的：继续以 Python 为行为基准移植 ROS 无关核心，实现安全源点/目标选择、连通域标记和角域调和势求解，同时保持正式 Python 控制器和外部 ROS 接口不变。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/angular_field_check.cpp`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 在 C++ 中建立与 Python `SimConfig` 对齐的配置结构，为本阶段和后续核心组合提供一致默认值；
  - 新增最近自由像素选择和 8 邻域连通域标记，保持 NumPy 行优先首次最小值、历史目标代价和确定性图像左偏置语义；
  - 新增安全目标选择，保持参考像素裁剪/偶数舍入、同连通域限制、欧氏净空奖励和目标滞回语义；净空距离当前使用精确直接搜索以优先锁定 SciPy 行为，尚未做 CPU 专项优化；
  - 新增源/目标圆盘掩码和 C++ 共轭梯度调和求解，保持四邻域 Laplace 离散、障碍物及 FOV 边界零法向通量、源值 1、目标值 0、重叠/空未知域分支、迭代上限及有效性语义；
  - 黄金数据新增原始可空安全源点/目标、连通域数量/逐像素标签，以及独立调用调和求解器得到的有效性和势场；这避免最终 `compute_guidance()` 的回退分支掩盖子模块错误；
  - 新增 `angular_field_check`，除 16 组黄金数据外，单独覆盖对角 8 邻域连通、左右同距时的固定左偏、源目标重叠、全 Dirichlet、源目标被遮挡和半径一圆盘等分支。
- 验证：
  - 生成器与 Python 黄金数据回归通过 `py_compile`，单独 Python 基准回归 1 项通过；
  - 16 组黄金数据连续两次强制生成的组合 SHA-256 均为 `dfff964b2efcda4706fdde9fde6831674c61f77f28bb9c473e76e6dab03f5338`，算法源码 SHA-256 仍为 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`；组合哈希变化来自新增选择、标签和独立势场字段；
  - 在阶段 2.2 构建目录增量编译成功，CTest 6/6 通过；新增测试对连通域标签和选定像素执行精确比较，对势场使用迁移计划规定的 `atol=1e-6`、`rtol=1e-6` 和精确 NaN 布局比较；
  - 使用全新的 `/tmp/fov_gvf_stage23_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 6/6 通过：三个 C++ 检查和三个 Python 测试注册项全部通过。
- 遗留问题：阶段 2.4 的离散路径、角速度限制、参考线收敛、制动和前向滚动尚未移植；正式运行入口仍为 Python；本阶段的精确直接欧氏距离计算适合当前 `48 × 36` 行为对齐，但需要在完成迁移后结合部署 CPU 指标决定是否替换为线性时间距离变换并重新验证。

### 2026-09-07 — 阶段 2.2：移植深度反投影与碰撞锥自由距离

- 目的：继续按 Python 行为基准分片移植 ROS 无关核心，在不切换控制器入口、不改变正常配置下算法行为的前提下，实现并独立验收 C++ 深度几何层。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/depth_geometry_check.cpp`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ `backprojectObstaclePoints()`，保持 Python 的有限值、正深度、最大量程减 `1e-6`、二维采样步长和行优先点序语义；
  - 新增 C++ `collisionConeFreeDistance()`，保持相机射线缓存、传感器最大可知距离、球形膨胀接触距离、严格碰撞锥判定和分块计算语义，并对不匹配的图像尺寸及无效调用参数给出异常；
  - 将 Python 反投影障碍点数量和坐标加入 16 组语言无关黄金数据，使 C++ 测试可分别定位反投影误差与自由距离误差；Python 回归同步验证新增中间量；
  - 新增 `depth_geometry_check`，逐场景检查障碍点数量/坐标和每个像素的自由距离，覆盖全零、全 NaN、步长保留/漏采单像素、不同分块大小及错误输入；
  - 更新迁移计划，将下一检查点推进到阶段 2.3；Python 控制器、三个公开可执行入口和 ROS 接口均未切换。
- 验证：
  - 生成器与 Python 回归测试通过 `py_compile`，单独黄金数据回归 1 项通过；
  - 16 组黄金数据连续两次强制生成的组合 SHA-256 均为 `5e85e861d15d63dcf251d7d8497d03e35e6db0d7b44f5a88cc0ae2e5cc3fed6a`，算法源码 SHA-256 仍为 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`；组合哈希变化仅来自新增确定性障碍点字段；
  - 在阶段 2.1 临时目录完成增量编译，CTest 5 项全部通过；
  - 使用全新的 `/tmp/fov_gvf_stage22_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 5/5 通过：两个 C++ 检查和三个 Python 测试注册项全部通过。
- 遗留问题：阶段 2.3 及之后的目标选择、连通域、调和场、路径/参考/制动和整体组合尚未移植；正式运行入口仍为 Python；本阶段只验证算法数值和全新构建，未重复 ROS 闭环演示，因为运行实现没有切换。

### 2026-09-07 — 阶段 2.1：建立 C++ 数学、相机与混合构建基础

- 目的：在不切换运行入口、不改变避障算法行为的前提下，在 `pc_gvf` 同一 ROS 包内建立首个可独立验收的 C++ 基础层，并让 C++ 测试能够读取阶段 1 的语言无关黄金数据。
- 涉及文件：`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/package.xml`、`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/test/cpp/fixture_reader.hpp`、`src/pc_gvf/test/cpp/camera_geometry_check.cpp`、`src/pc_gvf/scripts/depth_angular_controller`、`src/pc_gvf/scripts/depth_angular_demo_sim`、`src/pc_gvf/scripts/depth_angular_core`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 将 `pc_gvf` 从纯 `ament_python` 构建改为 `ament_cmake` + `ament_cmake_python` 混合构建，在迁移期间继续安装原 Python 模块、launch 和三个原名可执行入口；
  - 新增独立 C++ 静态库 `pc_gvf_depth_angular_core`，实现与 Python 对齐的向量归一化/限幅、固定相机安装旋转、四元数旋转矩阵、针孔相机内参、像素—射线变换和射线缓存；
  - 新增只用于测试的标准库 `key=value` 黄金数据读取器，能够读取 16 组基准及 `nan/inf` 数组；
  - 新增 `camera_geometry_check`，检查默认和替代相机参数的 Python 数值基准、坐标轴约定、四元数有效性、全部黄金数据形状、每个像素的缓存射线和像素—射线往返；
  - 新增显式 Python 入口包装器。最初直接安装模块文件时，控制器和模拟器因没有模块级 `__main__` 调用而启动后立即正常退出；包装器恢复 setuptools console-script 原有的 `main()` 调用语义。
- 验证：
  - 使用独立的 `/tmp/fov_gvf_stage21_{build,install,log}` 前缀完成 `pc_gvf_msgs`、`pc_gvf_platforms` 和混合 `pc_gvf` 构建；`pc_gvf` 的 `colcon_build.rc` 为 0，C++ 库、头文件、Python 模块、launch 和三个可执行入口均安装成功；
  - CTest 共 4 项全部通过：`camera_geometry_check`、原 Python 核心测试、可视化测试和 16 组黄金行为回归；其中 Python 测试函数合计 6 项通过；
  - `ros2 pkg executables pc_gvf` 仍列出 `depth_angular_controller`、`depth_angular_core` 和 `depth_angular_demo_sim`，`depth_angular_core --help` 正常；
  - 沙箱内首次短时启动因 DDS 无权创建 UDP socket 而无法验证，获批在沙箱外以 `ROS_LOCALHOST_ONLY=1` 重跑；修复入口包装器后，无界面 `single_pillar` 演示依次进入 `WAITING_ODOMETRY`、`WAITING_DEPTH`、`NAVIGATING` 和 `GOAL_REACHED`；8 秒测试结束时 `timeout` 的重复 SIGINT 在三个 Python 进程销毁阶段产生 `KeyboardInterrupt` traceback，导航目标已在中断前到达；
  - 验证过程中另有三次命令环境错误并已纠正：`--log-base` 最初放在 `build` 子命令后、全新前缀只选择目标包而未先构建工作区依赖、一次直接 `ctest` 在加载 ROS 环境前执行；这些失败均未归因于源码测试。
- 遗留问题：阶段 2.2 及之后的深度几何、调和场和运动预测尚未移植；当前正式控制器仍是 Python；混合包暂时保留不再由 ament 构建使用的 `setup.py/setup.cfg`，待 C++ 切换和清理阶段处理；短时启动测试的超时退出不作为优雅关停验收。

### 2026-09-07 — 阶段 1：冻结 Python 行为基准

- 目的：按照“先以 Python 为行为基准、再并行实现 C++、验收后切换和删除旧实现”的迁移路线，建立第一阶段可由 Python 和后续 C++ 共同读取的确定性行为基准。
- 涉及文件：`CPP_MIGRATION_PLAN.md`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/MANIFEST.txt`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`WORK_LOG.md`。
- 修改内容：
  - 新建分阶段迁移计划，定义迁移不变量、运行范围、六个阶段检查点、ROS 公共接口和数值一致性容差；
  - 新增 Python 行为基准生成器，以无需 JSON/NumPy 读取器的 UTF-8 `key=value` 格式保存输入、中间量和最终输出，便于后续 C++ 标准库直接解析；
  - 生成 16 组黄金数据，覆盖七个内置场景，以及全零、全 NaN、保留/漏采单像素、历史方向与侧向速度、相机偏航、替代内参、参考直线偏移和相机后方目标；
  - 冻结深度、方向自由距离、禁行掩码、调和势、源/目标/命令像素、场有效性和最终核心速度；墙钟耗时因不可重复而未写入；
  - 新增自动回归测试，确认当前 Python 核心仍与全部黄金数据一致；清单记录算法源码 SHA-256 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`。
- 验证：
  - `/usr/bin/python3 -m py_compile` 检查生成器和回归测试通过；
  - 黄金数据生成成功，共 16 组、约 876 KiB；连续两次强制生成的组合 SHA-256 均为 `b9275a9ebdbc7f416a2864b1232d3796c31ceb619c2af580fa210002841e2791`；
  - 新增黄金数据回归测试：1 项通过；
  - `src/pc_gvf/test` 全部测试：6 项通过；首次全包测试因测试命令覆盖 ROS 注入的 `PYTHONPATH` 而在收集阶段缺少 `rclpy`，改为在原 ROS 路径前追加源码目录后通过，属于验证环境命令问题而非代码失败。
- 遗留问题：当前基准有意保留 Python 已知安全语义缺口，只用于迁移等价性；尚未实现 C++ 解析器和任何 C++ 算法模块；旧 C++ 独立功能是否保留要在删除阶段前明确决定。

### 2026-09-06 — 记录 FOV-GVF 原理、论文方法与 CPU 路线核对结果

- 目的：完整阅读用户提供的原理分析 Markdown 和四张论文方法截图，并把附件主张、项目实际实现、二者对应关系及后续判断边界形成可追溯记录。
- 涉及文件：`FOV_GVF_PRINCIPLES_RECORD.md`、`WORK_LOG.md`。
- 修改内容：
  - 新建原理核对记录，整理截图公式（1）至（22）中的粗—细调和场、状态锚定局部参考、横向误差反馈、场刷新和扰动最终界；
  - 区分默认 ROS 2 Python 角度域运行链路、C++ 二维 ESDF 流函数链路和 C++ 三维深度栅格势流链路；
  - 对照源码记录附件 CPU 方案中完整深度、未知语义、邻接一致性、最终输出认证、编译内核和整链路计时的已实现与未实现状态；
  - 记录附件文件哈希和核对范围，并明确附件外部实验材料不在本仓库，相关性能结论尚未在本机复现。
- 验证：完整读取 240 行附件 Markdown 和四张原始分辨率截图；静态核对项目 README、迁移说明、启动配置、Python 核心/控制器/测试、C++ 深度投影/求解/指导策略及回归测试；执行 Markdown 差异与空白检查。未运行算法性能或安全测试。
- 遗留问题：论文截图缺少题名、作者和完整上下文，暂不能确认论文身份；附件引用的外部实验脚本和结果不在当前仓库；原理记录列出的算法语义缺口均尚未修复。

### 2026-09-06 — 建立独立修改分支和工作日志

- 目的：保留上游 `dev` 基线，为后续定制修改建立可追溯记录。
- 涉及文件：`AGENTS.md`、`WORK_LOG.md`。
- 修改内容：
  - 从提交 `804f26e` 创建并切换到 `feature/local-modifications`；
  - 新建本工作日志并整理项目已有功能、实测结果、已知限制和记录规范；
  - 在 `AGENTS.md` 中增加“每次修改必须同步更新工作日志”的项目规则。
- 验证：确认当前分支为 `feature/local-modifications`；检查 Markdown 章节、表格和代码块结构。
- 遗留问题：`build_humble/`、`install_humble/` 和 `log_humble/` 为未跟踪构建产物，尚未调整忽略规则。

### 2026-09-10 — RViz 加入雷达点云与 EGO 膨胀占据地图

- 目的：在 USER-EGO 自动导航运行时同时观察传感器当前看到的三维障碍点，以及 EGO Cloud 场景真值占据栅格经机体安全半径膨胀后的局部地图，并保证新增可视化不改变深度直达控制路线。
- 涉及文件：`src/pc_gvf_platforms/pc_gvf_platforms/ego_map_visualizer.py`、`src/pc_gvf_platforms/setup.py`、`src/pc_gvf_platforms/package.xml`、`src/pc_gvf_platforms/test/test_ego_map_visualizer.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`scripts/run_isaac_fov_gvf_navigation.sh`、`USER_EGO_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增只读 `ego_map_visualizer` 节点，将 `32FC1/16UC1` 深度图以相机内参反投影为 X 前、Y 左、Z 上的 `radar` 坐标点云，并依据里程计持续发布 `world -> radar` 变换；点云发布到 `/pc_gvf/radar_points`，使用 Best Effort/Volatile QoS。
  - 读取与 Isaac ESDF 保护共用的 `400×300×50` EGO 场景 `occupancy.bin`，按导航空间 `0.4 m` 分辨率和机体半径加安全余量 `0.45 m` 做球形体素膨胀；发布以无人机为中心、水平半径 `25 m`、垂直半径 `10 m`、最多 60000 点的 `/pc_gvf/ego_inflated_occupancy`，使用 Reliable/Transient Local QoS。
  - RViz 默认新增青色实时雷达点与半透明红色膨胀体素两层；启动脚本统一导出占据文件路径，launch 仅在 RViz 启用时启动该节点。两个输出不订阅也不发布控制话题，不进入 C++ 控制器或 Isaac 碰撞判断。
- 验证：Python、launch 和 Shell 静态检查通过；新增反投影轴向、球形膨胀/裁剪、四元数与 PointCloud2 布局测试通过，`pc_gvf_platforms` 全包 9 项测试全部通过；ROS 2 三包重新构建成功。第一次短时联调因沙箱禁止 DDS/本地套接字而无法探测话题，随后在允许 ROS 通信的同等环境复验：控制器进入 `NAVIGATING`，两路 PointCloud2 均只有 1 个预期发布者且各有 RViz 订阅者，QoS 完全匹配；采样得到实时雷达云 4685 点、首帧局部膨胀地图 33649 点，RViz、控制器与可视化节点均正常清理退出。
- 遗留问题：当前红色层明确是 EGO Cloud 场景真值栅格的可视化，不是控制器在线重建地图；若以后把真实 EGO-Swarm `sdf_map` 接入本项目，应改订阅其 `/sdf_map/occupancy_inflate` 并关闭本节点的真值地图发布，避免把真值与估计地图混淆。

### 2026-09-10 — 以深度在线历史地图替换 RViz 场景真值地图

- 目的：彻底移除 `occupancy.bin -> /pc_gvf/ego_inflated_occupancy` 的 ROS/RViz 真值可视化链路，改为把无人机实际看见的逐帧深度点转换到 `world` 并持久累计，同时保持青色实时点云、C++ 深度直达导航和 Isaac 内部 ESDF 安全保护不变。
- 涉及文件：删除 `src/pc_gvf_platforms/pc_gvf_platforms/ego_map_visualizer.py`、`src/pc_gvf_platforms/test/test_ego_map_visualizer.py`；新增 `src/pc_gvf_platforms/pc_gvf_platforms/observed_map_visualizer.py`、`src/pc_gvf_platforms/test/test_observed_map_visualizer.py`、`tools/ros_observed_map_probe.py`；修改 `src/pc_gvf_platforms/setup.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`USER_EGO_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：
  - `observed_map_visualizer` 保留 `/pc_gvf/radar_points`；按深度时间戳从 120 个最近里程计样本中选取最近位姿，以 `/sim/odom` 给出的 `T_world_base` 和 USD 已验证的相机外参平移 `(0.22,0,0.02)` 组合 `T_world_radar`，执行 `p_world=R_world_base*p_radar+t_world_base+R_world_base*t_base_radar`。同一组合结果继续广播直接 `world -> radar` TF，最大位姿时间差默认 `0.20 s`。
  - 深度有效范围改为 `0.30–15.0 m`，按 4 像素步长反投影。世界点以 `0.15 m` 体素哈希去重，每张深度帧内每个体素最多计一次，跨帧命中至少 2 次才进入永久 `occupied_voxels`；确认体素不随视野移动或后续未观测而删除。
  - `/pc_gvf/observed_map` 使用 `world` frame、Reliable/Transient Local QoS 和 2 Hz ROS 仿真时间定时发布。内部全局地图不裁剪；发布端以 `0.30 m` 做不影响内部数据的全局二次体素化，必要时继续自适应增大显示分辨率以守住 120000 点上限，避免按动态步长抽样造成地图闪烁。
  - RViz 删除红色 `EGO inflated occupancy`，新增紫色 `Observed obstacle map`：RGB `140;64;217`、Alpha `0.45`、`Points`、2 像素；青色实时雷达云、无人机、轨迹、原始/平滑速度箭头均保留。
  - Launch 不再向可视化节点传递占据文件、真值原点、真值缩放、局部裁剪或 `0.45 m` 膨胀参数；旧可执行入口和增量安装前缀中的三个可再生成残留文件已删除。`scripts/run_isaac_fov_gvf_navigation.sh` 中的 `FOV_GVF_ESDF_OCCUPANCY` 仍保留，且只传给 Isaac 脚本用于独立碰撞保护。
- 验证：
  - 最终在线地图纯函数测试 7 项通过，覆盖轴向反投影、外参/位姿组合、单帧去重、两帧确认、静止重复 100 帧不增长、移动后同一世界体素不重复、历史保留、新区域增长、发布上限和二次体素化；Flake8、Python/launch 编译和 Shell 语法检查通过；ROS 2 三包构建成功。
  - 有效运行探针采集 25 张地图消息：无人机移动 `20.71 m`，首张非空地图 `18330` 点、末张 `69453` 点，宽度单调不减，首张所有体素在末张的缺失数为 `0`；`map_frames=[world]`、`radar_frames=[radar]`，ROS 消息时间计算频率 `2.034 Hz`，雷达云均值 `3809.2` 点/帧。另一次坐标对齐采样给出精确 `2.0 Hz`，剔除独立地面 Mesh 后 36413 个观测点在 EGO Cloud 真值 1 体素邻域内的匹配率为 `100%`，未发现整体偏移。第一次集合探针在 Isaac 完成启动前即超时得到 0 样本，修正为等待 `NAVIGATING` 后重跑才得到上述有效结果。
  - ROS 图实测 `/pc_gvf/observed_map` 与 `/pc_gvf/radar_points` 各有且仅有一个 `observed_map_visualizer` 发布者并与 RViz QoS 配对；旧 `/pc_gvf/ego_inflated_occupancy` 返回 `Unknown topic`；`/position_cmd` 仍只有 `depth_angular_controller` 一个发布者。
  - 最终全程 Isaac+ROS+RViz 闭环在 `70.87 s` 到达 `(64.233,12.129,5.087)` 并返回 0，无碰撞退出；内部确认地图在最后一次周期日志为 247755 个 `0.15 m` 体素，最终 RViz 发布 94528 个 `0.30 m` 点，141 个发布样本点数全程单调增长且没有下降，未触发 120000 点硬上限。
  - `pc_gvf_platforms` 最终全包 13 项测试全部通过（含在线地图新增 7 项）；`pc_gvf` 五项 C++ 核心测试全部通过。`pc_gvf` 两项旧 Python 测试仍在收集阶段因系统 NumPy 2.2.6 与旧 SciPy 二进制 ABI 不兼容失败，和此前记录一致，不是本次变更导致。
- 遗留问题：当前候选体素只在确认后移出计数表，单次噪声候选尚未设置时间淘汰；本次最长运行候选约 4.25 万，未影响闭环。若未来进行数十分钟以上连续探索，应增加只针对未确认候选的 TTL/容量策略，不能删除已确认历史障碍。

### 2026-09-10 — 增加逐次运行的控制链性能指标与持久记录

- 目的：在每次 USER-EGO 运行期间和结束时显示避障计算、控制帧率、相邻控制延迟及控制发布到无人机命令接口的延迟，并把结果按运行 ID 持久记录，作为后续避障算法优化的量化基线。
- 涉及文件：`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`performance/PERFORMANCE_METRICS.md`、`DONG.md`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：C++ 控制器使用 steady clock 统计 `computeGuidance()`、完整回调到发布、DDS publish、控制间隔，并使用 ROS 仿真时间统计深度帧年龄和 50 Hz 命令周期；命令桥统计 `/position_cmd` 时间戳到接收、Twist 转换与 `/sim/cmd_vel` 发布；Isaac 统计实际命令采样帧数、仿真/墙钟 FPS 和命令变化间隔。控制器与桥每 5 秒输出终端摘要，三段在退出时输出最终摘要并追加 Markdown。启动脚本自动创建运行 ID，也允许使用 `FOV_GVF_RUN_ID` 和 `FOV_GVF_PERFORMANCE_LOG` 覆盖。
- 验证：ROS 2 三包构建成功；五项 C++ 核心测试全部通过；Python/launch `py_compile` 与 Bash `bash -n` 通过；加载 Humble 和本项目 install 后，`test_platform_math.py` 5 项通过。第一次 C++ 回归因恢复源码保留旧时间戳而错误复用了此前已回滚实验的 `/tmp` 对象，强制重新编译后 5/5 通过；第一次沙箱内联调因 DDS 套接字权限失败，记录为 0 帧失败样本并保留在性能文档。获准在具备 ROS 通信权限的环境运行两次短时联调，最终修正口径的 `metrics_smoke_corrected_20260910` 在 5.017 s 仿真时间内得到控制命令 251 帧、ROS FPS 50.000 Hz、避障计算 mean/P95/max `1.025/1.406/3.181 ms`、深度到命令 mean/P95/max `16.801/16.667/33.333 ms`、控制器时间戳到桥接接收 mean/P95/max `0.066/0/16.667 ms`；运行按预设 5 s 超时结束，不是完整到达验收。
- 影响与边界：未修改 `depth_angular_core.cpp` 的避障数学、路径选择、安全距离、速度参数或控制输出值；性能统计会增加少量采样和退出写盘开销。`ROS2SubscribeTwist` 不带消息时间戳，因此没有伪造桥接发布到 OmniGraph 取用的逐消息精确延迟，改为分别记录桥接发布耗时和 Isaac 60 Hz 实际采样周期。
- 回滚/备份：修改前完整副本为 `/home/starry/isaac-data/备份/user_ego_before_avoidance_20260910_165519`；恢复时可从该副本取回上述文件并删除新增 `performance/`。
- 遗留问题：尚未运行约 70 秒的完整 Cloud 到达测试；后续正常运行会自动形成完整任务性能记录。

### 2026-09-10 — 提高 RViz 连贯性并降低纯显示链路负载

- 目的：解除 RViz 默认约 30 FPS 的显示限制，并减少高频完整路径、角域 Marker 和大规模历史点云重发造成的渲染阻塞，使第三人称跟随更连贯。
- 涉及文件：`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`src/pc_gvf_platforms/pc_gvf_platforms/navigation_visualizer.py`、`src/pc_gvf_platforms/test/test_robot_visualization.py`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`DONG.md`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：RViz 目标帧率由隐式默认值提高为 60 FPS；无人机/命令 Marker 保持最高 60 Hz，完整路径降为 10 Hz且位移至少 0.10 m 才新增轨迹点，缓存上限从 2000 降为 1000；角域 GVF Marker 降为 15 Hz；雷达/在线地图深度采样步长由 2 增至 4；历史地图显示由 `2 Hz/0.15 m/120000 点` 调整为 `1 Hz/0.25 m/60000 点`；两层 PointCloud2 队列设为 1 并关闭逐点选择，默认关闭与无人机 Mesh 重复的 Odometry 坐标轴。
- 验证：ROS 2 三包构建成功；五项 C++ 避障核心测试全部通过；可视化与在线地图 Python 测试 8 项全部通过；Python/launch 语法通过。使用安装后的 RViz 配置短时启动 8 秒，OpenGL 4.6 和 ThirdPersonFollower/显示配置正常加载，无插件错误；测试按 `timeout` 预期返回 124。安装目录已复核为 `Frame Rate: 60` 及上述降载参数。完整 `ctest` 另有 2 项 Python 基线测试在收集阶段失败，原因为系统 NumPy 2.2.6 与 Ubuntu SciPy（要求 NumPy <1.25）二进制不兼容，并非断言失败；随后再次定向执行本次相关的 5 项 C++ 和 8 项 Python 测试，全部通过。未执行完整 Isaac+RViz 到达测试，因此不宣称实际显示始终达到 60 FPS。
- 影响与边界：只降低 RViz 和辅助可视化的数据量/发布频率；`depth_angular_core.cpp`、控制器 `48×36` 深度规划输入、50 Hz `/position_cmd`、安全距离、速度和 Isaac ESDF 均未改变。历史地图内部仍以 0.08 m 累计，0.25 m只影响发布显示。
- 回滚/备份：修改前完整快照为 `/home/starry/isaac-data/备份/user_ego_before_rviz_fps_20260910_204300`，源/备份文件清单哈希均为 `e76c88f0ce0600ead2d36e1059259317742af3bbe9bc5d4d7557106c60b221e7`。
- 遗留问题：实际帧率仍受 GPU、窗口大小和同时显示项目影响；若用户机器上仍不够流畅，可在 RViz 中临时关闭 `Forward depth` 或 `FOV, depth hits, and angular GVF` 进一步降载。

### 2026-09-10 — 固化用户调整后的 RViz 默认配置

- 目的：保存用户在 RViz 界面中修改后的显示、视角与窗口布局，并使项目脚本和直接启动 RViz2 时都默认使用该配置。
- 涉及文件：`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`/home/starry/.rviz2/default.rviz`、`WORK_LOG.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：确认 RViz 已在退出时把最新界面状态写入临时安装目录，以该版本覆盖项目源码配置和用户级 `default.rviz`；保存白色背景、关闭地面网格、第三人称距离约 14.65、历史点云 20 px 和 1920×2032 窗口布局，同时保留 60 FPS、点云队列 1、禁用点选择等性能优化。项目 launch 继续通过 `-d` 默认加载该项目配置。
- 验证：源码、临时安装目录和用户级默认文件 SHA-256 均为 `a01b082c747da498046e578bcb1085334f3ecc469b00f4dbe8fd4e1fafc037a2`；重新构建 3 个 ROS 2 包成功。未运行 Isaac 仿真。
- 影响与边界：只改变并固化 RViz 显示配置，未修改避障算法、控制链、传感器参数或性能统计。
- 回滚/备份：旧项目 RViz 配置保存于 `/home/starry/isaac-data/备份/isaac_cloud_navigation_before_user_default_20260910_2105.rviz`；完整项目前置备份为 `/home/starry/isaac-data/备份/user_ego_before_rviz_fps_20260910_204300`。
- 遗留问题：无。

### 2026-09-11 — Cloud 场景由 4 倍缩小为 2 倍并调整起终点

- 目的：把 EGO-Swarm Cloud 物理场景从原始地图的 4 倍缩小为 2 倍，并让无人机任务坐标保持相同的相对位置。
- 涉及文件：`scripts/prepare_ego_swarm_cloud_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`、`DONG.md`、`USER_EGO_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`、`HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/ENVIRONMENT_AND_ASSETS.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：用户已先将 `--environment-scale` 从 4.0 改为 2.0，本次保留该修改；起点从 `(-64,-11.2,2.8)` 等比例改为 `(-32,-5.6,1.4)`，终点及 ROS 控制器固定目标从 `(64,11.2,4.8)` 改为 `(32,5.6,2.4)`。重新生成导航 USD，障碍空间约为 `80×60×10 m`；元数据驱动的 ESDF 分辨率由 0.4 m 变为 0.2 m，相机最大深度由 40 m 变为 20 m。无人机不缩放。
- 验证：Isaac Python 复核 USD 元数据为 `environmentScale=2.0`、起点 `[-32,-5.6,1.4]`、终点 `[32,5.6,2.4]`，障碍 Mesh 边界约 `(-40.1,-30.1,-0.1)` 至 `(39.9,29.9,9.9)`；ROS 2 三包构建成功，安装后的 launch 固定目标正确；Shell/launch 语法通过，5 项 C++ 核心测试和 8 项可视化/地图 Python 测试通过。未运行完整 Isaac 闭环飞行。
- 影响与边界：物理环境、任务起终点、ESDF 物理尺度和相机最大量程改变；避障算法、无人机尺寸、速度、安全参数、RViz 配置和在线观测地图参数不变。
- 回滚/备份：修改前完整快照为 `/home/starry/isaac-data/备份/user_ego_before_2x_map_20260911_184713`。
- 遗留问题：需要完整运行 2 倍场景，确认新起点净空和终点可达性。

### 2026-09-11 — 以当前 USER-EGO 重建 4 倍场景键盘自由控制版

- 目的：完整继承当前 USER-EGO 的连续调和避障、在线观测地图、RViz 降载和性能统计，同时把 USER 恢复为无固定起终点的键盘自由驾驶副本，并修复旧键盘方案依赖事件集合、可能漏松键的问题。
- 涉及文件：整体替换 `/home/starry/isaac-data/user`；重点修改 `scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/prepare_ego_swarm_cloud_navigation.sh`、`scripts/build_isaac_ros_workspace.sh`、`scripts/run_isaac_fov_gvf_navigation.sh`、其余绝对路径脚本、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`、`USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md` 及 HISTORY 记录。
- 修改内容：将旧 USER 移入备份后复制当前 USER-EGO，构建目录和锁文件换为 USER 专用名称。环境比例改回 4.0；手动场景不创建 `/World/UAV_Start`、`/World/UAV_Goal`，只存 `spawn=(-64,-11.2,2.8)`。关闭固定前进、固定目标和到点停止；每仿真帧直接轮询物理按键状态并将世界坐标 `W/S/A/D/R/F` 意图以 `TwistStamped` 发布到 `/human_intent`。无运动键时输入为零，最终执行器同帧再次强制零速；ESDF 只阻止危险位移，不退出会话。
- 验证：USD 元数据和 Prim 检查确认 4 倍、无起终点，障碍范围约 `160×120×20 m`；ROS 2 三包构建成功；C++ 核心 5/5、平台/可视化/地图 Python 13/13 通过，静态检查通过。3.017 s 无按键 Isaac+ROS 短测中控制器进入 `ZERO_INTENT`，位置从始至终为 `(-64.000,-11.200,2.800)`。`get_keyboard_value()` 可调用并返回未按下值；自动按键探针因 Isaac 6.0 运行时没有文档所列 `buffer_keyboard_key_event` 方法而失败，故没有把真实按键写成已验证。
- 影响与边界：只改 USER；USER-EGO 仍是 2 倍固定起终点自动版。C++ 避障数学、在线地图原则、RViz 和性能采样保持继承状态。
- 回滚/备份：旧 USER 完整保存于 `/home/starry/isaac-data/备份/user_before_user_ego_replace_20260911_190000`。
- 遗留问题：用户需在 Isaac 主视口人工验收单键、组合键、松键、Space、窗口失焦和碰撞保护后改向继续驾驶。

### 2026-09-12 — 临时切换为无障碍平地控制测试场景

- 目的：排除 Cloud 障碍与避障路径干扰，单独测试 USER 手动版的键盘控制、组合键、停止和升降。
- 涉及文件：新增 `scripts/isaac/create_flat_ground_scene.py`、`scripts/prepare_flat_ground_navigation.sh` 和 `scenes/flat_ground/` 两个 USD；修改 `scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/isaac/run_fov_gvf_navigation.py`、`USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md` 及 HISTORY 状态记录；性能文档自动记录两次探针。
- 修改内容：当前默认场景模式改为 `flat`，场地为 `200×200 m`、`z=0`，出生点 `(0,0,1.5)`。启动脚本支持 `FOV_GVF_SCENE_MODE=flat|cloud`，只自动加载所选场景同目录的占据文件，避免平地误用 Cloud ESDF。运行器允许零空中障碍，同时读取 `static_ground` 顶面并按0.25 m机体半径阻止穿地。原4倍 Cloud场景和 `occupancy.bin` 完整保留。
- 验证：Shell/Python语法检查通过；OpenUSD确认平地环境只有 `/World/Ground`，手动元数据、Robot和相机有效，无起终点。第一次3秒短测错误使用headless，手动键盘接口按设计拒绝启动并得到0帧；随后可见窗口短测成功进入 `ZERO_INTENT`，3.017 s内发布151个命令帧，无按键时位置保持 `(0,0,1.5)`，无碰撞阻断，按预设超时结束。
- 影响与边界：未修改C++避障算法、键位、速度、ROS/RViz或USER-EGO。真实按键事件无法由当前自动探针注入，仍需用户点击Isaac主视口后人工验证。
- 恢复方法：`FOV_GVF_SCENE_MODE=cloud bash scripts/run_isaac_fov_gvf_navigation.sh` 直接运行原4倍Cloud。
- 遗留问题：现场测试 `W/S/A/D/R/F`、组合键、松键、`Space`，以及下降到地面时的保护。

### 2026-09-12 — 摘出不含避障和ROS的 KEYBOARD 独立控制测试

- 目的：把物理键盘读取与无人机直接运动从完整USER控制链中单独摘出，避免ROS、深度避障、ESDF和RViz影响键盘实现测试。
- 涉及文件：新增 `KEYBOARD/run_keyboard_only.sh`、`KEYBOARD/keyboard_only.py`、`KEYBOARD/README.md`；更新 `USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md` 和统一HISTORY记录。
- 修改内容：独立程序每帧使用 `get_keyboard_value()` 读取 `W/S/A/D/R/F`、方向键、Page Up/Down和Space，组合键归一化为默认2 m/s世界坐标速度并直接积分Robot位置；水平移动时机头平滑朝向运动方向，`1/3/V`切换视角。没有ROS导入、节点、话题、FOV-GVF、深度控制、ESDF、RViz或在线地图；只保留 `z>=0.25 m` 防穿地钳制。终端在物理按键集合变化时输出 `[KEY STATE]`。
- 验证：Shell/Python语法通过，Isaac 6.0中所用键枚举全部存在；3秒可见窗口短测成功打开第三人称视图并明确打印全部控制链为off，运行181帧，无按键时位置保持 `(0,0,1.5)`，按预设结束。未发现ROS或避障进程。
- 影响与边界：原USER完整入口及USER-EGO均未改动；物理键盘事件不能由当前自动探针可靠注入，真实单键、组合键、松键和Space仍待用户现场验证。
- 运行：`cd /home/starry/isaac-data/user/Fov-gvf/KEYBOARD && bash run_keyboard_only.sh`。
- 遗留问题：用户现场观察 `[KEY STATE]` 与无人机运动是否一致。

### 2026-09-12 — 修正 KEYBOARD 过曝并增加地面方向参照

- 目的：解决独立键盘测试中无人机和地面大面积发白、难以判断飞行姿态与位移的问题。
- 涉及文件：`KEYBOARD/keyboard_only.py`、`KEYBOARD/README.md`、`WORK_LOG.md`、
  `HISTORY/CHANGELOG.md`。
- 修改内容：视口从会忽略物体颜色层次的 `MinimalRendering` 常量着色模式改为
  `RayTracedLighting`，关闭自动曝光，将太阳光强度从 `800` 降至 `2.5`、环境光设为
  `0.01`。运行时在出生点地面创建无碰撞中心台、红绿坐标十字及四个方向块，颜色约定
  为 `+X` 橙、`-X` 蓝、`+Y` 绿、`-Y` 品红；增加可选 `KEYBOARD_SCREENSHOT` 自动截图
  参数。这些 Prim 固定在世界坐标系，不随无人机移动。
- 验证：Python、Shell 语法和尾随空白检查通过；共进行四轮 4 s 可见窗口截图检查。
  第一轮为 `MinimalRendering + 250`，第二、三轮为 `RayTracedLighting + 250/25`，均仍
  存在明显高光裁切，未作为完成结果；最终太阳光
  `2.5` 的运行完成 241 帧并正常保存 `/tmp/keyboard_observation_final.png`。人工查看确认
  蓝色机身、深色电机、灰色桨叶、橙/绿坐标线和彩色方向块均可区分；全图任一通道
  `>=250` 的像素比例约 `0.066%`，截图 SHA-256 为
  `3ee319f732436feb0b9c883f0c42ec583d350cd9e11b296656b6677a0219adef`。
- 影响与边界：只改变 KEYBOARD 的显示渲染和无碰撞视觉参照；没有修改键位、速度、
  位置积分、最低高度限制、ROS、避障、完整 USER 入口或 USER-EGO。
- 回滚/备份：旧参数在本条记录中保留；恢复时将渲染器改回 `MinimalRendering`、太阳光
  改回 `800`，并删除 `KeyboardReferences` 与截图逻辑。
- 遗留问题：自动测试仍未注入真实物理按键，按键控制手感需用户现场验收。

### 2026-09-12 — KEYBOARD 独立入口改为 LiteRadio Mode 2 手柄控制

- 目的：以 Ubuntu 已识别的 BETAFPV LiteRadio 3 USB joystick 替换独立入口中的键盘
  飞行控制，并采用多旋翼常见 Mode 2 操作语义。
- 涉及文件：`KEYBOARD/keyboard_only.py`、`KEYBOARD/run_joystick_only.sh`、
  `KEYBOARD/run_keyboard_only.sh`、`KEYBOARD/README.md`、`USER_MANUAL_CONTROL_GUIDE.md`、
  `DONG.md`、`WORK_LOG.md`、`HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、
  `HISTORY/CHANGELOG.md`。
- 修改内容：直接使用 Linux `/dev/input/js` 非阻塞接口读取手柄，不依赖 Isaac/GLFW 的
  游戏手柄映射数据库。默认 AETR/HID 通道为轴0 Roll、轴1 Pitch、轴2 Throttle、轴3
  Yaw；Pitch/Roll 产生相对当前机头的前后/左右速度，Throttle 产生垂直速度，Yaw 独立
  积分机头角度，删除原来“移动方向自动带动偏航”的键盘行为。默认水平/垂直速度为
  `2.0/1.5 m/s`，最大偏航角速度 `75 deg/s`，死区 `0.08`。四杆回中保持 `0.5 s` 后
  才解锁，USB断连立即输出零运动；设备、轴、正负号、速度和死区均可由环境变量覆盖。
  新增 `run_joystick_only.sh` 主入口，旧 `run_keyboard_only.sh` 保留为兼容包装。
- 验证：主机只读检测确认 USB `0483:572b` 对应 `/dev/input/js0`，设备名
  `STMicroelectronics BETAFPV Joystick`，7轴、16按钮，内核绝对轴能力为 `0x7f`；Python
  和两个Shell脚本语法检查通过。5 s 可见 Isaac 联调成功直接打开设备并运行301帧；
  当时实测语义通道为 `roll=0.00, pitch=0.00, throttle=+0.64, yaw=0.00`，因油门未回中，
  安全门保持 `enabled=no`，无人机从始至终保持 `(0,0,1.5)`，证明非中位启动不会误动。
  GLFW 同时报“unknown remapping”，但本实现绕过GLFW直接读取js0，因此不影响采样。
- 影响与边界：只替换 `KEYBOARD/` 无ROS独立测试入口的输入与运动语义；完整 USER
  `scripts/run_isaac_fov_gvf_navigation.sh` 仍为键盘+避障，USER-EGO自动版不变。保留
  已验收光照、地面参照和最低高度保护。
- 回滚/备份：未建完整副本；旧键盘实现及参数可由前一条历史记录恢复，兼容Shell入口
  未删除。
- 遗留问题：需要用户把油门置中完成解锁，依次推动四个摇杆现场确认实际轴方向；若
  固件通道顺序不同，按 `KEYBOARD/README.md` 调整 `JOYSTICK_AXIS_*`/`SIGN_*`。

### 2026-09-12 — 独立手柄入口切换为 BEITONG A2P3A 映射

- 目的：用户用北通 A2P3A BFM 接收器替换 LiteRadio 后，使 Mode 2 控制使用新设备的
  实际 Linux joystick 轴布局。
- 涉及文件：`KEYBOARD/keyboard_only.py`、`KEYBOARD/run_joystick_only.sh`、
  `KEYBOARD/README.md`、`USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md`、`WORK_LOG.md`、
  `HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：默认设备由 `/dev/input/js0` 改为稳定的北通 by-id 路径。Linux轴码表为
  `0=ABS_X, 1=ABS_Y, 2=ABS_Z, 3=ABS_RZ, 4=ABS_GAS, 5=ABS_BRAKE,
  6/7=ABS_HAT0X/Y`，据此将默认 Mode 2 映射改为 `Yaw=0, Throttle=1, Roll=2,
  Pitch=3`；Gas/Brake和方向键不参与飞行控制。正负号、速度、死区与安全门不变。
- 验证：USB只读检测确认 `20bc:511c`、设备名
  `BEITONG BEITONG A2P3A BFM DONGLE`、8轴16按钮。第一次按通用Xbox假设配置
  `Roll=3, Pitch=4` 的5 s联调中，静止Pitch被读成 `+1.00`，回中门保持关闭且无人机
  未移动；随后读取 `JSIOCGAXMAP` 发现轴4实际为Gas，纠正为轴2/3右摇杆。最终5 s
  Isaac联调中四通道均为 `0.00`，0.5 s后正常 `enabled=yes`，运行301帧，位置始终
  `(0,0,1.5)`。Python、Shell语法和KEYBOARD改动文件尾随空白检查通过。
- 影响与边界：只改变KEYBOARD独立入口的默认设备和轴编号；完整USER键盘避障、
  USER-EGO、运动速度、光照、地面参照与断连停车逻辑不变。
- 回滚/备份：未建完整备份；前一条LiteRadio记录保留旧设备路径和轴映射，可按记录
  恢复，所有通道仍支持环境变量覆盖。
- 遗留问题：空闲、中位解锁和无漂移已验证；四个摇杆的实际运动方向仍需用户推动
  手柄现场确认，若单轴方向相反只需调整相应 `JOYSTICK_SIGN_*`。

### 2026-09-12 — 新增 MISSANDKEYBOARD 北通手柄 + FOV-GVF 避障入口

- 目的：把 `KEYBOARD/` 保留为已验证的无ROS手柄参考，在独立
  `MISSANDKEYBOARD/` 入口中将同一套北通 A2P3A Mode 2 输入接入 USER 的完整深度
  避障链。
- 涉及文件：新增 `MISSANDKEYBOARD/beitong_joystick.py`、
  `MISSANDKEYBOARD/run_joystick_avoidance.sh`、`MISSANDKEYBOARD/README.md`；修改
  `scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、
  `USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md`、`WORK_LOG.md`、`HISTORY/README.md`、
  `HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/CHANGELOG.md`；联调自动追加
  `performance/PERFORMANCE_METRICS.md`。
- 修改内容：完整运行器新增默认关闭的 `ISAAC_MANUAL_INPUT_MODE=joystick` 模式，原
  键盘默认行为不变。手柄采用实测北通映射 `Yaw=0, Throttle=1, Roll=2, Pitch=3`；
  Pitch/Roll 按当前 yaw 从机体系转换为 world 坐标，Throttle 作为 world Z，一并发布
  `/human_intent`，C++ FOV-GVF 修正后才经 `/position_cmd -> /sim/cmd_vel` 执行。
  Yaw 独立旋转机头和前向相机。保留0.5秒回中解锁、0.08死区、断连全停；末端 dead-man
  单独检查平移输入，确保只转Yaw时不会沿用旧平移命令。组合脚本默认选择4倍Cloud。
- 验证：Python与Shell语法通过。纯Python映射检查最初误把对角输入期望写成未归一化
  数值，断言失败；按程序既定的平面合速度归一化修正期望后通过，覆盖Mode 2符号、
  机体到world旋转、平面归一化、升降和偏航。受限环境无法读取`/dev/input`，启动脚本
  按预期拒绝运行；获得主机设备权限后执行5秒可见Isaac+ROS联调成功：识别北通8轴
  16按钮，`/position_cmd`发布者为1，加载4倍Cloud与0.40 m ESDF，四通道均为0，0.5秒
  后解锁，303个仿真控制帧内位置保持`(-64,-11.2,2.8)`、yaw为0、碰撞阻断为0，
  `MANUAL_TIMEOUT`后两个ROS节点干净退出。该次无人工推杆，控制器处于`ZERO_INTENT`、
  guidance帧为0，故未验证实际运动方向或动态绕障效果。
- 影响与边界：`KEYBOARD/` 未修改，仍是无ROS直控参考；原完整入口默认仍为键盘和平地。
  C++避障数学、launch参数、RViz与USER-EGO均未修改。手柄组合入口只做人工局部避障，
  不增加全局寻路能力。
- 回滚/备份：未建立完整副本；删除 `MISSANDKEYBOARD/`，并移除运行器的 joystick模式
  分支即可恢复。原键盘路径始终保留且默认关闭新模式。
- 遗留问题：用户需现场分别推动四杆确认方向，并在Cloud障碍前验证减速、绕行、回中
  停车和断连停车；未完成这些动作前不把动态避障标为通过。

### 2026-09-17 — 建立 EGO1P0 独立版本分支

- 目的：以 `user/Fov-gvf` 当前“北通手柄 + C++深度角域FOV-GVF避障”实现为主基线，
  建立后续可独立修改的 `EGO1P0` 目录版本，并完整说明现有功能、平台与参数。
- 涉及文件：从 `/home/starry/isaac-data/user/Fov-gvf` 复制当前源码、场景、配置、脚本、
  文档和性能基线到 `/home/starry/isaac-data/EGO1P0`；新增 `EGO1P0_VERSION.md`；只在
  构建、启动、场景准备/生成脚本中把绝对工程路径改为EGO1P0，并将colcon临时目录改为
  `/tmp/fov_gvf_ego1p0_isaac_{build,install,log}`。未复制可再生成的工程内
  `build/install/log`、`__pycache__`和`.pytest_cache`。
- 具体内容：算法源码、ROS launch参数、北通轴映射、USD/occupancy场景、RViz配置、
  ROS话题和默认行为均保持主基线建立时状态。`EGO1P0_VERSION.md`记录人工意图到避障
  再到Isaac的完整数据链，Ubuntu/ROS 2/Isaac/GPU/手柄平台，Isaac、手柄、控制器、
  命令桥、地图、RViz和性能参数，以及构建运行方法与验证边界。
- 验证：`rsync --checksum --dry-run`显示差异仅为新增版本说明和预期的路径适配
  文件；其余文件校验一致。关键Python文件语法检查与全部Shell脚本`bash -n`通过，
  源码脚本中不再残留`/home/starry/isaac-data/user/Fov-gvf`或旧
  `/tmp/fov_gvf_user_isaac_*`执行路径。使用EGO1P0独立临时目录构建3个ROS包成功；
  C++定向测试`camera_geometry_check`、`depth_geometry_check`、`angular_field_check`、
  `motion_safety_check`、`guidance_composition_check`共5/5通过。未运行Isaac动态仿真，
  未重复声明手柄动态绕障通过。
- 影响与边界：主基线`user/Fov-gvf`的算法和运行文件未改；EGO1P0不会自动跟随主基线
  后续变化。两者仍共用ROS域42和`/tmp/fov_gvf_user_navigation.lock`，因此有意禁止并行
  运行，防止同域重复控制器。
- 回滚/备份：EGO1P0本身即2026-09-17目录快照；删除该新目录和独立`/tmp`构建产物
  即可回滚，不影响主基线。
- 遗留问题：需用户现场推动四杆验证方向，并在Cloud障碍前验收动态局部避障、回中
  停车和USB断连停车。

### 2026-09-17 — EGO1P0 主入口默认切换为北通手柄控制

- 目的：按用户要求，使EGO1P0直接运行完整主脚本时采用与
  `user/Fov-gvf/MISSANDKEYBOARD`相同的北通A2P3A Mode 2手柄输入，不再需要进入
  `MISSANDKEYBOARD/`专用目录。
- 涉及文件：`scripts/run_isaac_fov_gvf_navigation.sh`、
  `scripts/isaac/run_fov_gvf_navigation.py`、`USER_MANUAL_CONTROL_GUIDE.md`、
  `EGO1P0_VERSION.md`、`DONG.md`、`performance/PERFORMANCE_METRICS.md`、
  `HISTORY/FOV_GVF_FAMILY.md`和`HISTORY/CHANGELOG.md`。
- 修改内容：主Shell入口与Python运行器的默认`ISAAC_MANUAL_INPUT_MODE`由`keyboard`
  改为`joystick`；主入口在启动ROS/Isaac前检查默认北通by-id设备是否可读，缺失时安全
  退出并提示连接设备。原键盘控制未删除，可通过
  `ISAAC_MANUAL_INPUT_MODE=keyboard`显式启用。手柄映射、速度、死区、0.5秒回中解锁、
  USB断连停车、机体系到world转换、360°水平避障和XY/Z解耦均继续复用既有实现。
- 验证：Shell与Python语法检查通过；用不存在的设备路径确认默认入口在拉起ROS前以
  状态1拒绝，用显式keyboard和不存在的ROS安装目录确认键盘分支绕过手柄检查；
  `test_manual_control_math.py`为4/4通过。主机5秒可见Isaac+ROS零输入联调成功，主入口
  打印`mode=joystick scene_mode=cloud`，识别北通8轴16按钮，控制器报告4个水平视角，
  加载Cloud ESDF，四杆为零并在0.5秒后解锁；303帧内位置保持
  `(-64,-11.2,2.8)`、yaw为0、碰撞阻断为0，随后超时且ROS节点干净退出。截图
  `/tmp/ego1p0_default_joystick.png`确认Cloud障碍场景正常显示。
- 影响与边界：这是默认输入选择和启动前安全检查的变化，没有修改FOV-GVF数学、ROS
  话题或手柄轴定义。零输入联调不能替代人工推杆验收；真实四轴方向、动态绕障、回中
  停车和拔掉USB停车仍需用户现场操作确认。
- 回滚/备份：将两个默认值恢复为`keyboard`并移除主Shell的手柄预检即可回滚；显式
  keyboard兼容路径目前可直接使用，无需回滚代码。

### 2026-09-17 — 修复右摇杆水平零输出并恢复点云输入密度

- 目的：处理“右摇杆方向在RViz可见但无人机不平移”，并恢复为
  `user/Fov-gvf/MISSANDKEYBOARD`的点云观感和传感器基线。
- 涉及文件：四相机场景准备器与两份生成USD、Isaac运行器、深度角域核心/控制节点、
  `horizontal_360_check.cpp`、版本/控制说明、自动运行性能记录，以及HISTORY状态与总账。
- 修改内容：通过状态切换诊断把请求速度、所选视角自由距离、制动/rollout限速和发布
  速度串联输出。实测四路120°相机把机体电机/桨叶投进碰撞锥，所有方向自由距离约
  `-0.33~-0.35 m`，所以安全层正确但错误地把水平速度压为0。四路改为中心间隔90°的
  90°FOV，并把每路从`160×120`恢复到`320×240`；角域相机模型同步为90°/68°。
  精确45°拼接边界在测试中复用运行时内侧像素钳位。手柄心跳增加enabled/connected。
- 验证：RViz源配置与USER逐字节相同，launch地图参数均为`depth_stride=4`、
  `map_resolution=0.08`、`publication_resolution=0.25`、`maximum_publish_points=160000`。
  ROS三包构建成功。第一次测试命令把`--log-base`放错位置而未运行；第二次未设
  `PYTHONNOUSERSITE`，导致2项Python收集受用户NumPy 2.2.6与系统SciPy不兼容影响，
  同时90°精确45°边界旧断言失败；修正测试并隔离用户site-packages后，9/9 CTest、
  汇总16项均通过。正式运行`20260917_143846`中右摇杆请求/制动/rollout均可达
  `2.000 m/s`、实际发布`1.998 m/s`，XY位置由`(-64.00,-11.20)`移动到约
  `(-29.85,-28.66)`；前向历史地图发布点数由约1922增长到52280。
  最终窗口`20260917_144635`已重新启动供继续测试；实时端点检查确认点云发布端与
  RViz订阅端均采用兼容的`BEST_EFFORT/VOLATILE` QoS。
- 影响与边界：未改轴映射、地图体素/抽样、RViz点大小和速度上限。近障碍时的安全
  减速仍保留；USB拔出停车本轮未现场执行。原USER/MISSANDKEYBOARD没有被修改。

### 2026-09-18 — 初始化EGO1P0 Git分支并准备推送

- 目的：把当前已验证的EGO1P0版本纳入用户GitHub仓库，同时不覆盖已有远程`dev`。
- 涉及文件：新增项目本地`.git/`元数据；修改本文件和`DONG.md`两处既有Markdown
  行尾空格。`build/`、`install/`、`log/`、Python缓存和测试缓存继续由`.gitignore`
  排除。
- 修改内容：以本机`Fov-gvf`历史提交`d267feb`为父节点初始化分支`ego1p0`，远程设置为
  `https://github.com/Chenwill1899/Fov-gvf.git`；提交当前源码、配置、场景、说明和性能
  记录共241个受控文件，源码快照提交为`d02b140`（`Add EGO1P0 joystick avoidance
  baseline`）。没有修改远程`dev`。
- 验证：远程分支检查确认仅有`dev`、没有`ego1p0`重名；有效文件最大约17 MB，未超过
  GitHub单文件限制；秘密模式扫描未发现私钥、GitHub令牌或常见API密钥；构建/安装/
  日志与缓存目录均显示为ignored。首次`git push -u origin ego1p0`失败，错误为HTTPS
  无法读取GitHub Username；进一步检查确认本机没有`gh`、SSH私钥、GitHub Token环境
  变量或已配置凭据助手。因此当前状态为“本地提交完成、远程推送等待用户认证”，
  不能写成已上传成功。
- 影响与边界：本轮没有修改算法、场景参数或运行方式；远程仓库尚未发生任何变化。
  完成认证后只需从本目录重新执行`git push -u origin ego1p0`。

### 2026-09-18 — 按用户确认改为直接从远程dev建立ego1p0

- 目的：满足“在dev主分支下新建分支，并把目前内容提交到新分支”的明确要求，避免
  把本机未推送的历史实验提交带入远程分支。
- 涉及文件：本地Git提交图和本工作日志；工作区其余文件内容保持当前EGO1P0快照。
- 修改内容：获取远程`dev=804f26e2e631d0571f5e1cc5088100481df1b16d`，计划以该提交
  作为`ego1p0`的直接父节点，并用当前完整文件树生成一个快照提交。此前基于本机
  `d267feb`建立但未推送的`d02b140/a24035c`不进入新的远程分支历史。
- 验证：`git fetch origin dev`成功，远程dev仍为`804f26e init`；当前工作区在重建前
  无未提交改动。GitHub页面显示`Chenwill1899`已邀请当前账号`dhmiaolovestar-hash`
  协作，但邀请尚待用户确认接受；本条记录时远程仍未创建`ego1p0`。
- 影响与边界：不改远程dev，不改算法、参数、场景或运行文件；只调整待推送分支的
  父提交和提交图。
