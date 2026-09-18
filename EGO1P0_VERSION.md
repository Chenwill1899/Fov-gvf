# EGO1P0 版本基线说明

## 1. 分支定位

- 版本名：`EGO1P0`（EGO 1.0）。
- 建立日期：2026-09-17。
- 主基线：`/home/starry/isaac-data/user/Fov-gvf`。
- 当前分支：`/home/starry/isaac-data/EGO1P0`。
- 基线来源：主基线中已经完成零输入联调的 `MISSANDKEYBOARD`“北通手柄 + 深度角域
  FOV-GVF 避障”版本。

本目录是完整独立副本，不是指向主基线的软链接。建立分支时没有修改算法、场景、
话题、控制参数或手柄映射；只把运行/构建脚本中的绝对工程路径改到 `EGO1P0`，并把
可再生成的 `/tmp` 构建前缀改为 `fov_gvf_ego1p0_isaac_*`，避免覆盖主基线。复制时没有
带入工程内旧的 `build/`、`install/`、`log/`、`__pycache__/` 和 `.pytest_cache/`。

今后若要试验新算法，应优先修改 `EGO1P0`；`user/Fov-gvf` 作为建立本版本时的主基线
保留。两个目录不会自动同步。

## 2. 当前已经实现的功能

1. 北通 A2P3A USB 手柄采用多旋翼 Mode 2 控制：Throttle、Yaw、Pitch、Roll。
2. Pitch/Roll 是相对当前机头的机体系输入，运行时转换成 world 坐标；Throttle控制
   world Z；三轴平移以带时间戳的 `/human_intent` 发布。
3. 四台 90°、`320×240` 水平深度相机以前/左/后/右四个视角拼接覆盖 360°；C++ 深度角域
   FOV-GVF 根据期望 XY 方向选择视角，不先构建全局三维地图，只修正水平意图，再与
   独立 Z 输入组合发布 `/position_cmd`。
4. Python命令桥在holonomic模式下把world速度转换为无人机机体系Twist，发布
   `/sim/cmd_vel`；Isaac Sim以固定仿真步更新无人机。
5. Yaw由手柄独立控制机头，同时旋转四台水平深度相机。当前FOV-GVF只规划 XY，不
   修改Z，也不规划Yaw。
6. 启动时四杆必须在死区内连续回中0.5秒才解锁；摇杆回中停止，USB断连清零平移与
   偏航；Isaac执行端另有逐帧平移dead-man，避免只转Yaw时沿用旧平移命令。
7. Cloud场景使用`occupancy.bin`生成ESDF做仿真末端碰撞保护；平地或其他场景使用
   地面高度和障碍AABB保护。该保护是最后一道执行边界，不是控制器的规划输入。
8. RViz显示无人机、轨迹、命令、深度图、角域导引、当前青色深度点云和历史紫色
   观测地图；历史地图只显示，不进入避障控制。
9. 控制器、命令桥和Isaac分别统计控制帧、运行频率、避障计算耗时、控制间隔与延迟，
   每次运行追加到`performance/PERFORMANCE_METRICS.md`。
10. 保留键盘人工避障入口以及`KEYBOARD/`无ROS、无避障的手柄直控参考入口。

当前实现属于“人工意图 + 水平360°局部避障”，不具备全局路径搜索或上下三维绕障，
也不保证在死胡同中自动找到出口。

## 3. 控制数据链

```text
BEITONG A2P3A Mode 2
        |
        v
Pitch/Roll机体系 -> world坐标 XY + 独立 Throttle Z -> /human_intent
        |
        +---- Yaw -> Isaac机头/四相机方向
        |
        v
前/左/后/右 Depth + CameraInfo + /sim/odom
        |
        v
C++ depth_angular_controller（按 desired XY 选视角并做水平FOV-GVF）
        |
        v
/position_cmd（world速度）
        |
        v
position_cmd_to_twist（holonomic world->body）
        |
        v
/sim/cmd_vel -> Isaac 60 Hz运动积分 -> ESDF/AABB末端保护
```

关键ROS 2话题：

| 话题 | 类型/作用 |
|---|---|
| `/human_intent` | `geometry_msgs/TwistStamped`，人工world平移意图 |
| `/sim/depth/image_raw` | 前向深度图 |
| `/sim/depth/camera_info` | 深度相机内参 |
| `/sim/depth_left/*` | 左向深度图及内参 |
| `/sim/depth_back/*` | 后向深度图及内参 |
| `/sim/depth_right/*` | 右向深度图及内参 |
| `/sim/odom` | `nav_msgs/Odometry`，world位姿和速度 |
| `/position_cmd` | `pc_gvf_msgs/PositionCommand`，避障后的world指令 |
| `/sim/cmd_vel` | `geometry_msgs/Twist`，给Isaac的机体系速度 |
| `/pc_gvf/angular_field` | RViz角域导引场 |
| `/pc_gvf/radar_points` | 当前帧青色深度点云 |
| `/pc_gvf/observed_map` | 紫色历史观测地图，只用于显示 |

前向相机路径和话题保持兼容；另增左、后、右三台相机。四台相机的机体系中心方向为
`0°/+90°/180°/-90°`，每台 FOV 90°，期望方向与所选相机中心最大相差 45°；视场
拼接边界采用内侧像素钳位。该参数与原 USER/MISSANDKEYBOARD 前向相机一致，避免
120°广角把机体电机/桨叶纳入碰撞锥并错误压制水平指令。
第三人称相机仍只用于观察，不输入避障算法。

## 4. 使用平台

### 软件平台

| 项目 | 当前基线 |
|---|---|
| 操作系统 | Ubuntu 22.04，Linux x86_64；建立版本时内核为`6.8.0-138-generic` |
| ROS 2 | Humble；本机`ros-humble-ros-base`包版本`0.10.0-1jammy.20260804.204550` |
| RMW | `rmw_fastrtps_cpp`，`ROS_LOCALHOST_ONLY=1` |
| Isaac Sim | `6.0.1-rc.7+release.42383.32955d8d.gl` |
| Isaac Python | Python 3.12.13 |
| 控制器 | ROS 2 C++节点`depth_angular_controller` |
| 平台桥与显示 | ROS 2 Python节点，`rclpy` |
| 场景格式 | OpenUSD/USD；Cloud场景附带二进制占据栅格`occupancy.bin` |
| 可视化 | RViz2，目标渲染帧率60 FPS |

### 硬件平台

| 项目 | 当前基线 |
|---|---|
| USB手柄 | BEITONG A2P3A BFM DONGLE，USB ID `20bc:511c`，Linux识别为8轴16按钮 |
| 默认设备路径 | `/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick` |
| 图形平台 | 最近一次组合联调使用NVIDIA GeForce RTX 5070，驱动580.173.02 |
| 仿真无人机 | USD四旋翼模型，运动学速度控制；不是实机飞控固件 |

## 5. 默认场景与Isaac参数

通过`EGO1P0/MISSANDKEYBOARD/run_joystick_avoidance.sh`启动时，默认场景模式是
`cloud`。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| Cloud整体缩放 | 4倍 | 约`160 × 120 × 20 m`的障碍范围 |
| Cloud出生位置 | `(-64.0, -11.2, 2.8)` | 只作为手动会话初始位姿，无固定终点 |
| 平地尺寸/出生位置 | `200 × 200 m` / `(0,0,1.5)` | `FOV_GVF_SCENE_MODE=flat`时使用 |
| 深度相机分辨率 | 四路各 `320 × 240` | 前向RViz点云密度与原USER/MISSANDKEYBOARD一致 |
| 单相机水平FOV | `90°` | 四路中心间隔90°，拼接覆盖水平360° |
| 仿真控制步长 | `1/60 s` | Isaac运动与里程计更新 |
| 机体碰撞半径 | `0.25 m` | Isaac末端保护与控制器一致 |
| Cloud ESDF分辨率 | `0.40 m` | 原`0.10 m`体素随4倍场景缩放 |
| 渲染器 | `MinimalRendering` | 可改为`RaytracedLighting` |
| 抗锯齿 | `TAA` | 可选`OFF/TAA/FXAA` |
| Minimal shading mode | `2` | Texture Diffuse |
| Sun/Dome/环境光 | `800 / 0 / 0.05` | 工程观察视图参数 |
| 视口抓图 | `/tmp/fov_gvf_isaac_viewport.png` | 启动后自动抓取一次 |
| 第一/第三人称 | `1 / 3 / V` | 选择或切换观察视角 |

主要Isaac环境变量：

| 环境变量 | 默认值/范围 |
|---|---|
| `ISAAC_MANUAL_INPUT_MODE` | `joystick`；可显式设为`keyboard`使用兼容键盘控制 |
| `FOV_GVF_SCENE_MODE` | 完整通用入口和手柄避障入口均默认`cloud`；可显式设为`flat` |
| `FOV_GVF_SCENE` | 空；设置后直接指定USD |
| `FOV_GVF_ESDF_OCCUPANCY` | 自动使用所选场景同目录`occupancy.bin` |
| `ISAAC_HEADLESS` | `0`；人工控制要求可见窗口 |
| `ISAAC_MANUAL_TIMEOUT` | `0`，表示不自动超时 |
| `ISAAC_VIEW_RENDERER` | `MinimalRendering` |
| `ISAAC_VIEW_ANTIALIASING` | `TAA` |
| `ISAAC_MINIMAL_SHADING_MODE` | `2` |
| `ISAAC_SUN_INTENSITY` | `800` |
| `ISAAC_DOME_INTENSITY` | `0` |
| `ISAAC_AMBIENT_LIGHT_INTENSITY` | `0.05` |
| `ISAAC_VIEW_EYE` | `0,-180,145` |
| `ISAAC_VIEW_TARGET` | `0,0,6` |

## 6. 手柄参数

| 参数/环境变量 | 默认值 | 作用 |
|---|---:|---|
| `JOYSTICK_AXIS_YAW` | `0` | 左摇杆左右 |
| `JOYSTICK_AXIS_THROTTLE` | `1` | 左摇杆上下 |
| `JOYSTICK_AXIS_ROLL` | `2` | 右摇杆左右 |
| `JOYSTICK_AXIS_PITCH` | `3` | 右摇杆上下 |
| `JOYSTICK_SIGN_YAW` | `-1` | Yaw方向修正 |
| `JOYSTICK_SIGN_THROTTLE` | `-1` | Throttle方向修正 |
| `JOYSTICK_SIGN_ROLL` | `+1` | Roll方向修正 |
| `JOYSTICK_SIGN_PITCH` | `-1` | Pitch方向修正 |
| `JOYSTICK_HORIZONTAL_SPEED` | `2.0 m/s` | Pitch/Roll最大水平合速度 |
| `JOYSTICK_VERTICAL_SPEED` | `1.5 m/s` | 输入端最大升降意图；桥接输出再限制为1.0 m/s |
| `JOYSTICK_YAW_RATE_DEG` | `75 deg/s` | 最大手柄偏航角速度 |
| `JOYSTICK_DEADZONE` | `0.08` | 四轴中心死区，必须小于1 |
| `JOYSTICK_NEUTRAL_HOLD` | `0.5 s` | 启动回中解锁保持时间 |

平面对角输入会归一化，水平合速度不会超过`JOYSTICK_HORIZONTAL_SPEED`。轴4/5的
Gas/Brake和轴6/7的方向键不参与飞行控制。

## 7. FOV-GVF控制与安全参数

权威运行值位于`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `human_intent_timeout` | `0.25 s` | 人工意图过期即停 |
| `max_depth_age` | `0.5 s` | 深度图过期即停 |
| `angular_width × angular_height` | `48 × 36` | 角域规划网格，使用C++默认值 |
| `max_depth` | `40.0 m` | 控制器最大规划深度 |
| `forward_lookahead` | `8.0 m` | 非固定目标的前向参考距离，使用C++默认值 |
| `obstacle_clear_frames` | `3` | 障碍连续消失3帧才释放 |
| `speed / max_speed` | `2.0 / 2.0 m/s` | 参考/最大水平合速度 |
| `max_vertical_speed` | `1.0 m/s` | 控制器最大垂直速度 |
| `horizontal_360_enabled` | `true` | 启用四视角水平避障和XY/Z分离 |
| `command_accel_limit` | `1.2 m/s²` | 命令加速度限制 |
| `command_jerk_limit` | `8.0 m/s³` | 命令jerk限制 |
| `command_response_time` | `0.22 s` | 平滑响应时间 |
| `speed_recovery_accel` | `1.0 m/s²` | 加速恢复限制 |
| `speed_brake_accel` | `2.5 m/s²` | 安全制动限制 |
| `max_direction_rate` | `1.60 rad/s` | 导引方向最大变化率 |
| `continuous_harmonic_guidance` | `true` | 连续调和梯度导引 |
| `harmonic_gradient_radius_cells` | `3` | 梯度估计半径 |
| `harmonic_gradient_lookahead_cells` | `2.5` | 梯度前视距离 |
| `safe_goal_hysteresis_weight` | `0.40` | 安全目标滞回权重 |
| `safe_goal_hold_time` | `0.35 s` | 安全目标最短保持 |
| `body_radius` | `0.25 m` | 无人机等效半径 |
| `safety_margin` | `0.20 m` | 规划安全余量 |
| `rollout_margin` | `0.10 m` | 前向预测附加余量 |
| `planning_horizon` | `3.0 s` | 预测时域 |
| `field_publish_rate` | `15 Hz` | RViz角域场发布率 |
| `performance_report_interval` | `5 s` | 终端性能摘要周期 |

控制器采用人工输入模式：`fixed_forward_intent=false`、`use_fixed_goal=false`、
`stop_at_goal=false`、`use_reference_line=false`，因此没有自动起点到终点任务。

## 8. 命令桥、地图与RViz参数

### 命令桥

| 参数 | 当前值 |
|---|---:|
| `mode` | `holonomic` |
| `max_vx / max_vy` | `2.0 / 2.0 m/s` |
| `max_vz` | `1.0 m/s` |
| `max_w` | `0.0 rad/s`；Yaw由Isaac手柄路径独立处理 |
| 输入/输出 | `/position_cmd` -> `/sim/cmd_vel` |

### 历史观测地图（只显示）

| 参数 | 当前值 |
|---|---:|
| `depth_stride` | `4` |
| `minimum_depth / maximum_depth` | `0.3 / 15.0 m` |
| `map_resolution` | `0.08 m`，内部累计体素 |
| `minimum_hits` | `2` |
| `map_publish_rate` | `1 Hz` |
| `publication_resolution` | `0.25 m`，RViz显示体素 |
| `maximum_publish_points` | `160000` |
| `maximum_pose_age` | `0.20 s` |
| `camera_offset` | `[0.22, 0.0, 0.02] m` |

导航显示中无人机Marker为60 Hz、Path为10 Hz，轨迹最小采样距离0.10 m、最多1000
个位姿；RViz配置目标帧率为60 FPS。

## 9. 构建与运行

首次使用本分支必须单独构建：

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/build_isaac_ros_workspace.sh
```

构建产物：

```text
/tmp/fov_gvf_ego1p0_isaac_build
/tmp/fov_gvf_ego1p0_isaac_install
/tmp/fov_gvf_ego1p0_isaac_log
```

北通手柄 + 4倍Cloud + 完整避障：

```bash
cd /home/starry/isaac-data/EGO1P0/MISSANDKEYBOARD
bash run_joystick_avoidance.sh
```

手柄 + 平地 + 完整ROS链：

```bash
FOV_GVF_SCENE_MODE=flat bash run_joystick_avoidance.sh
```

无ROS、无避障的手柄直控参考：

```bash
cd /home/starry/isaac-data/EGO1P0/KEYBOARD
bash run_joystick_only.sh
```

通用完整入口默认使用北通手柄和Cloud场景：

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/run_isaac_fov_gvf_navigation.sh
```

显式切回兼容键盘输入：

```bash
ISAAC_MANUAL_INPUT_MODE=keyboard bash scripts/run_isaac_fov_gvf_navigation.sh
```

默认`ROS_DOMAIN_ID=42`，使用`/tmp/fov_gvf_user_navigation.lock`和
`/position_cmd`单发布者检查。该锁有意与主基线共用，防止两个仿真实例在同一DDS域
同时控制；不要并行运行主基线和EGO1P0。

## 10. 建立本版本时的验证边界

- 主基线在2026-09-12完成过5秒北通手柄零输入组合联调：识别8轴16按钮、四杆回中
  解锁，4倍Cloud、ROS控制链和ESDF启动成功，303帧内无人机保持
  `(-64,-11.2,2.8)`，无漂移。
- EGO1P0建立时会执行源文件一致性检查、Python/Shell静态检查和独立ROS构建；这些
  结果记录在本分支`WORK_LOG.md`与统一`HISTORY/CHANGELOG.md`。
- 零输入联调不等于动态绕障通过。实际推杆方向、障碍前减速/绕行、回中停车和USB
  断连停车仍需要现场验收。
