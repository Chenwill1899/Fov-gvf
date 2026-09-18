# USER 键盘/手柄自由控制说明

`/home/starry/isaac-data/EGO1P0` 以当前 `user_ego` 的深度直达避障、性能统计、
在线观测地图和 RViz 优化为基础，用于北通 Mode 2 手柄自由驾驶。`user_ego` 自动导航
副本不受本目录修改影响。完整链路现在默认使用与 `user/Fov-gvf/MISSANDKEYBOARD`
相同的手柄输入、安全回中门和断连停车逻辑，键盘作为显式兼容模式保留。

## 构建和运行

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_isaac_fov_gvf_navigation.sh
```

主入口要求北通手柄已连接且设备可读；缺少设备时会在启动 ROS 和 Isaac 前退出。需要
临时使用旧键盘控制时显式执行：

```bash
ISAAC_MANUAL_INPUT_MODE=keyboard bash scripts/run_isaac_fov_gvf_navigation.sh
```

如果只想使用 BEITONG Mode 2 手柄直接移动无人机，不经过 ROS 或避障控制器，请使用
独立 `KEYBOARD/` 入口（目录名为兼容旧入口而保留）：

```bash
cd /home/starry/isaac-data/EGO1P0/KEYBOARD
bash run_joystick_only.sh
```

其手柄通道映射、安全回中解锁和终端状态输出见 `KEYBOARD/README.md`。

`MISSANDKEYBOARD/` 兼容入口仍可使用，行为与当前主入口一致（默认北通手柄和4倍Cloud）：

```bash
cd /home/starry/isaac-data/EGO1P0/MISSANDKEYBOARD
bash run_joystick_avoidance.sh
```

这里 Pitch/Roll 会先按当前机头转换为 world 坐标 `/human_intent`，FOV-GVF 修正平移
方向后才执行；Yaw 独立改变机头和四台水平深度相机朝向。详细参数和安全边界见
`MISSANDKEYBOARD/README.md`。

键盘兼容模式启动后先点击 Isaac Sim 主视口，使其获得键盘焦点。控制键按世界坐标解释：

- `W` / `S`：沿世界 `+X` / `-X`；
- `A` / `D`：沿世界 `+Y` / `-Y`；
- `R` / `F`：上升 / 下降；
- 方向键可替代 `W/S/A/D`，`Page Up/Page Down` 可替代 `R/F`；
- `Space`：立即停止；
- `1` / `3` / `V`：第一人称、第三人称、切换视角。

水平组合键只在 XY 平面内归一化，水平合速度不会超过
`ISAAC_KEYBOARD_SPEED`（默认 `2.0 m/s`）；Z 不参与该归一化，默认最大速度由
`ISAAC_KEYBOARD_VERTICAL_SPEED=1.0` 单独控制。因此 `W+D+R` 可以同时保持最大
水平合速度和最大上升速度。例如：

```bash
ISAAC_MANUAL_INPUT_MODE=keyboard ISAAC_KEYBOARD_SPEED=1.2 \
  bash scripts/run_isaac_fov_gvf_navigation.sh
```

## 控制和安全逻辑

场景没有 `/World/UAV_Start` 和 `/World/UAV_Goal`，没有固定目标、自动前进或到点停止。
当前默认平地 USD 保存 `spawn=(0,0,1.5)` 作为每次仿真的安全初始化位姿；切回 Cloud
模式时仍使用原来的 `spawn=(-64,-11.2,2.8)`。

Isaac 每个仿真帧通过 `get_keyboard_value()` 直接读取物理按键状态，不使用可能因窗口
失焦而遗漏松键的“按下集合”。键盘方向以 `TwistStamped` 发布到 `/human_intent`。
四台水平深度相机分别朝前、左、后、右，每台分辨率为 `320×240`、水平 FOV 为 90°；
四个视角拼接覆盖水平360°，恰在45°边界的方向会钳位到所选视角的内侧像素；
C++ 控制器根据期望水平速度角 `atan2(vy,vx)` 选择最接近的视角，在该视角中继续使用
原深度角域 FOV-GVF，只修正 XY，然后与原始 Z 重新组合并发布 `/position_cmd`。只按
`R/F` 时不调用水平规划，XY 严格为零。

执行器边界还有第二层 dead-man：当前帧没有运动键时，无论 ROS 中是否残留旧命令，
应用速度都立即强制为零。平地模式保留地面高度保护，Cloud 模式继续使用 ESDF。
若组合候选位置碰撞，末端保护会分别检查纯 Z 和纯 XY 候选，优先保留安全的 Z 分量；
这是一项基于仿真 ESDF/AABB 真值的 Z 紧急制动接口，不是三维路径规划。四台相机只
提供水平 360° 覆盖，不能据此宣称上下方向无障碍。

## 当前场景

- 默认模式：`cloud`；使用 `FOV_GVF_SCENE_MODE=flat` 才进入无障碍平地；
- 平地尺寸：`200×200 m`，地面高度 `z=0`；
- 出生位置：`(0,0,1.5)`；
- 环境中没有障碍物，也不会加载 Cloud 的 `occupancy.bin`；
- 无人机、深度相机、ROS 控制链、键盘控制和地面碰撞保护保持启用。

重新生成平地场景：

```bash
bash scripts/prepare_flat_ground_navigation.sh
```

显式启动4倍 Cloud 场景无需重新生成：

```bash
FOV_GVF_SCENE_MODE=cloud bash scripts/run_isaac_fov_gvf_navigation.sh
```

如需重新生成 Cloud 场景：

```bash
bash scripts/prepare_ego_swarm_cloud_navigation.sh
```

## 显示和性能记录

RViz 继续显示无人机、命令箭头、执行轨迹、深度图、角域场、实时青色点云和历史
紫色观测地图；在线地图只供显示，不参与控制。每次运行仍在终端周期输出性能摘要，
并追加到 `performance/PERFORMANCE_METRICS.md`。

手动会话默认没有超时，关闭 Isaac Sim 或按 `Ctrl-C` 结束。需要自动限时测试时可用：

```bash
ISAAC_MANUAL_TIMEOUT=10 bash scripts/run_isaac_fov_gvf_navigation.sh
```

## 关键文件

- `scripts/isaac/manual_control_math.py`：XY 独立归一化和分量式末端安全保护；
- `scripts/isaac/run_fov_gvf_navigation.py`：按键采样、四路深度、dead-man、运动和 ESDF；
- `MISSANDKEYBOARD/`：北通手柄 + 完整深度避障组合入口；
- `scripts/isaac/create_flat_ground_scene.py`：生成仅含平地的基础 USD；
- `scripts/prepare_flat_ground_navigation.sh`：生成当前默认平地手动场景；
- `scripts/isaac/prepare_uav_navigation_scene.py`：手动场景、出生位姿及四台水平相机；
- `src/pc_gvf/launch/isaac_cloud_navigation.launch.py`：关闭自动目标并配置控制器；
- `src/pc_gvf/src/depth_angular_controller_node.cpp`：C++ 避障和控制平滑；
- `src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`：RViz 默认配置。
