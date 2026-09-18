# Blender 云团场景 + Isaac Sim + FOV-GVF 导航

当前导航场景采用“外部 FOV-GVF ROS 2 控制器 + Isaac Sim 传感器/运动适配”的结构：

```text
Isaac Sim USD
  ├─ 14 个静态云团 Mesh 碰撞体
  ├─ /World/Robot（四旋翼可视模型）
  ├─ /World/Robot/CameraMount/DepthCamera（前向深度相机，兼容路径）
  ├─ /World/Robot/CameraMountLeft/DepthCamera（左向）
  ├─ /World/Robot/CameraMountBack/DepthCamera（后向）
  └─ /World/Robot/CameraMountRight/DepthCamera（右向）
        │  /sim/odom
        │  /sim/depth{,_left,_back,_right}/image_raw (32FC1)
        │  /sim/depth{,_left,_back,_right}/camera_info
        ▼
pc_gvf/depth_angular_controller
        │  /position_cmd
        ▼
position_cmd_to_twist → /sim/cmd_vel → Isaac Sim 运动更新
```

## 生成导航 USD

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/export_blender_cloud_to_isaac.sh 42
```

这会保留 Blender 母版，同时生成：

- `isolated_clouds_seed42_isaac.usd`：环境、材质、PhysicsScene 和静态障碍碰撞；
- `isolated_clouds_seed42_navigation.usd`：再加入四旋翼、四台水平深度相机和导航元数据。

起点与终点均为高度 `1.2 m`：

- 起点：`(-8, 0, 1.2)`；
- 终点：`(8, 0, 1.2)`。

## 构建 ROS 2 适配包

```bash
bash scripts/build_isaac_ros_workspace.sh
```

默认安装前缀为 `/tmp/fov_gvf_isaac_install`。该构建包含现有 C++ FOV-GVF 控制器、`PositionCommand` 速度适配器和新增 Isaac 导航 launch。

## 运行闭环

```bash
FOV_GVF_INSTALL=/tmp/fov_gvf_isaac_install \
bash scripts/run_isaac_fov_gvf_navigation.sh
```

脚本默认同时启动固定终点控制器、速度适配器、Isaac Sim 图形窗口和 RViz。Isaac Sim 使用覆盖全场的斜俯视观察相机；RViz 显示无人机、已执行轨迹、速度指令、前向深度、FOV、深度障碍点和角域 GVF。控制器参数为：定高 `z=1.2 m`、目标/最大速度 `2.0 m/s`、机体半径 `0.25 m`、安全余量 `0.20 m`，到达 `0.30 m` 终点容差后停止。接近障碍物时，安全制动和深度 rollout 仍可主动将实际命令降到 `2.0 m/s` 以下。

需要关闭某一个图形界面时可使用：

```bash
FOV_GVF_RVIZ=false bash scripts/run_isaac_fov_gvf_navigation.sh
ISAAC_HEADLESS=1 bash scripts/run_isaac_fov_gvf_navigation.sh
```

Isaac 主视口默认观察参数为：

```text
eye    = -12.5,-15.0,13.0
target = 0.0,0.0,1.25
```

这里的 `eye=x,y,z` 是观察相机在 Isaac 世界坐标系中的位置，
`target=x,y,z` 是镜头中心对准的位置；增大 `eye` 的 `z` 会抬高视点，
增大 `eye` 与 `target` 的距离会看到更大的场景范围。两点不要设置成完全相同。

可在启动时覆盖，例如切换成更接近顶视的画面：

```bash
ISAAC_VIEW_EYE="0,-10,20" \
ISAAC_VIEW_TARGET="0,0,1" \
bash scripts/run_isaac_fov_gvf_navigation.sh
```

若画面曝光不合适，还可以调整 Isaac RTX 使用的太阳光和环境光强度：

```bash
ISAAC_SUN_INTENSITY=3000 \
ISAAC_DOME_INTENSITY=900 \
bash scripts/run_isaac_fov_gvf_navigation.sh
```

Blender 导出的原始数值约为 `0.55/1.0`，对 Isaac RTX 来说过暗；运行脚本默认会将它们调整为上面的数值，但不会改写 Blender 母版。

脚本会将主视口截图保存为 `/tmp/fov_gvf_isaac_viewport.png`，并在终端输出
`[VIEWPORT CAPTURE] ... mean_rgb=...`。若 `mean_rgb` 明显大于 0，说明实际渲染内容不是黑帧。

运行中也可直接操作 Isaac Sim 主视口：按住 `Alt + 左键` 环绕，`Alt + 中键` 平移，
`Alt + 右键` 或鼠标滚轮缩放；在 Stage 中选中 `/World/Robot` 后按 `F` 可聚焦无人机。
若手动切换过相机，可在视口上方的 Camera 菜单中重新选择
`/World/NavigationOverviewCamera`。脚本每次启动都会重新创建并绑定该观察相机，
它与控制器使用的前/左/后/右四台水平深度相机相互独立。

## 本机验证边界

已验证：

- Blender USD → Isaac USD → 导航 USD 的生成链路；
- 14 个静态障碍碰撞体、地面、PhysicsScene、四旋翼和深度相机 prim；
- ROS 2 工作区干净构建成功。

闭环需要 Isaac Sim 获得 NVIDIA GPU/显示环境，并允许 ROS 2 DDS 创建本地 UDP socket。在受限沙箱中运行会出现 `getifaddrs: Operation not permitted`、UDP 创建失败或 `NVML_ERROR_DRIVER_NOT_LOADED`，此时控制器会安全停在起点并报告 `WAITING_ODOMETRY`/`STALE_DEPTH`，不能据此判断算法或场景失败。
