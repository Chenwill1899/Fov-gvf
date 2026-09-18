# EGO-Swarm Cloud 键盘避障驾驶

当前分支：`feature/ego-swarm-cloud-navigation`。

## 场景与控制

- 来源：`/home/starry/isaac-data/ego_swarm_cloud/ego_swarm_cloud.usd`。
- 原始空间：`40 × 30 × 5 m`，占据栅格 `400 × 300 × 50`，分辨率 `0.1 m`。
- 驾驶空间：环境 Mesh 统一放大 4 倍，约 `160 × 120 × 20 m`。
- ESDF：沿用相同占据索引，物理分辨率同步变为 `0.4 m`；不用整体 AABB。
- 出生位姿：`(-64.0, -11.2, 2.8)`，只用于仿真初始化，不是导航起点。
- 场景中没有起点和终点 Marker，也没有自动目标。
- 键盘速度上限：`2.0 m/s`；最大垂直速度：`1.0 m/s`。

## 生成与运行

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/prepare_ego_swarm_cloud_navigation.sh
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_isaac_fov_gvf_navigation.sh
```

最后一条命令默认同时打开 Isaac Sim 和 RViz。RViz 显示无人机、速度命令、轨迹、
前向深度图、FOV、深度命中点、角域场、青色雷达坐标实时点云与紫色历史观测
地图；可用 `FOV_GVF_RVIZ=false` 关闭。

点击 Isaac 主视口后使用 `W/S/A/D/R/F` 控制世界坐标三轴运动，`Space` 停止；
方向通过 C++ FOV-GVF 避障后才执行。具体键位和安全机制见
`USER_MANUAL_CONTROL_GUIDE.md`。

## 实现说明

`scripts/isaac/run_fov_gvf_navigation.py` 在 Isaac Python 环境中用
`scipy.ndimage.distance_transform_edt` 从 `occupancy.bin` 预计算有符号欧氏距离场。
每一帧对候选三维位置进行三线性 ESDF 查询，当障碍距离不大于 `0.25 m` 机体半径时，
独立碰撞保护阻止该帧危险位移，但不结束手动会话。ESDF 原点与分辨率随 4 倍场景
缩放同步变化。

控制器使用前、左、后、右四路重叠深度视角和里程计，根据期望XY方向选择对应视角做
水平局部导航；前向 `/sim/depth/*` 话题保留兼容。ESDF不输入控制器，只作为仿真真值
末端碰撞及Z紧急制动保护，因此没有把三维地图引入在线规划，也没有实现上下三维绕障。

`observed_map_visualizer` 只负责 RViz：它把深度图反投影到 X 前/Y 左/Z 上的
`radar` 坐标系并发布 `/pc_gvf/radar_points`；再按 `/sim/odom` 与相机固定外参将
每帧点变换到 `world`，以 `0.15 m` 哈希体素累计。每个体素至少跨两帧命中才确认，
确认后不再随当前视野消失。全局历史地图以 2 Hz 发布到
`/pc_gvf/observed_map`。内部地图保持 `0.15 m`，RViz 发布端以 `0.30 m` 二次体素化
降低视觉密度；它不做安全膨胀，也不参与控制决策。

旧 `/pc_gvf/ego_inflated_occupancy` 真值可视化已经移除。这里的移除不涉及 Isaac
运行脚本中的 ESDF：仿真仍读取 `occupancy.bin` 做独立碰撞保护，C++ 控制器仍只
使用实时深度、CameraInfo 和里程计导航。

## 历史参考

以下为被替换的自动导航版本历史验收记录，不是当前键盘控制的验收结果：2026-09-09
在 Isaac Sim 与 RViz 同时开启的条件下完成闭环复验。RViz 成功使用
OpenGL 4.6 启动，Isaac 主视口平均 RGB 为 `204.27`。无人机从
`(-64.0,-11.2,2.8)` 出发，在三维方向上主动调整高度，于 `68.52 s` 到达
`(63.331,11.927,4.734)` 并进入 `GOAL_REACHED`，未触发 ESDF 碰撞保护。
