2026-10-23 12:47 — **四张地图已换用完整ARL-Robot-1外观。** Cloud、lab、hangar、forest共用官方模型，保留10m/s请求、0.48m物理机体与四相机/原控制器。3包、外观/几何/起飞检查通过，实际四图渲染及16路深度A/B无机身遮挡。新增林地飞行73.241m后触发COLLISION_GUARD、未到达，不能继承旧349.255m成功；完整巡航/长期手柄未验收。[实际效果](../../../EGO2P6/docs/performance/arl_visual_20261023_122132/preview/forest_robot.png) · [完整验证与失败记录](../../../EGO2P6/docs/performance/arl_visual_20261023_122132/VALIDATION.json)。

# 原避障地图高度翻倍

更新：2026-10-22 20:44。EGO2P6默认Cloud地图由480×120×20m调整为**480×120×40m**。长宽与水平障碍布局保持，障碍物沿竖直方向拉伸2倍。

以原地面上表面z=-0.2m为锚点，采用 `z_new = 2*z_old + 0.2`。障碍物高度范围由约[-0.2,19.8]m变为[-0.2,39.8]m；本次高度修改时地面、无人机模型、相机位置和尺寸保持原值；2026-10-23后续独立外观替换已将无人机改为ARL-Robot-1，相机和物理机体不变。

颜色随原顶点一起拉伸：每个顶点的原颜色与颜色索引完全保留，底部红色到顶部紫色覆盖新的40m高度区间，没有上下重复一套渐变。

![增高后的原地图](../../../EGO2P6/docs/performance/cloud_height2x_20261022/preview/cloud_overview.png)

## 运行

原命令直接使用增高后的地图，保存配置请求上限仍为10m/s：

```bash
cd /home/starry/isaac-data/EGO2P6
bash scripts/run_manual.sh
```

手柄起点保持(-64,-11.2,2.8)。控制工作域高度上界同步到39.8m，机体须完整位于域内。已有正在运行的仿真需要退出后重新启动才能加载新文件；继续使用共享ROS域42和单控制入口锁。

`r01`至`r10`和`route300`的起终点原高度10m会在新几何中落入障碍，因此已按同一变换同步到20.2m，水平坐标保持。23个手柄/路线起终点的2.5m启动自由球均已重新认证。旧路线原件保存在源归档中，原实验成绩对应20m旧图，不视为新图成绩。

## 验证范围

- 逐点检查3块障碍网格共2,348,649个顶点；XY不变，Z按公式变换，面索引与颜色索引保持。
- 0.4m碰撞网格从1200×300×50变为1200×300×100，每个原竖直单元对应两个新单元，逐单元比对通过。
- 三包构建、5组几何/配色检查、63项核心、174项工具平台、23个起飞端点、6组源码/安装参数检查通过；已实际渲染地图和4路深度。
- **本次没有新地图飞行。** 验证启动时另一个EGO3P0仿真持有共享导航锁，已取消本任务的等待，不中断该仿真，不保留自动排队。工具保留了无飞行的RUNTIME_ERROR记录和缺失轨迹的审计错误，未把它计作飞行失败或成功。

完整结果与过程错误见[VALIDATION.json](../../../EGO2P6/docs/performance/cloud_height2x_20261022/VALIDATION.json)。前面新增的室内、机库和林地三图文件未变。

## 重建与恢复

`scenes/ego_swarm_cloud/height20_source.tar.gz`保存原20m模型、碰撞栅格、manifest及旧路线配置。生成器每次从该固定来源重建，不会重复运行后继续翻倍：

```bash
cd /home/starry/isaac-data
PYTHONNOUSERSITE=1 isaacsim/python.sh EGO2P6/scripts/build_cloud_height_map.py
EGO2P6_BUILD_TESTING=ON bash EGO2P6/scripts/build_isaac_ros_workspace.sh
PYTHONNOUSERSITE=1 isaacsim/python.sh EGO2P6/scripts/test_cloud_height_map.py
```

完整修改前保存点：`/home/starry/isaac-data/HISTORY/snapshots/EGO2P6_cloud_height2x_20261022_202603`。恢复旧图时还需恢复旧路线、19.8m高度上界和对应起飞校验配置后重新构建；不覆盖当前共享日志，不删除新旧实验记录。
