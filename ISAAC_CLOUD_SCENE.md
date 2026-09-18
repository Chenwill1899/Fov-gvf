# Isaac Sim 云状柱障碍场景

## 场景含义

- 二维俯视图中，黑色区域是障碍物，白色区域是可飞行空间。
- 黑色连通区域在 Isaac Sim 中从地面向上拉伸为静态三角网格柱体。
- 柱体具有真实 `CollisionAPI`，不是只用于显示的贴图。
- 同一个随机种子总是生成同一个场景。
- 生成器会清出一条弯曲通道，并验证起点和终点在满足净空阈值的同一连通区域内。

默认场景大小为 `18 m × 12 m`，网格分辨率为 `0.15 m`，柱高约 `3.4 m`，起点和
终点分别为 `(-8, 0, 1.2)` 和 `(8, 0, 1.2)`。

## 生成并打开

```bash
cd /home/starry/isaac-data/Fov-gvf
bash scripts/run_cloud_pillar_scene.sh 42
```

正常时终端最后会显示三行 `[SCENE READY]`，并自动采用能看到整个场地的斜俯视视角。

参数 `42` 是随机种子。换成其他整数即可生成另一张地图，例如：

```bash
bash scripts/run_cloud_pillar_scene.sh 103
```

若只想生成 USD、不打开 Isaac Sim：

```bash
/home/starry/isaac-data/isaacsim/python.sh \
  scripts/isaac/generate_cloud_pillar_scene.py \
  --seed 103 \
  --output scenes/cloud_pillars/cloud_pillars_seed103.usd
```

可调参数包括 `--occupancy`、`--cell-size`、`--corridor-radius`、`--height` 和
`--height-variation`。提高 `--occupancy` 会使黑色区域更密；减小
`--corridor-radius` 会缩窄保证通道，但不得低于无人机外接半径、安全余量和跟踪误差之和。

## 输出文件

每个场景包含：

- `.usd`：可直接载入 Isaac Sim 的三维场景；
- `.json`：种子、尺寸、起终点、实际障碍占比、网格统计和通道验证参数；
- `_map.png`：生成三维柱体时使用的原始黑白二维占据图。

无人机闭环接入时应将 ARL Robot 1 加到 `/World/Robot`，初始位置设置为 manifest 的
`start`，目标设置为 `goal`。避障算法只能使用深度传感器，不能读取 `_map.png` 或
manifest 中的通道信息。
