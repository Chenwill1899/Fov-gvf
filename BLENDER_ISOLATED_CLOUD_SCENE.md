# Blender 独立云团柱体场景

生成器直接通过 Blender Python API 创建可编辑三维对象，不依赖二维图片导入。每个障碍位于 `Cloud_Obstacles` 集合中，是单独的封闭柱体 Mesh；其不规则轮廓由多频径向扰动生成。

场景包含：

- 白色地面 `Ground`；
- 14 个互不接触的黑色云团柱体 `Cloud_01` 至 `Cloud_14`；
- 绿色起点圆盘和空对象 `UAV_Start`；
- 红色终点圆盘和空对象 `UAV_Goal`；
- 顶视相机和灯光。

`UAV_Start` 和 `UAV_Goal` 默认高度均为 `1.2 m`。在 Blender 中放置无人机时，可以将无人机的 Location 复制为对应空对象的 Location，或者把无人机临时设为该空对象的子对象并清零局部位姿。

生成并打开：

```bash
cd /home/starry/isaac-data/Fov-gvf
bash scripts/run_blender_isolated_cloud_scene.sh 42
```

只在后台生成：

```bash
/home/starry/isaac-data/blender/blender --background \
  --python scripts/blender/generate_isolated_cloud_scene.py -- \
  --seed 42 \
  --output scenes/blender_isolated_clouds/isolated_clouds_seed42.blend
```

可在 `--` 后调节 `--cloud-count`、`--min-radius`、`--max-radius`、`--min-gap`、`--height` 和 `--height-variation`。生成器使用保守外接圆约束障碍间距，因此真实 Mesh 边界间隙不会小于记录的保证值。

## 导出到 Isaac Sim

```bash
cd /home/starry/isaac-data/Fov-gvf
bash scripts/export_blender_cloud_to_isaac.sh 42
```

该流程保留 `.blend` 母版，先输出 Blender 原始 USD，再生成带 `PhysicsScene`、地面碰撞和 14 个精确静态 Mesh 碰撞体的 `_isaac.usd`。`UAV_Start` 与 `UAV_Goal` 会作为 Xform 标记一并导出。
