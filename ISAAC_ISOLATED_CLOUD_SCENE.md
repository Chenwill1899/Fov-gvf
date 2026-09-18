# 独立不规则云团柱体场景

此方法与原来的云状随机场不同：每个黑色障碍是一个独立的不规则云团，任意两团之间强制保留 `--min-gap` 的黑色边界间距；白色区域是无人机可穿越空间。默认每张地图有 14 团云，最小间距 `0.90 m`。生成器只保护起终点区域，不会人工挖出明显的空白通道；它会验证带安全净空的自由空间确实连通。

```bash
cd /home/starry/isaac-data/Fov-gvf
bash scripts/run_isolated_cloud_scene.sh 42
```

只生成 USD：

```bash
/home/starry/isaac-data/isaacsim/python.sh scripts/isaac/generate_isolated_cloud_scene.py \
  --seed 103 --output scenes/isolated_clouds/isolated_clouds_seed103.usd
```

可调参数：`--cloud-count`、`--min-radius`、`--max-radius`、`--min-gap` 和 `--endpoint-radius`。如果无人机外接半径加安全余量大于间隙的一半，应增大 `--min-gap`，必要时减少 `--cloud-count`。
