# Isaac Sim 连通裂隙迷宫场景

## 与云状障碍生成器的区别

原有 `generate_cloud_pillar_scene.py` 保持不变，它在白底上生成随机黑色云状障碍，并额外清出一条保证通道。

新的 `generate_crack_maze_scene.py` 采用相反过程：先令整个地图为黑色障碍，再用偏向长路径、同时随机回到旧节点生长分岔的迷宫算法生成一棵生成树，只沿树的节点和边挖出白色飞行通道。这样所有道路天然属于同一连通网络，不会产生藏在障碍物中的独立白色空洞。

道路具有随机宽度扰动和路口位置扰动。生成树会形成分岔、死胡同以及连续急转弯；树直径的两个端点自动作为起点和终点。生成结束后还会再次检查：

- 全部白色区域只有一个八邻域连通分量；
- 起点和终点在满足 `--min-clearance` 净空要求的区域内连通；
- 黑色部分已拉伸为带精确静态 Mesh 碰撞的三维障碍。

## 生成并打开

```bash
cd /home/starry/isaac-data/Fov-gvf
bash scripts/run_crack_maze_scene.sh 42
```

更换随机种子：

```bash
bash scripts/run_crack_maze_scene.sh 103
```

只生成、不打开 Isaac Sim：

```bash
/home/starry/isaac-data/isaacsim/python.sh \
  scripts/isaac/generate_crack_maze_scene.py \
  --seed 103 \
  --output scenes/crack_maze/crack_maze_seed103.usd
```

## 主要参数

- `--maze-columns`、`--maze-rows`：迷宫路口密度；
- `--corridor-width`：通道完整宽度，默认 `1.05 m`；
- `--width-variation`：不同道路的宽度变化比例；
- `--node-jitter`：路口偏离规则网格的程度；
- `--junction-radius`：分岔点及急转弯处的回旋空间；
- `--min-clearance`：起终点连通验证所要求的最小障碍净空；
- `--height`、`--height-variation`：黑色障碍柱体高度及变化。

无人机的外接半径、安全余量和跟踪误差之和必须小于 `--min-clearance`。闭环算法只能读取仿真深度相机，不得读取 `_map.png` 或 manifest 中的完整地图和迷宫拓扑。
