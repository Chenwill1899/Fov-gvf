# 2026-09-28 验收复现

工作目录 `/home/starry/isaac-data/EGO1P1`。构建入口：

```bash
bash scripts/build_isaac_ros_workspace.sh
PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p1_isaac_build/pc_gvf --output-on-failure
source /opt/ros/humble/setup.bash
source /tmp/fov_gvf_ego1p1_isaac_install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest -q src/pc_gvf_platforms/test
ROS_DOMAIN_ID=83 PYTHONNOUSERSITE=1 /usr/bin/python3 tools/ros_vector_guidance_probe.py --paper
```

下列每一轮顺序单独运行（共享单实例锁，不要同时启动）：

```bash
ISAAC_MANUAL_INPUT_MODE=trace ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false ROS_DOMAIN_ID=84 \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_envelope_recovery_trace.json" \
ISAAC_ACCEPTANCE_TRACE="$PWD/performance/envelope_20260928/repeated_compact.csv" \
ISAAC_MANUAL_TIMEOUT=94 FOV_GVF_RUN_ID=envelope_repeated_compact \
FOV_GVF_PERFORMANCE_LOG="$PWD/performance/envelope_20260928/repeated_performance.md" \
bash scripts/run_isaac_fov_gvf_navigation.sh
```

- `final_compact`（中间候选）：build8，默认0.48机体+0.02安全+0.02执行，最低前视0.10m，94秒。
- `final_wide`（中间候选）：同build8，同输入与前视，只增加 `FOV_GVF_SAFETY_MARGIN=0.20 FOV_GVF_ROLLOUT_MARGIN=0.10`。
- `final_expiry`：同build8默认，fixture换为`isaac_evidence_expiry_trace.json`，99秒。
- `final_maxspeed`：同build8默认，fixture换为`isaac_max_speed_trace.json`，24秒。
- `final_no_seed`：同build8默认，fixture换为`isaac_strict_observation_trace.json`，6秒，增加`FOV_GVF_VERIFIED_START=0`。
- `baseline`：修改前快照中的控制器，0.25机体/0.20安全/0.10执行/1m前视，只增加相同CSV遥测，94秒。

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/analyze_envelope_trace.py performance/envelope_20260928/final_compact.csv
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/calibrate_motion_discretization.py
/home/starry/isaac-data/isaacsim/python.sh tools/calibrate_vehicle_envelope.py scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd
```

中间探索版不是当前默认：build1统一范围；build2修复空间样本；build3最初0.4s运动证明；
build4完整制动证明；build5原始命中AABB筛选；build6前视环境变量/前缀诊断；
build7部分额外范围保留；build8最终参数与短前缀制动回归。
中间试验参数和失败见项目`ENVELOPE_ACCEPTANCE.md`。未保存每次中间源码快照；
修改前快照、最终源码哈希、输入、原始日志和CSV保留。日志计时/渲染可能使重复轨迹不同，
单次对照不构成统计显著性结论。


`optimized_*`为build9：最后一次前视尝试强制使用实测速度制动需求/最低前视下限。
`union_*`为build10：另加相邻认证球的解析并集包含检查。两批各自沿用上表中的
compact/wide/expiry/maxspeed/no_seed输入、时长与参数；以`union_*`作为最终几何代码的仿真证据。
`source_stage9/`保存加入两球解析查询前的相关源码，便于区分中间结果。
