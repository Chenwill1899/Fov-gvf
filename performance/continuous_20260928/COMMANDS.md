# 连续通道与恢复验收复现

工作目录：`/home/starry/isaac-data/EGO1P1`。所有仿真顺序运行，使用已有单实例锁。

```bash
bash scripts/build_isaac_ros_workspace.sh
PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p1_isaac_build/pc_gvf --output-on-failure
source /opt/ros/humble/setup.bash
source /tmp/fov_gvf_ego1p1_isaac_install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest -q src/pc_gvf_platforms/test
ROS_DOMAIN_ID=83 PYTHONNOUSERSITE=1 /usr/bin/python3 tools/ros_vector_guidance_probe.py --paper
```

最终组的94秒改向输入：

```bash
ISAAC_MANUAL_INPUT_MODE=trace ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false ROS_DOMAIN_ID=84 \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_envelope_recovery_trace.json" \
ISAAC_ACCEPTANCE_TRACE="$PWD/performance/continuous_20260928/reproduced.csv" \
ISAAC_MANUAL_TIMEOUT=94 FOV_GVF_RUN_ID=continuous_reproduced \
FOV_GVF_REPLAY_DIR="$PWD/performance/continuous_20260928/replay_reproduced" \
FOV_GVF_PERFORMANCE_LOG="$PWD/performance/continuous_20260928/performance.md" \
bash scripts/run_isaac_fov_gvf_navigation.sh
/usr/bin/python3 tools/analyze_envelope_trace.py performance/continuous_20260928/reproduced.csv
/usr/bin/python3 tools/verify_navigation_replays.py performance/continuous_20260928/replay_reproduced
```

- `continuous14`、`repeat14`：相同最终算法、场景、参数及94秒改向输入，两次实际运行。
- `expiry14`：改用`isaac_evidence_expiry_trace.json`，99秒；松手67秒后改向10秒。
- `maxspeed14`：改用`isaac_max_speed_trace.json`，24秒；2m/s输入。
- `no_seed14`：改用`isaac_strict_observation_trace.json`，6秒，并设置`FOV_GVF_VERIFIED_START=0`。

旧`continuous5/6/8/9/11`及`expiry9`均为中间版本，不能当作最终验收。
`paper_replay_stage8`和`paper_replay_stage11`保留对应阶段的单帧重放程序。
最终源码/场景/输入哈希另存`SOURCE_SHA256.txt`；构建及回归日志按阶段保留。
原用户16:15运行未记录完整深度/控制内部状态，因此不能声称已逐帧重放截图中的原始位置。
本轮重放验证的是新捕获的完整控制状态，不包含渲染调度或墙钟超时。
