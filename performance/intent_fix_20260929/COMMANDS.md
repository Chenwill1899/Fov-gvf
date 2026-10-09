# 本轮复现

在/home/starry/isaac-data/EGO1P1执行：

```bash
bash scripts/build_isaac_ros_workspace.sh
PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p1_isaac_build/pc_gvf --output-on-failure
source /opt/ros/humble/setup.bash
source /tmp/fov_gvf_ego1p1_isaac_install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest -q src/pc_gvf_platforms/test
ISAAC_MANUAL_INPUT_MODE=trace ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false ROS_DOMAIN_ID=84 \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_intent_response_trace.json" \
ISAAC_ACCEPTANCE_TRACE="$PWD/performance/intent_fix_20260929/reproduced.csv" \
ISAAC_MANUAL_TIMEOUT=84 FOV_GVF_RUN_ID=intent_reproduced \
FOV_GVF_REPLAY_DIR="$PWD/performance/intent_fix_20260929/replay_reproduced" \
FOV_GVF_PERFORMANCE_LOG="$PWD/performance/intent_fix_20260929/performance.md" \
bash scripts/run_isaac_fov_gvf_navigation.sh
/usr/bin/python3 tools/analyze_envelope_trace.py performance/intent_fix_20260929/reproduced.csv
/usr/bin/python3 tools/analyze_intent_response.py performance/intent_fix_20260929/reproduced.csv
/usr/bin/python3 tools/verify_navigation_replays.py performance/intent_fix_20260929/replay_reproduced
```

保存的paper_replay_final对应当前算法/V5；读取V3/V4是迁移读取，不等于旧算法逐值复现。
