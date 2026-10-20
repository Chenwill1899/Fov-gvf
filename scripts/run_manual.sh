#!/usr/bin/env bash
# EGO2P6 local RTX5070 / ROS Humble / physical BEITONG controller.
set -eo pipefail
cd /home/starry/isaac-data/EGO2P6
source scripts/manual_profile.env
source scripts/environment.sh
export ROS_DOMAIN_ID=42
export FOV_GVF_GOAL_CACHE_DISPLACEMENT=.75
export EGO5090_CONTROLLER_CPU_AFFINITY="${EGO2P6_CONTROLLER_CPU_AFFINITY:-}"
export FOV_GVF_RUN_ID="p6_manual_$(date +%Y%m%d_%H%M%S)_$$"
run_dir="$EGO2P6_RUNTIME/manual_runs/$FOV_GVF_RUN_ID"
mkdir -p "$run_dir"
export FOV_GVF_RUNTIME_MANIFEST="$run_dir/runtime_manifest.json"
export FOV_GVF_PERFORMANCE_LOG="$run_dir/PERFORMANCE.md"
# Full binary diagnostic snapshots are opt-in; they are not execution inputs.
if [[ "${EGO2P6_RECORD_REPLAYS:-0}" == 1 ]]; then
  export FOV_GVF_REPLAY_DIR="$run_dir/replays"
else
  export FOV_GVF_REPLAY_DIR=""
fi
export ISAAC_VIEW_CAPTURE="$run_dir/viewport.png"
printf '%s\n' "$run_dir" > "$EGO2P6_RUNTIME/latest_manual_run.txt"
exec > >(tee -a "$run_dir/console.log") 2>&1
printf '\nEGO2P6 手柄仿真（本机 Humble 构建；当前版本与验收范围见 docs/EGO2P6_VERSION.md）\n'
printf '右杆：前后左右；左杆：升降/偏航。四轴回中 0.5 秒解锁。\n'
printf '水平上限 10m/s，垂向 1m/s；近障与探索自动减速。\n'
printf 'Isaac窗口按 1/3/V 切换视角；关闭Isaac窗口或终端按 Ctrl+C 退出。\n日志：%s\n\n' "$run_dir"
exec bash scripts/run_isaac_fov_gvf_navigation.sh "$@"
