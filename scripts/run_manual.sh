#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Refuse before creating runtime directories or starting a simulator.
/usr/bin/python3 -B "$PROJECT_ROOT/scripts/require_new_controller.py" >/dev/null
cd "$PROJECT_ROOT"
source scripts/manual_profile.env
source scripts/environment.sh
export ROS_DOMAIN_ID=42
export FOV_GVF_CONTROLLER_EXECUTABLE="$EGO4P0_CONTROLLER_EXECUTABLE"
export FOV_GVF_RUN_ID="p4_manual_$(date +%Y%m%d_%H%M%S)_$$"
run_dir="$EGO4P0_RUNTIME/manual_runs/$FOV_GVF_RUN_ID"
mkdir -p "$run_dir"
export FOV_GVF_PERFORMANCE_LOG="$run_dir/PERFORMANCE.md"
export ISAAC_VIEW_CAPTURE="$run_dir/viewport.png"
exec > >(tee -a "$run_dir/console.log") 2>&1
printf 'EGO4P0 仿真接口，控制器: %s\n日志: %s\n' "$EGO4P0_CONTROLLER_EXECUTABLE" "$run_dir"
exec bash scripts/run_isaac_fov_gvf_navigation.sh "$@"
