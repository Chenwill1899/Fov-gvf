#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/starry/isaac-data/EGO2P6
cd "$ROOT"
source scripts/manual_profile.env
# Never inherit the server i9 CPU mask on this 12-thread Ryzen.
export EGO5090_CONTROLLER_CPU_AFFINITY="${EGO2P6_CONTROLLER_CPU_AFFINITY:-}"
source scripts/environment.sh
source /opt/ros/humble/setup.bash
source "$FOV_GVF_INSTALL/setup.bash"
set -u
export ISAAC_PLANT_ACCEL=16 ISAAC_PLANT_TAU=.10 ISAAC_COMMAND_FRAME=world
export FOV_GVF_EXECUTION_CONSISTENT=1 FOV_GVF_ASSISTED_VERTICAL=1 FOV_GVF_BOUNDED_EXPLORATION=1
export FOV_GVF_TRACKING_MARGIN=.015
export ROS_DOMAIN_ID=42
export EGO5090_MANUAL_POLICY="${EGO5090_MANUAL_POLICY:-1}"
if [[ "${EGO5090_MANUAL_POLICY:-0}" == 1 ]]; then
  export FOV_GVF_CONTROLLER_EXECUTABLE="$FOV_GVF_INSTALL/pc_gvf/lib/pc_gvf/depth_angular_controller"
else
  export FOV_GVF_CONTROLLER_EXECUTABLE="$FOV_GVF_INSTALL/pc_gvf/lib/pc_gvf/depth_angular_controller_goal"
fi
# Preserve the saved manual algorithm/profile; replace physical input by goal intent.
export FOV_GVF_HISTORY_MAX_OBSERVATIONS="${EGO5090_HISTORY_MAX_OBSERVATIONS:-16}"
label="${1:-local_p6_goal_$(date +%Y%m%d_%H%M%S)}"
mkdir -p docs/performance/p6_speed_20261019/local_ubuntu
exec /usr/bin/python3 -B scripts/run_route.py "$label" \
  --out "$ROOT/docs/performance/p6_speed_20261019/local_ubuntu" \
  --maps original --repeats 1 --speed 10 --depth 20 --horizon .5 \
  --cache --cache-ttl .3 --restart-clearance .02 --pipeline-delay "${FOV_GVF_PIPELINE_DELAY:-.20}" \
  --abort-stall "${EGO5090_ABORT_STALL_S:-60}"
