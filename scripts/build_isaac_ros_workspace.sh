#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="/home/starry/isaac-data/EGO2P6"
source "$PROJECT_ROOT/scripts/environment.sh"
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export PYTHONNOUSERSITE=1
colcon --log-base /home/starry/isaac-data/EGO2P6/docs/performance/runtime/colcon_log build --base-paths "$PROJECT_ROOT/src" \
  --build-base /home/starry/isaac-data/EGO2P6/docs/performance/runtime/build \
  --install-base /home/starry/isaac-data/EGO2P6/docs/performance/runtime/install \
  --packages-up-to pc_gvf pc_gvf_platforms \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="${EGO2P6_BUILD_TESTING:-OFF}"
