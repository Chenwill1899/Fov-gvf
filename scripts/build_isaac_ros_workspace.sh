#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
source /opt/ros/humble/setup.bash
set -u
export PYTHONNOUSERSITE=1
colcon --log-base /tmp/fov_gvf_ego1p0_isaac_log build --base-paths "$PROJECT_ROOT/src" \
  --build-base /tmp/fov_gvf_ego1p0_isaac_build \
  --install-base /tmp/fov_gvf_ego1p0_isaac_install \
  --packages-up-to pc_gvf pc_gvf_platforms \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
