#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/scripts/environment.sh"
/usr/bin/python3 -B "$PROJECT_ROOT/scripts/base_config.py" --check
/usr/bin/python3 -I "$PROJECT_ROOT/scripts/navigation_config.py" --check
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export PYTHONNOUSERSITE=1
colcon --log-base "$EGO4P2_RUNTIME/colcon_log" build --base-paths "$PROJECT_ROOT/src" \
  --build-base "$EGO4P2_RUNTIME/build" \
  --install-base "$EGO4P2_RUNTIME/install" \
  --packages-up-to pc_gvf pc_gvf_platforms new_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="${EGO4P2_BUILD_TESTING:-${EGO4P0_BUILD_TESTING:-OFF}}"
