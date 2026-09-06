#!/usr/bin/env bash
readonly WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
set -Eeuo pipefail
colcon --log-base "${WORKSPACE}/log" build \
  --base-paths "${WORKSPACE}/src" \
  --build-base "${WORKSPACE}/build" \
  --install-base "${WORKSPACE}/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
