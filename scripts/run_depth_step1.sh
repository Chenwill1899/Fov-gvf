#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/scripts/environment.sh"
source /opt/ros/humble/setup.bash
source "$FOV_GVF_INSTALL/setup.bash"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec /usr/bin/python3 -B "$PROJECT_ROOT/scripts/run_depth_step1.py" "$@"
