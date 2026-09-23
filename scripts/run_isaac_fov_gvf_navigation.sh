#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P1"
ISAAC_ROOT="/home/starry/isaac-data/isaacsim"
HUMBLE_BRIDGE_LIB="$ISAAC_ROOT/exts/isaacsim.ros2.core/humble/lib"
ROS_INSTALL="${FOV_GVF_INSTALL:-/tmp/fov_gvf_ego1p1_isaac_install}"
INPUT_MODE="${ISAAC_MANUAL_INPUT_MODE:-joystick}"
case "$INPUT_MODE" in
  joystick)
    # shellcheck source=scripts/lib/beitong_joystick.sh
    source "$PROJECT_ROOT/scripts/lib/beitong_joystick.sh"
    if ! resolve_beitong_joystick; then
      echo "For the legacy keyboard input, set ISAAC_MANUAL_INPUT_MODE=keyboard." >&2
      exit 1
    fi
    ;;
  keyboard)
    ;;
  *)
    echo "Unknown ISAAC_MANUAL_INPUT_MODE=$INPUT_MODE (expected joystick or keyboard)" >&2
    exit 1
    ;;
esac
export ISAAC_MANUAL_INPUT_MODE="$INPUT_MODE"
SCENE_MODE="${FOV_GVF_SCENE_MODE:-cloud}"
case "$SCENE_MODE" in
  flat)
    DEFAULT_SCENE="$PROJECT_ROOT/scenes/flat_ground/flat_ground_navigation.usd"
    ;;
  cloud)
    DEFAULT_SCENE="$PROJECT_ROOT/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd"
    ;;
  *)
    echo "Unknown FOV_GVF_SCENE_MODE=$SCENE_MODE (expected flat or cloud)" >&2
    exit 1
    ;;
esac
SCENE="${FOV_GVF_SCENE:-$DEFAULT_SCENE}"

if [[ ! -f "$ROS_INSTALL/setup.bash" ]]; then
  echo "ROS install not found: $ROS_INSTALL/setup.bash" >&2
  echo "Build first with scripts/build_isaac_ros_workspace.sh" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$ROS_INSTALL/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1
export ISAAC_HEADLESS="${ISAAC_HEADLESS:-0}"
export FOV_GVF_RVIZ="${FOV_GVF_RVIZ:-true}"
if [[ -z "${FOV_GVF_ESDF_OCCUPANCY+x}" ]]; then
  SCENE_OCCUPANCY="$(dirname "$SCENE")/occupancy.bin"
  if [[ -f "$SCENE_OCCUPANCY" ]]; then
    export FOV_GVF_ESDF_OCCUPANCY="$SCENE_OCCUPANCY"
  else
    unset FOV_GVF_ESDF_OCCUPANCY
  fi
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/fov_gvf_ego1p1_isaac_ros_log}"
export FOV_GVF_RUN_ID="${FOV_GVF_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export FOV_GVF_PERFORMANCE_LOG="${FOV_GVF_PERFORMANCE_LOG:-$PROJECT_ROOT/performance/PERFORMANCE_METRICS.md}"
mkdir -p "$ROS_LOG_DIR"
mkdir -p "$(dirname "$FOV_GVF_PERFORMANCE_LOG")"
printf '\n### Run %s\n\n- Started: %s\n- Scene: `%s`\n- ROS domain: `%s`\n- Manual input: `%s`\n' \
  "$FOV_GVF_RUN_ID" "$(date '+%Y-%m-%d %H:%M:%S %Z')" "$SCENE" "$ROS_DOMAIN_ID" \
  "$ISAAC_MANUAL_INPUT_MODE" \
  >> "$FOV_GVF_PERFORMANCE_LOG"
echo "[PERF] run=$FOV_GVF_RUN_ID log=$FOV_GVF_PERFORMANCE_LOG"
echo "[INPUT] mode=$ISAAC_MANUAL_INPUT_MODE scene_mode=$SCENE_MODE"

LOCK_FILE="/tmp/fov_gvf_user_navigation.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "USER manual navigation is already running (lock: $LOCK_FILE)" >&2
  exit 1
fi

position_cmd_publisher_count() {
  local topic_info publisher_count
  topic_info="$(timeout 3s ros2 topic info /position_cmd 2>/dev/null || true)"
  publisher_count="$(sed -n 's/^Publisher count: //p' <<<"$topic_info")"
  echo "${publisher_count:-0}"
}

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${ROS_LAUNCH_PID:-}" ]] && kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$ROS_LAUNCH_PID" 2>/dev/null || true
    for _cleanup_attempt in {1..30}; do
      if ! kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$ROS_LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$ROS_LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

existing_publishers="$(position_cmd_publisher_count)"
if (( existing_publishers != 0 )); then
  echo "Refusing to start: /position_cmd already has $existing_publishers publisher(s) in ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >&2
  ros2 topic info /position_cmd --verbose || true
  ros2 node list || true
  exit 1
fi

setsid ros2 launch pc_gvf isaac_cloud_navigation.launch.py rviz:="$FOV_GVF_RVIZ" &
ROS_LAUNCH_PID=$!
publisher_count=0
for _startup_attempt in {1..20}; do
  sleep 0.25
  publisher_count="$(position_cmd_publisher_count)"
  if (( publisher_count == 1 || publisher_count > 1 )); then
    break
  fi
done
if (( publisher_count != 1 )); then
  echo "Startup check failed: expected one /position_cmd publisher, found $publisher_count" >&2
  ros2 topic info /position_cmd --verbose || true
  ros2 node list || true
  exit 1
fi
echo "[ROS CHECK] ROS_DOMAIN_ID=$ROS_DOMAIN_ID /position_cmd publishers=1"
ros2 topic info /position_cmd
ros2 node list
env \
  -u PYTHONPATH \
  -u OLD_PYTHONPATH \
  -u AMENT_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  ROS_DISTRO=humble \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="$HUMBLE_BRIDGE_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  PYTHONNOUSERSITE=1 \
  "$ISAAC_ROOT/python.sh" \
  "$PROJECT_ROOT/scripts/isaac/run_fov_gvf_navigation.py" "$SCENE" "$@"
