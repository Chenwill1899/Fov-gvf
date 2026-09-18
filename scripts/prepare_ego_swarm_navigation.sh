#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
ISAAC_PYTHON="/home/starry/isaac-data/isaacsim/python.sh"
SOURCE="${EGO_SWARM_SOURCE:-/home/starry/isaac-data/ego_swarm_blender/ego_swarm.usd}"
OUTPUT="${EGO_SWARM_NAV_SCENE:-$PROJECT_ROOT/scenes/ego_swarm/ego_swarm_navigation.usd}"

if [[ ! -f "$SOURCE" ]]; then
  echo "EGO-Swarm Blender USD does not exist: $SOURCE" >&2
  exit 1
fi

"$ISAAC_PYTHON" "$PROJECT_ROOT/scripts/isaac/prepare_uav_navigation_scene.py" \
  "$SOURCE" "$OUTPUT" \
  --start=-30.0,0.0,1.2 \
  --goal=25.0,4.0,1.2

echo "EGO-Swarm navigation scene ready: $OUTPUT"
