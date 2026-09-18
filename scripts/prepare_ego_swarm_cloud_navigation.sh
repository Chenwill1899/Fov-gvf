#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
ISAAC_PYTHON="/home/starry/isaac-data/isaacsim/python.sh"
SOURCE_ROOT="${EGO_SWARM_CLOUD_SOURCE_ROOT:-/home/starry/isaac-data/ego_swarm_cloud}"
SOURCE_USD="$SOURCE_ROOT/ego_swarm_cloud.usd"
SOURCE_OCCUPANCY="$SOURCE_ROOT/occupancy.bin"
OUTPUT_ROOT="$PROJECT_ROOT/scenes/ego_swarm_cloud"
OUTPUT_USD="$OUTPUT_ROOT/ego_swarm_cloud_navigation.usd"

for required in "$SOURCE_USD" "$SOURCE_OCCUPANCY"; do
  if [[ ! -f "$required" ]]; then
    echo "EGO-Swarm Cloud input does not exist: $required" >&2
    exit 1
  fi
done

mkdir -p "$OUTPUT_ROOT"
install -m 0644 "$SOURCE_OCCUPANCY" "$OUTPUT_ROOT/occupancy.bin"
"$ISAAC_PYTHON" "$PROJECT_ROOT/scripts/isaac/prepare_uav_navigation_scene.py" \
  "$SOURCE_USD" "$OUTPUT_USD" \
  --environment-scale=4.0 \
  --manual-spawn=-64.0,-11.2,2.8

echo "EGO-Swarm Cloud navigation scene ready: $OUTPUT_USD"
