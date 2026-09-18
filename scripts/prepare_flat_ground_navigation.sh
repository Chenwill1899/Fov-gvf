#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
ISAAC_PYTHON="/home/starry/isaac-data/isaacsim/python.sh"
OUTPUT_ROOT="$PROJECT_ROOT/scenes/flat_ground"
BASE_USD="$OUTPUT_ROOT/flat_ground.usd"
OUTPUT_USD="$OUTPUT_ROOT/flat_ground_navigation.usd"

mkdir -p "$OUTPUT_ROOT"
"$ISAAC_PYTHON" "$PROJECT_ROOT/scripts/isaac/create_flat_ground_scene.py" \
  "$BASE_USD" --half-extent=100.0
"$ISAAC_PYTHON" "$PROJECT_ROOT/scripts/isaac/prepare_uav_navigation_scene.py" \
  "$BASE_USD" "$OUTPUT_USD" \
  --environment-scale=1.0 \
  --manual-spawn=0.0,0.0,1.5

echo "Flat-ground manual-control scene ready: $OUTPUT_USD"
