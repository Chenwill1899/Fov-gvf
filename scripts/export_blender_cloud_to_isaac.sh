#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
BLENDER="/home/starry/isaac-data/blender/blender"
ISAAC_PYTHON="/home/starry/isaac-data/isaacsim/python.sh"
SEED="${1:-42}"
BLEND="$PROJECT_ROOT/scenes/blender_isolated_clouds/isolated_clouds_seed${SEED}.blend"
RAW_USD="$PROJECT_ROOT/scenes/blender_isolated_clouds/isolated_clouds_seed${SEED}_blender.usd"
ISAAC_USD="$PROJECT_ROOT/scenes/blender_isolated_clouds/isolated_clouds_seed${SEED}_isaac.usd"
NAVIGATION_USD="$PROJECT_ROOT/scenes/blender_isolated_clouds/isolated_clouds_seed${SEED}_navigation.usd"

if [[ ! -f "$BLEND" ]]; then
  echo "Blender scene does not exist: $BLEND" >&2
  exit 1
fi
"$BLENDER" --background "$BLEND" --python "$PROJECT_ROOT/scripts/blender/export_scene_to_usd.py" -- --output "$RAW_USD"
"$ISAAC_PYTHON" "$PROJECT_ROOT/scripts/isaac/prepare_blender_cloud_usd.py" "$RAW_USD" "$ISAAC_USD"
"$ISAAC_PYTHON" "$PROJECT_ROOT/scripts/isaac/prepare_uav_navigation_scene.py" "$ISAAC_USD" "$NAVIGATION_USD"
echo "Isaac Sim environment ready: $ISAAC_USD"
echo "Isaac Sim navigation scene ready: $NAVIGATION_USD"
