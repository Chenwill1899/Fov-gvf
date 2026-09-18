#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
BLENDER="/home/starry/isaac-data/blender/blender"
SEED="${1:-42}"
if (($# > 0)); then shift; fi
OUTPUT="$PROJECT_ROOT/scenes/blender_isolated_clouds/isolated_clouds_seed${SEED}.blend"

"$BLENDER" --background --python "$PROJECT_ROOT/scripts/blender/generate_isolated_cloud_scene.py" -- \
  --seed "$SEED" --output "$OUTPUT" "$@"
exec "$BLENDER" "$OUTPUT"
