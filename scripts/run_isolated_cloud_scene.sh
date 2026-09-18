#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
ISAAC_SIM_ROOT="/home/starry/isaac-data/isaacsim"
SEED="${1:-42}"; if (($# > 0)); then shift; fi
SCENE="$PROJECT_ROOT/scenes/isolated_clouds/isolated_clouds_seed${SEED}.usd"
"$ISAAC_SIM_ROOT/python.sh" "$PROJECT_ROOT/scripts/isaac/generate_isolated_cloud_scene.py" --seed "$SEED" --output "$SCENE" "$@"
exec "$ISAAC_SIM_ROOT/python.sh" "$PROJECT_ROOT/scripts/isaac/view_isolated_cloud_scene.py" "$SCENE"
