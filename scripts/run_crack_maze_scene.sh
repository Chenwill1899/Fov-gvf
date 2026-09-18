#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
ISAAC_SIM_ROOT="/home/starry/isaac-data/isaacsim"
GENERATOR="$PROJECT_ROOT/scripts/isaac/generate_crack_maze_scene.py"
VIEWER="$PROJECT_ROOT/scripts/isaac/view_crack_maze_scene.py"
SEED="${1:-42}"
if (($# > 0)); then
    shift
fi
SCENE="$PROJECT_ROOT/scenes/crack_maze/crack_maze_seed${SEED}.usd"

"$ISAAC_SIM_ROOT/python.sh" "$GENERATOR" --seed "$SEED" --output "$SCENE" "$@"
exec "$ISAAC_SIM_ROOT/python.sh" "$VIEWER" "$SCENE"
