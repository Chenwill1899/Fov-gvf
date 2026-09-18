#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
ISAAC_SIM_ROOT="/home/starry/isaac-data/isaacsim"
GENERATOR="$PROJECT_ROOT/scripts/isaac/generate_cloud_pillar_scene.py"
VIEWER="$PROJECT_ROOT/scripts/isaac/view_cloud_pillar_scene.py"
SEED="${1:-42}"
if (($# > 0)); then
    shift
fi
SCENE="$PROJECT_ROOT/scenes/cloud_pillars/cloud_pillars_seed${SEED}.usd"

# USD generation uses Isaac Sim's bundled Python/OpenUSD but does not start Kit,
# so it also works in a terminal without an active RTX renderer.
"$ISAAC_SIM_ROOT/python.sh" "$GENERATOR" --seed "$SEED" --output "$SCENE" "$@"

# Open the stage explicitly and select a reproducible overview camera.  This
# lightweight standalone viewer does not enable the ROS bridge.
exec "$ISAAC_SIM_ROOT/python.sh" "$VIEWER" "$SCENE"
