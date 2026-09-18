#!/usr/bin/env bash
set -euo pipefail

JOYSTICK_ROOT="/home/starry/isaac-data/EGO1P0/KEYBOARD"
ISAAC_ROOT="/home/starry/isaac-data/isaacsim"
SCENE="${JOYSTICK_SCENE:-${KEYBOARD_SCENE:-/home/starry/isaac-data/EGO1P0/scenes/flat_ground/flat_ground_navigation.usd}}"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick}"

if [[ ! -f "$SCENE" ]]; then
  echo "Joystick test scene not found: $SCENE" >&2
  echo "Generate it with: bash ../scripts/prepare_flat_ground_navigation.sh" >&2
  exit 1
fi

if [[ ! -r "$JOYSTICK_DEVICE" ]]; then
  echo "Joystick is not readable: $JOYSTICK_DEVICE" >&2
  echo "Connect the BEITONG USB controller or set JOYSTICK_DEVICE." >&2
  exit 1
fi

exec env \
  -u PYTHONPATH \
  -u OLD_PYTHONPATH \
  -u AMENT_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  PYTHONNOUSERSITE=1 \
  JOYSTICK_DEVICE="$JOYSTICK_DEVICE" \
  "$ISAAC_ROOT/python.sh" "$JOYSTICK_ROOT/keyboard_only.py" "$SCENE" "$@"
