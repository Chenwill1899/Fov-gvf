#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/starry/isaac-data/EGO1P0"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick}"

if [[ ! -r "$JOYSTICK_DEVICE" ]]; then
  echo "Joystick is not readable: $JOYSTICK_DEVICE" >&2
  echo "Connect the BEITONG USB controller or set JOYSTICK_DEVICE." >&2
  exit 1
fi

export ISAAC_MANUAL_INPUT_MODE=joystick
export JOYSTICK_DEVICE
export FOV_GVF_SCENE_MODE="${FOV_GVF_SCENE_MODE:-cloud}"

exec bash "$PROJECT_ROOT/scripts/run_isaac_fov_gvf_navigation.sh" "$@"
