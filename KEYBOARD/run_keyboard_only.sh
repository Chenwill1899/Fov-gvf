#!/usr/bin/env bash
set -euo pipefail

echo "[COMPAT] run_keyboard_only.sh now starts the Mode-2 joystick controller." >&2
exec bash /home/starry/isaac-data/EGO1P0/KEYBOARD/run_joystick_only.sh "$@"
