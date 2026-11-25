#!/usr/bin/env bash
# Local runtime artifacts are generated under docs/performance/runtime.
export EGO4P2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Legacy environment names are adapter aliases, always pointing at P2.
export EGO4P0_ROOT="$EGO4P2_ROOT"
export EGO4P2_RUNTIME="$EGO4P2_ROOT/docs/performance/runtime"
export EGO4P0_RUNTIME="$EGO4P2_RUNTIME"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
EGO4P0_CACHE_BASE="${EGO4P2_CACHE_ROOT:-$EGO4P2_RUNTIME}"
export XDG_CACHE_HOME="$EGO4P0_CACHE_BASE/cache" XDG_CONFIG_HOME="$EGO4P0_CACHE_BASE/config" XDG_DATA_HOME="$EGO4P0_CACHE_BASE/data"
export CUDA_CACHE_PATH="$EGO4P0_CACHE_BASE/cuda" __GL_SHADER_DISK_CACHE_PATH="$EGO4P0_CACHE_BASE/shader" WARP_CACHE_PATH="$EGO4P0_CACHE_BASE/warp"
export TMPDIR="$EGO4P0_CACHE_BASE/tmp_current" ROS_LOG_DIR="$EGO4P0_CACHE_BASE/ros_log" ROS_HOME="$EGO4P0_CACHE_BASE/ros_home" ROS2CLI_NO_DAEMON=1
export FOV_GVF_INSTALL="$EGO4P0_RUNTIME/install"
mkdir -p "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME" "$XDG_DATA_HOME" "$CUDA_CACHE_PATH" "$__GL_SHADER_DISK_CACHE_PATH" "$WARP_CACHE_PATH" "$TMPDIR" "$ROS_LOG_DIR" "$ROS_HOME"
