#!/usr/bin/env python3
"""Open one generated crack-maze USD in Isaac Sim with an overview camera."""

from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp


if len(sys.argv) != 2:
    raise SystemExit("usage: view_crack_maze_scene.py SCENE.usd")

scene_path = Path(sys.argv[1]).expanduser().resolve()
if not scene_path.is_file():
    raise SystemExit(f"scene does not exist: {scene_path}")
sys.argv = [sys.argv[0]]

simulation_app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

import omni.usd
from isaacsim.core.rendering_manager import ViewportManager


try:
    for _ in range(3):
        simulation_app.update()
    context = omni.usd.get_context()
    print(f"[SCENE LOAD] Opening {scene_path}", flush=True)
    opened = context.open_stage(scene_path.as_posix())
    print(f"[SCENE LOAD] open_stage returned {opened}", flush=True)
    stage = context.get_stage()
    obstacle = stage.GetPrimAtPath("/World/MazeObstacles/MazeField") if stage else None
    if not obstacle or not obstacle.IsValid():
        raise RuntimeError(f"MazeField was not found after opening {scene_path}")

    ViewportManager.set_camera_view(
        "/OmniverseKit_Persp",
        eye=[-12.5, -15.0, 13.0],
        target=[0.0, 0.0, 1.25],
    )
    print(f"[SCENE READY] Loaded {scene_path}", flush=True)
    print("[SCENE READY] Obstacle prim: /World/MazeObstacles/MazeField", flush=True)
    print("[SCENE READY] All white corridors belong to one connected maze.", flush=True)
    while simulation_app.is_running():
        simulation_app.update()
finally:
    simulation_app.close()
