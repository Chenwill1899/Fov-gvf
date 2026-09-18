#!/usr/bin/env python3
"""Open a separated-cloud USD scene in Isaac Sim."""
from __future__ import annotations
import sys
from pathlib import Path
from isaacsim import SimulationApp

if len(sys.argv) != 2:
    raise SystemExit("usage: view_isolated_cloud_scene.py SCENE.usd")
scene_path = Path(sys.argv[1]).expanduser().resolve()
if not scene_path.is_file():
    raise SystemExit(f"scene does not exist: {scene_path}")
sys.argv = [sys.argv[0]]
simulation_app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})
import omni.usd
from isaacsim.core.rendering_manager import ViewportManager
try:
    for _ in range(3): simulation_app.update()
    context = omni.usd.get_context()
    print(f"[SCENE LOAD] Opening {scene_path}", flush=True)
    print(f"[SCENE LOAD] open_stage returned {context.open_stage(scene_path.as_posix())}", flush=True)
    stage = context.get_stage(); obstacle = stage.GetPrimAtPath("/World/CloudObstacles/CloudField") if stage else None
    if not obstacle or not obstacle.IsValid(): raise RuntimeError("CloudField was not found")
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[-12.5, -15.0, 13.0], target=[0.0, 0.0, 1.25])
    print(f"[SCENE READY] Loaded {scene_path}", flush=True)
    print("[SCENE READY] Separate irregular cloud pillars; white gaps are vehicle-free space.", flush=True)
    while simulation_app.is_running(): simulation_app.update()
finally:
    simulation_app.close()
