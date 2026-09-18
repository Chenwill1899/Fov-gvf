#!/usr/bin/env python3
"""Open the Blender-authored cloud scene in Isaac Sim."""
from __future__ import annotations
import sys
from pathlib import Path
from isaacsim import SimulationApp

if len(sys.argv) != 2:
    raise SystemExit("usage: view_blender_cloud_scene.py SCENE.usd")
scene_path = Path(sys.argv[1]).expanduser().resolve()
if not scene_path.is_file():
    raise SystemExit(f"scene does not exist: {scene_path}")
sys.argv = [sys.argv[0]]
app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})
import omni.usd
from isaacsim.core.rendering_manager import ViewportManager
try:
    for _ in range(3): app.update()
    context = omni.usd.get_context()
    print(f"[SCENE LOAD] Opening {scene_path}", flush=True)
    opened = context.open_stage(scene_path.as_posix())
    print(f"[SCENE LOAD] open_stage returned {opened}", flush=True)
    stage = context.get_stage()
    obstacles = [p for p in stage.Traverse() if p.GetTypeName() == "Mesh" and "/Cloud_" in str(p.GetPath())]
    if len(obstacles) != 14:
        raise RuntimeError(f"expected 14 cloud meshes, got {len(obstacles)}")
    for marker in ("/World/UAV_Start", "/World/UAV_Goal"):
        if not stage.GetPrimAtPath(marker).IsValid(): raise RuntimeError(f"missing {marker}")
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[-12.5, -15.0, 13.0], target=[0.0, 0.0, 1.25])
    print("[SCENE READY] Blender cloud scene loaded with 14 obstacles and UAV start/goal markers.", flush=True)
    while app.is_running(): app.update()
finally:
    app.close()
