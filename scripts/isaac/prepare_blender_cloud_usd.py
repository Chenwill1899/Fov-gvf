#!/usr/bin/env python3
"""Add Isaac Sim physics and explicit UAV marker metadata to a Blender USD."""
from __future__ import annotations
import argparse
from pathlib import Path
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("input", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
source = args.input.expanduser().resolve()
target = args.output.expanduser().resolve()
if not source.is_file():
    raise SystemExit(f"input USD does not exist: {source}")
stage = Usd.Stage.Open(str(source))
if stage is None:
    raise RuntimeError(f"could not open {source}")
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

obstacle_count = 0
ground_count = 0
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    path = str(prim.GetPath())
    is_obstacle = "/Cloud_" in path and path.endswith("_Mesh")
    is_ground = path == "/World/Ground/Cube"
    if not (is_obstacle or is_ground):
        continue
    UsdPhysics.CollisionAPI.Apply(prim)
    collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
    collision.CreateApproximationAttr().Set("none")
    prim.CreateAttribute("fov:collisionRole", Sdf.ValueTypeNames.String).Set(
        "static_obstacle" if is_obstacle else "static_ground"
    )
    obstacle_count += int(is_obstacle)
    ground_count += int(is_ground)

for path, role in (("/World/UAV_Start", "uav_start"), ("/World/UAV_Goal", "uav_goal")):
    marker = stage.GetPrimAtPath(path)
    if not marker.IsValid():
        raise RuntimeError(f"required marker missing: {path}")
    marker.CreateAttribute("fov:role", Sdf.ValueTypeNames.String).Set(role)

if obstacle_count != 14 or ground_count != 1:
    raise RuntimeError(f"expected 14 obstacles and one ground, got {obstacle_count} and {ground_count}")
target.parent.mkdir(parents=True, exist_ok=True)
stage.GetRootLayer().Export(str(target))
print(f"[ISAAC USD] Wrote {target}")
print(f"[ISAAC USD] Static obstacle meshes: {obstacle_count}; ground meshes: {ground_count}")
