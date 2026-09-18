#!/usr/bin/env python3
"""Create a simple Z-up, metre-scale flat-ground USD for control testing."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path)
parser.add_argument("--half-extent", type=float, default=100.0)
args = parser.parse_args()

if not math.isfinite(args.half_extent) or args.half_extent <= 1.0:
    raise SystemExit("--half-extent must be a finite value greater than 1 metre")

target = args.output.resolve()
target.parent.mkdir(parents=True, exist_ok=True)
stage = Usd.Stage.CreateNew(str(target))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

extent = float(args.half_extent)
ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
ground.CreatePointsAttr([
    Gf.Vec3f(-extent, -extent, 0.0),
    Gf.Vec3f(extent, -extent, 0.0),
    Gf.Vec3f(extent, extent, 0.0),
    Gf.Vec3f(-extent, extent, 0.0),
])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
ground.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
ground.CreateDoubleSidedAttr(True)
ground.CreateExtentAttr([
    Gf.Vec3f(-extent, -extent, 0.0),
    Gf.Vec3f(extent, extent, 0.0),
])
ground.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.36, 0.40)])
ground.GetPrim().CreateAttribute(
    "fov:collisionRole", Sdf.ValueTypeNames.String).Set("static_ground")

stage.GetRootLayer().Save()
print(f"[FLAT SCENE] Wrote {target}; ground={2.0 * extent:.1f} x "
      f"{2.0 * extent:.1f} m at z=0")
