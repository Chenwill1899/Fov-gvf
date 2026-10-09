#!/usr/bin/env python3
"""Read-only USD geometry audit; run with the existing Isaac Python runtime."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from pxr import Usd, UsdGeom, Gf

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('scene',type=Path)
args=parser.parse_args()
stage=Usd.Stage.Open(str(args.scene));robot=stage.GetPrimAtPath('/World/Robot')
if not robot:raise SystemExit('Missing /World/Robot')
xforms=UsdGeom.XformCache();bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
parts=[]
for prim in Usd.PrimRange(robot):
    if not prim.IsA(UsdGeom.Boundable) or prim.IsA(UsdGeom.Camera):continue
    transform,_=xforms.ComputeRelativeTransform(prim,robot)
    center=transform.Transform(Gf.Vec3d(0))
    identity=all(abs(transform[i][j]-(1 if i==j else 0))<1e-10 for i in range(3) for j in range(3))
    if prim.IsA(UsdGeom.Cylinder) and identity and UsdGeom.Cylinder(prim).GetAxisAttr().Get()=='Z':
        cylinder=UsdGeom.Cylinder(prim);r=cylinder.GetRadiusAttr().Get();h=cylinder.GetHeightAttr().Get()/2
        radial=math.hypot(center[0],center[1])+r
        radius=math.hypot(radial,abs(center[2])+h)
        method='exact Z-cylinder support'
    else:
        box=bounds.ComputeRelativeBound(prim,robot).ComputeAlignedRange()
        radius=max(math.sqrt(sum(float(x)**2 for x in box.GetCorner(i))) for i in range(8))
        method='conservative relative AABB corners'
    parts.append({'prim':str(prim.GetPath()),'bounding_sphere_radius_m':radius,'method':method})
maximum=max(p['bounding_sphere_radius_m'] for p in parts)
print(json.dumps({'scene':str(args.scene),'scene_sha256':hashlib.sha256(args.scene.read_bytes()).hexdigest(),
    'measured_radius_m':maximum,'rounded_body_radius_m':math.ceil(maximum*100)/100,
    'previous_body_radius_m':.25,'parts':parts},indent=2))
