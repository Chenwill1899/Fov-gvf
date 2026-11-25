#!/usr/bin/env python3
"""Export embedded robot mesh and exact camera transforms for a CPU BVH probe."""
import argparse,json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom
from robot_visual import ROOT

cli=argparse.ArgumentParser();cli.add_argument('--out',type=Path,required=True);args=cli.parse_args()
args.out.mkdir(parents=True,exist_ok=True)
s=Usd.Stage.Open(str(ROOT/'scenes/reference_forest/scene.usdc'))
cache=UsdGeom.XformCache();inverse=cache.GetLocalToWorldTransform(s.GetPrimAtPath('/World/Robot')).GetInverse()
points=[];faces=[];offset=0
for prim in Usd.PrimRange(s.GetPrimAtPath('/World/Robot/Visuals')):
    if not prim.IsA(UsdGeom.Mesh):continue
    mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
    matrix=np.asarray(cache.GetLocalToWorldTransform(prim)*inverse)
    points.append(v@matrix[:3,:3]+matrix[3,:3])
    counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get());indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    start=0
    for count in counts:
        polygon=indices[start:start+count]+offset;start+=count
        for i in range(1,int(count)-1):faces.append([polygon[0],polygon[i],polygon[i+1]])
    offset+=len(v)
np.savez_compressed(args.out/'probe_mesh.npz',vertices=np.concatenate(points),triangles=np.asarray(faces,dtype=np.int32))
cameras={}
for side,mount in [('front','CameraMount'),('left','CameraMountLeft'),('back','CameraMountBack'),('right','CameraMountRight')]:
    p=s.GetPrimAtPath('/World/Robot/'+mount+'/DepthCamera');camera=UsdGeom.Camera(p)
    cameras[side]=dict(matrix=np.asarray(cache.GetLocalToWorldTransform(p)*inverse).tolist(),
        focal=float(camera.GetFocalLengthAttr().Get()),horizontal_aperture=float(camera.GetHorizontalApertureAttr().Get()),
        vertical_aperture=float(camera.GetVerticalApertureAttr().Get()),clipping=list(camera.GetClippingRangeAttr().Get()))
(args.out/'probe_cameras.json').write_text(json.dumps(cameras,indent=2)+'\n')
print(len(np.concatenate(points)),'render vertices',len(faces),'triangles; four exact camera transforms')
