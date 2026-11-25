#!/usr/bin/env python3
"""Prepare official ARL base geometry for the unchanged EGO2P6 camera rig.

Isaac Python: prepare_arl_visual.py --source PATH/configuration/arl_robot_1_base.usd
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom
from robot_visual import ASSET,relocate_links


def prepare(path):
    stage=Usd.Stage.Open(str(path))
    root=stage.GetDefaultPrim()
    # Expand prototype meshes so CopySpec cannot omit flattened root prototypes.
    while True:
        instances=[p for p in stage.Traverse() if p.IsInstance()]
        if not instances:break
        for p in instances:p.SetInstanceable(False)
    world=[];triangles=0;mesh_count=0
    cache=UsdGeom.XformCache()
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
            matrix=np.asarray(cache.GetLocalToWorldTransform(prim))
            world.append(v@matrix[:3,:3]+matrix[3,:3]);mesh_count+=1
            triangles+=sum(max(0,int(n)-2) for n in mesh.GetFaceVertexCountsAttr().Get())
    points=np.concatenate(world);top=float(points[:,2].max())
    # Put the airframe below the unchanged camera plane (z=.02). Preserve full
    # geometry and choose a uniform scale fitting within the physical .48m ball.
    def transformed(scale):
        return points*scale+[0,0,-top*scale-.04]
    lo,hi=0.,3.
    for _ in range(60):
        mid=(lo+hi)/2
        if np.linalg.norm(transformed(mid),axis=1).max()<=.46:lo=mid
        else:hi=mid
    scale=lo;offset=-top*scale-.04
    target=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(target,'Z');UsdGeom.SetStageMetersPerUnit(target,1.)
    old=root.GetPath();dest=Sdf.Path('/ARLRobot1')
    flat=stage.Flatten(False)
    Sdf.CopySpec(flat,old,target.GetRootLayer(),dest)
    target.SetDefaultPrim(target.GetPrimAtPath(dest));relocate_links(target,old,dest)
    for p in list(target.Traverse()):
        if p.GetName()=='joints':target.RemovePrim(p.GetPath());continue
        if p.GetTypeName().startswith(('Physics','Physx','OmniGraph')):
            raise ValueError('Unexpected dynamics or graph in visual layer: '+str(p.GetPath()))
        if any('Physics' in a or 'Physx' in a for a in p.GetAppliedSchemas()):
            raise ValueError('Unexpected physics API in visual-only source')
        for attr in p.GetAttributes():
            if attr.GetTypeName()==Sdf.ValueTypeNames.Asset:
                value=attr.Get()
                if value and value.path:
                    if Path(value.path).name!='OmniPBR.mdl':raise ValueError(value.path)
                    attr.Set(Sdf.AssetPath('OmniPBR.mdl'))
    prim=target.GetDefaultPrim()
    for attr in list(prim.GetAttributes()):
        if attr.GetName().startswith('xformOp:') or attr.GetName()=='xformOpOrder':prim.RemoveProperty(attr.GetName())
    xf=UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0,0,offset))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(scale))
    ASSET.mkdir(parents=True,exist_ok=True)
    temporary=ASSET/'visual.tmp.usdc';target.GetRootLayer().Export(str(temporary));temporary.replace(ASSET/'visual.usdc')
    final=transformed(scale)
    report=dict(name='ARL-Robot-1',visual_revision=1,source_url='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/NTNU/ARL-Robot-1/configuration/arl_robot_1_base.usd',
        author='NTNU Autonomous Robots Lab',license='BSD-3-Clause',license_source='https://github.com/ntnu-arl/robot_model',
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),visual_sha256=hashlib.sha256((ASSET/'visual.usdc').read_bytes()).hexdigest(),
        mesh_count=mesh_count,triangles=triangles,uniform_scale=scale,translation=[0,0,offset],
        aabb_min=final.min(axis=0).tolist(),aabb_max=final.max(axis=0).tolist(),
        maximum_vertex_radius_m=float(np.linalg.norm(final,axis=1).max()),physical_body_radius_m=.48,
        purpose='visual only; existing robot rig, four cameras and kinematic controller retained',builtin_material_dependency='OmniPBR.mdl')
    (ASSET/'provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument('--source',type=Path,required=True)
    prepare(cli.parse_args().source.resolve())
