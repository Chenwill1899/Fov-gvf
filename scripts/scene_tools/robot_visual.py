"""Attach the prepared ARL appearance to the existing kinematic robot rig.

All geometry is embedded in the map; only Isaac's bundled OmniPBR MDL is used.
No articulation, controller, camera or collision geometry is imported.
"""
import json
from pathlib import Path
from pxr import Sdf, Usd, UsdGeom

ROOT=Path(__file__).resolve().parents[2]
ASSET=ROOT/'scenes/robots/arl_robot_1'
LEGACY=['Body','ArmForward','ArmSide']+[f'{kind}_{i}' for kind in ['Motor','Propeller'] for i in range(1,5)]


def relocate_links(stage, old, new):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(new)):
        for rel in prim.GetRelationships():
            if rel.GetTargets():rel.SetTargets([p.ReplacePrefix(old,new) for p in rel.GetTargets()])
        for attr in prim.GetAttributes():
            if attr.GetConnections():attr.SetConnections([p.ReplacePrefix(old,new) for p in attr.GetConnections()])


def attach(stage):
    robot=stage.GetPrimAtPath('/World/Robot')
    if not robot:raise ValueError('Expected the existing /World/Robot camera/control rig')
    source=Usd.Stage.Open(str(ASSET/'visual.usdc'))
    for name in LEGACY:
        stage.RemovePrim('/World/Robot/'+name)
    dest=Sdf.Path('/World/Robot/Visuals')
    stage.RemovePrim(dest)
    old=source.GetDefaultPrim().GetPath()
    Sdf.CopySpec(source.GetRootLayer(),old,stage.GetRootLayer(),dest)
    relocate_links(stage,old,dest)
    return json.loads((ASSET/'provenance.json').read_text())
