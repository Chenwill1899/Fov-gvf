#!/usr/bin/env python3
"""Build three editable, self-contained Isaac USD reference environments.

Run with isaacsim/python.sh (USD bindings); does not start SimulationApp.
Dimensions and layouts are authored estimates, not surveyed reconstructions.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
from reference_scene_geometry import voxelize
from reference_lab_layout import sample_layout, author_routes
from robot_visual import attach as attach_robot_visual

ROOT = Path(__file__).resolve().parents[2]
COLORS = {'grey': [.30,.34,.38], 'dark':[.065,.08,.10], 'green':[.28,.80,.035],
          'yellow':[.83,.63,.08], 'blue':[.08,.18,.40], 'wood':[.28,.18,.085]}


class Model:
    def __init__(self, name, extent, spawn, goal):
        self.name, self.extent, self.spawn, self.goal = name, extent, spawn, goal
        self.objects, self.gates = [], []
        self.route = [goal]
        self.revision = 1
        self.layout = {}
        self.ceiling_lights = [[x,0,extent[2]-.65] for x in [-extent[0]/3,0,extent[0]/3]]
        self.overview_eye = [-extent[0]*.65,-extent[1]*.8,extent[2]*.75]

    def add(self, name, kind, color, **kw):
        self.objects.append(dict(name=name, kind=kind, color=color, **kw))

    def box(self, name, center, size, color):
        self.add(name, 'box', color, center=list(center), size=list(size))

    def rod(self, name, a, b, radius, color):
        self.add(name, 'capsule', color, a=list(a), b=list(b), radius=radius)

    def sphere(self, name, center, radius, color):
        self.add(name, 'sphere', color, center=list(center), radius=radius)

    def gate(self, name, center, axis='x', width=3.4, height=3.4, color=None):
        color = color or COLORS['green']
        c = np.array(center, dtype=float); tangent = np.array([0.,1.,0.] if axis=='x' else [1.,0.,0.])
        thick = .16
        for sign in [-1,1]:
            p=c+sign*tangent*(width/2+thick/2)
            size=[thick,thick,height+2*thick]
            self.box(f'{name}_side_{"l" if sign<0 else "r"}',p,size,color)
            p=c+[0,0,sign*(height/2+thick/2)]
            size=[thick,width+2*thick,thick] if axis=='x' else [width+2*thick,thick,thick]
            self.box(f'{name}_bar_{"b" if sign<0 else "t"}',p,size,color)
        for sign in [-1,1]:
            p=c+sign*tangent*(width/2+thick/2)
            if c[2]-height/2 > .2:
                self.rod(f'{name}_stand_{sign+1}',[p[0],p[1],.08],[p[0],p[1],c[2]-height/2],.05,COLORS['dark'])
        self.gates.append(dict(name=name, center=list(center), normal=[1,0,0] if axis=='x' else [0,1,0],
                               inner_width=width,inner_height=height))

    def room(self, roof=True):
        x,y,z=self.extent
        self.box('Ground',[0,0,-.15],[x+1,y+1,.3],[.37,.39,.41])
        for sign in [-1,1]:
            self.box(f'WallX_{sign+1}',[sign*x/2,0,z/2],[.2,y,z],[.49,.52,.54])
            self.box(f'WallY_{sign+1}',[0,sign*y/2,z/2],[x,.2,z],[.43,.46,.49])
        if roof:
            self.box('Roof',[0,0,z+.12],[x,y,.24],[.19,.21,.23])
        for i,xx in enumerate(np.arange(-x/2+2,x/2,4)):
            self.box(f'Beam_{i}',[xx,0,z-.4],[.16,y,.24],COLORS['dark'])
            self.box(f'Light_{i}',[xx,0,z-.55],[.5,2,.08],[.95,.97,1])


def lab():
    m=Model('lab',[140,100,7],[-67,-45,3.],[66,-45,3.]);m.room()
    m.revision=3
    m.layout=sample_layout()
    m.overview_eye=[-100,-110,105]
    for entry in m.layout['placements']:
        x,y=entry['xy'];name=entry['name']
        if entry['kind']=='stack':
            h=entry['height']
            m.box(name+'_lower',[x,y,h/2],[1.8,1.8,h],COLORS[entry['color']])
            m.box(name+'_cap',[x,y,h+.45],[1.8,1.8,.9],COLORS['yellow'])
        else:
            m.gate(name,[x,y,entry['z']],axis=entry['axis'],color=[.57,.61,.69])
    m.ceiling_lights=[]
    for row,dy in enumerate([-40,-20,0,20,40]):
        for col,dx in enumerate([-56,-28,0,28,56]):
            for j,x in enumerate([-28/3,0,28/3]):
                m.ceiling_lights.append([x+dx,dy,6.35])
                m.box(f'Cell_{row}_{col}_CeilingFixture_{j}',[x+dx,dy,6.45],[.5,2,.08],[.95,.97,1])
    # Visible safety mesh: thin physical strands, never solid invisible walls.
    for i,x in enumerate(np.arange(-69,70,.8)):
        m.rod(f'Net_vertical_{i}',[x,49.6,0],[x,49.6,6.7],.012,[.18,.21,.24])
    for i,z in enumerate(np.arange(.5,7,.5)):
        m.rod(f'Net_horizontal_{i}',[-69.5,49.6,z],[69.5,49.6,z],.012,[.18,.21,.24])
    smoke_path,m.route=author_routes(m.objects,m.spawn,m.goal)
    m.layout['offline_smoke_connectivity_path']=smoke_path
    m.layout['route_authoring']='offline analytic clearance only; smoke runtime receives final goal, not this path'
    return m


def hangar():
    m=Model('hangar',[40,28,9],[-16,-7,3.],[15,-7,3.]);m.room(roof=False)
    m.revision=2  # Open roof admits daylight; keep the existing gate course and trusses.
    for i,x in enumerate([-18,-9,0,9,18]):
        for side in [-1,1]:
            m.box(f'Column_{i}_{side+1}',[x,side*12.8,4.5],[.35,.35,9],COLORS['grey'])
        for j,y in enumerate([-10,-5,0,5,10]):
            m.rod(f'Truss_{i}_{j}',[x,y-2.5,8],[x,y+2.5,8.7],.06,COLORS['grey'])
    gates=[([-10,-7,2.3],'x'),([2,-7,2.3],'x'),([12,-1,3.5],'y'),
           ([6,7,5.5],'x'),([6,7,2.0],'x'),([-6,7,2.3],'x'),([-14,1,3.0],'y')]
    for i,(c,axis) in enumerate(gates):m.gate(f'Gate_{i+1}',c,axis,width=3.4,height=3.)
    for i,x in enumerate([-15,-5,5,15]):
        m.box(f'Barrier_{i}',[x,12,1.0],[3,.45,2],[.10,.15,.19])
    m.route=[[-12,-7,2.3],[-8,-7,2.3],[0,-7,2.3],[4,-7,2.3],[12,-3,3.5],[12,1,3.5],
             [9,7,5.5],[3,7,5.5],[3,7,2.0],[9,7,2.0],[10,10,2.3],[-4,10,2.3],
             [-4,7,2.3],[-8,7,2.3],[-14,3,3.0],[-14,-1,3.0],[-16,-7,2.3]]
    return m


def forest():
    from reference_forest_layout import populate
    m=Model('forest',[360,80,24],[-172,0,6.],[172,0,6.])
    populate(m)
    return m


def build(m):
    folder=ROOT/'scenes'/f'reference_{m.name}';folder.mkdir(parents=True,exist_ok=True)
    scene=folder/'scene.usdc'
    stage=Usd.Stage.CreateNew(str(scene))
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
    source=Usd.Stage.Open(str(ROOT/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'))
    Sdf.CopySpec(source.GetRootLayer(),'/World/Robot',stage.GetRootLayer(),'/World/Robot')
    UsdGeom.XformCommonAPI(stage.GetPrimAtPath('/World/Robot')).SetTranslate(Gf.Vec3d(*m.spawn))
    robot_visual=attach_robot_visual(stage)
    UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
    materials={}
    for obj in m.objects:
        path='/World/Environment/'+obj['name']
        if m.name=='forest' and obj['name']=='Ground':
            prim=UsdGeom.Mesh.Define(stage,path)
            points=np.array([[-.5,-.5,-.5],[.5,-.5,-.5],[.5,.5,-.5],[-.5,.5,-.5],
                             [-.5,-.5,.5],[.5,-.5,.5],[.5,.5,.5],[-.5,.5,.5]])
            points=points*np.array(obj['size'])+np.array(obj['center'])
            faces=[[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
            indices=np.asarray(faces).ravel()
            prim.CreatePointsAttr([Gf.Vec3f(*v) for v in points])
            prim.CreateFaceVertexCountsAttr([4]*6);prim.CreateFaceVertexIndicesAttr(indices.tolist())
            prim.CreateSubdivisionSchemeAttr('none')
            UsdGeom.PrimvarsAPI(prim).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying').Set(
                [Gf.Vec2f(*v) for v in points[indices,:2]/(3.2*getattr(m,'texture_scale',1.))])
        elif obj['kind']=='box':
            prim=UsdGeom.Cube.Define(stage,path);prim.CreateSizeAttr(1.)
            api=UsdGeom.XformCommonAPI(prim);api.SetTranslate(Gf.Vec3d(*obj['center']));api.SetScale(Gf.Vec3f(*obj['size']))
        elif obj['kind']=='sphere':
            prim=UsdGeom.Sphere.Define(stage,path);prim.CreateRadiusAttr(float(obj['radius']))
            UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*obj['center']))
        elif obj['kind']=='convex':
            prim=UsdGeom.Mesh.Define(stage,path)
            prim.CreatePointsAttr([Gf.Vec3f(*v) for v in obj['vertices']])
            prim.CreateFaceVertexCountsAttr([3]*len(obj['faces']))
            prim.CreateFaceVertexIndicesAttr(np.asarray(obj['faces']).ravel().tolist())
            prim.CreateSubdivisionSchemeAttr('none')
            UsdPhysics.MeshCollisionAPI.Apply(prim.GetPrim()).CreateApproximationAttr('convexHull')
            if obj.get('hidden'):prim.MakeInvisible()
        else:
            a,b=np.array(obj['a']),np.array(obj['b']);delta=b-a
            prim=UsdGeom.Capsule.Define(stage,path);prim.CreateRadiusAttr(float(obj['radius']));prim.CreateHeightAttr(float(np.linalg.norm(delta)));prim.CreateAxisAttr('Z')
            xf=UsdGeom.Xformable(prim);xf.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*delta)).GetQuat())
        if obj.get('hidden'):prim.MakeInvisible()
        prim.CreateDisplayColorAttr([Gf.Vec3f(*obj['color'])]);UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
        key=tuple(round(float(c),2) for c in obj['color'])
        if key not in materials:
            material=UsdShade.Material.Define(stage,f'/World/Looks/Mat_{len(materials)}')
            shader=UsdShade.Shader.Define(stage,str(material.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*key))
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.88)
            shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');materials[key]=material
        UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(materials[key])
        prim.GetPrim().CreateAttribute('reference:kind',Sdf.ValueTypeNames.String).Set(obj['kind'])
    asset_hashes={}
    if hasattr(m,'vegetation'):
        from reference_forest_layout import add_visuals
        asset_hashes=add_visuals(stage,m.vegetation)
    if hasattr(m,'author_visuals'):
        asset_hashes.update(m.author_visuals(stage))
    light=UsdLux.DomeLight.Define(stage,'/World/Dome');light.CreateIntensityAttr(600)
    sun=UsdLux.DistantLight.Define(stage,'/World/Sun');sun.CreateIntensityAttr(1500)
    UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(-35,20,30))
    if m.name!='forest':
        for i,position in enumerate(m.ceiling_lights):
            light=UsdLux.RectLight.Define(stage,f'/World/CeilingLight_{i}')
            light.CreateWidthAttr(5);light.CreateHeightAttr(5);light.CreateIntensityAttr(850)
            light.CreateExposureAttr(6.)
            UsdGeom.XformCommonAPI(light).SetTranslate(Gf.Vec3d(*position))
    x,y,z=m.extent;res=getattr(m,'resolution',.15)
    origin=[-x/2-.45,-y/2-.45,-.45]
    shape=[int(math.ceil((v+.9)/res))+1 for v in [x,y,z]]
    metadata=json.loads(source.GetRootLayer().customLayerData['fovNavigationJson'])
    # Cloud-specific height transforms must not label or rescale reference maps.
    for key in ['verticalScale','heightStretchAnchorZ']:
        metadata.pop(key,None)
    metadata.update(mapIdentity=f'reference_{m.name}_v{m.revision}',spawn=m.spawn,manualControl=True,occupancyResolution=res,occupancyOrigin=origin,
                    occupancyShape=shape,environmentScale=1.,bodyRadiusM=.48,cameraMaxDepthM=20.)
    if asset_hashes:metadata['assetSha256']=asset_hashes
    stage.GetRootLayer().customLayerData={'fovNavigationJson':json.dumps(metadata)}
    stage.GetRootLayer().Save()
    grid=voxelize(m.objects,origin,shape,res);grid.tofile(folder/'occupancy.bin')
    manifest=dict(robot_visual=robot_visual,map_identity=f'reference_{m.name}_v{m.revision}',scene=scene.name,
        reconstruction=getattr(m,'reconstruction','reference-inspired; all dimensions and coordinates are authored estimates, not surveyed'),
        units='m',up_axis='Z',resolution=res,origin=origin,shape=shape,extent=m.extent,
        spawn=m.spawn,goal=m.goal,waypoints=m.route,gates=m.gates,objects=m.objects,
        collision_model=f'boxes, spheres, capsules and conservative convex mesh envelopes; {res:g}m cell circumsphere supercover',
        working_bounds=dict(enabled=True,min_x=-x/2+.3,max_x=x/2-.3,min_y=-y/2+.3,max_y=y/2-.3,min_z=0,max_z=z-.3),
        overview_eye=m.overview_eye,overview_target=[0,0,2.5],
        first_person_eye=m.spawn,first_person_target=[m.spawn[0]+12,m.spawn[1],m.spawn[2]],
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        occupancy_sha256=hashlib.sha256((folder/'occupancy.bin').read_bytes()).hexdigest(),
        robot_template_sha256=hashlib.sha256((ROOT/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd').read_bytes()).hexdigest())
    if asset_hashes:manifest['asset_sha256']=asset_hashes
    if m.layout:manifest['layout']=m.layout
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=None if m.name in ['forest','geo_forest','geo_clutter','geo_garage'] else 2)+'\n')
    print(f'{m.name}: {len(m.objects)} solids, grid {shape}, {int(grid.sum())} occupied cells',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',choices=['all','lab','hangar','forest'],default='all');a=p.parse_args()
    for name,factory in [('lab',lab),('hangar',hangar),('forest',forest)]:
        if a.scene in ['all',name]:build(factory())
    # Only exact complete scene/grid bytes may authorize the initial free ball.
    records=[]
    for name in ['lab','hangar','forest','geo_forest','geo_clutter','geo_garage']:
        p=ROOT/'scenes'/f'reference_{name}'/'manifest.json'
        if p.exists():
            m=json.loads(p.read_text())
            records.append('    {"%s", "%s", {%s}, {%s}, %.17g},' % (
                m['scene_sha256'],m['occupancy_sha256'],','.join(map(str,m['shape'])),
                ','.join(map(str,m['origin'])),m['resolution']))
    header='''// Generated by scripts/scene_tools/build_reference_scenes.py. Exact-byte startup allowlist.
struct ReferenceSceneIdentity {
    const char* scene; const char* occupancy; int shape[3]; double origin[3]; double resolution;
};
static const ReferenceSceneIdentity kReferenceScenes[] = {
'''+ '\n'.join(records)+'\n};\n'
    (ROOT/'src/pc_gvf/config/generated_scene_identity/reference_scene_registry.inc').write_text(header)
