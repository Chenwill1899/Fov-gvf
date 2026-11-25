"""Natural woodland layout and instanced CC0 trees; offline scene authoring only."""
import hashlib
import copy
import json
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
from reference_scene_geometry import bounds, signed_distance
from reference_lab_layout import author_routes

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'scenes/reference_forest/assets'
SPECIES = ['tree_small_02', 'island_tree_01']


def transformed_pieces(entry, templates):
    angle = np.deg2rad(entry['yaw'])
    rotation = np.array([[np.cos(angle),-np.sin(angle),0],
                         [np.sin(angle),np.cos(angle),0],[0,0,1]])
    scale, position = entry['scale'], np.array(entry['position'])
    for i, piece in enumerate(templates[entry['asset']]['pieces']):
        vertices = np.asarray(piece['vertices']) @ rotation.T * scale + position
        planes = np.asarray(piece['planes']).copy()
        planes[:,:3] = planes[:,:3] @ rotation.T
        planes[:,3] = planes[:,3] * scale - planes[:,:3] @ position
        yield dict(name=f"{entry['name']}_proxy_{i}", kind='convex',
            color=[.22,.31,.09], hidden=True, asset_instance=entry['name'],
            vertices=vertices.tolist(), faces=piece['faces'], planes=planes.tolist(),
            aabb_min=vertices.min(axis=0).tolist(), aabb_max=vertices.max(axis=0).tolist())


def populate_section(m, seed=20261023, build_routes=True):
    rng = np.random.default_rng(seed)
    templates = {name:json.loads((ASSETS/name/'collision.json').read_text()) for name in SPECIES}
    m.revision = 2
    m.overview_eye = [-44,-44,34]
    m.box('Ground',[0,0,-.15],[62,42,.3],[.20,.24,.11])
    entries, stems = [], []
    centers = np.array([[-19,-10],[-12,11],[0,-8],[9,9],[20,-10]])
    clearings = np.array([[-12,-1],[14,0]])

    def safe_endpoint(pieces):
        for p in [m.spawn, m.goal]:
            for obj in pieces:
                low, high = bounds(obj)
                if np.any(np.asarray(p)<low-2.8) or np.any(np.asarray(p)>high+2.8):
                    continue
                if float(signed_distance(p,obj)) < 2.8:
                    return False
        return True

    def plant(name, xy, scale):
        entry = dict(name=name, asset=SPECIES[int(rng.integers(2))],
            position=[float(xy[0]),float(xy[1]),0.],
            yaw=float(rng.uniform(0,360)), scale=float(scale))
        pieces = list(transformed_pieces(entry, templates))
        if not safe_endpoint(pieces):
            return False
        entries.append(entry); m.objects.extend(pieces)
        return True

    # A variable-density point process: clumps, scattered mature trees and clearings.
    # There are no planting rows, equal cell quotas or artificially cleared corridor.
    for attempt in range(30000):
        if len(stems) == 96:
            break
        if rng.random() < .68:
            xy = centers[rng.integers(len(centers))] + rng.normal(0,5.,2)
        else:
            xy = rng.uniform([-28,-18],[28,18])
        if np.any(xy < [-28,-18]) or np.any(xy > [28,18]):
            continue
        if np.min(np.linalg.norm(clearings-xy,axis=1)) < 3.0:
            continue
        if stems and np.min(np.linalg.norm(np.array(stems)-xy,axis=1)) < 3.05:
            continue
        if plant(f'Tree_{len(stems):03d}',xy,rng.uniform(.72,1.22)):
            stems.append(xy)
    if len(stems) != 96:
        raise RuntimeError('Unable to place all woodland trees')
    shrubs = 0
    for attempt in range(5000):
        if shrubs == 48:
            break
        xy = np.array(stems[rng.integers(len(stems))]) + rng.normal(0,1.6,2)
        if np.any(xy < [-28,-18]) or np.any(xy > [28,18]):
            continue
        if plant(f'Shrub_{shrubs:03d}',xy,rng.uniform(.10,.23)):
            shrubs += 1
    for i in range(22):
        xy = rng.uniform([-27,-17],[27,17])
        radius = float(rng.uniform(.25,.65))
        # Low boulders are partly buried, with a shared exact render/collision sphere.
        m.sphere(f'Rock_{i:02d}',[*xy,-radius*.22],radius,[.27,.28,.24])
    for i in range(8):
        xy = np.array(stems[int(rng.integers(len(stems)))])
        angle = rng.uniform(0,2*np.pi); length = rng.uniform(1.8,3.6)
        a = np.array([*xy,.22]); b = a + [length*np.cos(angle),length*np.sin(angle),.10]
        obj = dict(name=f'Fallen_log_{i:02d}',kind='capsule',color=[.23,.15,.075],
                   a=a.tolist(),b=b.tolist(),radius=float(rng.uniform(.12,.23)))
        if safe_endpoint([obj]):
            m.objects.append(obj)
    distance = np.linalg.norm(np.array(stems)[:,None]-np.array(stems),axis=2)
    np.fill_diagonal(distance,np.inf)
    m.vegetation = entries
    m.layout = dict(seed=seed,method='variable-density clustered minimum-distance sampling',
        tree_count=len(stems), shrub_count=shrubs, minimum_stem_spacing_m=3.05,
        nearest_neighbor_m=dict(min=float(distance.min()), median=float(np.median(distance.min(axis=1)))),
        placements=entries, clearings=clearings.tolist(),
        inspiration=['https://thu-uav.github.io/FlightBench/#scenarios',
                     'https://github.com/princeton-vl/infinigen',
                     'https://github.com/callumnewlands/ForestGenerator'],
        actual_assets=[f'https://polyhaven.com/a/{name}' for name in SPECIES],
        collision_note='Hidden conservative convex foliage envelopes, not exact leaf-gap collision; full rendered-triangle containment.')
    if not build_routes:
        return
    smoke, m.route = author_routes(m.objects,m.spawn,m.goal,
        x_limits=(-28,28),y_limits=(-18,18),grid_step=.3,
        anchors=[[23,-13],[15,12],[-9,13],[-22,-12],[-26,0]])
    m.layout['offline_smoke_connectivity_path'] = smoke
    m.layout['route_authoring'] = 'offline clearance only; runtime smoke receives the final goal, not this path'


def populate(m):
    """Keep the original woodland at 2x scale; extend X with two new random sections."""
    base=json.loads((ASSETS/'base_layout_v2.json').read_text())
    templates={name:json.loads((ASSETS/name/'collision.json').read_text()) for name in SPECIES}
    scale=2.
    m.revision=3
    m.overview_eye=[-235,-255,220]
    m.texture_scale=scale
    m.box('Ground',[0,0,-.3],[364,84,.6],[.20,.24,.11])
    entries=[];sections=[];clearings=[]
    for label,offset,seed in [('west',-60.,20261024),('center',0.,20261023),('east',60.,20261025)]:
        if label=='center':
            section=base
        else:
            local=type(m)('forest',[60,40,12],[-26,0,3.],[26,0,3.])
            populate_section(local,seed=seed,build_routes=False)
            section=dict(placements=local.vegetation,clearings=local.layout['clearings'],
                objects=[o for o in local.objects if o['kind']!='convex' and o['name']!='Ground'])
        translation=np.array([offset,0.,0.])
        for original in section['placements']:
            e=copy.deepcopy(original)
            if label!='center':
                kind,index=e['name'].split('_',1);e['name']=kind+'_'+label+'_'+index
            e['position']=((np.array(e['position'])+translation)*scale).tolist()
            e['scale']*=scale;e['section']=label
            entries.append(e);m.objects.extend(transformed_pieces(e,templates))
        for original in section['objects']:
            o=copy.deepcopy(original)
            if label!='center':o['name']=label+'_'+o['name']
            if o['kind']=='sphere':o['center']=((np.array(o['center'])+translation)*scale).tolist()
            elif o['kind']=='capsule':
                for key in ['a','b']:o[key]=((np.array(o[key])+translation)*scale).tolist()
            else:raise ValueError(o['kind'])
            o['radius']*=scale;m.objects.append(o)
        clearings.extend(((np.array(section['clearings'])+[offset,0])*scale).tolist())
        sections.append(dict(name=label,seed=seed,x_range=[(offset-30)*scale,(offset+30)*scale],
            original_layout_preserved=label=='center',tree_count=96,shrub_count=48))
    m.vegetation=entries
    stems=np.array([e['position'][:2] for e in entries if e['name'].startswith('Tree_')])
    distance=np.linalg.norm(stems[:,None]-stems,axis=2);np.fill_diagonal(distance,np.inf)
    m.layout=dict(seed=20261023,method='original layout uniformly scaled 2x; independently seeded west/east extensions',
        original_extent=base['extent'],uniform_scale=2.,length_extension_factor=3.,extension_axis='X',
        scaled_extent=[120,80,24],final_extent=m.extent,sections=sections,
        base_layout_sha256=hashlib.sha256((ASSETS/'base_layout_v2.json').read_bytes()).hexdigest(),
        tree_count=288,shrub_count=144,rock_count=66,fallen_log_count=24,
        minimum_stem_spacing_m=6.1,
        nearest_neighbor_m=dict(min=float(distance.min()),median=float(np.median(distance.min(axis=1)))),
        placements=entries,clearings=clearings,
        actual_assets=[f'https://polyhaven.com/a/{name}' for name in SPECIES],
        collision_note='Conservative convex foliage envelopes; same geometry scaled 2x, body and sensor parameters unchanged.')
    smoke,m.route=author_routes(m.objects,m.spawn,m.goal,x_limits=(-176,176),y_limits=(-36,36),
        grid_step=.6,route_z=6.,max_shortcut_m=35.,
        anchors=[[-120,-24],[-60,24],[0,-24],[60,24],[120,-24],[172,0]])
    m.layout['offline_smoke_connectivity_path']=smoke
    m.layout['route_authoring']='offline analytic geometry only; smoke runtime receives only final goal'


def add_visuals(stage, entries):
    # Internal instances keep geometry embedded in the scene SHA identity. Only
    # local texture dependencies remain external and are independently hashed.
    library = UsdGeom.Xform.Define(stage,'/World/TreeLibrary')
    library.MakeInvisible()
    for name in SPECIES:
        source = Usd.Stage.Open(str(ASSETS/name/'model.usdc'))
        root = source.GetDefaultPrim().GetPath()
        dest = Sdf.Path('/World/TreeLibrary/'+name)
        Sdf.CopySpec(source.GetRootLayer(),root,stage.GetRootLayer(),dest)
        for prim in Usd.PrimRange(stage.GetPrimAtPath(dest)):
            for rel in prim.GetRelationships():
                targets = rel.GetTargets()
                if targets:
                    rel.SetTargets([p.ReplacePrefix(root,dest) for p in targets])
            for attr in prim.GetAttributes():
                connections = attr.GetConnections()
                if connections:
                    attr.SetConnections([p.ReplacePrefix(root,dest) for p in connections])
                if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                    value = attr.Get()
                    if value and value.path:
                        local = Path(value.path).name
                        path = ASSETS/name/'textures'/local
                        if not path.is_file():
                            raise FileNotFoundError(path)
                        attr.Set(Sdf.AssetPath(f'assets/{name}/textures/{local}'))
            if prim.IsA(UsdGeom.Mesh):
                UsdGeom.Mesh(prim).CreateDoubleSidedAttr(True)
    for entry in entries:
        prim = stage.DefinePrim('/World/Vegetation/'+entry['name'],'Xform')
        prim.GetReferences().AddInternalReference('/World/TreeLibrary/'+entry['asset'])
        prim.SetInstanceable(True)
        xf = UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*entry['position']))
        xf.AddRotateZOp().Set(entry['yaw'])
        xf.AddScaleOp().Set(Gf.Vec3f(entry['scale']))
    ground = UsdShade.Material.Define(stage,'/World/Looks/ForestGround')
    surface = UsdShade.Shader.Define(stage,str(ground.GetPath())+'/Surface')
    surface.CreateIdAttr('UsdPreviewSurface')
    surface.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.9)
    uv = UsdShade.Shader.Define(stage,str(ground.GetPath())+'/UV')
    uv.CreateIdAttr('UsdPrimvarReader_float2')
    uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    for kind,slot in [('diff','diffuseColor'),('nor_gl','normal'),('rough','roughness')]:
        texture = UsdShade.Shader.Define(stage,str(ground.GetPath())+'/'+kind)
        texture.CreateIdAttr('UsdUVTexture')
        texture.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(
            f'assets/forest_ground_04/forest_ground_04_{kind}_1k.jpg'))
        texture.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
        texture.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB' if kind=='diff' else 'raw')
        for axis in ['S','T']:
            texture.CreateInput('wrap'+axis,Sdf.ValueTypeNames.Token).Set('repeat')
        if kind=='nor_gl':
            texture.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2))
            texture.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1))
        typename=Sdf.ValueTypeNames.Float if kind=='rough' else Sdf.ValueTypeNames.Color3f
        surface.CreateInput(slot,typename).ConnectToSource(texture.ConnectableAPI(),'r' if kind=='rough' else 'rgb')
    ground.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(),'surface')
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath('/World/Environment/Ground')).Bind(ground)
    return {str(p.relative_to(ASSETS.parent)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(ASSETS.rglob('*')) if p.is_file()}
