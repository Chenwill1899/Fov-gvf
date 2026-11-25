#!/usr/bin/env python3
"""Adapt the three official AeroScene environments to Isaac, keeping the c36 rig.

Use Isaac Python. Input is export_geomonoplan_unity.py's offline data, not Unity
executables. Meshes are embedded; shared local textures/converted source are kept.
"""
import argparse, collections, hashlib, json, math, shutil
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,UsdGeom,UsdLux,UsdShade
from build_reference_scenes import Model,build,ROOT
from reference_scene_geometry import bounds,swept_clearance

NAMES=['geo_forest','geo_clutter','geo_garage']
ASSETS=ROOT/'scenes/geomonoplan_assets'
C=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,1.,0.]])
C4=np.eye(4);C4[:3,:3]=C

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def matrix_points(p,m):return np.asarray(p)@m[:3,:3].T+m[:3,3]
def corners(lo,hi):return np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])

class Adapter:
    def __init__(self,source):
        self.source=source;self.data=json.loads((source/'scene_data.json').read_text())
        with np.load(source/'meshes.npz') as f:self.arrays={k:f[k] for k in f.files}
        self.meshes=self.data['meshes'];self.proxies={}
        ASSETS.mkdir(exist_ok=True)
        for name in ['scene_data.json','meshes.npz']+[p.name for p in source.glob('*.png')]:
            target=ASSETS/name
            if (source/name).resolve()!=target.resolve():shutil.copy2(source/name,target)
    def points(self,key):return self.arrays[self.meshes[key]['prefix']+'_points']@C.T
    def triangles(self,key):
        mesh=self.meshes[key];return [self.arrays[mesh['prefix']+'_tri_'+str(i)][:,::-1] for i in range(mesh['groups'])]
    def proxy_boxes(self,key,tree=False):
        ck=(key,tree)
        if ck not in self.proxies:
            points=self.points(key)
            if not tree:self.proxies[ck]=[(points.min(0),points.max(0))]
            else:
                tri=np.concatenate(self.triangles(key));faces=points[tri]
                # Vertical layers retain the trunk/crown height profile without
                # materializing hundreds of thousands of tiny USD colliders.
                ids=np.floor(faces.mean(1)[:,2]/1.).astype(int);_,inverse=np.unique(ids,return_inverse=True)
                self.proxies[ck]=[(faces[inverse==i].reshape(-1,3).min(0),faces[inverse==i].reshape(-1,3).max(0)) for i in range(int(inverse.max())+1)]
        return self.proxies[ck]
    def raw_instances(self,s):
        entries=[dict(r,tree=False) for r in s['instances']]
        t=s['terrain']
        if t:
            terrain_matrix=np.array(t['matrix'])
            for i,tree in enumerate(t['trees']):
                pos=np.array([tree['position'][a] for a in 'xyz'])*t['size']
                angle=tree['rotation'];co,si=math.cos(angle),math.sin(angle)
                rot=np.array([[co,0,si],[0,1,0],[-si,0,co]])
                tr=np.eye(4);tr[:3,:3]=rot@np.diag([tree['widthScale'],tree['heightScale'],tree['widthScale']]);tr[:3,3]=pos
                for j,r in enumerate(t['prototypes'][tree['index']]['renderers']):
                    entries.append(dict(r,name=f"Tree_{i}_{j}_{r['name']}",matrix=(terrain_matrix@tr@np.array(r['matrix'])).tolist(),tree=True,source_tree_index=i))
        return entries
    def model(self,s):
        entries=self.raw_instances(s);boxes=[]
        for r in entries:
            matrix=C4@np.array(r['matrix'])@C4.T
            v=matrix_points(self.points(r['mesh']),matrix);boxes.append((v.min(0),v.max(0)))
        terrain=s['terrain'];raw_ground=None
        if terrain:
            h=self.arrays[s['name']+'_heights'];n=terrain['resolution'];size=terrain['size']
            xx,yy=np.meshgrid(np.linspace(0,size[0],n),np.linspace(0,size[2],n),indexing='xy')
            raw_ground=matrix_points(np.column_stack([xx.ravel(),h.ravel(),yy.ravel()]),np.array(terrain['matrix']))@C.T
            boxes.append((raw_ground.min(0),raw_ground.max(0)))
            floor=raw_ground[:,2].max()
        else:
            candidates=[(hi[2],np.prod((hi-lo)[:2])) for r,(lo,hi) in zip(entries,boxes) if 'floor' in r['name'].lower() and hi[2]-lo[2]<.3]
            if not candidates:raise ValueError('Garage floor could not be identified')
            floor=max(candidates,key=lambda v:v[1])[0]
        low=np.min([b[0] for b in boxes],axis=0);high=np.max([b[1] for b in boxes],axis=0)
        zscale=2. if s['name']=='geo_garage' else 1.
        transform=np.eye(4);transform[2,2]=zscale;transform[:2,3]=-(low[:2]+high[:2])/2;transform[2,3]=-floor*zscale
        extent_xy=np.ceil((high[:2]-low[:2]+8)/2)*2
        height=float(math.ceil(max(10.,(high[2]-floor)*zscale+2.)))
        m=Model(s['name'],[float(extent_xy[0]),float(extent_xy[1]),height],[0,0,3.],[0,0,3.]);m.resolution=.20 if s['name']=='geo_clutter' else .15;m.ceiling_lights=[]
        m.reconstruction='Official GeoMonoPlan AeroScene v1.0.0 geometry/layout converted from Unity; LOD1 trees and 512px textures; Z-up centered coordinates; garage vertical scale 2, other scales 1; component convex hulls, conservative tree layers and terrain patch boxes.'
        m.overview_eye=[-m.extent[0]*.62,-m.extent[1]*.68,max(32.,m.extent[0]*.75)]
        transformed=[];maximum_error=0.;proxy_count=0
        for index,r in enumerate(entries):
            matrix=transform@C4@np.array(r['matrix'])@C4.T
            transformed.append(dict(r,matrix=matrix.tolist()))
            if not r['tree']:
                # Static batches already contain world-space vertices. Hull
                # their actual transformed component, rather than surrounding
                # tilted panels with large axis-aligned invisible boxes.
                from scipy.spatial import ConvexHull,QhullError
                vertices=np.unique(matrix_points(self.points(r['mesh']),matrix),axis=0)
                try:hull=ConvexHull(vertices)
                except QhullError:
                    directions=np.vstack([np.eye(3),-np.eye(3)])*.002
                    vertices=np.unique((vertices[:,None,:]+directions[None,:,:]).reshape(-1,3),axis=0)
                    hull=ConvexHull(vertices)
                used=hull.vertices;mapping=np.full(len(vertices),-1,dtype=int);mapping[used]=np.arange(len(used));points=vertices[used]
                _,unique=np.unique(np.round(hull.equations,8),axis=0,return_index=True)
                m.add(f'Imported_{index}_0','convex',[.25,.30,.25],hidden=True,vertices=points.tolist(),faces=mapping[hull.simplices].tolist(),planes=hull.equations[unique].tolist(),aabb_min=points.min(0).tolist(),aabb_max=points.max(0).tolist())
                proxy_count+=1
                continue
            for j,(lo,hi) in enumerate(self.proxy_boxes(r['mesh'],r['tree'])):
                v=matrix_points(corners(lo,hi),matrix);a,b=v.min(0)-.003,v.max(0)+.003
                m.box(f'Imported_{index}_{j}',(a+b)/2,b-a,[.25,.30,.25]);m.objects[-1]['hidden']=True
                proxy_count+=1
        ground=None
        if raw_ground is not None:
            n=terrain['resolution'];ground=matrix_points(raw_ground,transform).reshape(n,n,3)
            # Every original terrain triangle belongs to a rectangular 16-cell
            # patch. The patch box contains every vertex, including its boundary.
            for i in range(0,n-1,16):
                for j in range(0,n-1,16):
                    v=ground[i:min(i+17,n),j:min(j+17,n)].reshape(-1,3);a,b=v.min(0),v.max(0);a[2]-=.5;b[2]+=.003
                    m.box(f'Terrain_{i}_{j}',(a+b)/2,np.maximum(b-a,.006),[.2,.3,.1]);m.objects[-1]['hidden']=True
        # Foundation lies below the imported terrain and floor, without hiding it.
        base_z=float(min(-.25,min(bounds(o)[0][2] for o in m.objects))-.15)
        m.box('Ground',[0,0,base_z-.15],[m.extent[0]+1,m.extent[1]+1,.3],[.22,.25,.18] if terrain else [.25,.26,.27])
        m.layout=dict(source_repository='https://github.com/TJU-Aerial-Robotics/GeoMonoPlan',source_release='v1.0.0',source_archive_sha256='08f3f3170b03fd4a2af41a9febae8fa5aea33ee4d2fa43afaef94523c3e0d42a',source_level=s['source_level'],coordinate_conversion=C.tolist(),world_transform=transform.tolist(),vertical_scale=zscale,tree_count=len(terrain['trees']) if terrain else 0,renderer_count=len(entries),source_renderer_count=len(s['instances']),tree_lod=1,texture_max_edge=512,collision='component convex hulls of all source vertices; tree triangles grouped into 1m vertical layers; all original terrain vertices in 16-cell patch boxes',collision_proxy_count=proxy_count,online_map_truth=False)
        self.routes(m)
        m.author_visuals=lambda stage:self.author_visuals(stage,m,s,transformed,ground,transform)
        return m
    @staticmethod
    def routes(m):
        step=.75;x=np.arange(-m.extent[0]/2+3.3,m.extent[0]/2-3.3,step);y=np.arange(-m.extent[1]/2+3.3,m.extent[1]/2-3.3,step);z=3.
        if not len(x) or not len(y):raise ValueError('Empty flight domain')
        boxes=np.array([bounds(o) for o in m.objects]);lo,hi=boxes[:,0],boxes[:,1]
        def free_grid(radius):
            free=np.ones((len(x),len(y)),bool)
            dz=np.maximum(np.maximum(lo[:,2]-z,z-hi[:,2]),0)
            for a,b,h in zip(lo,hi,dz):
                if h>=radius:continue
                pad=math.sqrt(radius*radius-h*h)+.04
                l=np.maximum(0,np.floor((a[:2]-pad-[x[0],y[0]])/step).astype(int));u=np.minimum(free.shape,np.ceil((b[:2]+pad-[x[0],y[0]])/step).astype(int)+1)
                if np.all(l<u):free[l[0]:u[0],l[1]:u[1]]=False
            return free
        free=free_grid(.85);seeds=np.argwhere(free_grid(2.85)&free)
        if not len(seeds):raise ValueError('No original-scene location supports a verified 2.5m start ball')
        candidates=np.column_stack([x[seeds[:,0]],y[seeds[:,1]]]);target=np.array([-m.extent[0]*.32,0.]);start=tuple(seeds[np.argmin(np.linalg.norm(candidates-target,axis=1))])
        queue=collections.deque([start]);previous={start:None};distance={start:0}
        while queue:
            here=queue.popleft()
            for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nxt=(here[0]+dx,here[1]+dy)
                if 0<=nxt[0]<len(x) and 0<=nxt[1]<len(y) and free[nxt] and nxt not in previous:
                    previous[nxt]=here;distance[nxt]=distance[here]+1;queue.append(nxt)
        valid=[tuple(p) for p in seeds if tuple(p) in previous]
        end=max(valid,key=lambda p:(x[p[0]]-x[start[0]])-.15*abs(y[p[1]]-y[start[1]]))
        if distance[end]*step<10:raise ValueError('No useful connected original-scene route')
        raw=[];here=end
        while here is not None:raw.append([float(x[here[0]]),float(y[here[1]]),z]);here=previous[here]
        raw.reverse();course=[raw[0]];index=0
        # Remove collinear points, then shorten with independent continuous body
        # clearance. These offline points are never sent to smoke/manual modes.
        raw=[raw[0]]+[raw[i] for i in range(1,len(raw)-1) if not np.array_equal(np.subtract(raw[i],raw[i-1]),np.subtract(raw[i+1],raw[i]))]+[raw[-1]]
        while index<len(raw)-1:
            for j in range(len(raw)-1,index,-1):
                a,b=np.array(raw[index]),np.array(raw[j])
                if np.linalg.norm(a-b)>20 and j!=index+1:continue
                selected=np.all((lo<=np.maximum(a,b)+.85)&(hi>=np.minimum(a,b)-.85),axis=1)
                if swept_clearance(a,b,[m.objects[i] for i in np.flatnonzero(selected)],.75)>0:
                    course.append(raw[j]);index=j;break
            else:raise ValueError('Route shortcut failed continuous clearance')
        m.spawn=course[0];m.goal=course[-1];m.route=course[1:]
        m.layout.update(offline_smoke_connectivity_path=course,route_authoring='offline box-clearance connectivity only; smoke receives final goal, manual receives joystick',route_plane_z_m=z,start_conservative_clearance_m=min(float(np.linalg.norm(np.maximum(np.maximum(lo-np.array(m.spawn),np.array(m.spawn)-hi),0),axis=1).min()),1000.))
        print(m.name,'extent',m.extent,'spawn',m.spawn,'goal',m.goal,'course segments',len(m.route),'solids',len(m.objects),flush=True)
    def material(self,stage,key,used):
        path='/World/Looks/Geo_'+key.replace('.','_');existing=stage.GetPrimAtPath(path)
        if existing:return UsdShade.Material(existing)
        spec=self.data['materials'][key];mat=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec['color'][:3]));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(max(.22,spec['roughness']));shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(spec['metallic'])
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        from geomonoplan_profile import AMBIENT_TEXTURE_STRENGTH
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(np.array(spec['color'][:3])*AMBIENT_TEXTURE_STRENGTH)))
        if spec['texture']:
            used.add(spec['texture']);uv=UsdShade.Shader.Define(stage,path+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
            tr=UsdShade.Shader.Define(stage,path+'/Transform');tr.CreateIdAttr('UsdTransform2d');tr.CreateInput('in',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');tr.CreateInput('scale',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*spec['uv_scale']));tr.CreateInput('translation',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*spec['uv_offset']))
            tex=UsdShade.Shader.Define(stage,path+'/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('../geomonoplan_assets/'+spec['texture']));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(tr.ConnectableAPI(),'result')
            tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*spec['color']));tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3);tex.CreateOutput('a',Sdf.ValueTypeNames.Float)
            for axis in ['S','T']:tex.CreateInput('wrap'+axis,Sdf.ValueTypeNames.Token).Set('repeat')
            shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
            for axis in ['S','T']:tex.CreateInput('wrap'+axis,Sdf.ValueTypeNames.Token).Set('repeat')
            ambient=UsdShade.Shader.Define(stage,path+'/AmbientTexture');ambient.CreateIdAttr('UsdUVTexture')
            for name in ['file','sourceColorSpace','st','wrapS','wrapT']:
                original=tex.GetInput(name);copy=ambient.CreateInput(name,original.GetTypeName())
                connections=original.GetAttr().GetConnections()
                if connections:copy.GetAttr().SetConnections(connections)
                elif original.Get() is not None:copy.Set(original.Get())
            ambient.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*(np.array(spec['color'][:3])*AMBIENT_TEXTURE_STRENGTH),1.))
            ambient.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
            shader.GetInput('emissiveColor').ConnectToSource(ambient.ConnectableAPI(),'rgb')
            if spec['alpha_clip']:
                shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'a');shader.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(spec['cutoff'])
        return mat
    def author_visuals(self,stage,m,s,entries,ground,transform):
        from geomonoplan_profile import RENDER_ENV,AMBIENT_TEXTURE_STRENGTH
        m.layout['render_profile']=dict(RENDER_ENV,ambient_texture_strength=AMBIENT_TEXTURE_STRENGTH,real_time_shadows_preserved=True)
        used=set();library=UsdGeom.Xform.Define(stage,'/World/GeoLibrary');library.MakeInvisible();prototypes={}
        for i,r in enumerate(entries):
            token=(r['mesh'],tuple(r['materials']))
            if token not in prototypes:
                path='/World/GeoLibrary/P_'+str(len(prototypes));prototypes[token]=path;UsdGeom.Xform.Define(stage,path)
                mesh=UsdGeom.Mesh.Define(stage,path+'/Mesh');points=self.points(r['mesh']);mesh.CreatePointsAttr(points.tolist());groups=self.triangles(r['mesh']);tri=np.concatenate(groups);mesh.CreateFaceVertexCountsAttr([3]*len(tri));mesh.CreateFaceVertexIndicesAttr(tri.ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
                spec=self.meshes[r['mesh']];prefix=spec['prefix']
                if prefix+'_normals' in self.arrays:mesh.CreateNormalsAttr((self.arrays[prefix+'_normals']@C.T).tolist());mesh.SetNormalsInterpolation('vertex')
                if prefix+'_uv' in self.arrays:UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(self.arrays[prefix+'_uv'].tolist())
                offset=0
                for j,faces in enumerate(groups):
                    key=r['materials'][min(j,len(r['materials'])-1)] if r['materials'] else None
                    if key and len(faces):
                        subset=UsdGeom.Subset.CreateGeomSubset(mesh,'Material_'+str(j),UsdGeom.Tokens.face,list(range(offset,offset+len(faces))),UsdShade.Tokens.materialBind,UsdGeom.Tokens.partition)
                        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(self.material(stage,key,used))
                    offset+=len(faces)
            prim=stage.DefinePrim('/World/Imported/R_'+str(i),'Xform');prim.GetReferences().AddInternalReference(prototypes[token]);prim.SetInstanceable(True)
            UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(*np.asarray(r['matrix']).T.ravel()))
            prim.CreateAttribute('geomonoplan:sourceName',Sdf.ValueTypeNames.String).Set(r['name'])
        if ground is not None:
            n=ground.shape[0];mesh=UsdGeom.Mesh.Define(stage,'/World/Imported/Terrain');mesh.CreatePointsAttr(ground.reshape(-1,3).tolist())
            ix=np.arange(n*n).reshape(n,n);a,b,c,d=ix[:-1,:-1].ravel(),ix[1:,:-1].ravel(),ix[1:,1:].ravel(),ix[:-1,1:].ravel();tri=np.column_stack([np.column_stack([a,b,c]),np.column_stack([a,c,d])]).reshape(-1,3)
            # Input grid axes map to Isaac +X/-Y. Its winding is reversed once.
            mesh.CreateFaceVertexCountsAttr([3]*len(tri));mesh.CreateFaceVertexIndicesAttr(tri[:,::-1].ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            layer=s['terrain']['layers'][0];uv=ground[:,:,:2].reshape(-1,2)/np.array(layer['tile_size']);UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(uv.tolist())
            key='Terrain_'+m.name;self.data['materials'][key]=dict(color=[1,1,1,1],roughness=.95,metallic=0,texture=layer['texture'],uv_scale=[1,1],uv_offset=[0,0],alpha_clip=False)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(self.material(stage,key,used))
        if m.name=='geo_garage':
            for i,light in enumerate(s['lights'][::3]):
                if light['type']==1:continue
                p=matrix_points(np.array(light['matrix'])[:3,3][None]@C.T,transform)[0]
                rect=UsdLux.RectLight.Define(stage,'/World/GeoCeilingLight_'+str(i));rect.CreateWidthAttr(4);rect.CreateHeightAttr(2);rect.CreateIntensityAttr(700);rect.CreateExposureAttr(4);rect.CreateColorAttr(Gf.Vec3f(*light['color']));UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(*p))
        m.layout['embedded_prototypes']=len(prototypes)
        return {'../geomonoplan_assets/'+name:sha(ASSETS/name) for name in sorted(used)}

def registry():
    entries=[]
    for name in ['lab','hangar','forest']+NAMES:
        p=ROOT/'scenes'/('reference_'+name)/'manifest.json'
        if p.exists():
            m=json.loads(p.read_text());entries.append('    {"%s", "%s", {%s}, {%s}, %.17g},' % (m['scene_sha256'],m['occupancy_sha256'],','.join(map(str,m['shape'])),','.join(map(str,m['origin'])),m['resolution']))
    p=ROOT/'src/pc_gvf/config/generated_scene_identity/reference_scene_registry.inc';tmp=p.with_suffix('.tmp');tmp.write_text('// Generated reference and GeoMonoPlan exact-byte startup allowlist.\nstruct ReferenceSceneIdentity {\n    const char* scene; const char* occupancy; int shape[3]; double origin[3]; double resolution;\n};\nstatic const ReferenceSceneIdentity kReferenceScenes[] = {\n'+'\n'.join(entries)+'\n};\n');tmp.replace(p)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=ASSETS);p.add_argument('--scene',choices=['all']+NAMES,default='all');a=p.parse_args();adapter=Adapter(a.source)
    for s in adapter.data['scenes']:
        if a.scene in ['all',s['name']]:build(adapter.model(s))
    registry()
