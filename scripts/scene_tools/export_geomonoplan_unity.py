#!/usr/bin/env python3
"""Read the official AeroScene data with UnityPy; never execute Unity code.

UnityPy is an offline conversion dependency, installed separately from ROS/Isaac.
Outputs compact mesh arrays, 512px textures, transforms and source provenance.
"""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
import UnityPy
from UnityPy.helpers.MeshHelper import MeshHandler

CONVERSION=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,1.,0.]])

def vec(v,keys='xyz'):return np.array([v[k] for k in keys],dtype=float)

def trs(d):
    x,y,z,w=vec(d['m_LocalRotation'],'xyzw')
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    m=np.eye(4);m[:3,:3]=r@np.diag(vec(d['m_LocalScale']));m[:3,3]=vec(d['m_LocalPosition']);return m

class Exporter:
    def __init__(self,source,out):
        self.env=UnityPy.load(str(source));self.out=out;out.mkdir(parents=True,exist_ok=True)
        self.files={Path(n).name:f for n,f in self.env.files.items() if hasattr(f,'objects')}
        self.cache={};self.matrices={};self.arrays={};self.meshes={};self.materials={};self.textures={};self.decoded_meshes={}
    def ref(self,f,p):
        if not p or not p.get('m_PathID'):return None
        target=f if p['m_FileID']==0 else self.files[Path(f.externals[p['m_FileID']-1].path).name]
        return target,target.objects[p['m_PathID']]
    def read(self,f,o):
        key=(f.name,o.path_id)
        if key not in self.cache:self.cache[key]=o.read_typetree()
        return self.cache[key]
    def components(self,f,go):
        d=self.read(f,go);result={}
        for p in d['m_Component']:
            target,obj=self.ref(f,p['component']);result[obj.type.name]=(target,obj)
        return result
    def transform(self,f,o):
        key=(f.name,o.path_id)
        if key not in self.matrices:
            d=self.read(f,o);m=trs(d);parent=self.ref(f,d['m_Father'])
            if parent:m=self.transform(*parent)@m
            self.matrices[key]=m
        return self.matrices[key]
    def active(self,f,go):
        if not self.read(f,go)['m_IsActive']:return False
        comp=self.components(f,go);tr=comp.get('Transform')
        if tr:
            parent=self.ref(f,self.read(*tr)['m_Father'])
            if parent:
                pg=self.ref(parent[0],self.read(*parent)['m_GameObject'])
                return self.active(*pg)
        return True
    def texture(self,f,p):
        ref=self.ref(f,p)
        if not ref:return None
        target,obj=ref;key=Path(target.name).name+'_'+str(obj.path_id)
        if key not in self.textures:
            d=obj.read();image=d.image
            if image is None:raise ValueError('Texture decoding failed: '+key)
            image.thumbnail((512,512));name=key+'.png';image.save(self.out/name)
            self.textures[key]=dict(file=name,name=d.m_Name,size=list(image.size),sha256=hashlib.sha256((self.out/name).read_bytes()).hexdigest())
        return self.textures[key]['file']
    def material(self,f,p):
        ref=self.ref(f,p)
        if not ref:return None
        target,obj=ref;key=Path(target.name).name+'_'+str(obj.path_id)
        if key not in self.materials:
            d=self.read(target,obj);props=d['m_SavedProperties'];colors=dict(props.get('m_Colors',[]));floats=dict(props.get('m_Floats',[]));tex=dict(props.get('m_TexEnvs',[]))
            color=colors.get('_BaseColor',colors.get('_Color',dict(r=1,g=1,b=1,a=1)))
            diffuse=tex.get('_BaseMap',tex.get('_MainTex'))
            self.materials[key]=dict(name=d['m_Name'],color=vec(color,'rgba').tolist(),
                metallic=floats.get('_Metallic',0),roughness=1-floats.get('_Smoothness',floats.get('_Glossiness',.2)),
                cutoff=floats.get('_AlphaClipThreshold',floats.get('_Cutoff',.5)),alpha_clip=bool(floats.get('_AlphaClip',floats.get('_AlphaCutoffEnable',0)) or floats.get('_Mode')==1 or any(token in d['m_Name'].lower() for token in ['leaf','leav','foliage'])),
                texture=self.texture(target,diffuse['m_Texture']) if diffuse else None,
                uv_scale=vec(diffuse['m_Scale'],'xy').tolist() if diffuse else [1,1],
                uv_offset=vec(diffuse['m_Offset'],'xy').tolist() if diffuse else [0,0])
        return key
    def mesh(self,f,p,selection=None):
        target,obj=self.ref(f,p);key=Path(target.name).name+'_'+str(obj.path_id)
        if selection is not None:key+='_'+str(selection[0])+'_'+str(selection[1])
        if key not in self.meshes:
            source_key=(target.name,obj.path_id)
            if source_key not in self.decoded_meshes:
                data=obj.read();handler=MeshHandler(data);handler.process()
                self.decoded_meshes[source_key]=(handler,handler.get_triangles(),data.m_Name)
            handler,groups,mesh_name=self.decoded_meshes[source_key]
            if selection is not None:groups=groups[selection[0]:selection[0]+selection[1]]
            points=np.array(handler.m_Vertices,dtype=np.float32)
            triangles=[np.asarray(x,dtype=np.int32).reshape(-1,3) for x in groups]
            if not triangles or not len(points):raise ValueError('Empty mesh: '+key)
            used=np.unique(np.concatenate(triangles));mapping=np.full(len(points),-1,np.int32);mapping[used]=np.arange(len(used))
            prefix='m'+str(len(self.meshes));self.arrays[prefix+'_points']=points[used]
            if handler.m_Normals:self.arrays[prefix+'_normals']=np.array(handler.m_Normals,dtype=np.float32)[used]
            if handler.m_UV0:self.arrays[prefix+'_uv']=np.array(handler.m_UV0,dtype=np.float32)[used,:2]
            counts=[]
            for i,tri in enumerate(triangles):self.arrays[prefix+'_tri_'+str(i)]=mapping[tri];counts.append(len(tri))
            self.meshes[key]=dict(prefix=prefix,groups=len(triangles),triangles=sum(counts),name=mesh_name)
        return key
    def renderer(self,f,go,relative=None):
        comps=self.components(f,go)
        if 'MeshFilter' not in comps or 'MeshRenderer' not in comps:return None
        rf,ro=comps['MeshRenderer'];d=self.read(rf,ro)
        if not d['m_Enabled']:return None
        mf,mo=comps['MeshFilter'];mesh=self.read(mf,mo)['m_Mesh']
        if not mesh.get('m_PathID'):return None
        batch=d['m_StaticBatchInfo'];n=batch['subMeshCount']
        matrix=np.eye(4) if n else self.transform(*comps['Transform'])
        if relative is not None:matrix=relative@matrix
        return dict(name=self.read(f,go)['m_Name'],mesh=self.mesh(mf,mesh,(batch['firstSubMesh'],n) if n else None),
            matrix=matrix.tolist(),materials=[self.material(rf,p) for p in d['m_Materials']],static_batch=bool(n),source_id=go.path_id)
    def tree(self,f,p):
        target,go=self.ref(f,p);comp=self.components(target,go);root=comp['Transform'];root_inv=np.linalg.inv(self.transform(*root));result=[]
        preferred=None
        if 'LODGroup' in comp:
            ld=self.read(*comp['LODGroup'])['m_LODs'];index=min(1,len(ld)-1)
            preferred={r.get('renderer',r)['m_PathID'] for r in ld[index]['renderers']}
        def visit(ff,obj):
            cc=self.components(ff,obj)
            if 'MeshRenderer' in cc and (preferred is None or cc['MeshRenderer'][1].path_id in preferred):
                entry=self.renderer(ff,obj,root_inv)
                if entry:result.append(entry)
            if 'Transform' in cc:
                for child in self.read(*cc['Transform'])['m_Children']:
                    cf,co=self.ref(ff,child);gf,gg=self.ref(cf,self.read(cf,co)['m_GameObject']);visit(gf,gg)
        visit(target,go)
        if not result:raise ValueError('Tree prototype has no LOD1 renderers: '+self.read(target,go)['m_Name'])
        return dict(name=self.read(target,go)['m_Name'],renderers=result)
    def scene(self,name,level):
        f=self.files[level];instances=[];terrain=None;lights=[]
        for go in f.objects.values():
            if go.type.name!='GameObject' or not self.active(f,go):continue
            comps=self.components(f,go);d=self.read(f,go)
            entry=self.renderer(f,go)
            if entry:instances.append(entry)
            if 'Terrain' in comps:
                tf,to=comps['Terrain'];td=self.read(tf,to);df,do=self.ref(tf,td['m_TerrainData']);data=self.read(df,do)
                hm=data['m_Heightmap'];res=hm['m_Resolution'];heights=np.asarray(hm['m_Heights'],np.float32).reshape(res,res)/32767*hm['m_Scale']['y']
                self.arrays[name+'_heights']=heights
                db=data['m_DetailDatabase'];prototypes=[self.tree(df,p['prefab']) for p in db['m_TreePrototypes']]
                terrain=dict(resolution=res,size=[hm['m_Scale']['x']*(res-1),hm['m_Scale']['y'],hm['m_Scale']['z']*(res-1)],matrix=self.transform(*comps['Transform']).tolist(),trees=db['m_TreeInstances'],prototypes=prototypes)
                layers=data['m_SplatDatabase']['m_TerrainLayers']
                terrain['layers']=[]
                for lp in layers:
                    lf,lo=self.ref(df,lp);ld=self.read(lf,lo)
                    terrain['layers'].append(dict(name=ld['m_Name'],texture=self.texture(lf,ld['m_DiffuseTexture']),tile_size=vec(ld['m_TileSize'],'xy').tolist()))
            if 'Light' in comps:
                ld=self.read(*comps['Light'])
                if ld['m_Enabled']:lights.append(dict(matrix=self.transform(*comps['Transform']).tolist(),type=ld['m_Type'],color=vec(ld['m_Color'],'rgb').tolist(),intensity=ld['m_Intensity']))
        print(name,len(instances),'renderers',len(terrain['trees']) if terrain else 0,'trees',len(lights),'lights',flush=True)
        return dict(name=name,source_level=level,instances=instances,terrain=terrain,lights=lights)
    def run(self):
        scenes=[self.scene(n,l) for n,l in [('geo_forest','level1'),('geo_clutter','level2'),('geo_garage','level3')]]
        np.savez_compressed(self.out/'meshes.npz',**self.arrays)
        (self.out/'scene_data.json').write_text(json.dumps(dict(scenes=scenes,meshes=self.meshes,materials=self.materials,textures=self.textures,coordinate_conversion=CONVERSION.tolist(),tree_lod=1),separators=(',',':')))
        print('exported',len(self.meshes),'meshes',len(self.materials),'materials',len(self.textures),'textures',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();Exporter(a.source,a.out).run()
