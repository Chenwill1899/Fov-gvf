#!/usr/bin/env python3
"""Blender CPU BVH ray test; no GPU, ROS or downloaded source scripts executed."""
import argparse,json,os,sys
from pathlib import Path
import numpy as np
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

cli=argparse.ArgumentParser();cli.add_argument('--out',type=Path,required=True)
args=cli.parse_args(sys.argv[sys.argv.index('--')+1:]);out=args.out.resolve()
data=np.load(out/'probe_mesh.npz');vertices=data['vertices'];faces=data['triangles']
bvh=BVHTree.FromPolygons(vertices.tolist(),faces.tolist(),all_triangles=True)
cameras=json.loads((out/'probe_cameras.json').read_text());result={}
for side,camera in cameras.items():
    matrix=np.asarray(camera['matrix']);origin=matrix[3,:3]
    u,v=np.meshgrid(np.arange(320)+.5,np.arange(240)+.5)
    rays=np.column_stack([(u.ravel()-160)/320*camera['horizontal_aperture']/camera['focal'],
        -(v.ravel()-120)/240*camera['vertical_aperture']/camera['focal'],-np.ones(320*240)])
    near=camera['clipping'][0]
    lengths=np.linalg.norm(rays,axis=1);direction=(rays/lengths[:,None])@matrix[:3,:3]
    starts=origin+direction*(near*lengths[:,None])
    hits=[]
    for i,(start,d) in enumerate(zip(starts,direction)):
        point,normal,index,distance=bvh.ray_cast(Vector(start),Vector(d),2.)
        if point is not None:hits.append(dict(pixel=[i%320,i//320],distance=float(distance)))
    result[side]=dict(rays=len(rays),self_hits=len(hits),examples=hits[:10])
    print(side,result[side],flush=True)
(out/'cpu_ray_validation.json').write_text(json.dumps(result,indent=2)+'\n')
sys.stdout.flush();os._exit(1 if any(r['self_hits'] for r in result.values()) else 0)
