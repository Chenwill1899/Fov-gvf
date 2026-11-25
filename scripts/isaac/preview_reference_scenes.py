#!/usr/bin/env python3
"""Render the delivered USDs and record four actual depth views; no ROS control."""
import argparse
import json
import os
from pathlib import Path
from isaacsim import SimulationApp

ROOT=Path(__file__).resolve().parents[2]
cli=argparse.ArgumentParser(description=__doc__)
cli.add_argument('--scene',choices=['all','lab','hangar','forest','cloud'],default='all')
cli.add_argument('--out',type=Path,default=ROOT/'docs/performance/reference_scenes_20261022/preview')
args=cli.parse_args()
OUT=args.out.resolve()
OUT.mkdir(parents=True,exist_ok=True)
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':2,
    'active_gpu':0,'multi_gpu':False,'extra_args':['--portable-root',os.environ['EGO4P0_RUNTIME']+'/kit',
    '--/app/settings/persistent=false','--/log/file='+str(OUT/'render.log')]})
import numpy as np
from PIL import Image
from pxr import Usd, UsdGeom, Gf
import omni.usd
import omni.replicator.core as rep

report={}
try:
    for name in (['cloud'] if args.scene=='cloud' else ['lab','hangar','forest']):
        if args.scene not in ['all',name]:continue
        folder=ROOT/'scenes'/('ego_swarm_cloud' if name=='cloud' else f'reference_{name}')
        m=json.loads((folder/'manifest.json').read_text())
        omni.usd.get_context().open_stage(str(folder/m['scene']))
        for _ in range(30):app.update()
        stage=omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        camera=UsdGeom.Camera.Define(stage,'/World/PreviewCamera')
        camera.CreateFocalLengthAttr(22);camera.CreateHorizontalApertureAttr(36)
        camera.CreateVerticalApertureAttr(20.25);camera.CreateClippingRangeAttr(Gf.Vec2f(.1,1000))
        op=UsdGeom.Xformable(camera).AddTransformOp()
        product=rep.create.render_product(str(camera.GetPath()),(1280,720))
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
        for view in ['overview','eye_level']:
            hidden=[]
            if view=='overview':
                # Explicit cutaway for documentation only; saved map remains closed.
                for prim in stage.Traverse():
                    primname=prim.GetName()
                    cutaway=primname in ['Roof','WallX_0','WallY_0']
                    if name=='lab':
                        cutaway |= primname.startswith(('Beam_','Light_','CeilingLight_')) or 'CeilingFixture_' in primname
                    if cutaway:UsdGeom.Imageable(prim).MakeInvisible();hidden.append(prim)
                eye=m['overview_eye'];target=m['overview_target']
            else:
                for primname in ['Roof','WallX_0','WallY_0']:
                    prim=stage.GetPrimAtPath('/World/Environment/'+primname)
                    if prim:UsdGeom.Imageable(prim).MakeVisible()
                eye=m['first_person_eye'];target=m['first_person_target']
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Robot')).MakeInvisible()
            matrix=Gf.Matrix4d(1);matrix.SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1));op.Set(matrix.GetInverse())
            rep.orchestrator.step(rt_subframes=16)
            for _ in range(10):app.update()
            image=rgb.get_data()
            if image.size==0:raise RuntimeError('empty rendered image')
            Image.fromarray(image[:,:,:3]).save(OUT/f'{name}_{view}.png')
            for prim in hidden:UsdGeom.Imageable(prim).MakeVisible()
        rgb.detach(product);product.destroy()
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Robot')).MakeVisible()
        views={}
        for side,mount in [('front','CameraMount'),('left','CameraMountLeft'),('back','CameraMountBack'),('right','CameraMountRight')]:
            rp=rep.create.render_product(f'/World/Robot/{mount}/DepthCamera',(320,240))
            depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth.attach(rp)
            rep.orchestrator.step(rt_subframes=4)
            for _ in range(10):app.update()
            d=np.asarray(depth.get_data()).copy();finite=np.isfinite(d)&(d>0)
            views[side]={'shape':list(d.shape),'valid_pixels':int(finite.sum()),
                'min_m':float(d[finite].min()) if finite.any() else None,'max_m':float(d[finite].max()) if finite.any() else None}
            np.save(OUT/f'{name}_depth_{side}.npy',d)
            viz=(255*(1-np.clip(np.nan_to_num(d,nan=20,posinf=20),0,20)/20)).astype('uint8')
            Image.fromarray(viz).save(OUT/f'{name}_depth_{side}.png')
            depth.detach(rp);rp.destroy()
        report[name]=views
        print(name,views,flush=True)
    (OUT/'render_validation.json').write_text(json.dumps(report,indent=2)+'\n')
except Exception:
    import traceback
    traceback.print_exc()
    (OUT/'render_error.txt').write_text(traceback.format_exc())
finally:app.close()
