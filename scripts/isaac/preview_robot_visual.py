#!/usr/bin/env python3
"""Native ARL close-ups and depth A/B self-occlusion checks for all active maps."""
import argparse,json,os
from pathlib import Path
from isaacsim import SimulationApp

ROOT=Path(__file__).resolve().parents[2]
cli=argparse.ArgumentParser();cli.add_argument('--out',type=Path,required=True);args=cli.parse_args()
OUT=args.out.resolve();OUT.mkdir(parents=True,exist_ok=True)
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':2,
    'active_gpu':0,'multi_gpu':False,'extra_args':['--portable-root',os.environ['EGO4P0_RUNTIME']+'/kit',
    '--/app/settings/persistent=false','--/log/file='+str(OUT/'render.log')]})
import numpy as np
from PIL import Image
from pxr import UsdGeom,Gf
import omni.usd
import omni.replicator.core as rep

report={}
try:
    for name in ['forest','lab','hangar','cloud']:
        folder=ROOT/'scenes'/('ego_swarm_cloud' if name=='cloud' else f'reference_{name}')
        m=json.loads((folder/'manifest.json').read_text())
        omni.usd.get_context().open_stage(str(folder/m['scene']))
        for _ in range(30):app.update()
        stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
        visual=UsdGeom.Imageable(stage.GetPrimAtPath('/World/Robot/Visuals'))
        camera=UsdGeom.Camera.Define(stage,'/World/RobotInspectionCamera')
        camera.CreateFocalLengthAttr(32);camera.CreateHorizontalApertureAttr(36)
        camera.CreateVerticalApertureAttr(20.25);camera.CreateClippingRangeAttr(Gf.Vec2f(.02,1000))
        op=UsdGeom.Xformable(camera).AddTransformOp()
        spawn=np.array(m['spawn']);target=spawn+[0,0,-.16];eye=spawn+[1.05,-1.2,.85]
        matrix=Gf.Matrix4d(1);matrix.SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1));op.Set(matrix.GetInverse())
        product=rep.create.render_product(str(camera.GetPath()),(1280,720))
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
        rep.orchestrator.step(rt_subframes=16)
        for _ in range(10):app.update()
        pixels=np.asarray(rgb.get_data())
        if not pixels.size:raise RuntimeError('Empty robot RGB')
        Image.fromarray(pixels[:,:,:3]).save(OUT/f'{name}_robot.png')
        rgb.detach(product);product.destroy()
        views={}
        for side,mount in [('front','CameraMount'),('left','CameraMountLeft'),('back','CameraMountBack'),('right','CameraMountRight')]:
            product=rep.create.render_product('/World/Robot/'+mount+'/DepthCamera',(320,240))
            sensor=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');sensor.attach(product)
            frames=[]
            for visible in [True,False]:
                visual.MakeVisible() if visible else visual.MakeInvisible()
                rep.orchestrator.step(rt_subframes=4)
                for _ in range(10):app.update()
                frames.append(np.asarray(sensor.get_data()).copy())
            a,b=frames;finite=np.isfinite(a)&(a>0);both=finite&np.isfinite(b)&(b>0)
            near=int(np.sum(finite&(a<1.)))
            changed=int(np.sum((finite!= (np.isfinite(b)&(b>0))) | (both&(np.abs(np.nan_to_num(a,posinf=0)-np.nan_to_num(b,posinf=0))>.01))))
            views[side]=dict(shape=list(a.shape),valid_pixels=int(finite.sum()),
                min_depth_m=float(a[finite].min()) if finite.any() else None,
                self_hits_under_1m=near,visibility_changed_pixels_over_1cm=changed)
            np.save(OUT/f'{name}_depth_{side}_visible.npy',a)
            np.save(OUT/f'{name}_depth_{side}_hidden.npy',b)
            Image.fromarray((255*(1-np.clip(np.nan_to_num(a,nan=20,posinf=20),0,20)/20)).astype('uint8')).save(OUT/f'{name}_depth_{side}.png')
            if near or changed:raise RuntimeError(f'{name}/{side}: visual causes depth self-occlusion: {views[side]}')
            visual.MakeVisible();sensor.detach(product);product.destroy()
        report[name]=views
        (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
        print(name,views,flush=True)
except Exception:
    import traceback
    (OUT/'render_error.txt').write_text(traceback.format_exc());traceback.print_exc();raise
finally:app.close()
