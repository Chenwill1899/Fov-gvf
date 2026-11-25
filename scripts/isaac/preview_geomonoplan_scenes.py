#!/usr/bin/env python3
"""Render adapted AeroScene worlds and check all 12 native depth views."""
import argparse,json,os,sys
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/scene_tools'))
from geomonoplan_profile import RENDER_ENV
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':2,'active_gpu':0,'multi_gpu':False,'extra_args':['--portable-root',os.environ['EGO4P0_RUNTIME']+'/kit','--/app/settings/persistent=false','--/log/file='+str(out/'render.log')]})
from PIL import Image
from pxr import UsdGeom,UsdLux,Gf
import omni.usd
import omni.replicator.core as rep
import carb
settings=carb.settings.get_settings()
settings.set('/rtx/sceneDb/ambientLightColor',(1.,1.,1.))
settings.set('/rtx/sceneDb/ambientLightIntensity',float(RENDER_ENV['ISAAC_AMBIENT_LIGHT_INTENSITY']))
for flag in ['/rtx/directLighting/sampledLighting/enabled','/rtx/reflections/enabled','/rtx/reflections/sampledLighting/enabled','/rtx/ambientOcclusion/enabled','/rtx/indirectDiffuse/enabled']:settings.set(flag,False)
report={}
def capture(annotator):
 for attempt in range(12):
  rep.orchestrator.step(rt_subframes=8)
  for _ in range(12):app.update()
  pixels=np.asarray(annotator.get_data()).copy()
  if pixels.size and np.any(pixels!=0):return pixels
 raise RuntimeError('Renderer did not produce an annotator frame after 12 captures')
try:
 for name in ['geo_forest','geo_clutter','geo_garage']:
  folder=ROOT/'scenes'/('reference_'+name);m=json.loads((folder/'manifest.json').read_text());omni.usd.get_context().open_stage(str(folder/m['scene']))
  for frame in range(1800):
   app.update()
   if frame>40 and not omni.usd.get_context().get_stage_loading_status()[2]:break
  else:raise RuntimeError('Scene assets did not finish loading')
  print(name,'USD loaded',omni.usd.get_context().get_stage_loading_status(),flush=True)
  s=omni.usd.get_context().get_stage();s.SetEditTarget(s.GetSessionLayer())
  settings.set('/rtx/shadows/enabled',True)
  for prim in s.Traverse():
   if prim.IsA(UsdLux.DistantLight):
    fill=prim.GetAttribute('geomonoplan:ambientFill')
    if not (fill and fill.Get()):UsdLux.DistantLight(prim).GetIntensityAttr().Set(float(RENDER_ENV['ISAAC_SUN_INTENSITY']))
   elif prim.IsA(UsdLux.DomeLight):UsdLux.DomeLight(prim).GetIntensityAttr().Set(float(RENDER_ENV['ISAAC_DOME_INTENSITY']))
  camera=UsdGeom.Camera.Define(s,'/World/GeoPreviewCamera');camera.CreateFocalLengthAttr(22);camera.CreateHorizontalApertureAttr(36);camera.CreateVerticalApertureAttr(20.25);camera.CreateClippingRangeAttr(Gf.Vec2f(.05,1000));op=UsdGeom.Xformable(camera).AddTransformOp()
  product=rep.create.render_product(str(camera.GetPath()),(1280,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
  cutaway=[]
  for view in ['overview','eye_level','robot']:
   if view=='overview':
    if name=='geo_garage':
     for prim in s.GetPrimAtPath('/World/Imported').GetChildren():
      attr=prim.GetAttribute('geomonoplan:sourceName');label=attr.Get() if attr else ''
      if label.lower().startswith(('ceiling','north_','south_','east_','west_')):UsdGeom.Imageable(prim).MakeInvisible();cutaway.append(prim)
    eye=m['overview_eye'];target=m['overview_target']
   else:
    for prim in cutaway:UsdGeom.Imageable(prim).MakeVisible()
    spawn=np.array(m['spawn'])
    if view=='eye_level':eye=spawn;target=spawn+[12,0,0];UsdGeom.Imageable(s.GetPrimAtPath('/World/Robot')).MakeInvisible()
    else:eye=spawn+[1.05,-1.2,.85];target=spawn+[0,0,-.16];UsdGeom.Imageable(s.GetPrimAtPath('/World/Robot')).MakeVisible()
   matrix=Gf.Matrix4d(1);matrix.SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1));op.Set(matrix.GetInverse())
   pixels=capture(rgb);Image.fromarray(pixels[:,:,:3]).save(out/f'{name}_{view}.png')
  rgb.detach(product);product.destroy();UsdGeom.Imageable(s.GetPrimAtPath('/World/Robot')).MakeVisible();visual=UsdGeom.Imageable(s.GetPrimAtPath('/World/Robot/Visuals'));views={}
  for side,mount in [('front','CameraMount'),('left','CameraMountLeft'),('back','CameraMountBack'),('right','CameraMountRight')]:
   product=rep.create.render_product('/World/Robot/'+mount+'/DepthCamera',(320,240));annotator=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');annotator.attach(product);frames=[]
   for visible in [True,False]:
    visual.MakeVisible() if visible else visual.MakeInvisible()
    frames.append(capture(annotator))
   d,h=frames;finite=np.isfinite(d)&(d>0);other=np.isfinite(h)&(h>0);both=finite&other
   changed=int(np.sum((finite!=other)|(both&(np.abs(np.nan_to_num(d,posinf=0)-np.nan_to_num(h,posinf=0))>.01))))
   views[side]=dict(shape=list(d.shape),valid_pixels=int(finite.sum()),minimum_m=float(d[finite].min()) if finite.any() else None,self_hits_under_1m=int(np.sum(finite&(d<1))),visibility_changed_pixels_over_1cm=changed)
   # Open scene boundaries can legitimately return only sky (+inf). A complete
   # depth frame and the visibility A/B check still validate that direction.
   assert d.shape==(240,320) and not np.isnan(d).any(),views[side]
   assert not views[side]['self_hits_under_1m'] and not changed,views[side]
   np.save(out/f'{name}_depth_{side}.npy',d);Image.fromarray((255*(1-np.clip(np.nan_to_num(d,nan=20,posinf=20),0,20)/20)).astype('uint8')).save(out/f'{name}_depth_{side}.png')
   visual.MakeVisible();annotator.detach(product);product.destroy()
  assert any(v['valid_pixels'] for v in views.values()),'Every direction has no visible environment'
  report[name]=views;(out/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(name,views,flush=True)
except Exception:
 import traceback
 (out/'render_error.txt').write_text(traceback.format_exc());raise
finally:app.close()
