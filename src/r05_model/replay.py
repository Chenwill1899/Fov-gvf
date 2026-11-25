"""Re-render an audited P3 run using the exact original R05 presentation rules."""
import argparse,csv,hashlib,json,math,os,subprocess,sys
from pathlib import Path
import numpy as np
from PIL import Image
from presentation import ROOT,WIDTH,HEIGHT,FPS,ReferenceCamera,attach_arl,configure_camera,set_camera,make_hud

p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--preview',action='store_true');p.add_argument('--validate-depth',action='store_true');a=p.parse_args()
run=a.run.resolve();out=a.output.resolve();scene=ROOT/'scenes/r05_cloud/ego_swarm_cloud_navigation.usd'
digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
result=json.loads((run/'simulation/result.json').read_text());execution=json.loads((run/'execution.json').read_text());audit=json.loads((run/'simulation/audit.json').read_text())
if not execution.get('audit_passed') or result['status']!='ARRIVED' or result['scene_sha256']!=digest(scene):raise ValueError('requires an audited ARRIVED run of the original local Cloud scene')
trace=list(csv.DictReader((run/'simulation/trajectory.csv').open()));times=np.array([float(r['t']) for r in trace]);positions=np.array([[float(r[k]) for k in 'xyz'] for r in trace])
if 'yaw_rad' not in trace[0] or len(trace)-1!=audit['steps']:raise ValueError('recorded yaw/trajectory incomplete')
# Show the recorded endpoint on the first video frame at or after arrival.
# That frame can occur after the last physics tick when the clocks differ.
last_frame=math.ceil(round(float(times[-1])*FPS, 10))
count=last_frame+1
targets=np.minimum(np.arange(count)/FPS,times[-1])
idx=np.minimum(np.searchsorted(times,targets),len(times)-1);earlier=np.maximum(0,idx-1);idx=np.where(abs(times[earlier]-targets)<abs(times[idx]-targets),earlier,idx)
if np.max(np.abs(times[idx]-targets))>.00250001:raise ValueError('video samples are not aligned to recorded 5ms poses')
out.mkdir(parents=True,exist_ok=False);portable=out/'kit';portable.mkdir();(portable/'cache').symlink_to(ROOT/'docs/performance/isaac_closed_loop_20261117/shared_cache/kit',target_is_directory=True)
sys.argv=[sys.argv[0]]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','active_gpu':0,'multi_gpu':False,'limit_cpu_threads':8,
    'width':WIDTH,'height':HEIGHT,'disable_viewport_updates':True,'anti_aliasing':1,
    'extra_args':['--portable-root',str(portable),'--/app/settings/persistent=false','--/log/file='+str(out/'render.log')]})
encoder=None
try:
 import omni.usd,omni.replicator.core as rep
 from pxr import Gf,UsdGeom
 from imageio_ffmpeg import get_ffmpeg_exe
 if not omni.usd.get_context().open_stage(str(scene)):raise RuntimeError('scene open failed')
 for _ in range(8):app.update()
 app.reset_render_settings();stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 robot=UsdGeom.XformCommonAPI(stage.GetPrimAtPath('/World/Robot'))
 def pose(i):
  robot.SetTranslate(Gf.Vec3d(*positions[i]));robot.SetRotate(Gf.Vec3f(0,0,math.degrees(float(trace[i]['yaw_rad']))),UsdGeom.XformCommonAPI.RotationOrderXYZ)
 pose(0);rep.orchestrator.set_capture_on_play(False)
 depth_results=[]
 if a.validate_depth:
  sides=['','Left','Back','Right'];products=[];sensors=[]
  for side in sides:
   product=rep.create.render_product('/World/Robot/CameraMount'+side+'/DepthCamera',(320,240));sensor=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');sensor.attach(product);products.append(product);sensors.append(sensor)
  baseline=[]
  sample_ids=[0,len(trace)//4,len(trace)//2,3*len(trace)//4,len(trace)-1]
  for i in sample_ids:
   pose(i)
   for _ in range(3):rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False,wait_for_render=True)
   baseline.append([np.asarray(sensor.get_data()).copy() for sensor in sensors])
 visual=attach_arl(stage)
 if a.validate_depth:
  for k,i in enumerate(sample_ids):
   pose(i)
   for _ in range(3):rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False,wait_for_render=True)
   for c,sensor in enumerate(sensors):
    before=baseline[k][c];after=np.asarray(sensor.get_data());finite=np.isfinite(before)&(before>0);valid=np.isfinite(after)&(after>0)
    changed=int(np.count_nonzero(finite!=valid));both=finite&valid
    delta=float(np.max(np.abs(before[both]-after[both]))) if np.any(both) else 0.
    depth_results.append(dict(step=int(trace[i]['step']),camera=c,changed_validity_pixels=changed,max_depth_difference_m=delta,changed_pixels_over_1um=int(np.count_nonzero(np.abs(before[both]-after[both])>1e-6))))
  for sensor,product in zip(sensors,products):sensor.detach(product);product.destroy()
  (out/'depth_visual_check.json').write_text(json.dumps(depth_results,indent=2)+'\n')
  if any(r['changed_validity_pixels'] or r['changed_pixels_over_1um'] for r in depth_results):raise RuntimeError('new visual changes raw camera depth; see depth_visual_check.json')
 camera=configure_camera(stage,'/World/R05ReferenceCamera');grid=np.memmap(ROOT/'scenes/r05_cloud/occupancy.bin',dtype=np.uint8,mode='r',shape=(1200,300,50));rule=ReferenceCamera(grid)
 eyes,aims=rule.path(positions[idx],np.array([float(trace[i]['yaw_rad']) for i in idx]))
 pose(0);set_camera(camera,eyes[0],aims[0])
 product=rep.create.render_product(str(camera.GetPath()),(WIDTH,HEIGHT));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
 def render():
  rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False,wait_for_render=True)
  pixels=np.asarray(rgb.get_data())
  if pixels.shape[:2]!=(HEIGHT,WIDTH):raise RuntimeError('empty/wrong RGB output')
  return Image.fromarray(np.ascontiguousarray(pixels[:,:,:3]))
 for _ in range(4):render()
 hud=make_hud(trace,result);order=sorted({0,count//4,count//2,3*count//4,count-1}) if a.preview else range(count)
 video=out/'R05_EGO4P3_ARL_reference_view.mp4';pending=video.with_suffix('.partial.mp4');ff=get_ffmpeg_exe()
 if not a.preview:
  encoder=subprocess.Popen([ff,'-nostdin','-v','error','-n','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{WIDTH}x{HEIGHT}','-framerate',str(FPS),'-i','pipe:0','-an','-c:v','libx264','-threads','6','-preset','fast','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(pending)],stdin=subprocess.PIPE)
 with (out/'frames.csv').open('w',newline='') as f:
  writer=csv.writer(f);writer.writerow(['frame','trace_step','sample_time_s','playback_time_s','camera_x','camera_y','camera_z','line_occupied_samples'])
  for n in order:
   i=idx[n];pose(i);set_camera(camera,eyes[n],aims[n]);im=hud(render(),dict(frame=n,trace_step=int(i)),closing=n==count-1).convert('RGB')
   writer.writerow([n,int(trace[i]['step']),times[i],n/FPS,*eyes[n],int(rule.occluded(eyes[n][None,:],positions[i])[0])])
   if a.preview or n in {0,count//4,count//2,3*count//4,count-1}:im.save(out/f'view_{n:05d}.jpg',quality=95)
   if encoder:encoder.stdin.write(im.tobytes())
   if n%90==0:print(f'render {n+1}/{count}',flush=True)
 if encoder:
  for _ in range(60):encoder.stdin.write(im.tobytes())
  encoder.stdin.close()
  if encoder.wait(timeout=120):raise RuntimeError('video encoder failed')
  subprocess.run([ff,'-nostdin','-v','error','-xerror','-i',str(pending),'-f','null','-'],check=True)
  pending.rename(video)
 receipt=dict(source_run=str(run),source_trajectory_sha256=digest(run/'simulation/trajectory.csv'),source_scene_sha256=digest(scene),visual=visual,
  reference_camera='exact recovered final 2026-11-07 168-offset dynamic programming and Gaussian smoothing',
  dimensions=[WIDTH,HEIGHT],fps=FPS,source_frames=count,closing_hold_frames=60,total_frames=count+60,
  simulation_seconds=float(times[-1]),video_seconds=(count+60)/FPS,endpoint_frame_delay_s=last_frame/FPS-float(times[-1]),
  max_pose_sample_error_s=float(np.max(np.abs(times[idx]-targets))),
  preview_only=a.preview,full_decode_passed=not a.preview,algorithm_executed=False,visual_depth_check=depth_results,
  source_model_physics_unchanged=True,video_sha256=digest(video) if video.is_file() else None)
 (out/'video.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n');print('OUTPUT_DIR='+str(out),flush=True)
except BaseException as error:
 import traceback
 (out/'failure.json').write_text(json.dumps(dict(error=repr(error),traceback=traceback.format_exc()),indent=2));traceback.print_exc();sys.stderr.flush();raise
finally:
 if encoder and encoder.poll() is None:encoder.kill();encoder.wait()
 app.close()
