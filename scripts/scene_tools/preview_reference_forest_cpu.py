#!/usr/bin/env python3
"""CPU documentation preview when another task owns the shared Isaac/GPU channel.

Run with Blender --background --disable-autoexec --python this_file -- --out DIR.
Imports the actual USD geometry/materials, recreates daylight for Cycles. This is
not an Isaac RGB/depth or closed-loop validation and must be labeled accordingly.
"""
import bpy,sys,os,json,math
from mathutils import Vector
from pathlib import Path
root=Path(__file__).resolve().parents[2]
import argparse
cli=argparse.ArgumentParser()
cli.add_argument('--out',type=Path,required=True)
args=cli.parse_args(sys.argv[sys.argv.index('--')+1:])
out=args.out.resolve()
out.mkdir(exist_ok=True)
m=json.loads((root/'scenes/reference_forest/manifest.json').read_text())
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.usd_import(filepath=str(root/'scenes/reference_forest/scene.usdc'))
print('USD imported',len(bpy.data.objects),flush=True)
# USD proxies are invisible; remove them from this CPU-only documentation render.
for obj in list(bpy.data.objects):
 if '_proxy_' in obj.name or obj.type=='LIGHT' or obj.name.startswith('Robot'):
  bpy.data.objects.remove(obj,do_unlink=True)
# Blender's USD importer does not attach materials to implicit Sphere/Capsule
# primitives. Match their manifest diffuse colors for this documentation preview.
for entry in m['objects']:
 if entry['kind'] not in ['sphere','capsule']:
  continue
 obj=bpy.data.objects.get(entry['name'])
 if obj is not None and hasattr(obj.data,'materials'):
  mat=bpy.data.materials.new(entry['name']+'_Color');mat.use_nodes=True
  mat.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value=(*entry['color'],1.)
  mat.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value=.88
  obj.data.materials.clear();obj.data.materials.append(mat)
scene=bpy.context.scene
scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=16
scene.cycles.use_denoising=True
scene.render.threads_mode='FIXED';scene.render.threads=8
scene.render.resolution_x=1280;scene.render.resolution_y=720;scene.render.resolution_percentage=100
world=bpy.data.worlds.new('Daylight');world.use_nodes=True
world.node_tree.nodes['Background'].inputs[0].default_value=(.65,.78,1.,1)
world.node_tree.nodes['Background'].inputs[1].default_value=.5
scene.world=world
light=bpy.data.lights.new('Sun','SUN');light.energy=2.5;light.angle=.12
sun=bpy.data.objects.new('Sun',light);scene.collection.objects.link(sun);sun.rotation_euler=(.5,-.4,-.4)
cam=bpy.data.cameras.new('Preview');cam.lens=26
camera=bpy.data.objects.new('Preview',cam);scene.collection.objects.link(camera);scene.camera=camera
for name,eye,target in [('overview',m['overview_eye'],[0,0,3]),('eye_level',m['first_person_eye'],m['first_person_target'])]:
 camera.location=eye;camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
 scene.render.filepath=str(out/f'forest_{name}.png');bpy.ops.render.render(write_still=True)
 print('SAVED',scene.render.filepath,flush=True)
(out/'renderer.txt').write_text('Blender Cycles CPU documentation preview of exported USD, daylight recreated. Not Isaac RGB/depth or flight validation.\n')
sys.stdout.flush();os._exit(0)
