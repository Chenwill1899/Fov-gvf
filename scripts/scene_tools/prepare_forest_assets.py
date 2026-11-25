#!/usr/bin/env python3
"""Prepare CC0 Poly Haven baked LOD meshes using Blender, without source scripts.

blender --background --factory-startup -noaudio --disable-autoexec --python
scripts/scene_tools/prepare_forest_assets.py -- --source-dir PATH
Run build_forest_collision_assets.py with Isaac Python afterwards.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def prepare(name, source_dir):
    source = source_dir / name / f'{name}_1k.blend'
    folder = ROOT / 'scenes/reference_forest/assets' / name
    folder.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.open_mainfile(filepath=str(source), load_ui=False, use_scripts=False)
    obj = bpy.data.objects[name + '_LOD1']
    # Isolate the baked mesh: never evaluate the downloaded Geometry Nodes graph.
    mesh = obj.data.copy()
    matrix = obj.matrix_world.copy()
    for old in list(bpy.data.objects):
        bpy.data.objects.remove(old, do_unlink=True)
    obj = bpy.data.objects.new('Tree', mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.matrix_world = matrix
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    original_faces = len(mesh.polygons)
    decimate = obj.modifiers.new('RealtimeLOD', 'DECIMATE')
    decimate.ratio = min(1., 60000 / original_faces)
    decimate.use_collapse_triangulate = True
    bpy.ops.object.modifier_apply(modifier=decimate.name)
    tri = obj.modifiers.new('Triangles', 'TRIANGULATE')
    bpy.ops.object.modifier_apply(modifier=tri.name)
    mesh = obj.data
    coords = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get('co', coords)
    coords = coords.reshape(-1, 3)
    low, high = coords.min(axis=0), coords.max(axis=0)
    # Keep trunk origin XY, ground the mesh, and author two differently shaped trees.
    coords[:, 2] -= low[2]
    scale_z = 9.4 / (high[2] - low[2])
    scale_xy = min(scale_z, 4.3 / max(high[:2] - low[:2]))
    coords *= [scale_xy, scale_xy, scale_z]
    mesh.vertices.foreach_set('co', coords.ravel())
    mesh.update()
    for img in bpy.data.images:
        if img.source == 'FILE':
            path = source.parent / 'textures' / Path(img.filepath).name
            if not path.is_file():
                raise FileNotFoundError(path)
            img.filepath = str(path)
            img.reload()
    # Portable USD Preview Surface with leaf cutout alpha and double-sided cards.
    for mat in mesh.materials:
        mat.use_backface_culling = False
        if 'leaves' in mat.name:
            # Source leaves use a Blender-specific translucent shader mix, which
            # USD Preview Surface cannot export. Rebuild its texture-based PBR
            # equivalent explicitly; otherwise the foliage exports black/unbound.
            mat.use_nodes = True
            nodes, links = mat.node_tree.nodes, mat.node_tree.links
            nodes.clear()
            output = nodes.new('ShaderNodeOutputMaterial')
            shader = nodes.new('ShaderNodeBsdfPrincipled')
            links.new(shader.outputs['BSDF'], output.inputs['Surface'])
            shader.inputs['Roughness'].default_value = .82
            uv = nodes.new('ShaderNodeUVMap')
            uv.uv_map = mesh.uv_layers.active.name
            for suffix, socket in [('diff','Base Color'),('alpha','Alpha'),('rough','Roughness')]:
                node = nodes.new('ShaderNodeTexImage')
                path = source.parent / 'textures' / f'{name}_leaves_{suffix}_1k.png'
                node.image = bpy.data.images.load(str(path), check_existing=True)
                node.image.colorspace_settings.name = 'sRGB' if suffix == 'diff' else 'Non-Color'
                links.new(uv.outputs['UV'], node.inputs['Vector'])
                links.new(node.outputs['Color'], shader.inputs[socket])
    bpy.ops.wm.usd_export(filepath=str(folder / 'model.usdc'),
        selected_objects_only=True, export_animation=False,
        export_materials=True, generate_preview_surface=True,
        generate_materialx_network=False, export_textures_mode='NEW',
        overwrite_textures=True, relative_paths=True, root_prim_path='/Asset',
        export_armatures=False, export_shapekeys=False,
        export_lights=False, export_cameras=False, export_curves=False,
        export_points=False, export_volumes=False, convert_world_material=False,
        triangulate_meshes=True, convert_scene_units='METERS', meters_per_unit=1.)
    triangles = np.array([p.vertices[:] for p in mesh.polygons], dtype=np.int32)
    np.savez_compressed(folder / 'collision_source.npz', vertices=coords, triangles=triangles)
    report = dict(asset=name, license='CC0-1.0', url=f'https://polyhaven.com/a/{name}',
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source_object=name+'_LOD1', original_faces=original_faces,
        triangles=len(triangles), vertices=len(coords),
        aabb_min=coords.min(axis=0).tolist(), aabb_max=coords.max(axis=0).tolist(),
        transform_scale=[float(scale_xy), float(scale_xy), float(scale_z)], blender=bpy.app.version_string)
    (folder / 'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--source-dir', type=Path, required=True)
    args = cli.parse_args(sys.argv[sys.argv.index('--') + 1:])
    for name in ['tree_small_02', 'island_tree_01']:
        prepare(name, args.source_dir)
    # Some host audio backends hang Blender shutdown after headless export.
    # All synchronous exports and provenance files are closed before this exit.
    sys.stdout.flush()
    os._exit(0)
