#!/usr/bin/env python3
"""Generate editable, separated cloud-pillar obstacles in Blender.

Run with Blender, not the system Python:
  blender --background --python generate_isolated_cloud_scene.py -- [options]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import sys

import bpy


def parse_args() -> argparse.Namespace:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--width", type=float, default=18.0)
    parser.add_argument("--depth", type=float, default=12.0)
    parser.add_argument("--cloud-count", type=int, default=14)
    parser.add_argument("--min-radius", type=float, default=0.55)
    parser.add_argument("--max-radius", type=float, default=1.05)
    parser.add_argument("--min-gap", type=float, default=0.90)
    parser.add_argument("--endpoint-radius", type=float, default=1.05)
    parser.add_argument("--height", type=float, default=3.4)
    parser.add_argument("--height-variation", type=float, default=0.45)
    parser.add_argument("--vertices", type=int, default=48)
    parser.add_argument(
        "--output", type=Path,
        default=Path("/home/starry/isaac-data/EGO1P0/scenes/blender_isolated_clouds/isolated_clouds_seed42.blend"),
    )
    parser.add_argument("--preview", type=Path, default=None)
    return parser.parse_args(raw)


def material(name: str, rgba: tuple[float, float, float, float], roughness: float = 0.65):
    value = bpy.data.materials.new(name)
    value.diffuse_color = rgba
    value.use_nodes = True
    principled = value.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = rgba
    principled.inputs["Roughness"].default_value = roughness
    return value


def link_object(obj, collection) -> None:
    collection.objects.link(obj)


def cloud_outline(rng: random.Random, min_radius: float, max_radius: float, count: int):
    base_x = rng.uniform(min_radius, max_radius)
    base_y = rng.uniform(0.72 * min_radius, 0.90 * max_radius)
    rotation = rng.uniform(0.0, 2.0 * math.pi)
    phases = [rng.uniform(0.0, 2.0 * math.pi) for _ in range(3)]
    outline = []
    for index in range(count):
        theta = 2.0 * math.pi * index / count
        radial = 1.0 + 0.14 * math.sin(3.0 * theta + phases[0])
        radial += 0.09 * math.sin(5.0 * theta + phases[1])
        radial += 0.055 * math.sin(8.0 * theta + phases[2])
        x, y = base_x * radial * math.cos(theta), base_y * radial * math.sin(theta)
        ca, sa = math.cos(rotation), math.sin(rotation)
        outline.append((ca * x - sa * y, sa * x + ca * y))
    extent = max(math.hypot(x, y) for x, y in outline)
    return outline, extent


def make_prism(name: str, outline, height: float, collection, obstacle_material):
    count = len(outline)
    vertices = [(x, y, 0.0) for x, y in outline] + [(x, y, height) for x, y in outline]
    faces = [tuple(reversed(range(count))), tuple(range(count, 2 * count))]
    faces.extend((i, (i + 1) % count, (i + 1) % count + count, i + count) for i in range(count))
    mesh = bpy.data.meshes.new(f"{name}_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(obstacle_material)
    obj = bpy.data.objects.new(name, mesh)
    link_object(obj, collection)
    obj["fov_geometry"] = "isolated_irregular_cloud_pillar"
    obj["is_obstacle"] = True
    bevel = obj.modifiers.new("Small edge bevel", "BEVEL")
    bevel.width = 0.035
    bevel.segments = 2
    return obj


def add_cube(name, location, scale, mat, collection):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    obj.data.materials.append(mat)
    for old_collection in list(obj.users_collection):
        old_collection.objects.unlink(obj)
    link_object(obj, collection)
    return obj


def add_marker(name: str, location, color, collection):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=0.34, depth=0.025, location=(location[0], location[1], 0.015))
    pad = bpy.context.object
    pad.name = f"{name}_Pad"
    pad.data.materials.append(color)
    for old_collection in list(pad.users_collection):
        old_collection.objects.unlink(pad)
    link_object(pad, collection)

    marker = bpy.data.objects.new(name, None)
    marker.empty_display_type = "ARROWS"
    marker.empty_display_size = 0.65
    marker.location = location
    marker["role"] = "uav_start" if name == "UAV_Start" else "uav_goal"
    marker["position_m"] = list(location)
    link_object(marker, collection)
    return marker


def build(args: argparse.Namespace) -> dict:
    if args.cloud_count < 2 or args.vertices < 16:
        raise ValueError("cloud-count must be >= 2 and vertices must be >= 16")
    if args.min_radius <= 0.0 or args.max_radius < args.min_radius or args.min_gap <= 0.0:
        raise ValueError("invalid radius or gap")
    rng = random.Random(args.seed)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.name = "Isolated Cloud UAV Scene"
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"

    environment = bpy.data.collections.new("Environment")
    obstacles = bpy.data.collections.new("Cloud_Obstacles")
    markers = bpy.data.collections.new("Flight_Markers")
    scene.collection.children.link(environment)
    scene.collection.children.link(obstacles)
    scene.collection.children.link(markers)

    black = material("Obstacle_Black", (0.008, 0.012, 0.018, 1.0), 0.78)
    white = material("Ground_White", (0.92, 0.92, 0.92, 1.0), 0.82)
    green = material("Start_Green", (0.04, 0.80, 0.12, 1.0), 0.45)
    red = material("Goal_Red", (0.92, 0.04, 0.03, 1.0), 0.45)
    add_cube("Ground", (0.0, 0.0, -0.05), (args.width / 2.0 + 1.0, args.depth / 2.0 + 1.0, 0.05), white, environment)

    start = (-args.width / 2.0 + 1.0, 0.0, 1.2)
    goal = (args.width / 2.0 - 1.0, 0.0, 1.2)
    placed = []
    guaranteed_gap = float("inf")
    for index in range(args.cloud_count):
        accepted = None
        for _ in range(4000):
            outline, extent = cloud_outline(rng, args.min_radius, args.max_radius, args.vertices)
            x = rng.uniform(-args.width / 2.0 + extent + 0.35, args.width / 2.0 - extent - 0.35)
            y = rng.uniform(-args.depth / 2.0 + extent + 0.35, args.depth / 2.0 - extent - 0.35)
            if any(math.hypot(x - point[0], y - point[1]) < extent + args.endpoint_radius for point in (start, goal)):
                continue
            if any(math.hypot(x - px, y - py) < extent + previous_extent + args.min_gap for px, py, previous_extent in placed):
                continue
            accepted = (outline, extent, x, y)
            break
        if accepted is None:
            raise RuntimeError(f"could only place {index} of {args.cloud_count} clouds; reduce count/radius/gap")
        outline, extent, x, y = accepted
        if placed:
            guaranteed_gap = min(guaranteed_gap, *(math.hypot(x - px, y - py) - extent - pe for px, py, pe in placed))
        height = args.height + rng.uniform(-0.5, 0.5) * args.height_variation
        cloud = make_prism(f"Cloud_{index + 1:02d}", outline, height, obstacles, black)
        cloud.location = (x, y, 0.0)
        cloud["height_m"] = height
        cloud["bounding_radius_m"] = extent
        placed.append((x, y, extent))

    add_marker("UAV_Start", start, green, markers)
    add_marker("UAV_Goal", goal, red, markers)

    bpy.ops.object.light_add(type="SUN", location=(0.0, 0.0, 10.0))
    sun = bpy.context.object; sun.name = "Sun"; sun.data.energy = 2.2; sun.rotation_euler = (math.radians(25), math.radians(-18), math.radians(25))
    bpy.ops.object.camera_add(location=(0.0, 0.0, 25.0), rotation=(0.0, 0.0, 0.0))
    camera = bpy.context.object; camera.name = "Overview_Camera"; camera.data.type = "ORTHO"; camera.data.ortho_scale = 20.5
    camera.rotation_euler = (0.0, 0.0, 0.0)
    # Cameras look down local -Z; no rotation is needed at +Z.
    scene.camera = camera
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1200; scene.render.resolution_y = 800; scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.world = bpy.data.worlds.new("World")
    scene.world.color = (0.06, 0.06, 0.06)

    scene["generator"] = "blender_isolated_clouds_v1"
    scene["seed"] = args.seed
    scene["minimum_requested_gap_m"] = args.min_gap
    scene["guaranteed_bounding_circle_gap_m"] = guaranteed_gap
    scene["start_xyz_m"] = list(start)
    scene["goal_xyz_m"] = list(goal)
    args.output = args.output.expanduser().resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output))

    preview = args.preview.expanduser().resolve() if args.preview else args.output.with_name(args.output.stem + "_preview.png")
    preview.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(preview)
    bpy.ops.render.render(write_still=True)
    manifest = {
        "generator": scene["generator"], "seed": args.seed, "blend": str(args.output), "preview": str(preview),
        "width_m": args.width, "depth_m": args.depth, "cloud_count": args.cloud_count,
        "minimum_requested_gap_m": args.min_gap, "guaranteed_gap_m": guaranteed_gap,
        "start_object": "UAV_Start", "start": list(start), "goal_object": "UAV_Goal", "goal": list(goal),
        "obstacle_collection": "Cloud_Obstacles",
    }
    manifest_path = args.output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return manifest


if __name__ == "__main__":
    build(parse_args())
