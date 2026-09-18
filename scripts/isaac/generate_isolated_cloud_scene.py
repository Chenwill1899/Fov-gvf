#!/usr/bin/env python3
"""Generate separated, irregular cloud-pillar obstacles for Isaac Sim.

The map starts empty and places individual cloud-shaped components.  A
distance buffer is enforced between every pair of components, so the black
obstacles never touch and the white gaps remain wide enough for a vehicle.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

from generate_cloud_pillar_scene import add_static_box, make_cloud_mesh


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--width", type=float, default=18.0)
    parser.add_argument("--depth", type=float, default=12.0)
    parser.add_argument("--cell-size", type=float, default=0.12)
    parser.add_argument("--cloud-count", type=int, default=14)
    parser.add_argument("--min-radius", type=float, default=0.55)
    parser.add_argument("--max-radius", type=float, default=1.05)
    parser.add_argument("--min-gap", type=float, default=0.90, help="minimum black-to-black gap in metres")
    parser.add_argument("--endpoint-radius", type=float, default=0.90)
    parser.add_argument("--height", type=float, default=3.4)
    parser.add_argument("--height-variation", type=float, default=0.45)
    parser.add_argument(
        "--output", type=Path,
        default=Path("/home/starry/isaac-data/EGO1P0/scenes/isolated_clouds/isolated_clouds_seed42.usd"),
    )
    return parser.parse_args()


def make_blob(rng: np.random.Generator, ny: int, nx: int, cx: float, cy: float,
              radius_x: float, radius_y: float, angle: float, cell: float) -> np.ndarray:
    """Make one connected, lumpy ellipse on the world grid."""
    yy, xx = np.indices((ny, nx), dtype=np.float64)
    x = (xx - cx) * cell
    y = (yy - cy) * cell
    ca, sa = np.cos(angle), np.sin(angle)
    u = (ca * x + sa * y) / radius_x
    v = (-sa * x + ca * y) / radius_y
    theta = np.arctan2(v, u)
    # Low-frequency angular perturbations make the perimeter cloud-like while
    # retaining a single connected component after mild closing.
    perturb = (
        1.0
        + 0.13 * np.sin(3.0 * theta + rng.uniform(0.0, 2.0 * np.pi))
        + 0.09 * np.sin(5.0 * theta + rng.uniform(0.0, 2.0 * np.pi))
        + 0.05 * np.sin(8.0 * theta + rng.uniform(0.0, 2.0 * np.pi))
    )
    mask = (u * u + v * v) <= perturb * perturb
    mask = ndimage.binary_closing(mask, structure=np.ones((3, 3), dtype=bool), iterations=2)
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=np.uint8))
    if count > 1:
        sizes = np.bincount(labels.ravel())
        mask = labels == int(np.argmax(sizes[1:]) + 1)
    return mask


def isolated_cloud_mask(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, dict]:
    if args.cloud_count < 2:
        raise ValueError("cloud-count must be at least 2")
    if args.min_radius <= 0 or args.max_radius < args.min_radius:
        raise ValueError("invalid radius range")
    if args.min_gap <= 0 or args.endpoint_radius <= 0:
        raise ValueError("min-gap and endpoint-radius must be positive")

    nx, ny = int(round(args.width / args.cell_size)), int(round(args.depth / args.cell_size))
    if nx < 40 or ny < 40:
        raise ValueError("width/depth must contain at least 40 cells")
    rng = np.random.default_rng(args.seed)
    occupied = np.zeros((ny, nx), dtype=bool)

    xs = (np.arange(nx) + 0.5) * args.cell_size - 0.5 * args.width
    ys = (np.arange(ny) + 0.5) * args.cell_size - 0.5 * args.depth
    xx, yy = np.meshgrid(xs, ys)
    start_x, goal_x = -0.5 * args.width + 1.0, 0.5 * args.width - 1.0
    endpoint_keepout = (
        ((xx - start_x) ** 2 + yy ** 2 <= args.endpoint_radius ** 2)
        | ((xx - goal_x) ** 2 + yy ** 2 <= args.endpoint_radius ** 2)
    )
    accepted = []
    max_attempts = args.cloud_count * 300
    for _ in range(max_attempts):
        if len(accepted) >= args.cloud_count:
            break
        radius_x = rng.uniform(args.min_radius, args.max_radius)
        radius_y = rng.uniform(args.min_radius * 0.70, args.max_radius * 0.90)
        margin_x, margin_y = args.max_radius + args.min_gap, args.max_radius + args.min_gap
        cx = rng.uniform(margin_x, args.width - margin_x) / args.cell_size
        cy = rng.uniform(margin_y, args.depth - margin_y) / args.cell_size
        angle = rng.uniform(0.0, 2.0 * np.pi)
        blob = make_blob(rng, ny, nx, cx, cy, radius_x, radius_y, angle, args.cell_size)
        if not blob.any() or np.count_nonzero(blob) < 20:
            continue
        # Protect only the launch and goal disks; the route between them must
        # emerge from the separated-obstacle geometry instead of being carved.
        if np.any(blob & endpoint_keepout):
            continue
        if occupied.any():
            distance_to_existing = ndimage.distance_transform_edt(~occupied) * args.cell_size
            if float(distance_to_existing[blob].min()) < args.min_gap:
                continue
        occupied |= blob
        accepted.append((float(radius_x), float(radius_y)))
    if len(accepted) != args.cloud_count:
        raise RuntimeError(f"placed {len(accepted)} of {args.cloud_count} isolated clouds; reduce count/radii or gap")

    # Verify each black component remains separate, and the guaranteed corridor
    # has the requested vehicle clearance from every cloud.
    labels, component_count = ndimage.label(occupied, structure=np.ones((3, 3), dtype=np.uint8))
    if component_count != args.cloud_count:
        raise RuntimeError(f"expected {args.cloud_count} black components, got {component_count}")
    actual_min_gap = float("inf")
    for component in range(1, component_count + 1):
        this_cloud = labels == component
        other_clouds = occupied & ~this_cloud
        distance_to_others = ndimage.distance_transform_edt(~other_clouds) * args.cell_size
        actual_min_gap = min(actual_min_gap, float(distance_to_others[this_cloud].min()))
    if actual_min_gap + 1.0e-9 < args.min_gap:
        raise RuntimeError(f"actual cloud gap {actual_min_gap:.3f} m is below requested {args.min_gap:.3f} m")
    free = ~occupied
    distance = ndimage.distance_transform_edt(free) * args.cell_size
    start = np.array([start_x, 0.0])
    goal = np.array([goal_x, 0.0])
    start_cell = np.unravel_index(np.argmin((xx - start[0]) ** 2 + yy**2), xx.shape)
    goal_cell = np.unravel_index(np.argmin((xx - goal[0]) ** 2 + yy**2), xx.shape)
    required_clearance = min(0.40, 0.42 * args.min_gap)
    traversable = free & (distance >= required_clearance)
    free_labels, free_count = ndimage.label(traversable, structure=np.ones((3, 3), dtype=np.uint8))
    if free_labels[start_cell] == 0 or free_labels[start_cell] != free_labels[goal_cell]:
        raise RuntimeError("corridor validation failed")

    noise = ndimage.gaussian_filter(rng.normal(size=(ny, nx)), sigma=3.0, mode="wrap")
    noise -= noise.min()
    noise /= max(float(noise.max()), 1.0e-9)
    top_height = np.maximum(args.height + args.height_variation * (noise - 0.5), 0.75 * args.height)
    metadata = {
        "start_x": float(start[0]), "start_y": float(start[1]),
        "goal_x": float(goal[0]), "goal_y": float(goal[1]),
        "cloud_count": int(args.cloud_count), "black_component_count": int(component_count),
        "free_component_count": int(free_count), "occupied_fraction": float(occupied.mean()),
        "minimum_requested_gap_m": float(args.min_gap),
        "actual_minimum_gap_m": actual_min_gap,
        "validated_vehicle_clearance_m": float(required_clearance),
    }
    return occupied, top_height, metadata


def build_scene(args: argparse.Namespace) -> dict:
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    occupied, top_height, metadata = isolated_cloud_mask(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(args.output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    add_static_box(stage, "/World/Ground", (args.width + 2.0, args.depth + 2.0, 0.10), (0.0, 0.0, -0.05), (0.92, 0.92, 0.92))
    _, vertex_count, face_count = make_cloud_mesh(stage, occupied, top_height, args.width, args.depth, args.cell_size)
    for path, x_key, color in (("/World/Start", "start_x", (0.1, 0.8, 0.2)), ("/World/Goal", "goal_x", (0.95, 0.2, 0.1))):
        marker = UsdGeom.Cylinder.Define(stage, path)
        marker.CreateRadiusAttr(0.28); marker.CreateHeightAttr(0.025)
        UsdGeom.XformCommonAPI(marker).SetTranslate(Gf.Vec3d(metadata[x_key], 0.0, 0.015))
        marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0); dome.CreateColorAttr(Gf.Vec3f(0.65, 0.75, 0.90))
    manifest = {"generator": "isolated_irregular_clouds_v1", "seed": args.seed, "width_m": args.width, "depth_m": args.depth,
                "cell_size_m": args.cell_size, "cloud_count": args.cloud_count, "min_gap_m": args.min_gap,
                "endpoint_radius_m": args.endpoint_radius,
                "obstacle_height_m": args.height, "height_variation_m": args.height_variation,
                "obstacle_prim": "/World/CloudObstacles/CloudField", "start": [metadata["start_x"], 0.0, 1.2],
                "goal": [metadata["goal_x"], 0.0, 1.2], "stats": metadata,
                "mesh_vertices": vertex_count, "mesh_faces": face_count}
    manifest_path = args.output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    map_path = args.output.with_name(args.output.stem + "_map.png")
    image = np.where(occupied, 0, 255).astype(np.uint8)
    Image.fromarray(np.flipud(image), mode="L").resize((image.shape[1] * 8, image.shape[0] * 8), Image.Resampling.NEAREST).save(map_path)
    stage.Save()
    return {"stage": str(args.output), "manifest": str(manifest_path), "map": str(map_path), **manifest}


if __name__ == "__main__":
    print(json.dumps(build_scene(parse_args()), indent=2))
