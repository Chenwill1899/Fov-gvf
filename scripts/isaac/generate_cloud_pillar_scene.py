#!/usr/bin/env python3
"""Generate a reproducible cloud-footprint obstacle field for Isaac Sim.

The obstacle footprint is a smoothed random binary field.  Occupied cells are
merged into one static triangle mesh and extruded from the floor, so the white
regions remain real free-space gaps rather than visual decoration.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--width", type=float, default=18.0, help="world X size in metres")
    parser.add_argument("--depth", type=float, default=12.0, help="world Y size in metres")
    parser.add_argument("--cell-size", type=float, default=0.15)
    parser.add_argument("--occupancy", type=float, default=0.58)
    parser.add_argument("--corridor-radius", type=float, default=0.95)
    parser.add_argument("--height", type=float, default=3.4)
    parser.add_argument("--height-variation", type=float, default=0.45)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/starry/isaac-data/EGO1P0/scenes/cloud_pillars/cloud_pillars_seed42.usd"),
    )
    return parser.parse_args()


def cloud_mask(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    """Create a smooth obstacle mask and a varying top-height map."""
    nx = int(round(args.width / args.cell_size))
    ny = int(round(args.depth / args.cell_size))
    if nx < 20 or ny < 20:
        raise ValueError("width/depth must contain at least 20 cells")
    if not 0.15 <= args.occupancy <= 0.70:
        raise ValueError("occupancy must be in [0.15, 0.70]")

    rng = np.random.default_rng(args.seed)
    coarse = ndimage.gaussian_filter(rng.normal(size=(ny, nx)), sigma=4.0, mode="wrap")
    fine = ndimage.gaussian_filter(rng.normal(size=(ny, nx)), sigma=1.5, mode="wrap")
    field = 0.78 * coarse + 0.22 * fine
    threshold = float(np.quantile(field, 1.0 - args.occupancy))
    occupied = field >= threshold
    occupied = ndimage.binary_closing(occupied, structure=np.ones((3, 3), dtype=bool), iterations=2)
    occupied = ndimage.binary_opening(occupied, structure=np.ones((2, 2), dtype=bool))

    xs = (np.arange(nx) + 0.5) * args.cell_size - 0.5 * args.width
    ys = (np.arange(ny) + 0.5) * args.cell_size - 0.5 * args.depth
    xx, yy = np.meshgrid(xs, ys)

    # Carve a guaranteed white, gently bending flight corridor.  It creates a
    # solvable benchmark without exposing this corridor to the planner.
    start_x = -0.5 * args.width + 1.0
    goal_x = 0.5 * args.width - 1.0
    phase = (xx - start_x) / max(goal_x - start_x, args.cell_size)
    centre_y = 0.75 * np.sin(2.0 * np.pi * phase + 0.35) + 0.32 * np.sin(5.0 * np.pi * phase)
    corridor = (
        (xx >= start_x)
        & (xx <= goal_x)
        & (np.abs(yy - centre_y) <= args.corridor_radius)
    )
    occupied[corridor] = False

    # Keep the launch and goal disks clear even when the corridor bends near an edge.
    start = np.array([start_x, 0.0])
    goal = np.array([goal_x, 0.0])
    occupied[((xx - start[0]) ** 2 + yy**2) <= (args.corridor_radius * 1.2) ** 2] = False
    occupied[((xx - goal[0]) ** 2 + yy**2) <= (args.corridor_radius * 1.2) ** 2] = False

    height_noise = ndimage.gaussian_filter(rng.normal(size=(ny, nx)), sigma=3.0, mode="wrap")
    height_noise -= height_noise.min()
    height_noise /= max(float(height_noise.max()), 1.0e-9)
    top_height = args.height + args.height_variation * (height_noise - 0.5)
    top_height = np.maximum(top_height, 0.75 * args.height)

    free = ~occupied
    distance = ndimage.distance_transform_edt(free) * args.cell_size
    start_cell = np.unravel_index(np.argmin((xx - start[0]) ** 2 + (yy - start[1]) ** 2), xx.shape)
    goal_cell = np.unravel_index(np.argmin((xx - goal[0]) ** 2 + (yy - goal[1]) ** 2), xx.shape)
    traversable = free & (distance >= max(0.5, 0.55 * args.corridor_radius))
    labels, count = ndimage.label(traversable, structure=np.ones((3, 3), dtype=np.uint8))
    if labels[start_cell] == 0 or labels[start_cell] != labels[goal_cell]:
        raise RuntimeError("internal corridor validation failed; try another seed")

    metadata = {
        "start_x": float(start[0]),
        "start_y": float(start[1]),
        "goal_x": float(goal[0]),
        "goal_y": float(goal[1]),
        "free_clearance_min_m": float(distance[traversable].min()),
        "occupied_fraction": float(occupied.mean()),
        "free_component_count": int(count),
    }
    return occupied, top_height, metadata


def add_static_box(stage, path: str, size: tuple[float, float, float], position: tuple[float, float, float], color):
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*position))
    xform.SetScale(Gf.Vec3f(size[0], size[1], size[2]))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def make_cloud_mesh(stage, occupied: np.ndarray, top_height: np.ndarray, width: float, depth: float, cell: float):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    ny, nx = occupied.shape
    vertices: list[tuple[float, float, float]] = []
    counts: list[int] = []
    indices: list[int] = []

    def quad(points: list[tuple[float, float, float]]) -> None:
        base = len(vertices)
        vertices.extend(points)
        counts.append(4)
        indices.extend([base, base + 1, base + 2, base + 3])

    def height(row: int, col: int) -> float:
        return float(top_height[row, col])

    for row in range(ny):
        for col in range(nx):
            if not occupied[row, col]:
                continue
            x0 = -0.5 * width + col * cell
            x1 = x0 + cell
            y0 = -0.5 * depth + row * cell
            y1 = y0 + cell
            z1 = height(row, col)
            z0 = 0.0
            # Top and bottom close every occupied cell.  Adjacent coplanar faces
            # are harmless, while boundary faces below define the cloud outline.
            quad([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)])
            quad([(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)])
            neighbours = ((0, -1, x0, y0, x0, y1), (0, 1, x1, y1, x1, y0),
                          (-1, 0, x0, y1, x1, y1), (1, 0, x1, y0, x0, y0))
            for dr, dc, xa, ya, xb, yb in neighbours:
                nr, nc = row + dr, col + dc
                neighbour_top = 0.0 if not (0 <= nr < ny and 0 <= nc < nx and occupied[nr, nc]) else height(nr, nc)
                if neighbour_top >= z1 - 1.0e-6:
                    continue
                lower = max(z0, neighbour_top)
                if dr == 0 and dc == -1:
                    quad([(xa, ya, lower), (xb, yb, lower), (xb, yb, z1), (xa, ya, z1)])
                elif dr == 0 and dc == 1:
                    quad([(xb, yb, lower), (xa, ya, lower), (xa, ya, z1), (xb, yb, z1)])
                elif dr == -1:
                    quad([(xa, ya, lower), (xb, yb, lower), (xb, yb, z1), (xa, ya, z1)])
                else:
                    quad([(xb, yb, lower), (xa, ya, lower), (xa, ya, z1), (xb, yb, z1)])

    mesh = UsdGeom.Mesh.Define(stage, "/World/CloudObstacles/CloudField")
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in vertices])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.008, 0.012, 0.018)])
    mesh.GetPrim().CreateAttribute("fov:geometry", Sdf.ValueTypeNames.String).Set("cloud_extruded_grid")
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    collision.CreateApproximationAttr().Set("none")
    return mesh, len(vertices), len(counts)


def build_scene(args: argparse.Namespace) -> dict:
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    occupied, top_height, metadata = cloud_mask(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(args.output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

    add_static_box(stage, "/World/Ground", (args.width + 2.0, args.depth + 2.0, 0.10), (0.0, 0.0, -0.05), (0.92, 0.92, 0.92))
    _, vertex_count, face_count = make_cloud_mesh(
        stage, occupied, top_height, args.width, args.depth, args.cell_size
    )

    # Colored, non-colliding start/goal pads make the intended test endpoints visible.
    for path, x, color in (("/World/Start", metadata["start_x"], (0.1, 0.8, 0.2)), ("/World/Goal", metadata["goal_x"], (0.95, 0.2, 0.1))):
        marker = UsdGeom.Cylinder.Define(stage, path)
        marker.CreateRadiusAttr(0.28)
        marker.CreateHeightAttr(0.025)
        UsdGeom.XformCommonAPI(marker).SetTranslate(Gf.Vec3d(x, 0.0, 0.015))
        marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.65, 0.75, 0.90))
    overview = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    overview.CreateFocalLengthAttr(24.0)
    overview.CreateClippingRangeAttr((0.1, 100.0))
    UsdGeom.XformCommonAPI(overview).SetTranslate(Gf.Vec3d(-10.5, -13.0, 11.0))
    UsdGeom.XformCommonAPI(overview).SetRotate(Gf.Vec3f(48.0, 0.0, -39.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    manifest = {
        "seed": args.seed,
        "width_m": args.width,
        "depth_m": args.depth,
        "cell_size_m": args.cell_size,
        "obstacle_height_m": args.height,
        "height_variation_m": args.height_variation,
        "requested_occupancy": args.occupancy,
        "corridor_radius_m": args.corridor_radius,
        "validated_corridor_clearance_m": max(0.5, 0.55 * args.corridor_radius),
        "obstacle_prim": "/World/CloudObstacles/CloudField",
        "start": [metadata["start_x"], metadata["start_y"], 1.2],
        "goal": [metadata["goal_x"], metadata["goal_y"], 1.2],
        "stats": metadata,
        "mesh_vertices": vertex_count,
        "mesh_faces": face_count,
    }
    manifest_path = args.output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    # Save the source black/white footprint used to create the 3-D collision mesh.
    map_path = args.output.with_name(args.output.stem + "_map.png")
    image = np.where(occupied, 0, 255).astype(np.uint8)
    Image.fromarray(np.flipud(image), mode="L").resize(
        (image.shape[1] * 8, image.shape[0] * 8), Image.Resampling.NEAREST
    ).save(map_path)
    stage.Save()
    return {"stage": str(args.output), "manifest": str(manifest_path), "map": str(map_path), **manifest}


def main() -> None:
    args = parse_args()
    result = build_scene(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
