#!/usr/bin/env python3
"""Generate a connected crack-maze obstacle field for Isaac Sim.

Unlike the cloud-field generator, this generator starts with an occupied black
map and carves only the corridors of a randomized perfect maze.  Every white
cell therefore belongs to one connected road network; no isolated free-space
pockets are introduced.
"""

from __future__ import annotations

import argparse
from collections import deque
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
    parser.add_argument("--cell-size", type=float, default=0.12)
    parser.add_argument("--maze-columns", type=int, default=8)
    parser.add_argument("--maze-rows", type=int, default=5)
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--corridor-width", type=float, default=1.05, help="nominal full width in metres")
    parser.add_argument("--width-variation", type=float, default=0.16, help="per-road relative variation")
    parser.add_argument("--junction-radius", type=float, default=0.60)
    parser.add_argument("--node-jitter", type=float, default=0.22, help="fraction of grid spacing")
    parser.add_argument("--min-clearance", type=float, default=0.35)
    parser.add_argument("--height", type=float, default=3.4)
    parser.add_argument("--height-variation", type=float, default=0.30)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/starry/isaac-data/EGO1P0/scenes/crack_maze/crack_maze_seed42.usd"),
    )
    return parser.parse_args()


def randomized_growing_tree(rows: int, columns: int, rng: np.random.Generator) -> list[tuple[int, int]]:
    """Return a winding, branching spanning tree over a 4-neighbour grid.

    Selecting the newest active cell most of the time preserves long maze-like
    runs.  Occasionally selecting an older active cell creates substantially
    more junctions than a pure recursive backtracker.
    """
    start = int(rng.integers(rows * columns))
    visited = {start}
    active = [start]
    edges: list[tuple[int, int]] = []

    while active:
        active_index = len(active) - 1 if rng.random() < 0.68 else int(rng.integers(len(active)))
        node = active[active_index]
        row, column = divmod(node, columns)
        candidates = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = row + dr, column + dc
            neighbour = nr * columns + nc
            if 0 <= nr < rows and 0 <= nc < columns and neighbour not in visited:
                candidates.append(neighbour)
        if not candidates:
            active.pop(active_index)
            continue
        neighbour = candidates[int(rng.integers(len(candidates)))]
        visited.add(neighbour)
        edges.append((node, neighbour))
        active.append(neighbour)

    return edges


def bresenham(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    """Rasterize an 8-connected line using integer row/column coordinates."""
    row0, col0 = start
    row1, col1 = end
    dc = abs(col1 - col0)
    dr = -abs(row1 - row0)
    step_c = 1 if col0 < col1 else -1
    step_r = 1 if row0 < row1 else -1
    error = dc + dr
    points = []
    while True:
        points.append((row0, col0))
        if row0 == row1 and col0 == col1:
            return points
        twice = 2 * error
        if twice >= dr:
            error += dr
            col0 += step_c
        if twice <= dc:
            error += dc
            row0 += step_r


def tree_diameter(node_count: int, edges: list[tuple[int, int]]) -> tuple[int, int, int]:
    adjacency = [[] for _ in range(node_count)]
    for first, second in edges:
        adjacency[first].append(second)
        adjacency[second].append(first)

    def farthest(source: int) -> tuple[int, int]:
        distance = [-1] * node_count
        distance[source] = 0
        queue = deque([source])
        while queue:
            node = queue.popleft()
            for neighbour in adjacency[node]:
                if distance[neighbour] < 0:
                    distance[neighbour] = distance[node] + 1
                    queue.append(neighbour)
        target = max(range(node_count), key=distance.__getitem__)
        return target, distance[target]

    first, _ = farthest(0)
    second, length = farthest(first)
    return first, second, length


def crack_maze_mask(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, dict]:
    if args.maze_columns < 3 or args.maze_rows < 3:
        raise ValueError("maze-columns and maze-rows must both be at least 3")
    if args.cell_size <= 0.0 or args.corridor_width <= 0.0:
        raise ValueError("cell-size and corridor-width must be positive")
    if not 0.0 <= args.width_variation <= 0.35:
        raise ValueError("width-variation must be in [0, 0.35]")
    if not 0.0 <= args.node_jitter <= 0.30:
        raise ValueError("node-jitter must be in [0, 0.30]")
    if args.margin <= 0.5 * args.corridor_width:
        raise ValueError("margin must exceed half the corridor width")
    if args.width <= 2.0 * args.margin or args.depth <= 2.0 * args.margin:
        raise ValueError("margin leaves no maze area")

    nx = int(round(args.width / args.cell_size))
    ny = int(round(args.depth / args.cell_size))
    if nx < 40 or ny < 40:
        raise ValueError("width/depth must contain at least 40 cells")

    rng = np.random.default_rng(args.seed)
    edges = randomized_growing_tree(args.maze_rows, args.maze_columns, rng)
    x_values = np.linspace(-0.5 * args.width + args.margin, 0.5 * args.width - args.margin, args.maze_columns)
    y_values = np.linspace(-0.5 * args.depth + args.margin, 0.5 * args.depth - args.margin, args.maze_rows)
    spacing_x = float(x_values[1] - x_values[0])
    spacing_y = float(y_values[1] - y_values[0])

    nodes = np.empty((args.maze_rows * args.maze_columns, 2), dtype=np.float64)
    for row in range(args.maze_rows):
        for column in range(args.maze_columns):
            node = row * args.maze_columns + column
            jitter_x = rng.uniform(-1.0, 1.0) * args.node_jitter * spacing_x
            jitter_y = rng.uniform(-1.0, 1.0) * args.node_jitter * spacing_y
            nodes[node] = (x_values[column] + jitter_x, y_values[row] + jitter_y)

    def world_to_cell(point: np.ndarray) -> tuple[int, int]:
        column = int(np.clip(np.floor((point[0] + 0.5 * args.width) / args.cell_size), 0, nx - 1))
        row = int(np.clip(np.floor((point[1] + 0.5 * args.depth) / args.cell_size), 0, ny - 1))
        return row, column

    free = np.zeros((ny, nx), dtype=bool)
    edge_widths = []
    for first, second in edges:
        skeleton = np.zeros_like(free)
        for row, column in bresenham(world_to_cell(nodes[first]), world_to_cell(nodes[second])):
            skeleton[row, column] = True
        width_scale = rng.uniform(1.0 - args.width_variation, 1.0 + args.width_variation)
        edge_width = args.corridor_width * width_scale
        edge_widths.append(edge_width)
        free |= ndimage.distance_transform_edt(~skeleton) * args.cell_size <= 0.5 * edge_width

    # Junction disks make all incident roads overlap despite rasterization and
    # provide room for a vehicle to negotiate the deliberately sharp turns.
    for node in nodes:
        row, column = world_to_cell(node)
        marker = np.ones_like(free)
        marker[row, column] = False
        free |= ndimage.distance_transform_edt(marker) * args.cell_size <= args.junction_radius

    occupied = ~free
    connectivity = np.ones((3, 3), dtype=np.uint8)
    free_labels, free_components = ndimage.label(free, structure=connectivity)
    if free_components != 1:
        raise RuntimeError(f"maze construction produced {free_components} free-space components")

    start_node, goal_node, diameter_edges = tree_diameter(len(nodes), edges)
    start_cell = world_to_cell(nodes[start_node])
    goal_cell = world_to_cell(nodes[goal_node])
    clearance = ndimage.distance_transform_edt(free) * args.cell_size
    centreline_free = free & (clearance >= args.min_clearance)
    centre_labels, centre_components = ndimage.label(centreline_free, structure=connectivity)
    if (
        centre_labels[start_cell] == 0
        or centre_labels[goal_cell] == 0
        or centre_labels[start_cell] != centre_labels[goal_cell]
    ):
        raise RuntimeError("start and goal are not connected at the requested minimum clearance")

    degree = np.zeros(len(nodes), dtype=np.int32)
    for first, second in edges:
        degree[first] += 1
        degree[second] += 1

    height_noise = ndimage.gaussian_filter(rng.normal(size=(ny, nx)), sigma=4.0, mode="nearest")
    height_noise -= height_noise.min()
    height_noise /= max(float(height_noise.max()), 1.0e-9)
    top_height = args.height + args.height_variation * (height_noise - 0.5)
    top_height = np.maximum(top_height, 0.75 * args.height)

    metadata = {
        "start_x": float(nodes[start_node, 0]),
        "start_y": float(nodes[start_node, 1]),
        "goal_x": float(nodes[goal_node, 0]),
        "goal_y": float(nodes[goal_node, 1]),
        "free_fraction": float(free.mean()),
        "occupied_fraction": float(occupied.mean()),
        "free_component_count": int(free_components),
        "clearance_component_count": int(centre_components),
        "tree_edge_count": len(edges),
        "tree_diameter_edges": int(diameter_edges),
        "branch_point_count": int(np.count_nonzero(degree >= 3)),
        "dead_end_count": int(np.count_nonzero(degree == 1)),
        "minimum_edge_width_m": float(min(edge_widths)),
        "maximum_edge_width_m": float(max(edge_widths)),
    }
    return occupied, top_height, metadata


def add_static_box(stage, path: str, size: tuple[float, float, float], position: tuple[float, float, float], color):
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    transform = UsdGeom.XformCommonAPI(cube)
    transform.SetTranslate(Gf.Vec3d(*position))
    transform.SetScale(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def make_maze_mesh(stage, occupied: np.ndarray, top_height: np.ndarray, width: float, depth: float, cell: float):
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

    for row in range(ny):
        for column in range(nx):
            if not occupied[row, column]:
                continue
            x0 = -0.5 * width + column * cell
            x1 = x0 + cell
            y0 = -0.5 * depth + row * cell
            y1 = y0 + cell
            z1 = float(top_height[row, column])
            quad([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)])
            quad([(x0, y1, 0.0), (x1, y1, 0.0), (x1, y0, 0.0), (x0, y0, 0.0)])
            neighbours = ((0, -1, x0, y0, x0, y1), (0, 1, x1, y1, x1, y0),
                          (-1, 0, x0, y1, x1, y1), (1, 0, x1, y0, x0, y0))
            for dr, dc, xa, ya, xb, yb in neighbours:
                nr, nc = row + dr, column + dc
                neighbour_top = (
                    0.0
                    if not (0 <= nr < ny and 0 <= nc < nx and occupied[nr, nc])
                    else float(top_height[nr, nc])
                )
                if neighbour_top >= z1 - 1.0e-6:
                    continue
                lower = max(0.0, neighbour_top)
                if dr == 0 and dc == -1:
                    quad([(xa, ya, lower), (xb, yb, lower), (xb, yb, z1), (xa, ya, z1)])
                elif dr == 0 and dc == 1:
                    quad([(xb, yb, lower), (xa, ya, lower), (xa, ya, z1), (xb, yb, z1)])
                elif dr == -1:
                    quad([(xa, ya, lower), (xb, yb, lower), (xb, yb, z1), (xa, ya, z1)])
                else:
                    quad([(xb, yb, lower), (xa, ya, lower), (xa, ya, z1), (xb, yb, z1)])

    mesh = UsdGeom.Mesh.Define(stage, "/World/MazeObstacles/MazeField")
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in vertices])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.008, 0.012, 0.018)])
    mesh.GetPrim().CreateAttribute("fov:geometry", Sdf.ValueTypeNames.String).Set("connected_crack_maze")
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    collision.CreateApproximationAttr().Set("none")
    return len(vertices), len(counts)


def build_scene(args: argparse.Namespace) -> dict:
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    occupied, top_height, metadata = crack_maze_mask(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(args.output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

    add_static_box(
        stage,
        "/World/Ground",
        (args.width + 2.0, args.depth + 2.0, 0.10),
        (0.0, 0.0, -0.05),
        (0.92, 0.92, 0.92),
    )
    vertex_count, face_count = make_maze_mesh(
        stage, occupied, top_height, args.width, args.depth, args.cell_size
    )

    for path, x_key, y_key, color in (
        ("/World/Start", "start_x", "start_y", (0.1, 0.8, 0.2)),
        ("/World/Goal", "goal_x", "goal_y", (0.95, 0.2, 0.1)),
    ):
        marker = UsdGeom.Cylinder.Define(stage, path)
        marker.CreateRadiusAttr(0.28)
        marker.CreateHeightAttr(0.025)
        UsdGeom.XformCommonAPI(marker).SetTranslate(
            Gf.Vec3d(metadata[x_key], metadata[y_key], 0.015)
        )
        marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.65, 0.75, 0.90))
    overview = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    overview.CreateFocalLengthAttr(24.0)
    overview.CreateClippingRangeAttr((0.1, 100.0))
    UsdGeom.XformCommonAPI(overview).SetTranslate(Gf.Vec3d(-10.5, -13.0, 11.0))
    UsdGeom.XformCommonAPI(overview).SetRotate(
        Gf.Vec3f(48.0, 0.0, -39.0), UsdGeom.XformCommonAPI.RotationOrderXYZ
    )

    manifest = {
        "generator": "connected_crack_maze_v1",
        "seed": args.seed,
        "width_m": args.width,
        "depth_m": args.depth,
        "cell_size_m": args.cell_size,
        "maze_columns": args.maze_columns,
        "maze_rows": args.maze_rows,
        "corridor_width_m": args.corridor_width,
        "corridor_width_variation": args.width_variation,
        "junction_radius_m": args.junction_radius,
        "minimum_validated_clearance_m": args.min_clearance,
        "obstacle_height_m": args.height,
        "height_variation_m": args.height_variation,
        "obstacle_prim": "/World/MazeObstacles/MazeField",
        "start": [metadata["start_x"], metadata["start_y"], 1.2],
        "goal": [metadata["goal_x"], metadata["goal_y"], 1.2],
        "stats": metadata,
        "mesh_vertices": vertex_count,
        "mesh_faces": face_count,
    }
    manifest_path = args.output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    map_path = args.output.with_name(args.output.stem + "_map.png")
    image = np.where(occupied, 0, 255).astype(np.uint8)
    Image.fromarray(np.flipud(image), mode="L").resize(
        (image.shape[1] * 6, image.shape[0] * 6), Image.Resampling.NEAREST
    ).save(map_path)
    stage.Save()
    return {"stage": str(args.output), "manifest": str(manifest_path), "map": str(map_path), **manifest}


def main() -> None:
    args = parse_args()
    result = build_scene(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
