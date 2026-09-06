#!/usr/bin/env python3
"""Closed-loop numerical simulation for depth-only angular harmonic guidance.

The online guidance algorithm receives only a pinhole depth image, camera
calibration, the current state, and a reference velocity.  Analytic 3-D
geometry is used only by the depth renderer and the truth evaluator.

Coordinate convention
---------------------
World:  X forward, Y right, Z up.
Camera: x right, y down, z forward.
"""

from __future__ import annotations

import argparse
import csv
import inspect
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.linalg import cg


EPS = 1.0e-9


def normalize(v: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > EPS and np.all(np.isfinite(v)):
        return v / n
    if fallback is not None:
        return np.asarray(fallback, dtype=float).copy()
    return np.zeros_like(v, dtype=float)


def clamp_norm(v: np.ndarray, limit: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= limit or n <= EPS:
        return v
    return v * (limit / n)


@dataclass
class Camera:
    width: int = 48
    height: int = 36
    hfov_deg: float = 90.0
    vfov_deg: float = 68.0
    max_depth: float = 8.0

    def __post_init__(self) -> None:
        self.cx = 0.5 * (self.width - 1)
        self.cy = 0.5 * (self.height - 1)
        self.fx = 0.5 * self.width / math.tan(math.radians(self.hfov_deg) / 2.0)
        self.fy = 0.5 * self.height / math.tan(math.radians(self.vfov_deg) / 2.0)
        uu, vv = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.u_grid = uu.astype(float)
        self.v_grid = vv.astype(float)
        rays = np.stack(
            [
                (self.u_grid - self.cx) / self.fx,
                (self.v_grid - self.cy) / self.fy,
                np.ones_like(self.u_grid),
            ],
            axis=-1,
        )
        self.rays_c = rays / np.linalg.norm(rays, axis=-1, keepdims=True)

    def ray_from_pixel(self, q: np.ndarray) -> np.ndarray:
        u, v = float(q[0]), float(q[1])
        return normalize(np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0]))

    def pixel_from_direction(self, d_c: np.ndarray) -> Optional[np.ndarray]:
        if not np.all(np.isfinite(d_c)) or d_c[2] <= 1.0e-5:
            return None
        q = np.array(
            [self.fx * d_c[0] / d_c[2] + self.cx,
             self.fy * d_c[1] / d_c[2] + self.cy],
            dtype=float,
        )
        if not self.inside(q, margin=0.0):
            return None
        return q

    def inside(self, q: np.ndarray, margin: float = 0.0) -> bool:
        return (
            margin <= q[0] <= self.width - 1 - margin
            and margin <= q[1] <= self.height - 1 - margin
        )


# Camera axes expressed in the body/world frame for the standard forward mount
# Rz(-pi/2) * Rx(-pi/2): camera z->body X, x->body -Y, y->body -Z.
R_WC_FIXED = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


class Obstacle:
    name: str

    def ray_intersections(self, origin: np.ndarray, directions: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def segment_collision(self, a: np.ndarray, b: np.ndarray, body_radius: float) -> bool:
        raise NotImplementedError


@dataclass
class SphereObstacle(Obstacle):
    center: np.ndarray
    radius: float
    name: str = "sphere"

    def ray_intersections(self, origin: np.ndarray, directions: np.ndarray) -> np.ndarray:
        oc = origin - self.center
        b = directions @ oc
        c = float(oc @ oc - self.radius * self.radius)
        disc = b * b - c
        out = np.full(directions.shape[0], np.inf)
        valid = disc >= 0.0
        root = np.zeros_like(disc)
        root[valid] = np.sqrt(disc[valid])
        t0 = -b - root
        t1 = -b + root
        t = np.where(t0 > 1.0e-6, t0, np.where(t1 > 1.0e-6, t1, np.inf))
        out[valid] = t[valid]
        return out

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        return np.linalg.norm(points - self.center, axis=-1) - self.radius

    def segment_collision(self, a: np.ndarray, b: np.ndarray, body_radius: float) -> bool:
        ab = b - a
        denom = float(ab @ ab)
        if denom <= EPS:
            d = float(np.linalg.norm(a - self.center))
        else:
            t = float(np.clip((self.center - a) @ ab / denom, 0.0, 1.0))
            d = float(np.linalg.norm(a + t * ab - self.center))
        return d <= self.radius + body_radius


@dataclass
class AABBObstacle(Obstacle):
    minimum: np.ndarray
    maximum: np.ndarray
    name: str = "box"

    def ray_intersections(self, origin: np.ndarray, directions: np.ndarray) -> np.ndarray:
        inv = np.empty_like(directions)
        np.divide(1.0, directions, out=inv, where=np.abs(directions) > 1.0e-12)
        inv[np.abs(directions) <= 1.0e-12] = np.inf
        t0 = (self.minimum - origin) * inv
        t1 = (self.maximum - origin) * inv
        tmin = np.max(np.minimum(t0, t1), axis=1)
        tmax = np.min(np.maximum(t0, t1), axis=1)
        valid = (tmax >= np.maximum(tmin, 0.0)) & (tmax > 1.0e-6)
        t = np.where(tmin > 1.0e-6, tmin, tmax)
        return np.where(valid, t, np.inf)

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        center = 0.5 * (self.minimum + self.maximum)
        half = 0.5 * (self.maximum - self.minimum)
        q = np.abs(points - center) - half
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
        inside = np.minimum(np.max(q, axis=-1), 0.0)
        return outside + inside

    def segment_collision(self, a: np.ndarray, b: np.ndarray, body_radius: float) -> bool:
        lo = self.minimum - body_radius
        hi = self.maximum + body_radius
        d = b - a
        t_enter, t_exit = 0.0, 1.0
        for k in range(3):
            if abs(d[k]) < 1.0e-12:
                if a[k] < lo[k] or a[k] > hi[k]:
                    return False
                continue
            ta = (lo[k] - a[k]) / d[k]
            tb = (hi[k] - a[k]) / d[k]
            if ta > tb:
                ta, tb = tb, ta
            t_enter = max(t_enter, ta)
            t_exit = min(t_exit, tb)
            if t_enter > t_exit:
                return False
        return True


@dataclass
class Scene:
    name: str
    obstacles: list[Obstacle]
    start: np.ndarray
    goal: np.ndarray
    max_time: float = 18.0

    def render_depth(self, camera: Camera, position_w: np.ndarray, R_wc: np.ndarray) -> np.ndarray:
        rays_w = camera.rays_c.reshape(-1, 3) @ R_wc.T
        best_t = np.full(rays_w.shape[0], np.inf)
        for obstacle in self.obstacles:
            best_t = np.minimum(best_t, obstacle.ray_intersections(position_w, rays_w))
        ray_z = camera.rays_c.reshape(-1, 3)[:, 2]
        depth_z = best_t * ray_z
        hit = np.isfinite(depth_z) & (depth_z > 0.0) & (depth_z <= camera.max_depth)
        depth = np.full(best_t.shape[0], camera.max_depth, dtype=float)
        depth[hit] = depth_z[hit]
        return depth.reshape(camera.height, camera.width)

    def surface_clearance(self, position_w: np.ndarray, body_radius: float) -> float:
        if not self.obstacles:
            return float("inf")
        point = position_w.reshape(1, 3)
        return min(float(obs.signed_distance(point)[0] - body_radius) for obs in self.obstacles)

    def segment_collision(self, a: np.ndarray, b: np.ndarray, body_radius: float) -> bool:
        return any(obs.segment_collision(a, b, body_radius) for obs in self.obstacles)


@dataclass
class SimConfig:
    body_radius: float = 0.25
    safety_margin: float = 0.12
    reference_speed: float = 1.2
    max_accel: float = 3.0
    brake_accel: float = 2.5
    velocity_tau: float = 0.22
    delay: float = 0.10
    planning_horizon: float = 0.90
    control_dt: float = 0.05
    dynamics_dt: float = 0.01
    rollout_dt: float = 0.05
    rollout_horizon: float = 0.80
    rollout_margin: float = 0.10
    max_direction_rate: float = 1.60
    goal_tolerance: float = 0.28
    clearance_reward: float = 0.018
    hysteresis_weight: float = 0.20
    deterministic_left_bias: float = 0.012
    convergence_distance: float = 2.5
    convergence_margin: float = 1.0
    convergence_length: float = 1.5
    convergence_max_angle: float = 0.6
    source_radius_cells: int = 1
    goal_radius_cells: int = 1
    field_tolerance: float = 1.0e-7
    field_max_iterations: int = 1200
    depth_point_stride: int = 2
    cone_chunk_size: int = 256


@dataclass
class AngularSolution:
    depth: np.ndarray
    free_distance: np.ndarray
    planning_mask: np.ndarray
    potential: np.ndarray
    q_ref: np.ndarray
    q_source: np.ndarray
    q_goal: np.ndarray
    q_cmd: np.ndarray
    solve_ms: float
    field_valid: bool


@dataclass
class RunResult:
    scene: str
    success: bool
    collided: bool
    stopped_safely: bool
    timeout: bool
    final_distance: float
    min_clearance: float
    path_length: float
    elapsed_time: float
    max_speed: float
    max_accel: float
    rms_accel: float
    rms_jerk: float
    direction_flip_count: int
    solve_ms_p50: float
    solve_ms_p95: float
    solve_ms_p99: float
    trajectory: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    clearances: np.ndarray
    times: np.ndarray
    representative: Optional[AngularSolution]

    def summary_dict(self) -> dict:
        d = asdict(self)
        for key in [
            "trajectory", "velocities", "accelerations", "clearances", "times", "representative"
        ]:
            d.pop(key, None)
        for key, value in list(d.items()):
            if isinstance(value, float) and not math.isfinite(value):
                d[key] = None
        return d


def make_scenes() -> dict[str, Scene]:
    floor = AABBObstacle(
        minimum=np.array([-1.0, -5.0, -1.0]),
        maximum=np.array([12.0, 5.0, 0.0]),
        name="floor",
    )
    return {
        "empty": Scene(
            "empty", [], np.array([0.0, 0.0, 1.2]), np.array([7.0, 0.0, 1.2]), 10.0
        ),
        "single_pillar": Scene(
            "single_pillar",
            [
                AABBObstacle(
                    np.array([3.0, -0.40, 0.0]),
                    np.array([3.8, 0.40, 2.8]),
                    "single_pillar",
                )
            ],
            np.array([0.0, 0.0, 1.2]),
            np.array([7.2, 0.0, 1.2]),
            17.0,
        ),
        "offset_box": Scene(
            "offset_box",
            [
                AABBObstacle(
                    np.array([3.0, -0.45, 0.25]),
                    np.array([3.8, 1.05, 2.25]),
                    "offset_box",
                )
            ],
            np.array([0.0, 0.0, 1.2]),
            np.array([7.5, 0.0, 1.2]),
            16.0,
        ),
        "center_sphere": Scene(
            "center_sphere",
            [SphereObstacle(np.array([3.2, 0.0, 1.2]), 0.72, "center_sphere")],
            np.array([0.0, 0.0, 1.2]),
            np.array([7.2, 0.0, 1.2]),
            17.0,
        ),
        "overhead_bar": Scene(
            "overhead_bar",
            [
                floor,
                AABBObstacle(
                    np.array([3.0, -20.0, 0.15]),
                    np.array([3.7, 20.0, 1.35]),
                    "horizontal_bar",
                ),
            ],
            np.array([0.0, 0.0, 1.2]),
            np.array([7.5, 0.0, 1.2]),
            18.0,
        ),
        "diagonal_gap": Scene(
            "diagonal_gap",
            [
                floor,
                AABBObstacle(
                    np.array([3.0, -2.2, 0.0]),
                    np.array([3.8, -0.18, 1.18]),
                    "lower_left_block",
                ),
                AABBObstacle(
                    np.array([3.0, 0.18, 1.48]),
                    np.array([3.8, 2.2, 3.0]),
                    "upper_right_block",
                ),
            ],
            np.array([0.0, 0.0, 1.2]),
            np.array([7.5, 0.0, 1.3]),
            18.0,
        ),
        "narrow_gate": Scene(
            "narrow_gate",
            [
                floor,
                AABBObstacle(
                    np.array([3.0, -2.2, 0.0]),
                    np.array([3.8, -0.24, 2.8]),
                    "gate_left",
                ),
                AABBObstacle(
                    np.array([3.0, 0.24, 0.0]),
                    np.array([3.8, 2.2, 2.8]),
                    "gate_right",
                ),
            ],
            np.array([0.0, 0.0, 1.2]),
            np.array([7.0, 0.0, 1.2]),
            10.0,
        ),
    }


def backproject_obstacle_points(depth: np.ndarray, camera: Camera, stride: int) -> np.ndarray:
    hit = np.isfinite(depth) & (depth > 0.0) & (depth < camera.max_depth - 1.0e-6)
    if stride > 1:
        keep = np.zeros_like(hit)
        keep[::stride, ::stride] = True
        hit &= keep
    if not np.any(hit):
        return np.empty((0, 3))
    z = depth[hit]
    x = z * (camera.u_grid[hit] - camera.cx) / camera.fx
    y = z * (camera.v_grid[hit] - camera.cy) / camera.fy
    return np.stack([x, y, z], axis=1)


def collision_cone_free_distance(
    points_c: np.ndarray,
    camera: Camera,
    effective_radius: float,
    chunk_size: int,
) -> np.ndarray:
    """First sphere-swept contact distance along every candidate camera ray."""
    rays = camera.rays_c.reshape(-1, 3)
    # Even with no return, the algorithm only knows free space up to sensor range.
    free = np.full(rays.shape[0], max(0.0, camera.max_depth - effective_radius))
    if points_c.shape[0] == 0:
        return free.reshape(camera.height, camera.width)
    p2 = np.sum(points_c * points_c, axis=1)
    r2 = effective_radius * effective_radius
    for start in range(0, rays.shape[0], chunk_size):
        stop = min(start + chunk_size, rays.shape[0])
        rr = rays[start:stop]
        longitudinal = rr @ points_c.T
        lateral2 = np.maximum(0.0, p2[None, :] - longitudinal * longitudinal)
        valid = (longitudinal > 0.0) & (lateral2 < r2)
        root = np.sqrt(np.maximum(0.0, r2 - lateral2))
        contact = np.where(valid, longitudinal - root, np.inf)
        nearest = np.min(contact, axis=1)
        free[start:stop] = np.minimum(free[start:stop], nearest)
    return free.reshape(camera.height, camera.width)


def bilinear_sample(image: np.ndarray, q: np.ndarray, default: float = np.nan) -> float:
    h, w = image.shape
    u, v = float(q[0]), float(q[1])
    if u < 0.0 or v < 0.0 or u > w - 1 or v > h - 1:
        return default
    u0 = int(math.floor(u))
    v0 = int(math.floor(v))
    u1 = min(u0 + 1, w - 1)
    v1 = min(v0 + 1, h - 1)
    tu, tv = u - u0, v - v0
    values = np.array([image[v0, u0], image[v0, u1], image[v1, u0], image[v1, u1]])
    if not np.all(np.isfinite(values)):
        return default
    return float(
        (1.0 - tv) * ((1.0 - tu) * values[0] + tu * values[1])
        + tv * ((1.0 - tu) * values[2] + tu * values[3])
    )


def nearest_free_pixel(
    q: np.ndarray,
    free_mask: np.ndarray,
    q_history: Optional[np.ndarray],
    left_bias: float,
) -> Optional[np.ndarray]:
    vv, uu = np.nonzero(free_mask)
    if uu.size == 0:
        return None
    du = (uu - q[0]) / max(1.0, free_mask.shape[1])
    dv = (vv - q[1]) / max(1.0, free_mask.shape[0])
    cost = du * du + dv * dv
    if q_history is not None:
        cost += 0.15 * (
            ((uu - q_history[0]) / free_mask.shape[1]) ** 2
            + ((vv - q_history[1]) / free_mask.shape[0]) ** 2
        )
    # Negative image-u is left. Positive left_bias consistently favors it.
    cost += left_bias * (uu - q[0]) / max(1.0, free_mask.shape[1])
    k = int(np.argmin(cost))
    return np.array([float(uu[k]), float(vv[k])])


def choose_safe_goal(
    q_ref: np.ndarray,
    q_source: np.ndarray,
    mask: np.ndarray,
    q_goal_prev: Optional[np.ndarray],
    cfg: SimConfig,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
    free = ~mask
    labels, _ = ndimage.label(free, structure=np.ones((3, 3), dtype=int))
    q_source_safe = nearest_free_pixel(
        q_source, free, q_goal_prev, cfg.deterministic_left_bias
    )
    if q_source_safe is None:
        return None, None, labels
    su = int(round(q_source_safe[0]))
    sv = int(round(q_source_safe[1]))
    component = labels[sv, su]
    if component <= 0:
        return None, None, labels
    candidate = labels == component
    ru = int(round(np.clip(q_ref[0], 0, mask.shape[1] - 1)))
    rv = int(round(np.clip(q_ref[1], 0, mask.shape[0] - 1)))
    if candidate[rv, ru]:
        return q_source_safe, np.array([float(ru), float(rv)]), labels

    clearance = ndimage.distance_transform_edt(free)
    vv, uu = np.nonzero(candidate)
    if uu.size == 0:
        return None, None, labels
    du = (uu - q_ref[0]) / max(1.0, mask.shape[1])
    dv = (vv - q_ref[1]) / max(1.0, mask.shape[0])
    cost = du * du + dv * dv
    cost -= cfg.clearance_reward * clearance[vv, uu]
    if q_goal_prev is not None:
        cost += cfg.hysteresis_weight * (
            ((uu - q_goal_prev[0]) / mask.shape[1]) ** 2
            + ((vv - q_goal_prev[1]) / mask.shape[0]) ** 2
        )
    cost += cfg.deterministic_left_bias * (uu - q_ref[0]) / mask.shape[1]
    k = int(np.argmin(cost))
    return q_source_safe, np.array([float(uu[k]), float(vv[k])]), labels


def disk_mask(shape: tuple[int, int], q: np.ndarray, radius: int) -> np.ndarray:
    vv, uu = np.ogrid[: shape[0], : shape[1]]
    return (uu - q[0]) ** 2 + (vv - q[1]) ** 2 <= radius * radius


def solve_angular_harmonic(
    mask: np.ndarray,
    q_source: np.ndarray,
    q_goal: np.ndarray,
    cfg: SimConfig,
) -> tuple[np.ndarray, bool]:
    """Solve Laplace(phi)=0 with source/goal Dirichlet and wall Neumann BCs."""
    h, w = mask.shape
    source = disk_mask(mask.shape, q_source, cfg.source_radius_cells) & ~mask
    goal = disk_mask(mask.shape, q_goal, cfg.goal_radius_cells) & ~mask
    if not np.any(source) or not np.any(goal):
        return np.full(mask.shape, np.nan), False
    if np.any(source & goal):
        phi = np.full(mask.shape, np.nan)
        phi[~mask] = 0.0
        phi[source] = 1.0
        return phi, True

    dirichlet = source | goal
    unknown = (~mask) & (~dirichlet)
    index = -np.ones(mask.shape, dtype=int)
    index[unknown] = np.arange(np.count_nonzero(unknown))
    n = int(np.count_nonzero(unknown))
    if n == 0:
        phi = np.full(mask.shape, np.nan)
        phi[source] = 1.0
        phi[goal] = 0.0
        return phi, True

    rows: list[int] = []
    cols: list[int] = []
    data: list[float] = []
    rhs = np.zeros(n)
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for v, u in np.argwhere(unknown):
        row = index[v, u]
        degree = 0.0
        for dv, du in neighbors:
            nv, nu = v + dv, u + du
            # Outside FOV and obstacle faces are zero-normal-flux boundaries.
            if nv < 0 or nv >= h or nu < 0 or nu >= w or mask[nv, nu]:
                continue
            degree += 1.0
            if unknown[nv, nu]:
                rows.append(row)
                cols.append(index[nv, nu])
                data.append(-1.0)
            elif source[nv, nu]:
                rhs[row] += 1.0
            # goal value is zero; no RHS contribution.
        if degree <= 0.0:
            degree = 1.0
        rows.append(row)
        cols.append(row)
        data.append(degree)
    A = sparse.csr_matrix((data, (rows, cols)), shape=(n, n))
    tolerance_arg = "rtol" if "rtol" in inspect.signature(cg).parameters else "tol"
    x, info = cg(
        A,
        rhs,
        atol=0.0,
        maxiter=cfg.field_max_iterations,
        **{tolerance_arg: cfg.field_tolerance},
    )
    phi = np.full(mask.shape, np.nan)
    phi[source] = 1.0
    phi[goal] = 0.0
    if info != 0 or not np.all(np.isfinite(x)):
        return phi, False
    phi[unknown] = np.clip(x, 0.0, 1.0)
    return phi, True


def discrete_harmonic_path(
    phi: np.ndarray,
    mask: np.ndarray,
    q_source: np.ndarray,
    q_goal: np.ndarray,
    max_steps: int = 500,
) -> list[np.ndarray]:
    h, w = mask.shape
    current = np.array(
        [int(round(np.clip(q_source[0], 0, w - 1))), int(round(np.clip(q_source[1], 0, h - 1)))],
        dtype=int,
    )
    goal = np.array([int(round(q_goal[0])), int(round(q_goal[1]))], dtype=int)
    path = [current.astype(float)]
    visited = {(int(current[0]), int(current[1]))}
    for _ in range(max_steps):
        if np.linalg.norm(current - goal) <= 1.5:
            path.append(goal.astype(float))
            break
        candidates = []
        for dv in (-1, 0, 1):
            for du in (-1, 0, 1):
                if du == 0 and dv == 0:
                    continue
                u, v = current[0] + du, current[1] + dv
                if u < 0 or u >= w or v < 0 or v >= h or mask[v, u]:
                    continue
                value = phi[v, u]
                if not np.isfinite(value):
                    continue
                repeat_penalty = 0.2 if (u, v) in visited else 0.0
                goal_tie = 1.0e-4 * np.linalg.norm(np.array([u, v]) - goal)
                candidates.append((value + repeat_penalty + goal_tie, u, v))
        if not candidates:
            break
        _, u, v = min(candidates, key=lambda z: z[0])
        nxt = np.array([u, v], dtype=int)
        if np.array_equal(nxt, current):
            break
        current = nxt
        path.append(current.astype(float))
        visited.add((u, v))
    return path


def angular_distance(camera: Camera, qa: np.ndarray, qb: np.ndarray) -> float:
    ra = camera.ray_from_pixel(qa)
    rb = camera.ray_from_pixel(qb)
    return float(math.acos(np.clip(ra @ rb, -1.0, 1.0)))


def angular_rate_limit(
    camera: Camera,
    q_from: np.ndarray,
    q_to: np.ndarray,
    max_angle: float,
) -> np.ndarray:
    angle = angular_distance(camera, q_from, q_to)
    if angle <= max_angle or angle <= EPS:
        return q_to.copy()
    return q_from + (max_angle / angle) * (q_to - q_from)


def reference_converged_direction(
    field_direction_w: np.ndarray,
    position_w: np.ndarray,
    reference_origin_w: np.ndarray,
    reference_direction_w: np.ndarray,
    observed_clearance: float,
    cfg: SimConfig,
) -> np.ndarray:
    e_xy = normalize(reference_direction_w[:2])
    horizontal_speed = float(np.linalg.norm(field_direction_w[:2]))
    if horizontal_speed <= EPS or float(np.linalg.norm(e_xy)) <= EPS:
        return field_direction_w.copy()
    n_xy = np.array([-e_xy[1], e_xy[0]])
    phi = float(np.dot(position_w[:2] - reference_origin_w[:2], n_xy))
    theta_field = math.atan2(field_direction_w[1], field_direction_w[0])
    theta_target = math.atan2(e_xy[1], e_xy[0]) - math.atan(
        phi / max(EPS, cfg.convergence_length)
    )
    delta = math.atan2(
        math.sin(theta_target - theta_field), math.cos(theta_target - theta_field)
    )
    xi = np.clip(
        (observed_clearance - cfg.convergence_distance)
        / max(EPS, cfg.convergence_margin),
        0.0,
        1.0,
    )
    gate = float(xi * xi * (3.0 - 2.0 * xi))
    delta = float(np.clip(delta, -cfg.convergence_max_angle, cfg.convergence_max_angle)) * gate
    theta = theta_field + delta
    corrected = field_direction_w.copy()
    corrected[0] = horizontal_speed * math.cos(theta)
    corrected[1] = horizontal_speed * math.sin(theta)
    return normalize(corrected, field_direction_w)


def select_command_direction(
    camera: Camera,
    q_previous: np.ndarray,
    q_source_safe: np.ndarray,
    q_goal: np.ndarray,
    phi: np.ndarray,
    mask: np.ndarray,
    cfg: SimConfig,
) -> np.ndarray:
    max_angle = cfg.max_direction_rate * cfg.control_dt
    if angular_distance(camera, q_previous, q_source_safe) > 0.5 * max_angle:
        target = q_source_safe
    else:
        path = discrete_harmonic_path(phi, mask, q_source_safe, q_goal)
        target = path[min(2, len(path) - 1)] if path else q_goal
    q_cmd = angular_rate_limit(camera, q_previous, target, max_angle)
    q_cmd[0] = np.clip(q_cmd[0], 0.0, camera.width - 1.0)
    q_cmd[1] = np.clip(q_cmd[1], 0.0, camera.height - 1.0)
    return q_cmd


def braking_speed(distance: float, reference_speed: float, cfg: SimConfig) -> float:
    usable = max(0.0, distance - 0.05)
    ab = cfg.brake_accel
    safe = -ab * cfg.delay + math.sqrt((ab * cfg.delay) ** 2 + 2.0 * ab * usable)
    return max(0.0, min(reference_speed, safe))


def rollout_is_safe(
    position_w: np.ndarray,
    velocity_w: np.ndarray,
    command_w: np.ndarray,
    points_w: np.ndarray,
    camera_position_w: np.ndarray,
    R_cw: np.ndarray,
    camera: Camera,
    cfg: SimConfig,
) -> bool:
    if points_w.shape[0] == 0:
        return True
    p = position_w.copy()
    v = velocity_w.copy()
    # Keep the requested geometric inflation in the angular mask.  Rollout
    # alone gets a speed-scaled model-error allowance, which vanishes at rest.
    # At 2 m/s it makes the vehicle turn before a nearby pillar fills the whole
    # forward FOV instead of permanently enlarging the geometric obstacle.
    margin_scale = min(
        1.0,
        float(np.linalg.norm(velocity_w)) / max(EPS, cfg.reference_speed),
    )
    effective_radius = (
        cfg.body_radius + cfg.safety_margin + margin_scale * cfg.rollout_margin
    )
    steps = int(math.ceil(cfg.rollout_horizon / cfg.rollout_dt))
    for _ in range(steps):
        a = clamp_norm((command_w - v) / cfg.velocity_tau, cfg.max_accel)
        v_next = v + cfg.rollout_dt * a
        p_next = p + cfg.rollout_dt * v_next
        # Dense point-surface approximation to a swept sphere.
        seg = p_next - p
        denom = float(seg @ seg)
        if denom <= EPS:
            d2 = np.sum((points_w - p) ** 2, axis=1)
        else:
            t = np.clip(((points_w - p) @ seg) / denom, 0.0, 1.0)
            closest = p[None, :] + t[:, None] * seg[None, :]
            d2 = np.sum((points_w - closest) ** 2, axis=1)
        if float(np.min(d2)) <= effective_radius * effective_radius:
            return False
        rel_c = R_cw @ (p_next - camera_position_w)
        q = camera.pixel_from_direction(rel_c)
        if q is None:
            return False
        p, v = p_next, v_next
    return True


def depth_rollout_limit(
    desired_speed: float,
    ray_w: np.ndarray,
    position_w: np.ndarray,
    velocity_w: np.ndarray,
    points_w: np.ndarray,
    R_cw: np.ndarray,
    camera: Camera,
    cfg: SimConfig,
) -> float:
    if desired_speed <= 1.0e-5:
        return 0.0
    command = desired_speed * ray_w
    if rollout_is_safe(
        position_w, velocity_w, command, points_w, position_w, R_cw, camera, cfg
    ):
        return desired_speed
    lo, hi = 0.0, desired_speed
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        command = mid * ray_w
        if rollout_is_safe(
            position_w, velocity_w, command, points_w, position_w, R_cw, camera, cfg
        ):
            lo = mid
        else:
            hi = mid
    return lo


def compute_guidance(
    scene: Scene,
    camera: Camera,
    cfg: SimConfig,
    position_w: np.ndarray,
    velocity_w: np.ndarray,
    q_previous: np.ndarray,
    q_goal_previous: Optional[np.ndarray],
    R_wc: np.ndarray,
    reference_origin_w: Optional[np.ndarray] = None,
    reference_direction_w: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, np.ndarray, AngularSolution]:
    t0 = time.perf_counter()
    R_cw = R_wc.T
    depth = scene.render_depth(camera, position_w, R_wc)
    points_c = backproject_obstacle_points(depth, camera, cfg.depth_point_stride)
    effective_radius = cfg.body_radius + cfg.safety_margin
    free_distance = collision_cone_free_distance(
        points_c, camera, effective_radius, cfg.cone_chunk_size
    )

    goal_delta = scene.goal - position_w
    ref_speed = min(
        cfg.reference_speed,
        math.sqrt(max(0.0, 2.0 * cfg.brake_accel * np.linalg.norm(goal_delta))),
    )
    ref_dir_w = normalize(goal_delta, np.array([1.0, 0.0, 0.0]))
    ref_dir_c = R_cw @ ref_dir_w
    q_ref = camera.pixel_from_direction(ref_dir_c)
    if q_ref is None:
        if ref_dir_c[2] > EPS:
            q_ref = np.array(
                [
                    camera.fx * ref_dir_c[0] / ref_dir_c[2] + camera.cx,
                    camera.fy * ref_dir_c[1] / ref_dir_c[2] + camera.cy,
                ]
            )
        else:
            q_ref = np.array(
                [camera.width - 2.0 if ref_dir_c[0] >= 0.0 else 1.0, camera.cy]
            )
        q_ref[0] = np.clip(q_ref[0], 1.0, camera.width - 2.0)
        q_ref[1] = np.clip(q_ref[1], 1.0, camera.height - 2.0)

    stop_distance = ref_speed * cfg.delay + ref_speed * ref_speed / (2.0 * cfg.brake_accel)
    planning_distance = min(
        camera.max_depth - effective_radius,
        ref_speed * cfg.planning_horizon + stop_distance + 0.35,
    )
    planning_mask = free_distance <= planning_distance
    # Keep one pixel at the image boundary unavailable so the finite body does
    # not leave the observed FOV through a discretization gap.
    planning_mask[[0, -1], :] = True
    planning_mask[:, [0, -1]] = True

    q_source_safe, q_goal, _ = choose_safe_goal(
        q_ref, q_previous, planning_mask, q_goal_previous, cfg
    )
    if q_source_safe is None or q_goal is None:
        q_source_safe = q_previous.copy()
        q_goal = q_previous.copy()
        phi = np.full(planning_mask.shape, np.nan)
        field_valid = False
        q_cmd = q_previous.copy()
    elif angular_distance(camera, q_source_safe, q_goal) < math.radians(0.4):
        phi = np.full(planning_mask.shape, np.nan)
        phi[~planning_mask] = 0.0
        field_valid = True
        q_cmd = angular_rate_limit(
            camera, q_previous, q_goal, cfg.max_direction_rate * cfg.control_dt
        )
    else:
        phi, field_valid = solve_angular_harmonic(
            planning_mask, q_source_safe, q_goal, cfg
        )
        if field_valid:
            q_cmd = select_command_direction(
                camera, q_previous, q_source_safe, q_goal, phi, planning_mask, cfg
            )
        else:
            q_cmd = angular_rate_limit(
                camera, q_previous, q_goal, cfg.max_direction_rate * cfg.control_dt
            )

    ray_c = camera.ray_from_pixel(q_cmd)
    ray_w = R_wc @ ray_c
    if reference_origin_w is not None and reference_direction_w is not None:
        observed_clearance = (
            float(np.min(np.linalg.norm(points_c, axis=1)))
            if points_c.shape[0] > 0
            else camera.max_depth
        )
        ray_w = reference_converged_direction(
            ray_w,
            position_w,
            reference_origin_w,
            reference_direction_w,
            observed_clearance,
            cfg,
        )
        ray_c = R_cw @ ray_w
        q_corrected = camera.pixel_from_direction(ray_c)
        if q_corrected is not None:
            q_cmd = q_corrected
        elif ray_c[2] > EPS:
            q_cmd = np.array(
                [
                    camera.fx * ray_c[0] / ray_c[2] + camera.cx,
                    camera.fy * ray_c[1] / ray_c[2] + camera.cy,
                ]
            )
            q_cmd[0] = np.clip(q_cmd[0], 1.0, camera.width - 2.0)
            q_cmd[1] = np.clip(q_cmd[1], 1.0, camera.height - 2.0)
        ray_c = camera.ray_from_pixel(q_cmd)
        ray_w = R_wc @ ray_c
    d_free = bilinear_sample(free_distance, q_cmd, default=0.0)
    speed = braking_speed(d_free, ref_speed, cfg)
    points_w = position_w[None, :] + points_c @ R_wc.T
    speed = depth_rollout_limit(
        speed, ray_w, position_w, velocity_w, points_w, R_cw, camera, cfg
    )
    command_w = speed * ray_w
    solve_ms = 1000.0 * (time.perf_counter() - t0)
    solution = AngularSolution(
        depth=depth,
        free_distance=free_distance,
        planning_mask=planning_mask,
        potential=phi,
        q_ref=q_ref,
        q_source=q_source_safe,
        q_goal=q_goal,
        q_cmd=q_cmd,
        solve_ms=solve_ms,
        field_valid=field_valid,
    )
    return command_w, q_goal, solution


def simulate_scene(scene: Scene, camera: Camera, cfg: SimConfig) -> RunResult:
    position = scene.start.astype(float).copy()
    velocity = np.zeros(3)
    acceleration = np.zeros(3)
    R_wc = R_WC_FIXED.copy()
    q_previous = np.array([camera.cx, camera.cy])
    q_goal_previous: Optional[np.ndarray] = None
    command = np.zeros(3)
    next_plan = 0.0
    representative: Optional[AngularSolution] = None

    positions = [position.copy()]
    velocities = [velocity.copy()]
    accelerations = [acceleration.copy()]
    clearances = [scene.surface_clearance(position, cfg.body_radius)]
    times = [0.0]
    solve_times: list[float] = []
    direction_signs: list[int] = []
    collided = False
    success = False
    stopped_safely = False
    t = 0.0
    previous_acceleration = acceleration.copy()
    jerks: list[float] = []

    while t < scene.max_time:
        if t + 1.0e-9 >= next_plan:
            command, q_goal_previous, solution = compute_guidance(
                scene,
                camera,
                cfg,
                position,
                velocity,
                q_previous,
                q_goal_previous,
                R_wc,
            )
            q_previous = solution.q_cmd.copy()
            solve_times.append(solution.solve_ms)
            lateral_pixel = float(q_previous[0] - camera.cx)
            direction_signs.append(0 if abs(lateral_pixel) < 1.0 else int(np.sign(lateral_pixel)))
            obstacle_fraction = float(np.mean(solution.planning_mask))
            if representative is None or obstacle_fraction > float(
                np.mean(representative.planning_mask)
            ):
                representative = solution
            next_plan += cfg.control_dt

        acceleration = clamp_norm((command - velocity) / cfg.velocity_tau, cfg.max_accel)
        velocity_next = velocity + cfg.dynamics_dt * acceleration
        position_next = position + cfg.dynamics_dt * velocity_next
        if scene.segment_collision(position, position_next, cfg.body_radius):
            collided = True
            position = position_next
            velocity = velocity_next
            t += cfg.dynamics_dt
            positions.append(position.copy())
            velocities.append(velocity.copy())
            accelerations.append(acceleration.copy())
            clearances.append(scene.surface_clearance(position, cfg.body_radius))
            times.append(t)
            break
        jerk = float(np.linalg.norm((acceleration - previous_acceleration) / cfg.dynamics_dt))
        jerks.append(jerk)
        previous_acceleration = acceleration.copy()
        position, velocity = position_next, velocity_next
        t += cfg.dynamics_dt
        positions.append(position.copy())
        velocities.append(velocity.copy())
        accelerations.append(acceleration.copy())
        clearances.append(scene.surface_clearance(position, cfg.body_radius))
        times.append(t)

        if np.linalg.norm(position - scene.goal) <= cfg.goal_tolerance:
            success = True
            break

    if scene.name == "narrow_gate" and not collided:
        # A correct outcome is a stable stop before the geometrically closed gate.
        stopped_safely = (
            position[0] < 2.65
            and np.linalg.norm(velocity) < 0.12
            and t > 3.0
        )

    pos_arr = np.asarray(positions)
    vel_arr = np.asarray(velocities)
    acc_arr = np.asarray(accelerations)
    clr_arr = np.asarray(clearances)
    time_arr = np.asarray(times)
    segment_lengths = np.linalg.norm(np.diff(pos_arr, axis=0), axis=1)
    speed = np.linalg.norm(vel_arr, axis=1)
    accel_norm = np.linalg.norm(acc_arr, axis=1)
    nonzero_signs = [s for s in direction_signs if s != 0]
    flips = sum(a != b for a, b in zip(nonzero_signs, nonzero_signs[1:]))
    solve = np.asarray(solve_times) if solve_times else np.zeros(1)
    return RunResult(
        scene=scene.name,
        success=success,
        collided=collided,
        stopped_safely=stopped_safely,
        timeout=(not success and not collided and not stopped_safely),
        final_distance=float(np.linalg.norm(position - scene.goal)),
        min_clearance=float(np.min(clr_arr)),
        path_length=float(np.sum(segment_lengths)),
        elapsed_time=float(t),
        max_speed=float(np.max(speed)),
        max_accel=float(np.max(accel_norm)),
        rms_accel=float(np.sqrt(np.mean(accel_norm * accel_norm))),
        rms_jerk=float(np.sqrt(np.mean(np.square(jerks)))) if jerks else 0.0,
        direction_flip_count=int(flips),
        solve_ms_p50=float(np.percentile(solve, 50)),
        solve_ms_p95=float(np.percentile(solve, 95)),
        solve_ms_p99=float(np.percentile(solve, 99)),
        trajectory=pos_arr,
        velocities=vel_arr,
        accelerations=acc_arr,
        clearances=clr_arr,
        times=time_arr,
        representative=representative,
    )


def draw_box(ax, box: AABBObstacle, color: str = "tab:red", alpha: float = 0.22) -> None:
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    lo, hi = box.minimum, box.maximum
    corners = np.array(
        [[x, y, z] for x in [lo[0], hi[0]] for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]]
    )
    faces_idx = [
        [0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4],
        [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5],
    ]
    faces = [[corners[i] for i in face] for face in faces_idx]
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, alpha=alpha, edgecolor=color))


def draw_scene_3d(ax, scene: Scene, result: RunResult, cfg: SimConfig) -> None:
    for obstacle in scene.obstacles:
        if obstacle.name == "floor":
            continue
        if isinstance(obstacle, SphereObstacle):
            theta, phi = np.mgrid[0 : 2 * np.pi : 28j, 0 : np.pi : 14j]
            x = obstacle.center[0] + obstacle.radius * np.cos(theta) * np.sin(phi)
            y = obstacle.center[1] + obstacle.radius * np.sin(theta) * np.sin(phi)
            z = obstacle.center[2] + obstacle.radius * np.cos(phi)
            ax.plot_surface(x, y, z, color="tab:red", alpha=0.28, linewidth=0)
        elif isinstance(obstacle, AABBObstacle):
            draw_box(ax, obstacle)
    p = result.trajectory
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color="tab:blue", linewidth=2.2, label="trajectory")
    ax.scatter(*scene.start, color="tab:green", s=45, label="start")
    ax.scatter(*scene.goal, color="gold", edgecolor="black", s=60, label="goal")
    ax.set_xlabel("world X / m")
    ax.set_ylabel("world Y / m")
    ax.set_zlabel("world Z / m")
    ax.set_title(f"3-D closed-loop trajectory: {scene.name}")
    ax.legend(loc="upper left", fontsize=8)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((1.8, 1.0, 0.8))


def potential_gradient(phi: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    fill = phi.copy()
    if not np.any(np.isfinite(fill)):
        return np.zeros_like(fill), np.zeros_like(fill)
    nearest = ndimage.distance_transform_edt(~np.isfinite(fill), return_distances=False, return_indices=True)
    fill[~np.isfinite(fill)] = fill[tuple(nearest[:, ~np.isfinite(fill)])]
    gy, gx = np.gradient(fill)
    gx[mask] = 0.0
    gy[mask] = 0.0
    return -gx, -gy


def plot_run(scene: Scene, result: RunResult, camera: Camera, cfg: SimConfig, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401: registers projection="3d"

    fig = plt.figure(figsize=(15, 9))
    ax3 = fig.add_subplot(2, 2, 1, projection="3d")
    draw_scene_3d(ax3, scene, result, cfg)

    ax_depth = fig.add_subplot(2, 2, 2)
    rep = result.representative
    if rep is not None:
        image = ax_depth.imshow(rep.depth, cmap="viridis", origin="upper", vmin=0, vmax=camera.max_depth)
        overlay = np.ma.masked_where(~rep.planning_mask, rep.planning_mask)
        ax_depth.imshow(overlay, cmap="Reds", alpha=0.32, origin="upper")
        if rep.field_valid and np.any(np.isfinite(rep.potential)):
            fx, fy = potential_gradient(rep.potential, rep.planning_mask)
            magnitude = np.hypot(fx, fy)
            valid_vector = magnitude > 1.0e-8
            fx = np.divide(fx, magnitude, out=np.zeros_like(fx), where=valid_vector)
            fy = np.divide(fy, magnitude, out=np.zeros_like(fy), where=valid_vector)
            step = 4
            uu, vv = np.meshgrid(np.arange(camera.width), np.arange(camera.height))
            ax_depth.quiver(
                uu[::step, ::step], vv[::step, ::step],
                fx[::step, ::step], fy[::step, ::step],
                color="white", alpha=0.82, scale=16.0, width=0.003,
            )
        ax_depth.scatter(*rep.q_ref, c="cyan", marker="x", s=70, label="q_ref")
        ax_depth.scatter(*rep.q_goal, c="lime", marker="o", s=45, label="q_goal")
        ax_depth.scatter(*rep.q_cmd, c="white", marker="*", s=85, label="q_cmd")
        ax_depth.legend(loc="lower right", fontsize=8)
        fig.colorbar(image, ax=ax_depth, fraction=0.046, pad=0.04, label="optical-z depth / m")
    ax_depth.set_title("Representative depth, unsafe mask and angular field")
    ax_depth.set_xlabel("u / pixel")
    ax_depth.set_ylabel("v / pixel")

    ax_time = fig.add_subplot(2, 2, 3)
    speed = np.linalg.norm(result.velocities, axis=1)
    accel = np.linalg.norm(result.accelerations, axis=1)
    ax_time.plot(result.times, speed, label="speed / m s$^{-1}$")
    ax_time.plot(result.times, accel, label="acceleration / m s$^{-2}$", alpha=0.8)
    ax_time.set_xlabel("time / s")
    ax_time.grid(True, alpha=0.3)
    ax_time.legend()
    ax_time.set_title("Closed-loop motion")

    ax_clear = fig.add_subplot(2, 2, 4)
    finite = np.isfinite(result.clearances)
    ax_clear.plot(result.times[finite], result.clearances[finite], color="tab:purple", label="body-surface clearance")
    ax_clear.axhline(cfg.safety_margin, color="tab:orange", linestyle="--", label="requested margin")
    ax_clear.axhline(0.0, color="tab:red", linestyle=":", label="collision")
    ax_clear.set_xlabel("time / s")
    ax_clear.set_ylabel("clearance / m")
    ax_clear.grid(True, alpha=0.3)
    ax_clear.legend()
    status = "SUCCESS" if result.success else "SAFE STOP" if result.stopped_safely else "COLLISION" if result.collided else "TIMEOUT"
    ax_clear.set_title(f"{status}; min clearance={result.min_clearance:.3f} m")

    fig.suptitle("Depth-only angular harmonic guidance numerical simulation", fontsize=14)
    fig.tight_layout(pad=1.5)
    fig.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(fig)


def plot_summary(results: list[RunResult], output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [r.scene for r in results]
    x = np.arange(len(results))
    colors = ["tab:green" if r.success else "tab:blue" if r.stopped_safely else "tab:red" for r in results]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    clearance_values = [r.min_clearance if math.isfinite(r.min_clearance) else np.nan for r in results]
    axes[0].bar(x, clearance_values, color=colors)
    axes[0].axhline(0.0, color="black", linewidth=1)
    axes[0].set_title("Truth minimum body clearance")
    axes[0].set_ylabel("m")
    axes[1].bar(x, [r.path_length for r in results], color=colors)
    axes[1].set_title("Path length")
    axes[1].set_ylabel("m")
    axes[2].bar(x, [r.solve_ms_p95 for r in results], color=colors)
    axes[2].set_title("Guidance runtime p95")
    axes[2].set_ylabel("ms")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=25, ha="right")
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout(pad=1.5)
    fig.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(fig)


def write_results(results: list[RunResult], output_dir: Path, config: SimConfig, camera: Camera) -> None:
    summaries = [r.summary_dict() for r in results]
    payload = {
        "config": asdict(config),
        "camera": {
            "width": camera.width,
            "height": camera.height,
            "fx": camera.fx,
            "fy": camera.fy,
            "cx": camera.cx,
            "cy": camera.cy,
            "max_depth": camera.max_depth,
        },
        "results": summaries,
    }
    (output_dir / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    if summaries:
        with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(summaries[0].keys()))
            writer.writeheader()
            writer.writerows(summaries)
    for result in results:
        np.savez_compressed(
            output_dir / f"{result.scene}_log.npz",
            trajectory=result.trajectory,
            velocities=result.velocities,
            accelerations=result.accelerations,
            clearances=result.clearances,
            times=result.times,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        default="all",
        help="Scenario name or 'all': empty, offset_box, center_sphere, overhead_bar, diagonal_gap, narrow_gate",
    )
    parser.add_argument("--output", default="results", help="Output directory")
    parser.add_argument("--speed", type=float, default=1.2, help="Reference speed in m/s")
    parser.add_argument("--width", type=int, default=48, help="Depth/angular grid width")
    parser.add_argument("--height", type=int, default=36, help="Depth/angular grid height")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    camera = Camera(width=args.width, height=args.height)
    cfg = SimConfig(reference_speed=args.speed)
    scenes = make_scenes()
    if args.scenario == "all":
        selected = list(scenes.values())
    else:
        if args.scenario not in scenes:
            raise SystemExit(f"unknown scenario: {args.scenario}; choices={list(scenes)}")
        selected = [scenes[args.scenario]]

    results: list[RunResult] = []
    for scene in selected:
        print(f"[simulate] {scene.name}", flush=True)
        result = simulate_scene(scene, camera, cfg)
        results.append(result)
        plot_run(scene, result, camera, cfg, output_dir / f"{scene.name}.png")
        print(json.dumps(result.summary_dict(), ensure_ascii=False), flush=True)
    write_results(results, output_dir, cfg, camera)
    plot_summary(results, output_dir / "summary.png")
    return 0 if all(r.success or r.stopped_safely for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
