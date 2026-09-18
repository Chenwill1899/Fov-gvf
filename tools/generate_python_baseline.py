#!/usr/bin/env python3
"""Generate deterministic fixtures for the pre-C++ Python guidance behavior.

The fixtures intentionally preserve the current behavior, including known edge-case
semantics.  They are a migration oracle, not a statement that those semantics are safe.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE_ROOT = REPOSITORY_ROOT / "src" / "pc_gvf"
sys.path.insert(0, str(PYTHON_PACKAGE_ROOT))

from pc_gvf.depth_angular_core import (  # noqa: E402
    Camera,
    R_WC_FIXED,
    SimConfig,
    angular_distance,
    angular_rate_limit,
    backproject_obstacle_points,
    bilinear_sample,
    braking_speed,
    choose_safe_goal,
    compute_guidance,
    depth_rollout_limit,
    discrete_harmonic_path,
    make_scenes,
    reference_converged_direction,
    rollout_is_safe,
    select_command_direction,
    solve_angular_harmonic,
)


FORMAT_VERSION = 1
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "src" / "pc_gvf" / "test" / "fixtures" / "python_behavior_v1"
)


class FrozenDepthScene:
    def __init__(self, depth: np.ndarray, goal: np.ndarray) -> None:
        self.name = "frozen_depth"
        self.depth = np.asarray(depth, dtype=float).copy()
        self.goal = np.asarray(goal, dtype=float).copy()

    def render_depth(self, _camera, _position, _rotation):
        return self.depth.copy()


@dataclasses.dataclass
class Case:
    name: str
    depth: np.ndarray
    camera: Camera
    cfg: SimConfig
    position_w: np.ndarray
    velocity_w: np.ndarray
    goal_w: np.ndarray
    q_previous: np.ndarray
    q_goal_previous: Optional[np.ndarray]
    R_wc: np.ndarray
    reference_origin_w: Optional[np.ndarray] = None
    reference_direction_w: Optional[np.ndarray] = None


def rotation_z(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def scene_case(name: str, scene_name: str, x: float) -> Case:
    scene = make_scenes()[scene_name]
    camera = Camera()
    cfg = SimConfig()
    position = np.array([x, 0.0, 1.2])
    depth = scene.render_depth(camera, position, R_WC_FIXED)
    return Case(
        name=name,
        depth=depth,
        camera=camera,
        cfg=cfg,
        position_w=position,
        velocity_w=np.zeros(3),
        goal_w=scene.goal.copy(),
        q_previous=np.array([camera.cx, camera.cy]),
        q_goal_previous=None,
        R_wc=R_WC_FIXED.copy(),
    )


def frozen_case(
    name: str,
    depth: np.ndarray,
    *,
    camera: Optional[Camera] = None,
    position: Optional[np.ndarray] = None,
    velocity: Optional[np.ndarray] = None,
    goal: Optional[np.ndarray] = None,
    q_previous: Optional[np.ndarray] = None,
    q_goal_previous: Optional[np.ndarray] = None,
    R_wc: Optional[np.ndarray] = None,
    reference_origin: Optional[np.ndarray] = None,
    reference_direction: Optional[np.ndarray] = None,
) -> Case:
    camera = Camera() if camera is None else camera
    return Case(
        name=name,
        depth=np.asarray(depth, dtype=float),
        camera=camera,
        cfg=SimConfig(),
        position_w=np.array([0.0, 0.0, 1.2]) if position is None else position,
        velocity_w=np.zeros(3) if velocity is None else velocity,
        goal_w=np.array([7.0, 0.0, 1.2]) if goal is None else goal,
        q_previous=(
            np.array([camera.cx, camera.cy]) if q_previous is None else q_previous
        ),
        q_goal_previous=q_goal_previous,
        R_wc=R_WC_FIXED.copy() if R_wc is None else R_wc,
        reference_origin_w=reference_origin,
        reference_direction_w=reference_direction,
    )


def build_cases() -> list[Case]:
    cases = [
        scene_case("empty_start", "empty", 0.0),
        scene_case("single_pillar_near", "single_pillar", 1.8),
        scene_case("offset_box_near", "offset_box", 1.8),
        scene_case("center_sphere_near", "center_sphere", 1.8),
        scene_case("overhead_bar_near", "overhead_bar", 1.8),
        scene_case("diagonal_gap_near", "diagonal_gap", 1.8),
        scene_case("narrow_gate_near", "narrow_gate", 1.8),
    ]

    default_camera = Camera()
    clear = np.full((default_camera.height, default_camera.width), default_camera.max_depth)
    cases.append(frozen_case("all_zero_depth", np.zeros_like(clear)))
    cases.append(frozen_case("all_nan_depth", np.full_like(clear, np.nan)))

    kept = clear.copy()
    kept[18, 24] = 0.60
    cases.append(frozen_case("single_kept_near_pixel", kept))

    skipped = clear.copy()
    skipped[17, 23] = 0.60
    cases.append(frozen_case("single_skipped_near_pixel", skipped))

    obstacle_scene = make_scenes()["single_pillar"]
    obstacle_position = np.array([1.8, 0.0, 1.2])
    obstacle_depth = obstacle_scene.render_depth(
        default_camera, obstacle_position, R_WC_FIXED
    )
    cases.append(
        frozen_case(
            "history_and_lateral_velocity",
            obstacle_depth,
            position=obstacle_position,
            velocity=np.array([0.35, 0.55, 0.0]),
            goal=obstacle_scene.goal.copy(),
            q_previous=np.array([default_camera.cx + 4.0, default_camera.cy - 3.0]),
            q_goal_previous=np.array([default_camera.cx - 6.0, default_camera.cy + 2.0]),
        )
    )

    yawed_R_wc = rotation_z(math.radians(20.0)) @ R_WC_FIXED
    cases.append(
        frozen_case(
            "yawed_camera_clear",
            clear,
            R_wc=yawed_R_wc,
            q_previous=np.array([default_camera.cx - 2.0, default_camera.cy]),
        )
    )

    small_camera = Camera(width=32, height=24, hfov_deg=70.0, vfov_deg=55.0, max_depth=6.0)
    small_depth = np.full((small_camera.height, small_camera.width), small_camera.max_depth)
    small_depth[10:14, 14:18] = 1.15
    cases.append(frozen_case("alternate_intrinsics_patch", small_depth, camera=small_camera))

    cases.append(
        frozen_case(
            "reference_line_offset",
            obstacle_depth,
            position=np.array([1.8, 0.45, 1.2]),
            velocity=np.array([0.4, 0.0, 0.0]),
            goal=obstacle_scene.goal.copy(),
            reference_origin=np.array([0.0, 0.0, 1.2]),
            reference_direction=np.array([1.0, 0.0, 0.0]),
        )
    )

    cases.append(
        frozen_case(
            "goal_behind_camera",
            clear,
            goal=np.array([-3.0, 0.0, 1.2]),
        )
    )
    return cases


def scalar(value) -> str:
    if isinstance(value, (bool, np.bool_)):
        return "1" if value else "0"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    value = float(value)
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0.0 else "-inf"
    return format(value, ".17g")


def vector(value: Optional[np.ndarray]) -> str:
    if value is None:
        return "none"
    return ",".join(scalar(item) for item in np.asarray(value).reshape(-1))


def write_fixture(case: Case, output: Path) -> None:
    scene = FrozenDepthScene(case.depth, case.goal_w)
    obstacle_points = backproject_obstacle_points(
        case.depth, case.camera, case.cfg.depth_point_stride
    )
    command_w, q_goal_returned, solution = compute_guidance(
        scene,
        case.camera,
        case.cfg,
        case.position_w,
        case.velocity_w,
        case.q_previous,
        case.q_goal_previous,
        case.R_wc,
        reference_origin_w=case.reference_origin_w,
        reference_direction_w=case.reference_direction_w,
    )
    selected_source, selected_goal, component_labels = choose_safe_goal(
        solution.q_ref,
        case.q_previous,
        solution.planning_mask,
        case.q_goal_previous,
        case.cfg,
    )
    if selected_source is None or selected_goal is None:
        harmonic_potential = None
        harmonic_valid = False
        harmonic_path = None
        previous_source_angle = None
        rate_limited_goal = None
        direct_selected_command = None
        pre_reference_command = case.q_previous.copy()
    else:
        harmonic_potential, harmonic_valid = solve_angular_harmonic(
            solution.planning_mask, selected_source, selected_goal, case.cfg
        )
        harmonic_path = discrete_harmonic_path(
            harmonic_potential,
            solution.planning_mask,
            selected_source,
            selected_goal,
        )
        previous_source_angle = angular_distance(
            case.camera, case.q_previous, selected_source
        )
        rate_limited_goal = angular_rate_limit(
            case.camera,
            case.q_previous,
            selected_goal,
            case.cfg.max_direction_rate * case.cfg.control_dt,
        )
        direct_selected_command = select_command_direction(
            case.camera,
            case.q_previous,
            selected_source,
            selected_goal,
            harmonic_potential,
            solution.planning_mask,
            case.cfg,
        )
        if angular_distance(case.camera, selected_source, selected_goal) < math.radians(0.4):
            pre_reference_command = rate_limited_goal
        elif harmonic_valid:
            pre_reference_command = direct_selected_command
        else:
            pre_reference_command = rate_limited_goal

    raw_field_direction_w = case.R_wc @ case.camera.ray_from_pixel(
        pre_reference_command
    )
    observed_clearance = (
        float(np.min(np.linalg.norm(obstacle_points, axis=1)))
        if obstacle_points.shape[0] > 0
        else case.camera.max_depth
    )
    if case.reference_origin_w is not None and case.reference_direction_w is not None:
        reference_corrected_direction_w = reference_converged_direction(
            raw_field_direction_w,
            case.position_w,
            case.reference_origin_w,
            case.reference_direction_w,
            observed_clearance,
            case.cfg,
        )
    else:
        reference_corrected_direction_w = None

    command_free_distance = bilinear_sample(
        solution.free_distance, solution.q_cmd, default=0.0
    )
    goal_delta = case.goal_w - case.position_w
    reference_speed = min(
        case.cfg.reference_speed,
        math.sqrt(max(0.0, 2.0 * case.cfg.brake_accel * np.linalg.norm(goal_delta))),
    )
    pre_rollout_speed = braking_speed(
        command_free_distance, reference_speed, case.cfg
    )
    final_ray_w = case.R_wc @ case.camera.ray_from_pixel(solution.q_cmd)
    points_w = case.position_w[None, :] + obstacle_points @ case.R_wc.T
    full_speed_rollout_safe = rollout_is_safe(
        case.position_w,
        case.velocity_w,
        pre_rollout_speed * final_ray_w,
        points_w,
        case.position_w,
        case.R_wc.T,
        case.camera,
        case.cfg,
    )
    rollout_limited_speed = depth_rollout_limit(
        pre_rollout_speed,
        final_ray_w,
        case.position_w,
        case.velocity_w,
        points_w,
        case.R_wc.T,
        case.camera,
        case.cfg,
    )

    lines = [
        f"format_version={FORMAT_VERSION}",
        f"name={case.name}",
        f"camera.width={case.camera.width}",
        f"camera.height={case.camera.height}",
        f"camera.hfov_deg={scalar(case.camera.hfov_deg)}",
        f"camera.vfov_deg={scalar(case.camera.vfov_deg)}",
        f"camera.max_depth={scalar(case.camera.max_depth)}",
        f"camera.fx={scalar(case.camera.fx)}",
        f"camera.fy={scalar(case.camera.fy)}",
        f"camera.cx={scalar(case.camera.cx)}",
        f"camera.cy={scalar(case.camera.cy)}",
    ]
    for field in dataclasses.fields(case.cfg):
        lines.append(f"cfg.{field.name}={scalar(getattr(case.cfg, field.name))}")
    lines.extend(
        [
            f"input.position_w={vector(case.position_w)}",
            f"input.velocity_w={vector(case.velocity_w)}",
            f"input.goal_w={vector(case.goal_w)}",
            f"input.q_previous={vector(case.q_previous)}",
            f"input.q_goal_previous={vector(case.q_goal_previous)}",
            f"input.R_wc={vector(case.R_wc)}",
            f"input.reference_origin_w={vector(case.reference_origin_w)}",
            f"input.reference_direction_w={vector(case.reference_direction_w)}",
            f"input.depth={vector(case.depth)}",
            f"expected.obstacle_point_count={obstacle_points.shape[0]}",
            f"expected.obstacle_points={vector(obstacle_points)}",
            f"expected.command_w={vector(command_w)}",
            f"expected.q_goal_returned={vector(q_goal_returned)}",
            f"expected.free_distance={vector(solution.free_distance)}",
            "expected.planning_mask="
            + ",".join("1" if item else "0" for item in solution.planning_mask.reshape(-1)),
            f"expected.selected_q_source={vector(selected_source)}",
            f"expected.selected_q_goal={vector(selected_goal)}",
            f"expected.component_count={int(np.max(component_labels))}",
            f"expected.component_labels={vector(component_labels)}",
            f"expected.harmonic_potential={vector(harmonic_potential)}",
            f"expected.harmonic_valid={scalar(harmonic_valid)}",
            "expected.harmonic_path_count="
            + ("0" if harmonic_path is None else str(len(harmonic_path))),
            f"expected.harmonic_path={vector(harmonic_path)}",
            "expected.angular_distance_previous_source="
            + ("none" if previous_source_angle is None else scalar(previous_source_angle)),
            f"expected.rate_limited_goal={vector(rate_limited_goal)}",
            f"expected.direct_selected_q_cmd={vector(direct_selected_command)}",
            f"expected.pre_reference_q_cmd={vector(pre_reference_command)}",
            f"expected.observed_clearance={scalar(observed_clearance)}",
            "expected.reference_corrected_direction_w="
            + vector(reference_corrected_direction_w),
            f"expected.command_free_distance={scalar(command_free_distance)}",
            f"expected.reference_speed={scalar(reference_speed)}",
            f"expected.braking_speed={scalar(pre_rollout_speed)}",
            f"expected.full_speed_rollout_safe={scalar(full_speed_rollout_safe)}",
            f"expected.rollout_limited_speed={scalar(rollout_limited_speed)}",
            f"expected.potential={vector(solution.potential)}",
            f"expected.q_ref={vector(solution.q_ref)}",
            f"expected.q_source={vector(solution.q_source)}",
            f"expected.q_goal={vector(solution.q_goal)}",
            f"expected.q_cmd={vector(solution.q_cmd)}",
            f"expected.field_valid={scalar(solution.field_valid)}",
        ]
    )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        existing = list(output_dir.glob("*.fixture"))
        if existing and not args.force:
            raise SystemExit(
                f"refusing to overwrite {len(existing)} fixtures; pass --force"
            )
        if args.force:
            for fixture in existing:
                fixture.unlink()
    else:
        output_dir.mkdir(parents=True)

    cases = build_cases()
    for case in cases:
        write_fixture(case, output_dir / f"{case.name}.fixture")

    core_path = PYTHON_PACKAGE_ROOT / "pc_gvf" / "depth_angular_core.py"
    digest = hashlib.sha256(core_path.read_bytes()).hexdigest()
    manifest = [
        f"format_version={FORMAT_VERSION}",
        "generator=tools/generate_python_baseline.py",
        "algorithm=src/pc_gvf/pc_gvf/depth_angular_core.py",
        f"algorithm_sha256={digest}",
        f"case_count={len(cases)}",
        "cases=" + ",".join(case.name for case in cases),
    ]
    (output_dir / "MANIFEST.txt").write_text(
        "\n".join(manifest) + "\n", encoding="utf-8"
    )
    print(f"wrote {len(cases)} fixtures to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
