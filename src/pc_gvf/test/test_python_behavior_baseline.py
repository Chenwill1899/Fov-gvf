"""Lock the Python implementation while the equivalent C++ core is developed."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np

from pc_gvf.depth_angular_core import (
    Camera,
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
    reference_converged_direction,
    rollout_is_safe,
    select_command_direction,
    solve_angular_harmonic,
)


FIXTURE_DIR = Path(__file__).parent / "fixtures" / "python_behavior_v1"


class FrozenDepthScene:
    def __init__(self, depth: np.ndarray, goal: np.ndarray) -> None:
        self.depth = depth.copy()
        self.goal = goal.copy()

    def render_depth(self, _camera, _position, _rotation):
        return self.depth.copy()


def read_values(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


def array(value: str, shape=None):
    if value == "none":
        return None
    if value == "":
        return np.empty(0, dtype=float)
    result = np.fromstring(value, dtype=float, sep=",")
    return result if shape is None else result.reshape(shape)


def fixture_camera(values: dict[str, str]) -> Camera:
    camera = Camera(
        width=int(values["camera.width"]),
        height=int(values["camera.height"]),
        hfov_deg=float(values["camera.hfov_deg"]),
        vfov_deg=float(values["camera.vfov_deg"]),
        max_depth=float(values["camera.max_depth"]),
    )
    camera.fx = float(values["camera.fx"])
    camera.fy = float(values["camera.fy"])
    camera.cx = float(values["camera.cx"])
    camera.cy = float(values["camera.cy"])
    rays = np.stack(
        [
            (camera.u_grid - camera.cx) / camera.fx,
            (camera.v_grid - camera.cy) / camera.fy,
            np.ones_like(camera.u_grid),
        ],
        axis=-1,
    )
    camera.rays_c = rays / np.linalg.norm(rays, axis=-1, keepdims=True)
    return camera


def fixture_config(values: dict[str, str]) -> SimConfig:
    cfg = SimConfig()
    for field in dataclasses.fields(cfg):
        current = getattr(cfg, field.name)
        raw = values[f"cfg.{field.name}"]
        setattr(cfg, field.name, int(raw) if isinstance(current, int) else float(raw))
    return cfg


def assert_optional_array(actual, expected, context: str) -> None:
    if expected is None:
        assert actual is None, context
        return
    assert actual is not None, context
    np.testing.assert_allclose(
        actual, expected, rtol=1e-12, atol=1e-12, err_msg=context
    )


def test_python_behavior_matches_frozen_fixtures():
    fixtures = sorted(FIXTURE_DIR.glob("*.fixture"))
    manifest = read_values(FIXTURE_DIR / "MANIFEST.txt")
    assert int(manifest["format_version"]) == 1
    assert len(fixtures) == int(manifest["case_count"]) >= 10

    for fixture in fixtures:
        values = read_values(fixture)
        camera = fixture_camera(values)
        shape = (camera.height, camera.width)
        cfg = fixture_config(values)
        depth = array(values["input.depth"], shape)
        scene = FrozenDepthScene(depth, array(values["input.goal_w"]))
        obstacle_points = backproject_obstacle_points(
            depth, camera, cfg.depth_point_stride
        )

        assert obstacle_points.shape == (
            int(values["expected.obstacle_point_count"]), 3
        )
        np.testing.assert_allclose(
            obstacle_points.reshape(-1),
            array(values["expected.obstacle_points"]),
            rtol=1e-12,
            atol=1e-12,
            err_msg=f"fixture {fixture.name} obstacle points",
        )

        command, q_goal_returned, solution = compute_guidance(
            scene,
            camera,
            cfg,
            array(values["input.position_w"]),
            array(values["input.velocity_w"]),
            array(values["input.q_previous"]),
            array(values["input.q_goal_previous"]),
            array(values["input.R_wc"], (3, 3)),
            reference_origin_w=array(values["input.reference_origin_w"]),
            reference_direction_w=array(values["input.reference_direction_w"]),
        )
        selected_source, selected_goal, component_labels = choose_safe_goal(
            solution.q_ref,
            array(values["input.q_previous"]),
            solution.planning_mask,
            array(values["input.q_goal_previous"]),
            cfg,
        )

        context = f"fixture {fixture.name}"
        assert_optional_array(
            selected_source,
            array(values["expected.selected_q_source"]),
            f"{context} selected source",
        )
        assert_optional_array(
            selected_goal,
            array(values["expected.selected_q_goal"]),
            f"{context} selected goal",
        )
        np.testing.assert_array_equal(
            component_labels,
            array(values["expected.component_labels"], shape).astype(int),
            err_msg=f"{context} component labels",
        )
        assert int(np.max(component_labels)) == int(values["expected.component_count"])
        expected_harmonic = array(values["expected.harmonic_potential"])
        if selected_source is None or selected_goal is None:
            assert expected_harmonic is None, context
            assert not bool(int(values["expected.harmonic_valid"])), context
            assert values["expected.harmonic_path"] == "none", context
            assert values["expected.angular_distance_previous_source"] == "none", context
            assert values["expected.rate_limited_goal"] == "none", context
            assert values["expected.direct_selected_q_cmd"] == "none", context
            pre_reference_command = array(values["input.q_previous"])
        else:
            harmonic_potential, harmonic_valid = solve_angular_harmonic(
                solution.planning_mask, selected_source, selected_goal, cfg
            )
            np.testing.assert_allclose(
                harmonic_potential,
                expected_harmonic.reshape(shape),
                rtol=1e-10,
                atol=1e-10,
                equal_nan=True,
                err_msg=f"{context} direct harmonic solve",
            )
            assert harmonic_valid == bool(
                int(values["expected.harmonic_valid"])
            ), context
            harmonic_path = discrete_harmonic_path(
                harmonic_potential,
                solution.planning_mask,
                selected_source,
                selected_goal,
            )
            assert len(harmonic_path) == int(
                values["expected.harmonic_path_count"]
            ), context
            np.testing.assert_allclose(
                np.asarray(harmonic_path).reshape(-1),
                array(values["expected.harmonic_path"]),
                rtol=1e-12,
                atol=1e-12,
                err_msg=f"{context} harmonic path",
            )
            previous_source_angle = angular_distance(
                camera, array(values["input.q_previous"]), selected_source
            )
            np.testing.assert_allclose(
                previous_source_angle,
                float(values["expected.angular_distance_previous_source"]),
                rtol=1e-12,
                atol=1e-12,
                err_msg=f"{context} previous/source angle",
            )
            rate_limited_goal = angular_rate_limit(
                camera,
                array(values["input.q_previous"]),
                selected_goal,
                cfg.max_direction_rate * cfg.control_dt,
            )
            np.testing.assert_allclose(
                rate_limited_goal,
                array(values["expected.rate_limited_goal"]),
                rtol=1e-12,
                atol=1e-12,
                err_msg=f"{context} rate-limited goal",
            )
            direct_selected_command = select_command_direction(
                camera,
                array(values["input.q_previous"]),
                selected_source,
                selected_goal,
                harmonic_potential,
                solution.planning_mask,
                cfg,
            )
            np.testing.assert_allclose(
                direct_selected_command,
                array(values["expected.direct_selected_q_cmd"]),
                rtol=1e-12,
                atol=1e-12,
                err_msg=f"{context} selected command",
            )
            if angular_distance(camera, selected_source, selected_goal) < np.deg2rad(0.4):
                pre_reference_command = rate_limited_goal
            elif harmonic_valid:
                pre_reference_command = direct_selected_command
            else:
                pre_reference_command = rate_limited_goal

        np.testing.assert_allclose(
            pre_reference_command,
            array(values["expected.pre_reference_q_cmd"]),
            rtol=1e-12,
            atol=1e-12,
            err_msg=f"{context} pre-reference command",
        )
        rotation_world_from_camera = array(values["input.R_wc"], (3, 3))
        raw_field_direction = rotation_world_from_camera @ camera.ray_from_pixel(
            pre_reference_command
        )
        observed_clearance = (
            float(np.min(np.linalg.norm(obstacle_points, axis=1)))
            if obstacle_points.shape[0] > 0
            else camera.max_depth
        )
        np.testing.assert_allclose(
            observed_clearance,
            float(values["expected.observed_clearance"]),
            rtol=1e-12,
            atol=1e-12,
            err_msg=f"{context} observed clearance",
        )
        reference_origin = array(values["input.reference_origin_w"])
        reference_direction = array(values["input.reference_direction_w"])
        expected_reference = array(values["expected.reference_corrected_direction_w"])
        if reference_origin is None or reference_direction is None:
            assert expected_reference is None, context
        else:
            corrected = reference_converged_direction(
                raw_field_direction,
                array(values["input.position_w"]),
                reference_origin,
                reference_direction,
                observed_clearance,
                cfg,
            )
            np.testing.assert_allclose(
                corrected,
                expected_reference,
                rtol=1e-12,
                atol=1e-12,
                err_msg=f"{context} reference correction",
            )

        command_free_distance = bilinear_sample(
            solution.free_distance, solution.q_cmd, default=0.0
        )
        np.testing.assert_allclose(
            command_free_distance,
            float(values["expected.command_free_distance"]),
            rtol=1e-12,
            atol=1e-12,
            err_msg=f"{context} command free distance",
        )
        goal_delta = array(values["input.goal_w"]) - array(values["input.position_w"])
        reference_speed = min(
            cfg.reference_speed,
            np.sqrt(max(0.0, 2.0 * cfg.brake_accel * np.linalg.norm(goal_delta))),
        )
        pre_rollout_speed = braking_speed(command_free_distance, reference_speed, cfg)
        np.testing.assert_allclose(
            [reference_speed, pre_rollout_speed],
            [float(values["expected.reference_speed"]),
             float(values["expected.braking_speed"])],
            rtol=1e-12,
            atol=1e-12,
            err_msg=f"{context} braking speed",
        )
        final_ray_world = rotation_world_from_camera @ camera.ray_from_pixel(
            solution.q_cmd
        )
        position = array(values["input.position_w"])
        velocity = array(values["input.velocity_w"])
        points_world = position[None, :] + obstacle_points @ rotation_world_from_camera.T
        full_speed_safe = rollout_is_safe(
            position,
            velocity,
            pre_rollout_speed * final_ray_world,
            points_world,
            position,
            rotation_world_from_camera.T,
            camera,
            cfg,
        )
        assert full_speed_safe == bool(
            int(values["expected.full_speed_rollout_safe"])
        ), context
        rollout_speed = depth_rollout_limit(
            pre_rollout_speed,
            final_ray_world,
            position,
            velocity,
            points_world,
            rotation_world_from_camera.T,
            camera,
            cfg,
        )
        np.testing.assert_allclose(
            rollout_speed,
            float(values["expected.rollout_limited_speed"]),
            rtol=1e-12,
            atol=1e-12,
            err_msg=f"{context} rollout-limited speed",
        )
        np.testing.assert_allclose(
            command, array(values["expected.command_w"]), rtol=1e-12, atol=1e-12,
            err_msg=context,
        )
        np.testing.assert_allclose(
            q_goal_returned,
            array(values["expected.q_goal_returned"]),
            rtol=1e-12,
            atol=1e-12,
            err_msg=context,
        )
        np.testing.assert_allclose(
            solution.free_distance,
            array(values["expected.free_distance"], shape),
            rtol=1e-12,
            atol=1e-12,
            err_msg=context,
        )
        np.testing.assert_array_equal(
            solution.planning_mask,
            array(values["expected.planning_mask"], shape).astype(bool),
            err_msg=context,
        )
        np.testing.assert_allclose(
            solution.potential,
            array(values["expected.potential"], shape),
            rtol=1e-10,
            atol=1e-10,
            equal_nan=True,
            err_msg=context,
        )
        for name in ("q_ref", "q_source", "q_goal", "q_cmd"):
            np.testing.assert_allclose(
                getattr(solution, name),
                array(values[f"expected.{name}"]),
                rtol=1e-12,
                atol=1e-12,
                err_msg=f"{context} {name}",
            )
        assert solution.field_valid == bool(int(values["expected.field_valid"])), context
