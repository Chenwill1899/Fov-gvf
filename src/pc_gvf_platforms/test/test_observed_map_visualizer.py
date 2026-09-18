import math

from builtin_interfaces.msg import Time
import numpy as np

from pc_gvf_platforms.observed_map_visualizer import (
    ObservedVoxelMap,
    backproject_to_radar,
    point_cloud,
    quaternion_matrix,
    transform_radar_to_world,
)


def test_backprojection_uses_forward_left_up_radar_axes():
    depth = np.asarray([[2.0, 2.0], [2.0, np.inf]], dtype=np.float32)
    points = backproject_to_radar(
        depth, fx=2.0, fy=2.0, cx=0.0, cy=0.0,
        stride=1, minimum=0.3, maximum=15.0)
    assert np.allclose(points, [
        [2.0, 0.0, 0.0], [2.0, -1.0, 0.0], [2.0, 0.0, -1.0]])


def test_world_transform_composes_body_pose_and_camera_extrinsic():
    rotation = quaternion_matrix(
        0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    world, radar_origin = transform_radar_to_world(
        np.asarray([[2.0, 0.0, 0.0]], dtype=np.float32),
        body_position=(10.0, 20.0, 3.0), body_rotation=rotation,
        camera_offset=(0.22, 0.0, 0.02))
    assert np.allclose(radar_origin, [10.0, 20.22, 3.02], atol=1.0e-7)
    assert np.allclose(world, [[10.0, 22.22, 3.02]], atol=1.0e-6)


def test_hit_confirmation_is_per_frame_and_stationary_map_stays_bounded():
    observed = ObservedVoxelMap(resolution=0.15, minimum_hits=2)
    duplicates = np.repeat([[1.01, 2.01, 3.01]], 100, axis=0)
    assert observed.integrate(duplicates) == 0
    assert len(observed.voxel_hits) == 1
    assert observed.integrate(duplicates) == 1
    assert len(observed.occupied_voxels) == 1
    for _ in range(100):
        assert observed.integrate(duplicates) == 0
    assert len(observed.occupied_voxels) == 1
    assert len(observed.voxel_hits) == 0


def test_motion_keeps_old_world_voxel_and_adds_new_region():
    observed = ObservedVoxelMap(resolution=0.15, minimum_hits=2)
    identity = np.eye(3)
    camera_offset = np.zeros(3)
    first_world, _ = transform_radar_to_world(
        np.asarray([[5.0, 0.0, 0.0]], dtype=np.float32),
        (0.0, 0.0, 0.0), identity, camera_offset)
    same_obstacle_after_motion, _ = transform_radar_to_world(
        np.asarray([[4.0, 0.0, 0.0]], dtype=np.float32),
        (1.0, 0.0, 0.0), identity, camera_offset)
    observed.integrate(first_world)
    observed.integrate(same_obstacle_after_motion)
    assert len(observed.occupied_voxels) == 1

    new_region = np.asarray([[8.0, 1.0, 0.0]], dtype=np.float32)
    observed.integrate(new_region)
    observed.integrate(new_region)
    assert len(observed.occupied_voxels) == 2
    centers = observed.points(publication_resolution=0.15, maximum_points=100)
    assert np.any(np.linalg.norm(centers - [5.025, 0.075, 0.075], axis=1) < 1e-5)
    assert np.any(np.linalg.norm(centers - [8.025, 0.975, 0.075], axis=1) < 1e-5)


def test_publication_limit_does_not_delete_internal_map():
    observed = ObservedVoxelMap(resolution=0.15, minimum_hits=1)
    points = np.column_stack((np.arange(20), np.zeros(20), np.zeros(20)))
    observed.integrate(points)
    published = observed.points(publication_resolution=0.15, maximum_points=5)
    assert len(observed.occupied_voxels) == 20
    assert published.shape[0] <= 5


def test_publication_coarsening_is_global_and_does_not_change_internal_map():
    observed = ObservedVoxelMap(resolution=0.15, minimum_hits=1)
    points = np.asarray([
        [0.01, 0.01, 0.01], [0.16, 0.01, 0.01], [2.01, 0.01, 0.01]])
    observed.integrate(points)
    published = observed.points(
        publication_resolution=0.30, maximum_points=100)
    assert len(observed.occupied_voxels) == 3
    assert published.shape == (2, 3)
    assert np.allclose(published[:, 0], [0.15, 1.95])


def test_point_cloud_layout_and_world_header():
    message = point_cloud(
        Time(sec=3, nanosec=4), "world",
        np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32))
    assert message.header.frame_id == "world"
    assert message.width == 1 and message.point_step == 12
    assert len(message.data) == 12
