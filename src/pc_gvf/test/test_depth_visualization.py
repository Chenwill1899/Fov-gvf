"""ROS 2 marker regression: geometry is pose-equivariant and arrows are visible."""
from types import SimpleNamespace

import numpy as np
from rclpy.time import Time

from pc_gvf.depth_angular_controller import DepthAngularController
from pc_gvf.depth_angular_core import Camera, R_WC_FIXED, SimConfig


def test_fov_arrows_follow_camera_pose():
    camera = Camera(width=12, height=12)
    shape = (camera.height, camera.width)
    solution = SimpleNamespace(
        planning_mask=np.zeros(shape, dtype=bool), field_valid=False,
        potential=np.zeros(shape), free_distance=np.full(shape, 8.0),
        q_goal=np.array([9.0, 6.0]), q_ref=np.array([6.0, 6.0]),
        q_cmd=np.array([8.0, 6.0]), depth=np.full(shape, camera.max_depth))
    messages = []
    node = SimpleNamespace(
        camera=camera, cfg=SimConfig(), field_radius=1.5, frame_id="world",
        point=DepthAngularController.point, color=DepthAngularController.color,
        field_pub=SimpleNamespace(publish=messages.append))
    node.marker = lambda *args: DepthAngularController.marker(node, *args)
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    offset = np.array([3., -2., 1.])
    DepthAngularController.publish_field(node, Time(seconds=1), np.zeros(3), R_WC_FIXED, solution)
    DepthAngularController.publish_field(node, Time(seconds=1), offset, rotation @ R_WC_FIXED, solution)
    for first, moved in zip(messages[0].markers, messages[1].markers):
        assert first.header.frame_id == "world"
        if first.points:
            points = np.array([[p.x, p.y, p.z] for p in first.points])
            transformed = np.array([[p.x, p.y, p.z] for p in moved.points])
            np.testing.assert_allclose(transformed, points @ rotation.T + offset, atol=1e-12)
    arrows = next(m for m in messages[0].markers if m.ns == "angular_gvf")
    assert len(arrows.points) > 0 and len(arrows.points) % 14 == 0
    assert len(arrows.colors) == len(arrows.points)
    assert arrows.color.a == 1.0
    assert len({round(c.a, 4) for c in arrows.colors}) > 2
    solution.planning_mask[:] = True
    DepthAngularController.publish_field(node, Time(seconds=2), offset, R_WC_FIXED, solution)
    assert not next(m for m in messages[-1].markers if m.ns == "angular_gvf").points
