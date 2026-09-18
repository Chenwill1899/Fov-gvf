from collections import deque
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from pc_gvf_platforms.navigation_visualizer import NavigationVisualizer


def test_quadrotor_resource_pose_and_ground_box():
    messages = []
    paths = []
    node = SimpleNamespace(
        frame="world", shape="quadrotor", mesh_scale=1.0, path=deque(maxlen=10),
        path_min_distance=0.10, path_publish_rate=10.0,
        marker_publish_rate=60.0, last_path_publish_stamp=None,
        last_marker_publish_stamp=None,
        raw_command=None, filtered_velocity=None, point=NavigationVisualizer.point,
        path_pub=SimpleNamespace(publish=paths.append),
        marker_pub=SimpleNamespace(publish=messages.append))
    odom = Odometry()
    odom.header.frame_id = "world"
    odom.pose.pose.position.x = 3.0
    odom.pose.pose.orientation.z = 0.6
    odom.pose.pose.orientation.w = 0.8
    NavigationVisualizer.odom(node, odom)
    assert len(paths) == 1 and len(paths[-1].poses) == 1
    robot = messages[-1].markers[0]
    assert robot.type == robot.MESH_RESOURCE
    assert robot.pose == odom.pose.pose
    assert robot.scale.x == robot.scale.y == robot.scale.z == 1.0
    assert {marker.ns for marker in messages[-1].markers[1:]} == {
        "command_raw", "command_smoothed"}
    mesh = Path(get_package_share_directory("pc_gvf_platforms")) / "meshes/hummingbird.mesh"
    assert mesh.is_file()
    node.shape = "box"
    odom.header.stamp.nanosec = 100_000_000
    NavigationVisualizer.odom(node, odom)
    assert messages[-1].markers[0].type == robot.CUBE
    assert len(paths) == 2 and len(paths[-1].poses) == 1
