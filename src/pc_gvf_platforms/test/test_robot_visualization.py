from collections import deque
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from pc_gvf_platforms.navigation_visualizer import NavigationVisualizer


def test_quadrotor_resource_pose_and_ground_box():
    messages = []
    node = SimpleNamespace(
        frame="world", shape="quadrotor", mesh_scale=1.0, path=deque(maxlen=10),
        command=None, point=NavigationVisualizer.point,
        path_pub=SimpleNamespace(publish=lambda _: None),
        marker_pub=SimpleNamespace(publish=messages.append))
    odom = Odometry()
    odom.header.frame_id = "world"
    odom.pose.pose.position.x = 3.0
    odom.pose.pose.orientation.z = 0.6
    odom.pose.pose.orientation.w = 0.8
    NavigationVisualizer.odom(node, odom)
    robot = messages[-1].markers[0]
    assert robot.type == robot.MESH_RESOURCE
    assert robot.pose == odom.pose.pose
    assert robot.scale.x == robot.scale.y == robot.scale.z == 1.0
    mesh = Path(get_package_share_directory("pc_gvf_platforms")) / "meshes/hummingbird.mesh"
    assert mesh.is_file()
    node.shape = "box"
    NavigationVisualizer.odom(node, odom)
    assert messages[-1].markers[0].type == robot.CUBE
