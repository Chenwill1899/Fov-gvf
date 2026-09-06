#!/usr/bin/env python3
"""Publish bounded standard RViz markers and path from PC-GVF topics."""
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from pc_gvf_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class NavigationVisualizer(Node):
    def __init__(self):
        super().__init__("pc_gvf_visualizer")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = p("frame_id", "world")
        self.shape = p("robot_shape", "quadrotor")
        self.mesh_scale = p("robot_scale", 1.0)
        self.max_poses = max(10, p("path_max_poses", 2000))
        self.path = deque(maxlen=self.max_poses)
        self.command = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/pc_gvf/path", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/pc_gvf/markers", qos)
        self.create_subscription(Odometry, p("odom_topic", "/sim/odom"), self.odom, 10)
        self.create_subscription(PositionCommand, p("command_topic", "/position_cmd"), self.command_cb, 10)

    def command_cb(self, msg):
        self.command = msg

    @staticmethod
    def point(x, y, z):
        value = Point(); value.x, value.y, value.z = float(x), float(y), float(z)
        return value

    def odom(self, msg):
        pose = PoseStamped(); pose.header = msg.header; pose.pose = msg.pose.pose
        self.path.append(pose)
        path = Path(); path.header = msg.header; path.header.frame_id = self.frame; path.poses = list(self.path)
        self.path_pub.publish(path)

        robot = Marker(); robot.header = msg.header; robot.header.frame_id = self.frame
        robot.ns, robot.id, robot.action = "robot", 0, Marker.ADD
        robot.type = Marker.CUBE if self.shape == "box" else Marker.SPHERE
        robot.pose = msg.pose.pose
        robot.scale.x = 0.9 if self.shape == "box" else 0.5
        robot.scale.y = 0.5
        robot.scale.z = 0.25 if self.shape == "box" else 0.5
        robot.color.r, robot.color.g, robot.color.b, robot.color.a = 0.1, 0.55, 0.95, 0.9

        if self.shape == "quadrotor":
            robot.type = Marker.MESH_RESOURCE
            robot.mesh_resource = "package://pc_gvf_platforms/meshes/hummingbird.mesh"
            robot.scale.x = robot.scale.y = robot.scale.z = float(self.mesh_scale)
            robot.color.a = 1.0

        arrow = Marker(); arrow.header = robot.header; arrow.ns, arrow.id = "command", 1
        arrow.type, arrow.action = Marker.ARROW, Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.12, 0.16
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.35, 0.05, 1.0
        start = msg.pose.pose.position
        vx = self.command.velocity.x if self.command else 0.0
        vy = self.command.velocity.y if self.command else 0.0
        vz = self.command.velocity.z if self.command else 0.0
        arrow.points = [self.point(start.x, start.y, start.z),
                        self.point(start.x + vx, start.y + vy, start.z + vz)]
        self.marker_pub.publish(MarkerArray(markers=[robot, arrow]))


def main(args=None):
    rclpy.init(args=args); node = NavigationVisualizer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
