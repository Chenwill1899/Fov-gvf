#!/usr/bin/env python3
"""Publish bounded standard RViz markers and path from PC-GVF topics."""
from collections import deque
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from pc_gvf_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
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
        self.path_min_distance = max(0.0, float(p("path_min_distance", 0.10)))
        self.path_publish_rate = max(0.5, float(p("path_publish_rate", 10.0)))
        self.marker_publish_rate = max(1.0, float(p("marker_publish_rate", 60.0)))
        self.path = deque(maxlen=self.max_poses)
        self.filter_tau = max(0.01, float(p("display_filter_tau", 0.20)))
        self.raw_command = None
        self.filtered_velocity = None
        self.last_command_stamp = None
        self.last_path_publish_stamp = None
        self.last_marker_publish_stamp = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/pc_gvf/path", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/pc_gvf/markers", qos)
        self.create_subscription(Odometry, p("odom_topic", "/sim/odom"), self.odom, 10)
        self.create_subscription(PositionCommand, p("command_topic", "/position_cmd"), self.command_cb, 10)

    def command_cb(self, msg):
        self.raw_command = msg
        velocity = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        if self.filtered_velocity is None or self.last_command_stamp is None:
            self.filtered_velocity = velocity
        else:
            dt = stamp - self.last_command_stamp
            if dt <= 0.0 or dt > 0.5:
                dt = 0.02
            alpha = 1.0 - math.exp(-dt / self.filter_tau)
            self.filtered_velocity = tuple(
                previous + alpha * (current - previous)
                for previous, current in zip(self.filtered_velocity, velocity)
            )
        self.last_command_stamp = stamp

    @staticmethod
    def point(x, y, z):
        value = Point(); value.x, value.y, value.z = float(x), float(y), float(z)
        return value

    def odom(self, msg):
        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        append_pose = not self.path
        if self.path:
            previous = self.path[-1].pose.position
            current = msg.pose.pose.position
            append_pose = math.sqrt(
                (current.x - previous.x) ** 2
                + (current.y - previous.y) ** 2
                + (current.z - previous.z) ** 2) >= self.path_min_distance
        if append_pose:
            pose = PoseStamped(); pose.header = msg.header; pose.pose = msg.pose.pose
            self.path.append(pose)
        if (
            self.last_path_publish_stamp is None
            or stamp < self.last_path_publish_stamp
            or stamp - self.last_path_publish_stamp >= 1.0 / self.path_publish_rate
        ):
            path = Path(); path.header = msg.header; path.header.frame_id = self.frame
            path.poses = list(self.path)
            self.path_pub.publish(path)
            self.last_path_publish_stamp = stamp

        if (
            self.last_marker_publish_stamp is not None
            and stamp >= self.last_marker_publish_stamp
            and stamp - self.last_marker_publish_stamp < 1.0 / self.marker_publish_rate
        ):
            return
        self.last_marker_publish_stamp = stamp

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

        start = msg.pose.pose.position
        raw_velocity = (
            (self.raw_command.velocity.x, self.raw_command.velocity.y,
             self.raw_command.velocity.z) if self.raw_command else (0.0, 0.0, 0.0)
        )
        filtered_velocity = self.filtered_velocity or (0.0, 0.0, 0.0)

        filtered = Marker(); filtered.header = robot.header
        filtered.ns, filtered.id = "command_smoothed", 2
        filtered.type, filtered.action = Marker.ARROW, Marker.ADD
        filtered.scale.x, filtered.scale.y, filtered.scale.z = 0.10, 0.17, 0.21
        filtered.color.r, filtered.color.g, filtered.color.b, filtered.color.a = 0.65, 0.15, 1.0, 0.85
        filtered.points = [self.point(start.x, start.y, start.z), self.point(
            start.x + filtered_velocity[0], start.y + filtered_velocity[1],
            start.z + filtered_velocity[2])]

        raw = Marker(); raw.header = robot.header
        raw.ns, raw.id = "command_raw", 1
        raw.type, raw.action = Marker.ARROW, Marker.ADD
        raw.scale.x, raw.scale.y, raw.scale.z = 0.045, 0.09, 0.13
        raw.color.r, raw.color.g, raw.color.b, raw.color.a = 1.0, 0.35, 0.05, 1.0
        raw.points = [self.point(start.x, start.y, start.z), self.point(
            start.x + raw_velocity[0], start.y + raw_velocity[1],
            start.z + raw_velocity[2])]
        self.marker_pub.publish(MarkerArray(markers=[robot, filtered, raw]))


def main(args=None):
    rclpy.init(args=args); node = NavigationVisualizer()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
