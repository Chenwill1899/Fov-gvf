#!/usr/bin/env python3
"""Small ROS 2 closed-loop depth-camera simulator for PC-GVF integration checks."""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

from pc_gvf.depth_angular_core import AABBObstacle, Camera, R_WC_FIXED, SphereObstacle, make_scenes


class DepthAngularDemo(Node):
    def __init__(self) -> None:
        super().__init__("depth_angular_demo_sim")
        scenarios = make_scenes()
        scenario = self.declare_parameter("scenario", "single_pillar").value
        if scenario not in scenarios:
            raise ValueError(f"unknown scenario {scenario!r}; choices={sorted(scenarios)}")
        self.scene = scenarios[scenario]
        self.camera = Camera(width=48, height=36, max_depth=10.0)
        self.position = self.scene.start.copy()
        self.velocity = np.zeros(3)
        self.command = np.zeros(3)
        self.accel_limit = 4.0
        self.last_depth = self.get_clock().now()
        self.collided = False
        self.odom_pub = self.create_publisher(Odometry, "/sim/odom", 10)
        self.depth_pub = self.create_publisher(Image, "/sim/depth/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/sim/depth/camera_info", 10)
        self.intent_pub = self.create_publisher(TwistStamped, "/human_intent", 10)
        self.scenario_pub = self.create_publisher(
            MarkerArray, "/pc_gvf/scenario",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PositionCommand, "/position_cmd", self.command_callback, 10)
        self.timer = self.create_timer(0.02, self.step)
        self.publish_scenario()

    @staticmethod
    def point(value):
        from geometry_msgs.msg import Point
        result = Point(); result.x, result.y, result.z = map(float, value)
        return result

    def publish_scenario(self):
        markers = []
        stamp = self.get_clock().now().to_msg()
        for marker_id, obstacle in enumerate(self.scene.obstacles):
            if obstacle.name == "floor":
                continue
            marker = Marker(); marker.header.stamp = stamp; marker.header.frame_id = "world"
            marker.ns, marker.id, marker.action = "scenario_obstacles", marker_id, Marker.ADD
            if isinstance(obstacle, SphereObstacle):
                marker.type = Marker.SPHERE; marker.pose.position = self.point(obstacle.center)
                marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * obstacle.radius
            elif isinstance(obstacle, AABBObstacle):
                marker.type = Marker.CUBE
                marker.pose.position = self.point(0.5 * (obstacle.minimum + obstacle.maximum))
                marker.scale.x, marker.scale.y, marker.scale.z = map(float, obstacle.maximum - obstacle.minimum)
            marker.pose.orientation.w = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.92, 0.18, 0.12, 0.55
            markers.append(marker)
        for marker_id, (name, value, color) in enumerate([
            ("start", self.scene.start, (0.1, 0.9, 0.2)),
            ("goal", self.scene.goal, (1.0, 0.82, 0.0)),
        ], start=100):
            marker = Marker(); marker.header.stamp = stamp; marker.header.frame_id = "world"
            marker.ns, marker.id, marker.type, marker.action = name, marker_id, Marker.SPHERE, Marker.ADD
            marker.pose.position = self.point(value); marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.22
            marker.color.r, marker.color.g, marker.color.b = color; marker.color.a = 1.0
            markers.append(marker)
        self.scenario_pub.publish(MarkerArray(markers=markers))

    def command_callback(self, msg: PositionCommand) -> None:
        self.command = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])

    def step(self) -> None:
        if self.collided:
            return
        dt = 0.02
        delta = self.command - self.velocity
        norm = float(np.linalg.norm(delta))
        if norm > self.accel_limit * dt:
            delta *= self.accel_limit * dt / norm
        next_velocity = self.velocity + delta
        next_position = self.position + dt * next_velocity
        if self.scene.segment_collision(self.position, next_position, 0.25):
            self.get_logger().error("collision in ROS 2 integration demo")
            self.collided = True
            self.command[:] = 0.0
            next_velocity[:] = 0.0
        self.position, self.velocity = next_position, next_velocity
        stamp = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp.to_msg(), "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, self.position)
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, self.velocity)
        self.odom_pub.publish(odom)
        intent = TwistStamped()
        intent.header.stamp, intent.header.frame_id = stamp.to_msg(), "world"
        intent.twist.linear.x = 1.0
        self.intent_pub.publish(intent)
        if (stamp - self.last_depth).nanoseconds >= 100_000_000:
            self.publish_depth(stamp)
            self.last_depth = stamp

    def publish_depth(self, stamp) -> None:
        depth = self.scene.render_depth(self.camera, self.position, R_WC_FIXED).astype("<f4")
        image = Image()
        image.header.stamp, image.header.frame_id = stamp.to_msg(), "camera_optical"
        image.height, image.width, image.encoding = self.camera.height, self.camera.width, "32FC1"
        image.is_bigendian, image.step, image.data = False, self.camera.width * 4, depth.tobytes()
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = image.height, image.width
        info.k = [self.camera.fx, 0.0, self.camera.cx, 0.0, self.camera.fy, self.camera.cy, 0.0, 0.0, 1.0]
        info.p = [self.camera.fx, 0.0, self.camera.cx, 0.0, 0.0, self.camera.fy,
                  self.camera.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        self.depth_pub.publish(image)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthAngularDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
