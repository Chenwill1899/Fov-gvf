#!/usr/bin/env python3
"""Exercise the public C++ controller's safety-state transitions."""
from __future__ import annotations

import json
import math
import sys
import time
from array import array

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from pc_gvf.depth_angular_core import Camera


class StateProbe(Node):
    def __init__(self) -> None:
        super().__init__("ros_cpp_state_probe")
        self.started = time.monotonic()
        self.camera = Camera(48, 36, max_depth=10.0)
        self.status = None
        self.status_history: list[str] = []
        self.command_norms: dict[str, list[float]] = {}
        self.command_count = 0
        self.max_navigation_command = 0.0
        self.message_contract_failures: list[str] = []

        self.odom_pub = self.create_publisher(
            Odometry, "/state_test/odom", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, "/state_test/camera_info", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            Image, "/state_test/depth", qos_profile_sensor_data)
        self.intent_pub = self.create_publisher(
            TwistStamped, "/state_test/intent", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/depth_angular_controller/status", self.status_callback,
            latched)
        self.create_subscription(
            PositionCommand, "/state_test/command", self.command_callback, 100)
        self.create_timer(0.05, self.publish_phase)

    def status_callback(self, message: String) -> None:
        self.status = message.data
        if not self.status_history or self.status_history[-1] != message.data:
            self.status_history.append(message.data)

    def command_callback(self, message: PositionCommand) -> None:
        norm = math.sqrt(
            message.velocity.x ** 2
            + message.velocity.y ** 2
            + message.velocity.z ** 2)
        self.command_count += 1
        if self.status is not None:
            self.command_norms.setdefault(self.status, []).append(norm)
        if self.status == "NAVIGATING":
            self.max_navigation_command = max(self.max_navigation_command, norm)
        if message.header.frame_id != "world":
            self.message_contract_failures.append("command frame is not world")
        if message.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY:
            self.message_contract_failures.append("trajectory flag is not READY")
        untouched = [
            message.acceleration.x, message.acceleration.y, message.acceleration.z,
            message.jerk.x, message.jerk.y, message.jerk.z,
            message.yaw_dot, *message.kx, *message.kv, message.trajectory_id,
        ]
        if any(value != 0 for value in untouched):
            self.message_contract_failures.append(
                "fields left at zero by Python changed in C++")

    def odometry(self, position_x: float = 0.0, invalid_quaternion: bool = False):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.pose.pose.position.x = position_x
        message.pose.pose.position.z = 1.2
        message.pose.pose.orientation.w = 0.0 if invalid_quaternion else 1.0
        self.odom_pub.publish(message)

    def intent(self, speed: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = speed
        self.intent_pub.publish(message)

    def depth(self, use_uint16: bool = False) -> None:
        stamp = self.get_clock().now().to_msg()
        information = CameraInfo()
        information.header.stamp = stamp
        information.width = 48
        information.height = 36
        information.k = [
            self.camera.fx, 0.0, self.camera.cx,
            0.0, self.camera.fy, self.camera.cy,
            0.0, 0.0, 1.0,
        ]
        self.info_pub.publish(information)
        image = Image()
        image.header.stamp = stamp
        image.width = 48
        image.height = 36
        image.is_bigendian = 0
        if use_uint16:
            image.encoding = "16UC1"
            image.step = 48 * 2
            image.data = array("H", [10000] * (48 * 36)).tobytes()
        else:
            image.encoding = "32FC1"
            image.step = 48 * 4
            image.data = array("f", [10.0] * (48 * 36)).tobytes()
        self.depth_pub.publish(image)

    def publish_phase(self) -> None:
        elapsed = time.monotonic() - self.started
        if elapsed < 0.4:
            return
        if elapsed < 0.9:
            self.odometry()
            return
        if elapsed < 1.4:
            self.odometry()
            self.intent(0.0)
            return
        if elapsed < 1.9:
            self.odometry()
            self.intent(1.0)
            return
        if elapsed < 2.8:
            self.odometry()
            self.intent(1.0)
            self.depth()
            return
        if elapsed < 3.5:
            self.odometry()
            self.intent(1.0)
            return
        if elapsed < 4.2:
            self.odometry()
            self.depth(use_uint16=True)
            return
        if elapsed < 4.8:
            self.odometry(position_x=math.nan)
            self.intent(1.0)
            self.depth(use_uint16=True)
            return
        if elapsed < 5.4:
            self.odometry(invalid_quaternion=True)
            self.intent(1.0)
            self.depth(use_uint16=True)
            return
        self.odometry(position_x=7.2)
        self.intent(1.0)
        self.depth(use_uint16=True)

    def publisher_names(self, topic: str) -> list[str]:
        return sorted(info.node_name for info in self.get_publishers_info_by_topic(topic))

    def result(self) -> tuple[bool, dict]:
        expected = [
            "WAITING_ODOMETRY", "STALE_INTENT", "ZERO_INTENT",
            "WAITING_DEPTH", "NAVIGATING", "STALE_DEPTH",
            "INVALID_ODOMETRY", "INVALID_GUIDANCE", "GOAL_REACHED",
        ]
        missing = [value for value in expected if value not in self.status_history]
        failures = list(dict.fromkeys(self.message_contract_failures))
        if missing:
            failures.append(f"missing statuses: {missing}")
        for status in [
                "STALE_INTENT", "ZERO_INTENT", "WAITING_DEPTH", "STALE_DEPTH",
                "INVALID_ODOMETRY", "INVALID_GUIDANCE", "GOAL_REACHED"]:
            samples = self.command_norms.get(status, [])
            zero_samples = sum(value <= 1.0e-9 for value in samples)
            if zero_samples < 3:
                failures.append(
                    f"{status} did not sustain zero command: {samples[-3:]}")
        if self.max_navigation_command <= 0.1:
            failures.append(
                f"NAVIGATING never produced motion: {self.max_navigation_command}")
        publishers = self.publisher_names("/state_test/command")
        if publishers != ["depth_angular_controller"]:
            failures.append(f"unexpected command publishers: {publishers}")
        if self.status != "GOAL_REACHED":
            failures.append(f"final status is {self.status}, not GOAL_REACHED")
        summary = {
            "status_history": self.status_history,
            "zero_command_states": {
                key: len(self.command_norms.get(key, []))
                for key in expected if key != "WAITING_ODOMETRY"
            },
            "max_navigation_command": self.max_navigation_command,
            "command_samples": self.command_count,
            "command_publishers": publishers,
            "failures": failures,
        }
        return not failures, summary


def main() -> int:
    rclpy.init()
    node = StateProbe()
    try:
        while rclpy.ok() and time.monotonic() - node.started < 6.2:
            rclpy.spin_once(node, timeout_sec=0.05)
        passed, summary = node.result()
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
