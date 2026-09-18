#!/usr/bin/env python3
"""Observe one synthetic ROS scene driven by the public C++ controller."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from pc_gvf.depth_angular_core import make_scenes


class ClosedLoopProbe(Node):
    def __init__(self, scenario: str, duration: float) -> None:
        super().__init__("ros_cpp_closed_loop_probe")
        self.scenario_name = scenario
        self.scene = make_scenes()[scenario]
        self.goal = np.array([7.2, 0.0, 1.2])
        self.duration = duration
        self.started = time.monotonic()
        self.finish_time = None
        self.stop_candidate = None
        self.status = None
        self.status_history: list[str] = []
        self.previous_position = None
        self.position = None
        self.velocity = None
        self.command = None
        self.command_times: list[float] = []
        self.odom_count = 0
        self.field_count = 0
        self.collided = False
        self.min_clearance = math.inf

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Odometry, "/sim/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            PositionCommand, "/position_cmd", self.command_callback, 100)
        self.create_subscription(
            String, "/depth_angular_controller/status", self.status_callback,
            latched)
        self.create_subscription(
            MarkerArray, "/pc_gvf/angular_field", self.field_callback, latched)

    def status_callback(self, message: String) -> None:
        self.status = message.data
        if not self.status_history or self.status_history[-1] != message.data:
            self.status_history.append(message.data)
        if message.data == "GOAL_REACHED" and self.finish_time is None:
            self.finish_time = time.monotonic() + 0.3

    def command_callback(self, message: PositionCommand) -> None:
        self.command_times.append(time.monotonic())
        self.command = np.array([
            message.velocity.x, message.velocity.y, message.velocity.z])

    def field_callback(self, _message: MarkerArray) -> None:
        self.field_count += 1

    def odom_callback(self, message: Odometry) -> None:
        self.odom_count += 1
        position = np.array([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ])
        self.velocity = np.array([
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        ])
        if self.previous_position is not None and self.scene.segment_collision(
                self.previous_position, position, 0.25):
            self.collided = True
        self.previous_position = position.copy()
        self.position = position
        self.min_clearance = min(
            self.min_clearance,
            float(self.scene.surface_clearance(position, 0.25)))

        elapsed = time.monotonic() - self.started
        stopped = (
            elapsed >= 3.0
            and self.command is not None
            and float(np.linalg.norm(self.command)) <= 0.05
            and float(np.linalg.norm(self.velocity)) <= 0.05
            and position[0] >= 0.4
        )
        if stopped:
            if self.stop_candidate is None:
                self.stop_candidate = time.monotonic()
            elif time.monotonic() - self.stop_candidate >= 1.5:
                self.finish_time = time.monotonic()
        else:
            self.stop_candidate = None

    def complete(self) -> bool:
        return self.finish_time is not None and time.monotonic() >= self.finish_time

    def publisher_names(self, topic: str) -> list[str]:
        return sorted(info.node_name for info in self.get_publishers_info_by_topic(topic))

    def result(self) -> tuple[bool, dict]:
        failures: list[str] = []
        if self.position is None or self.velocity is None:
            failures.append("no odometry received")
            final_position = [math.nan] * 3
            final_speed = math.inf
            goal_distance = math.inf
        else:
            final_position = self.position.tolist()
            final_speed = float(np.linalg.norm(self.velocity))
            goal_distance = float(np.linalg.norm(self.position - self.goal))
        if self.collided:
            failures.append("trajectory intersects an inflated obstacle")
        if not any(value in {"NAVIGATING", "DEGRADED"}
                   for value in self.status_history):
            failures.append(f"controller never navigated: {self.status_history}")
        if self.field_count == 0:
            failures.append("no C++ angular-field visualization received")

        command_rate = 0.0
        if len(self.command_times) >= 10:
            # Discard startup and shutdown edges before measuring steady output.
            times = np.asarray(self.command_times, dtype=float)
            lower = times[0] + min(0.5, 0.1 * (times[-1] - times[0]))
            upper = times[-1] - min(0.2, 0.05 * (times[-1] - times[0]))
            steady = times[(times >= lower) & (times <= upper)]
            if steady.size >= 2:
                command_rate = float((steady.size - 1) / (steady[-1] - steady[0]))
        if command_rate < 40.0:
            failures.append(f"command rate below 40 Hz: {command_rate:.2f}")

        command_publishers = self.publisher_names("/position_cmd")
        field_publishers = self.publisher_names("/pc_gvf/angular_field")
        if command_publishers != ["depth_angular_controller"]:
            failures.append(f"unexpected command publishers: {command_publishers}")
        if field_publishers != ["depth_angular_controller"]:
            failures.append(f"unexpected field publishers: {field_publishers}")

        if self.status == "GOAL_REACHED":
            outcome = "goal_reached"
            if goal_distance > 0.35:
                failures.append(
                    f"GOAL_REACHED too far from goal: {goal_distance:.3f} m")
        elif self.stop_candidate is not None and final_speed <= 0.05:
            outcome = "safe_stop"
        else:
            outcome = "timeout"
            failures.append(
                f"neither goal nor stable safe stop: status={self.status}, "
                f"speed={final_speed:.3f}")

        summary = {
            "scenario": self.scenario_name,
            "outcome": outcome,
            "elapsed": time.monotonic() - self.started,
            "status": self.status,
            "status_history": self.status_history,
            "final_position": final_position,
            "final_speed": final_speed,
            "goal_distance": goal_distance,
            "min_surface_clearance": self.min_clearance,
            "odometry_samples": self.odom_count,
            "command_samples": len(self.command_times),
            "command_rate_hz": command_rate,
            "field_samples": self.field_count,
            "publishers": {
                "command": command_publishers,
                "field": field_publishers,
            },
            "failures": failures,
        }
        return not failures, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True, choices=sorted(make_scenes()))
    parser.add_argument("--duration", type=float)
    arguments = parser.parse_args()
    duration = arguments.duration
    if duration is None:
        duration = make_scenes()[arguments.scenario].max_time + 4.0

    rclpy.init()
    node = ClosedLoopProbe(arguments.scenario, duration)
    try:
        while (rclpy.ok()
               and time.monotonic() - node.started < duration
               and not node.complete()):
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
