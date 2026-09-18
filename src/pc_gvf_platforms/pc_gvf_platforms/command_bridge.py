#!/usr/bin/env python3
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TimingSeries:
    def __init__(self):
        self.values = []

    def add(self, value):
        if math.isfinite(value) and value >= 0.0:
            self.values.append(float(value))

    def mean(self):
        return statistics.fmean(self.values) if self.values else 0.0

    def percentile(self, fraction):
        if not self.values:
            return 0.0
        ordered = sorted(self.values)
        index = round(fraction * (len(ordered) - 1))
        return ordered[index]

    def maximum(self):
        return max(self.values, default=0.0)


def convert(vx, vy, yaw, mode, max_vx, max_vy, max_w, yaw_kp, deadband,
            yaw_dot=0.0, vz=0.0, max_vz=0.0):
    out = Twist()
    if mode == "diff_drive":
        speed = math.hypot(vx, vy)
        error = wrap(math.atan2(vy, vx) - yaw) if speed > deadband else 0.0
        out.linear.x = min(max_vx, max(0.0, speed * max(0.0, math.cos(error))))
        out.angular.z = min(max_w, max(-max_w, yaw_kp * error + yaw_dot))
    else:
        c, s = math.cos(yaw), math.sin(yaw)
        out.linear.x = min(max_vx, max(-max_vx, c * vx + s * vy))
        out.linear.y = min(max_vy, max(-max_vy, -s * vx + c * vy))
        out.angular.z = min(max_w, max(-max_w, yaw_dot))
    out.linear.z = min(max_vz, max(-max_vz, vz))
    return out


class CommandBridge(Node):
    def __init__(self):
        super().__init__("position_cmd_to_twist")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.mode = p("mode", "holonomic")
        if self.mode not in {"holonomic", "diff_drive"}:
            raise ValueError("mode must be holonomic or diff_drive")
        self.max_vx, self.max_vy, self.max_vz, self.max_w = (
            p("max_vx", 0.6), p("max_vy", 0.4), p("max_vz", 0.0), p("max_w", 1.2))
        self.yaw_kp, self.deadband = p("yaw_kp", 2.0), p("heading_deadband", 0.05)
        self.performance_report_interval = max(
            1.0, float(p("performance_report_interval", 5.0)))
        self.performance_log_path = str(p("performance_log_path", ""))
        self.performance_run_id = str(p("performance_run_id", "unspecified"))
        self.yaw = None
        self.performance_start = time.perf_counter()
        self.last_command_wall = None
        self.first_command_wall = None
        self.last_command_stamp_ns = None
        self.received_frames = 0
        self.published_frames = 0
        self.command_interval_ms = TimingSeries()
        self.command_ros_interval_ms = TimingSeries()
        self.controller_to_bridge_ms = TimingSeries()
        self.bridge_processing_ms = TimingSeries()
        self.performance_written = False
        self.pub = self.create_publisher(Twist, p("cmd_out_topic", "/cmd_vel"), 10)
        self.create_subscription(Odometry, p("odom_topic", "/sim/odom"), self.odom, 10)
        self.create_subscription(PositionCommand, p("cmd_in_topic", "/position_cmd"), self.command, 10)
        self.create_timer(self.performance_report_interval, self.report_performance)

    def odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def command(self, msg):
        callback_start = time.perf_counter()
        self.received_frames += 1
        if self.first_command_wall is None:
            self.first_command_wall = callback_start
        if self.last_command_wall is not None:
            self.command_interval_ms.add(
                (callback_start - self.last_command_wall) * 1000.0)
        self.last_command_wall = callback_start
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec)
        if stamp_ns > 0:
            if self.last_command_stamp_ns is not None:
                self.command_ros_interval_ms.add(max(
                    0.0, (stamp_ns - self.last_command_stamp_ns) / 1.0e6))
            self.last_command_stamp_ns = stamp_ns
            self.controller_to_bridge_ms.add(max(
                0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e6))
        if self.yaw is not None:
            self.pub.publish(convert(msg.velocity.x, msg.velocity.y, self.yaw, self.mode,
                                     self.max_vx, self.max_vy, self.max_w, self.yaw_kp,
                                     self.deadband, msg.yaw_dot,
                                     vz=msg.velocity.z, max_vz=self.max_vz))
            self.published_frames += 1
        self.bridge_processing_ms.add(
            (time.perf_counter() - callback_start) * 1000.0)

    def report_performance(self, final=False):
        elapsed = max(1.0e-9, time.perf_counter() - self.performance_start)
        active_elapsed = max(
            1.0e-9,
            (self.last_command_wall - self.first_command_wall)
            if self.first_command_wall is not None and self.last_command_wall is not None
            else elapsed,
        )
        wall_fps = (
            (self.published_frames - 1) / active_elapsed
            if self.published_frames > 1 else 0.0)
        ros_fps = (
            1000.0 / self.command_ros_interval_ms.mean()
            if self.command_ros_interval_ms.mean() > 0.0 else 0.0)
        suffix = " FINAL" if final else ""
        self.get_logger().info(
            f"[PERF BRIDGE{suffix}] run={self.performance_run_id} "
            f"elapsed={elapsed:.2f}s received={self.received_frames} "
            f"published={self.published_frames} fps(ros/wall)="
            f"{ros_fps:.2f}/{wall_fps:.2f} "
            "controller_to_bridge_ms(mean/p50/p95/max)="
            f"{self.controller_to_bridge_ms.mean():.3f}/"
            f"{self.controller_to_bridge_ms.percentile(0.50):.3f}/"
            f"{self.controller_to_bridge_ms.percentile(0.95):.3f}/"
            f"{self.controller_to_bridge_ms.maximum():.3f} "
            "bridge_process_ms(mean/p95/max)="
            f"{self.bridge_processing_ms.mean():.3f}/"
            f"{self.bridge_processing_ms.percentile(0.95):.3f}/"
            f"{self.bridge_processing_ms.maximum():.3f}"
        )
        if not final or self.performance_written or not self.performance_log_path:
            return
        self.performance_written = True
        record = (
            f"\n#### Command bridge — {self.performance_run_id}\n\n"
            f"- Wall duration: {elapsed:.3f} s\n"
            f"- Received / published control frames: {self.received_frames} / "
            f"{self.published_frames}\n"
            f"- Published control FPS (ROS simulation time / active wall time): "
            f"{ros_fps:.3f} / {wall_fps:.3f} Hz\n"
            "- Command stamp interval mean / P50 / P95 / max: "
            f"{self.command_ros_interval_ms.mean():.3f} / "
            f"{self.command_ros_interval_ms.percentile(0.50):.3f} / "
            f"{self.command_ros_interval_ms.percentile(0.95):.3f} / "
            f"{self.command_ros_interval_ms.maximum():.3f} ms (ROS simulation time)\n"
            "- Controller publish stamp to bridge receive mean / P50 / P95 / max: "
            f"{self.controller_to_bridge_ms.mean():.3f} / "
            f"{self.controller_to_bridge_ms.percentile(0.50):.3f} / "
            f"{self.controller_to_bridge_ms.percentile(0.95):.3f} / "
            f"{self.controller_to_bridge_ms.maximum():.3f} ms (ROS simulation time)\n"
            "- Command receive interval mean / P50 / P95 / max: "
            f"{self.command_interval_ms.mean():.3f} / "
            f"{self.command_interval_ms.percentile(0.50):.3f} / "
            f"{self.command_interval_ms.percentile(0.95):.3f} / "
            f"{self.command_interval_ms.maximum():.3f} ms (steady clock)\n"
            "- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: "
            f"{self.bridge_processing_ms.mean():.3f} / "
            f"{self.bridge_processing_ms.percentile(0.50):.3f} / "
            f"{self.bridge_processing_ms.percentile(0.95):.3f} / "
            f"{self.bridge_processing_ms.maximum():.3f} ms (steady clock)\n"
        )
        try:
            Path(self.performance_log_path).open("a", encoding="utf-8").write(record)
        except OSError as exc:
            self.get_logger().warning(
                f"cannot append performance log {self.performance_log_path}: {exc}")


def main(args=None):
    rclpy.init(args=args); node = CommandBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.report_performance(final=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
