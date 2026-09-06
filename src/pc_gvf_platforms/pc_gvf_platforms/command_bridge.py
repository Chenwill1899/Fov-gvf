#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def convert(vx, vy, yaw, mode, max_vx, max_vy, max_w, yaw_kp, deadband, yaw_dot=0.0):
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
    return out


class CommandBridge(Node):
    def __init__(self):
        super().__init__("position_cmd_to_twist")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.mode = p("mode", "holonomic")
        if self.mode not in {"holonomic", "diff_drive"}:
            raise ValueError("mode must be holonomic or diff_drive")
        self.max_vx, self.max_vy, self.max_w = p("max_vx", 0.6), p("max_vy", 0.4), p("max_w", 1.2)
        self.yaw_kp, self.deadband = p("yaw_kp", 2.0), p("heading_deadband", 0.05)
        self.yaw = None
        self.pub = self.create_publisher(Twist, p("cmd_out_topic", "/cmd_vel"), 10)
        self.create_subscription(Odometry, p("odom_topic", "/sim/odom"), self.odom, 10)
        self.create_subscription(PositionCommand, p("cmd_in_topic", "/position_cmd"), self.command, 10)

    def odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def command(self, msg):
        if self.yaw is not None:
            self.pub.publish(convert(msg.velocity.x, msg.velocity.y, self.yaw, self.mode,
                                     self.max_vx, self.max_vy, self.max_w, self.yaw_kp,
                                     self.deadband, msg.yaw_dot))


def main(args=None):
    rclpy.init(args=args); node = CommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
