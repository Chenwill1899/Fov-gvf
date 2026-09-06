#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class KinematicSim(Node):
    def __init__(self):
        super().__init__("pc_gvf_kinematic_sim")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.mode = p("mode", "holonomic")
        if self.mode not in {"holonomic", "diff_drive"}:
            raise ValueError("mode must be holonomic or diff_drive")
        self.x, self.y, self.z, self.yaw = p("init_x", 0.0), p("init_y", 0.0), p("init_z", 0.5), p("init_yaw", 0.0)
        self.frame, self.child = p("frame_id", "world"), p("child_frame_id", "base_link")
        self.timeout = max(0.05, p("cmd_timeout", 0.3))
        self.command, self.command_time = Twist(), None
        self.last = self.get_clock().now()
        self.pub = self.create_publisher(Odometry, p("odom_topic", "/sim/odom"), 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, p("cmd_topic", "/cmd_vel"), self.receive, 10)
        self.timer = self.create_timer(1.0 / max(1.0, p("rate", 50.0)), self.step)

    def receive(self, msg):
        self.command, self.command_time = msg, self.get_clock().now()

    def step(self):
        stamp = self.get_clock().now()
        dt = max(0.0, min(0.2, (stamp - self.last).nanoseconds * 1e-9)); self.last = stamp
        active = self.command_time is not None and (stamp - self.command_time).nanoseconds * 1e-9 <= self.timeout
        vx, vy, wz = ((self.command.linear.x, self.command.linear.y, self.command.angular.z)
                      if active else (0.0, 0.0, 0.0))
        if self.mode == "diff_drive": vy = 0.0
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt; self.y += (s * vx + c * vy) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom = Odometry(); odom.header.stamp = stamp.to_msg(); odom.header.frame_id = self.frame; odom.child_frame_id = self.child
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.x, self.y, self.z
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = vx, vy, wz
        self.pub.publish(odom)
        transform = TransformStamped(); transform.header = odom.header; transform.child_frame_id = self.child
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.x, self.y, self.z
        transform.transform.rotation = odom.pose.pose.orientation; self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args); node = KinematicSim()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
