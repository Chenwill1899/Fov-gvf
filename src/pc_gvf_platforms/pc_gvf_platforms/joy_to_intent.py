#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


def map_axes(axes, forward_axis, lateral_axis, forward_sign, lateral_sign, deadzone, max_speed):
    if len(axes) <= max(forward_axis, lateral_axis):
        return None
    forward, lateral = forward_sign * float(axes[forward_axis]), lateral_sign * float(axes[lateral_axis])
    if not math.isfinite(forward) or not math.isfinite(lateral):
        return None
    forward, lateral = max(-1.0, min(1.0, forward)), max(-1.0, min(1.0, lateral))
    magnitude = math.hypot(forward, lateral)
    if magnitude <= deadzone:
        return 0.0, 0.0
    scale = max_speed * (min(1.0, magnitude) - deadzone) / ((1.0 - deadzone) * magnitude)
    return forward * scale, lateral * scale


class JoyToIntent(Node):
    def __init__(self):
        super().__init__("joy_to_intent")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = p("frame_id", "world")
        self.forward_axis, self.lateral_axis = p("forward_axis", 1), p("lateral_axis", 0)
        self.forward_sign, self.lateral_sign = p("forward_sign", 1.0), p("lateral_sign", 1.0)
        self.deadzone, self.max_speed = p("deadzone", 0.15), p("max_speed", 1.0)
        self.timeout = max(0.05, p("joy_timeout", 0.5))
        if self.forward_axis < 0 or self.lateral_axis < 0 or self.forward_axis == self.lateral_axis:
            raise ValueError("axis indices must be distinct and non-negative")
        if not 0.0 <= self.deadzone < 1.0 or self.max_speed < 0.0:
            raise ValueError("invalid deadzone or speed")
        self.axes, self.received = None, None
        self.pub = self.create_publisher(TwistStamped, p("intent_topic", "/human_intent"), 10)
        self.create_subscription(Joy, p("joy_topic", "/joy"), self.callback, 10)
        self.timer = self.create_timer(1.0 / max(1.0, p("intent_rate_hz", 20.0)), self.publish)

    def callback(self, msg):
        self.axes, self.received = list(msg.axes), self.get_clock().now()

    def publish(self):
        stamp = self.get_clock().now()
        stale = self.received is None or (stamp - self.received).nanoseconds * 1e-9 > self.timeout
        velocity = (0.0, 0.0) if stale else map_axes(self.axes, self.forward_axis, self.lateral_axis,
            self.forward_sign, self.lateral_sign, self.deadzone, self.max_speed)
        if velocity is None: velocity = (0.0, 0.0)
        msg = TwistStamped(); msg.header.stamp = stamp.to_msg(); msg.header.frame_id = self.frame
        msg.twist.linear.x, msg.twist.linear.y = velocity; self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = JoyToIntent()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
