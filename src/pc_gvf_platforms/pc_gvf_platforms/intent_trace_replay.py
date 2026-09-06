#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
import yaml


def load_trace(path):
    data = yaml.safe_load(Path(path).read_text())
    events = data.get("events") if isinstance(data, dict) else None
    if not isinstance(events, list) or not events:
        raise ValueError("trace requires events")
    parsed, previous = [], -1.0
    for event in events:
        values = float(event["at"]), float(event["forward"]), float(event["lateral"])
        if not all(math.isfinite(v) for v in values) or values[0] <= previous or values[0] < 0.0:
            raise ValueError("trace times must be finite, non-negative, and increasing")
        if abs(values[1]) > 1.0 or abs(values[2]) > 1.0:
            raise ValueError("trace axes must be in [-1, 1]")
        parsed.append(values); previous = values[0]
    if abs(parsed[-1][1]) > 1e-9 or abs(parsed[-1][2]) > 1e-9:
        raise ValueError("final event must stop")
    return parsed, data


class IntentTraceReplay(Node):
    def __init__(self):
        super().__init__("intent_trace_replay")
        path = self.declare_parameter("trace_file", "").value
        if not path: raise ValueError("trace_file is required")
        self.events, config = load_trace(path)
        self.scale = self.declare_parameter("max_speed", 1.0).value
        self.frame = self.declare_parameter("frame_id", "world").value
        self.index, self.start = 0, self.get_clock().now()
        self.pub = self.create_publisher(TwistStamped,
            self.declare_parameter("intent_topic", "/human_intent").value, 10)
        rate = self.declare_parameter("publish_rate_hz", float(config.get("publish_rate_hz", 20.0))).value
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.publish)

    def publish(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        while self.index + 1 < len(self.events) and elapsed >= self.events[self.index + 1][0]: self.index += 1
        _, forward, lateral = self.events[self.index]
        msg = TwistStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = self.frame
        msg.twist.linear.x, msg.twist.linear.y = self.scale * forward, self.scale * lateral
        self.pub.publish(msg)
        if self.index == len(self.events) - 1 and elapsed >= self.events[-1][0] + 1.0:
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args); node = IntentTraceReplay()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
