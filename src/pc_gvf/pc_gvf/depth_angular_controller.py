#!/usr/bin/env python3
"""ROS 2 adapter for the depth-only angular harmonic guidance core."""
from __future__ import annotations

import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from pc_gvf.depth_angular_core import (
    Camera,
    R_WC_FIXED,
    SimConfig,
    backproject_obstacle_points,
    braking_speed,
    compute_guidance,
    potential_gradient,
)


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid odometry quaternion")
    scale = 2.0 / norm
    return np.array([
        [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
        [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
        [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
    ])


class LiveDepthScene:
    def __init__(self) -> None:
        self.name = "live_depth"
        self.start = np.zeros(3)
        self.goal = np.array([8.0, 0.0, 0.0])
        self.obstacles = []
        self.depth = None

    def render_depth(self, _camera, _position, _rotation):
        return self.depth


class DepthAngularController(Node):
    def __init__(self) -> None:
        super().__init__("depth_angular_controller")
        self.lock = threading.Lock()
        self.odom = None
        self.camera_info = None
        self.intent = None
        self.intent_time = None
        self.depth_time = None
        self.camera = None
        self.scene = LiveDepthScene()
        self.q_previous = None
        self.q_goal_previous = None
        self.reference_origin = None
        self.reference_direction = None
        self.command = np.zeros(3)
        self.last_command_time = None
        self.last_status = None
        self.goal_reached = False
        self.depth_version = 0
        self.last_field_version = -1

        self.frame_id = self.parameter("frame_id", "world")
        self.odom_topic = self.parameter("odom_topic", "/sim/odom")
        self.depth_topic = self.parameter("depth_topic", "/sim/depth/image_raw")
        self.camera_info_topic = self.parameter("camera_info_topic", "/sim/depth/camera_info")
        self.intent_topic = self.parameter("human_intent_topic", "/human_intent")
        self.cmd_topic = self.parameter("cmd_topic", "/position_cmd")
        self.fixed_forward_intent = self.parameter("fixed_forward_intent", False)
        self.intent_timeout = max(0.05, self.parameter("human_intent_timeout", 0.5))
        self.max_depth_age = max(0.05, self.parameter("max_depth_age", 0.3))
        self.target_width = max(8, self.parameter("angular_width", 48))
        self.target_height = max(8, self.parameter("angular_height", 36))
        self.max_depth = max(0.2, self.parameter("max_depth", 10.0))
        self.forward_lookahead = max(1.0, self.parameter("forward_lookahead", 8.0))
        self.use_fixed_goal = self.parameter("use_fixed_goal", False)
        self.fixed_goal = np.array([
            self.parameter("goal_x", 7.2), self.parameter("goal_y", 0.0),
            self.parameter("goal_z", 1.2)])
        self.use_reference_line = self.parameter("use_reference_line", True)
        self.stop_at_goal = self.parameter("stop_at_goal", False)
        self.goal_tolerance = max(0.05, self.parameter("goal_tolerance", 0.3))
        self.command_accel_limit = max(0.1, self.parameter("command_accel_limit", 4.0))
        self.max_vertical_speed = max(0.0, self.parameter("max_vertical_speed", 0.0))
        self.max_reference_speed = max(0.05, self.parameter("max_speed", 2.0))
        self.command_yaw = self.parameter("command_yaw", 0.0)
        self.field_radius = max(0.2, self.parameter("field_radius", 1.5))
        self.reanchor_cos = math.cos(math.radians(self.parameter("direction_reanchor_deg", 15.0)))

        self.cfg = SimConfig(reference_speed=self.parameter("speed", 1.0))
        self.cfg.max_accel = self.command_accel_limit
        self.cfg.body_radius = self.parameter("body_radius", 0.25)
        self.cfg.safety_margin = self.parameter("safety_margin", 0.175)
        self.cfg.rollout_margin = self.parameter("rollout_margin", 0.10)
        self.cfg.planning_horizon = self.parameter("planning_horizon", 3.0)

        self.command_pub = self.create_publisher(PositionCommand, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.intent_pub = self.create_publisher(TwistStamped, self.intent_topic, 10)
        self.field_pub = self.create_publisher(
            MarkerArray, "/pc_gvf/angular_field",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, self.intent_topic, self.intent_callback, 10)
        self.timer = self.create_timer(0.02, self.control)

    def parameter(self, name, default):
        return self.declare_parameter(name, default).value

    def set_status(self, value: str) -> None:
        if value != self.last_status:
            self.status_pub.publish(String(data=value))
            self.get_logger().info(value)
            self.last_status = value

    def odom_callback(self, msg: Odometry) -> None:
        with self.lock:
            self.odom = msg

    def info_callback(self, msg: CameraInfo) -> None:
        with self.lock:
            self.camera_info = msg

    def intent_callback(self, msg: TwistStamped) -> None:
        if self.fixed_forward_intent:
            return
        with self.lock:
            self.intent = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            self.intent_time = self.get_clock().now()

    @staticmethod
    def decode_depth(msg: Image) -> np.ndarray:
        if msg.encoding == "32FC1":
            dtype, item_size, scale = (">f4" if msg.is_bigendian else "<f4"), 4, 1.0
        elif msg.encoding == "16UC1":
            dtype, item_size, scale = (">u2" if msg.is_bigendian else "<u2"), 2, 0.001
        else:
            raise ValueError("depth encoding must be 32FC1 or 16UC1")
        if msg.step < msg.width * item_size or msg.step % item_size:
            raise ValueError("invalid depth row step")
        row_items = msg.step // item_size
        raw = np.frombuffer(msg.data, dtype=dtype, count=msg.height * row_items)
        return raw.reshape(msg.height, row_items)[:, :msg.width].astype(float) * scale

    def depth_callback(self, msg: Image) -> None:
        with self.lock:
            info = self.camera_info
        if info is None or info.width != msg.width or info.height != msg.height:
            return
        try:
            source = self.decode_depth(msg)
            uu = np.rint(np.linspace(0, msg.width - 1, self.target_width)).astype(int)
            vv = np.rint(np.linspace(0, msg.height - 1, self.target_height)).astype(int)
            depth = source[np.ix_(vv, uu)].copy()
            depth[~np.isfinite(depth) | (depth < 0.0)] = 0.0
            depth[depth > self.max_depth] = self.max_depth
            camera = Camera(self.target_width, self.target_height, max_depth=self.max_depth)
            sx, sy = self.target_width / msg.width, self.target_height / msg.height
            camera.fx, camera.fy = float(info.k[0]) * sx, float(info.k[4]) * sy
            camera.cx, camera.cy = (float(info.k[2]) + 0.5) * sx - 0.5, (float(info.k[5]) + 0.5) * sy - 0.5
            uu_grid, vv_grid = np.meshgrid(np.arange(camera.width), np.arange(camera.height))
            camera.u_grid, camera.v_grid = uu_grid.astype(float), vv_grid.astype(float)
            rays = np.stack(((camera.u_grid - camera.cx) / camera.fx,
                             (camera.v_grid - camera.cy) / camera.fy,
                             np.ones_like(camera.u_grid)), axis=-1)
            camera.rays_c = rays / np.linalg.norm(rays, axis=-1, keepdims=True)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(str(exc))
            return
        with self.lock:
            self.scene.depth, self.camera = depth, camera
            stamp = Time.from_msg(msg.header.stamp)
            self.depth_time = stamp if stamp.nanoseconds else self.get_clock().now()
            if self.q_previous is None:
                self.q_previous = np.array([camera.cx, camera.cy])
            self.depth_version += 1

    @staticmethod
    def age(now_time: Time, older: Time | None) -> float:
        return math.inf if older is None else (now_time - older).nanoseconds * 1.0e-9

    def limited(self, desired: np.ndarray, stamp: Time) -> np.ndarray:
        if self.last_command_time is None:
            self.last_command_time = stamp
        dt = max(0.0, min(0.2, (stamp - self.last_command_time).nanoseconds * 1.0e-9))
        delta = desired - self.command
        limit, norm = self.command_accel_limit * dt, float(np.linalg.norm(delta))
        if norm > limit and norm > 1.0e-9:
            delta *= limit / norm
        self.command += delta
        self.last_command_time = stamp
        return self.command.copy()

    def publish_command(self, stamp: Time, position: np.ndarray, velocity: np.ndarray) -> None:
        msg = PositionCommand()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        msg.position.x, msg.position.y, msg.position.z = map(float, position)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = map(float, velocity)
        msg.yaw = float(self.command_yaw)
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.command_pub.publish(msg)

    def stop(self, stamp: Time, position: np.ndarray, status: str) -> None:
        self.command[:] = 0.0
        self.publish_command(stamp, position, self.command)
        self.set_status(status)

    @staticmethod
    def point(value) -> Point:
        result = Point(); result.x, result.y, result.z = map(float, value)
        return result

    @staticmethod
    def color(red, green, blue, alpha=1.0) -> ColorRGBA:
        result = ColorRGBA(); result.r, result.g, result.b, result.a = red, green, blue, alpha
        return result

    def marker(self, stamp, namespace, marker_id, marker_type):
        result = Marker(); result.header.stamp = stamp.to_msg(); result.header.frame_id = self.frame_id
        result.ns, result.id, result.type, result.action = namespace, marker_id, marker_type, Marker.ADD
        result.pose.orientation.w = 1.0
        return result

    def publish_field(self, stamp, position, rotation_wc, solution):
        camera, radius, step = self.camera, self.field_radius, 3
        if camera is None:
            return
        project = lambda q: position + rotation_wc @ (radius * camera.ray_from_pixel(np.asarray(q, dtype=float)))
        markers = []

        unsafe = self.marker(stamp, "fov_unsafe", 0, Marker.POINTS)
        unsafe.scale.x = unsafe.scale.y = 0.045
        unsafe.color = self.color(1.0, 0.12, 0.05, 0.55)
        for v in range(0, camera.height, step):
            for u in range(0, camera.width, step):
                if solution.planning_mask[v, u]:
                    unsafe.points.append(self.point(project((u, v))))
        markers.append(unsafe)

        if solution.field_valid and np.any(np.isfinite(solution.potential)):
            fx, fy = potential_gradient(solution.potential, solution.planning_mask)
        else:
            fx = np.zeros(solution.planning_mask.shape); fy = np.zeros(solution.planning_mask.shape)
        # Match the ESDF quiver semantics: visualize the command field, not
        # only the obstacle-induced potential gradient.  In direct-flight
        # cells the nominal field points toward the selected angular goal.
        quiver = self.marker(stamp, "angular_gvf", 0, Marker.LINE_LIST)
        quiver.scale.x = 0.012
        quiver.color = self.color(1.0, 1.0, 1.0, 1.0)
        goal_u, goal_v = solution.q_goal
        pulse_time = stamp.nanoseconds * 1.0e-9

        def add_segment(a, b, rgba_a, rgba_b):
            quiver.points.extend((self.point(a), self.point(b)))
            quiver.colors.extend((rgba_a, rgba_b))

        for v in range(1, camera.height - 1, step):
            for u in range(1, camera.width - 1, step):
                if solution.planning_mask[v, u]:
                    continue
                flow_u, flow_v = float(fx[v, u]), float(fy[v, u])
                magnitude = math.hypot(flow_u, flow_v)
                if magnitude <= 1.0e-8:
                    flow_u, flow_v = float(goal_u - u), float(goal_v - v)
                    magnitude = math.hypot(flow_u, flow_v)
                if magnitude <= 1.0e-8:
                    continue

                q_end = np.array(
                    [u + 2.0 * flow_u / magnitude, v + 2.0 * flow_v / magnitude]
                )
                q_end[0] = np.clip(q_end[0], 0.0, camera.width - 1.0)
                q_end[1] = np.clip(q_end[1], 0.0, camera.height - 1.0)
                p0 = project((u, v))
                tangent = project(q_end) - p0
                tangent_norm = float(np.linalg.norm(tangent))
                if tangent_norm <= 1.0e-8:
                    continue
                tangent /= tangent_norm

                safe_speed = braking_speed(
                    float(solution.free_distance[v, u]),
                    self.cfg.reference_speed,
                    self.cfg,
                )
                speed_ratio = np.clip(
                    safe_speed / max(self.cfg.reference_speed, 1.0e-6), 0.0, 1.0
                )
                shaft = 0.18 * radius * speed_ratio
                if shaft <= 1.0e-4:
                    continue
                p1 = p0 + shaft * tangent
                base_color = self.color(
                    float(1.0 - speed_ratio), float(speed_ratio), 0.2, 0.85
                )

                pulse = (float(u) / camera.width - 2.5 * pulse_time) % 1.0
                subdivisions = 5
                for sub in range(subdivisions):
                    s0 = sub / subdivisions
                    s1 = (sub + 1) / subdivisions
                    d0 = (s0 - pulse + 0.5) % 1.0 - 0.5
                    d1 = (s1 - pulse + 0.5) % 1.0 - 0.5
                    alpha0 = base_color.a * (0.12 + 0.88 * math.exp(-0.5 * (d0 / 0.16) ** 2))
                    alpha1 = base_color.a * (0.12 + 0.88 * math.exp(-0.5 * (d1 / 0.16) ** 2))
                    c0 = self.color(base_color.r, base_color.g, base_color.b, alpha0)
                    c1 = self.color(base_color.r, base_color.g, base_color.b, alpha1)
                    add_segment(p0 + s0 * (p1 - p0), p0 + s1 * (p1 - p0), c0, c1)

                normal = (p0 - position) / max(float(np.linalg.norm(p0 - position)), 1.0e-8)
                side = np.cross(normal, tangent)
                side_norm = float(np.linalg.norm(side))
                if side_norm > 1.0e-8:
                    side /= side_norm
                    head = min(0.35 * shaft, 0.055 * radius)
                    head_left = p1 - 0.866 * head * tangent + 0.5 * head * side
                    head_right = p1 - 0.866 * head * tangent - 0.5 * head * side
                    add_segment(p1, head_left, base_color, base_color)
                    add_segment(p1, head_right, base_color, base_color)

        markers.append(quiver)

        corners = [(0, 0), (camera.width - 1, 0),
                   (camera.width - 1, camera.height - 1), (0, camera.height - 1)]
        boundary = [project(q) for q in corners]
        frustum = self.marker(stamp, "camera_fov", 0, Marker.LINE_LIST)
        frustum.scale.x = 0.025; frustum.color = self.color(0.15, 0.65, 1.0, 0.85)
        for corner in boundary:
            frustum.points.extend([self.point(position), self.point(corner)])
        for index in range(4):
            frustum.points.extend([self.point(boundary[index]), self.point(boundary[(index + 1) % 4])])
        markers.append(frustum)

        for marker_id, (name, q, rgba) in enumerate([
            ("reference_ray", solution.q_ref, self.color(0.0, 1.0, 1.0)),
            ("goal_ray", solution.q_goal, self.color(0.1, 1.0, 0.15)),
            ("command_ray", solution.q_cmd, self.color(1.0, 1.0, 1.0)),
        ]):
            ray = self.marker(stamp, name, marker_id, Marker.LINE_LIST)
            ray.scale.x = 0.04; ray.color = rgba
            ray.points = [self.point(position), self.point(project(q))]
            markers.append(ray)

        hits = self.marker(stamp, "depth_hits", 0, Marker.POINTS)
        hits.scale.x = hits.scale.y = 0.055; hits.color = self.color(0.1, 0.9, 0.7, 0.9)
        for camera_point in backproject_obstacle_points(solution.depth, camera, self.cfg.depth_point_stride):
            hits.points.append(self.point(position + rotation_wc @ camera_point))
        markers.append(hits)
        self.field_pub.publish(MarkerArray(markers=markers))

    def control(self) -> None:
        stamp = self.get_clock().now()
        with self.lock:
            odom, camera = self.odom, self.camera
            intent = None if self.intent is None else self.intent.copy()
            intent_time, depth_time, depth_version = self.intent_time, self.depth_time, self.depth_version
        if odom is None:
            self.set_status("WAITING_ODOMETRY")
            return
        pose, twist = odom.pose.pose, odom.twist.twist
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return self.stop(stamp, np.nan_to_num(position), "INVALID_ODOMETRY")
        if self.use_fixed_goal and self.stop_at_goal:
            if self.goal_reached or float(np.linalg.norm(position - self.fixed_goal)) <= self.goal_tolerance:
                self.goal_reached = True
                return self.stop(stamp, position, "GOAL_REACHED")
        if self.fixed_forward_intent:
            reference = np.array([self.cfg.reference_speed, 0.0, 0.0])
        elif intent is None or self.age(stamp, intent_time) > self.intent_timeout:
            return self.stop(stamp, position, "STALE_INTENT")
        else:
            reference = intent
        speed = min(float(np.linalg.norm(reference)), self.max_reference_speed)
        if speed <= 1.0e-5:
            return self.stop(stamp, position, "ZERO_INTENT")
        if camera is None:
            return self.stop(stamp, position, "WAITING_DEPTH")
        if self.age(stamp, depth_time) > self.max_depth_age:
            return self.stop(stamp, position, "STALE_DEPTH")
        direction = reference / float(np.linalg.norm(reference))
        if self.reference_direction is None or np.dot(direction, self.reference_direction) < self.reanchor_cos:
            self.reference_origin, self.reference_direction = position.copy(), direction.copy()
            self.q_goal_previous = None
        self.cfg.reference_speed = speed
        self.scene.start = position.copy()
        self.scene.goal = self.fixed_goal.copy() if self.use_fixed_goal else position + self.forward_lookahead * direction
        q = pose.orientation
        try:
            rotation_wc = quaternion_matrix(q.x, q.y, q.z, q.w) @ R_WC_FIXED
            line = ({"reference_origin_w": self.reference_origin,
                     "reference_direction_w": self.reference_direction}
                    if self.use_reference_line else {})
            desired, self.q_goal_previous, solution = compute_guidance(
                self.scene, camera, self.cfg, position, velocity, self.q_previous,
                self.q_goal_previous, rotation_wc, **line)
        except (ValueError, FloatingPointError) as exc:
            self.get_logger().warning(str(exc))
            return self.stop(stamp, position, "INVALID_GUIDANCE")
        self.q_previous = solution.q_cmd.copy()
        if depth_version != self.last_field_version:
            self.publish_field(stamp, position, rotation_wc, solution)
            self.last_field_version = depth_version
        if not np.all(np.isfinite(desired)):
            return self.stop(stamp, position, "INVALID_GUIDANCE")
        desired[2] = np.clip(desired[2], -self.max_vertical_speed, self.max_vertical_speed)
        self.publish_command(stamp, position, self.limited(desired, stamp))
        self.set_status("NAVIGATING" if solution.field_valid else "DEGRADED")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthAngularController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
