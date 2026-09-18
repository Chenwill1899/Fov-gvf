#!/usr/bin/env python3
"""Publish live radar points and a persistent world-frame observed map."""

from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def quaternion_matrix(x, y, z, w):
    """Return a normalized 3x3 world-from-body rotation matrix."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def backproject_to_radar(depth, fx, fy, cx, cy, stride, minimum, maximum):
    """Backproject optical depth into radar axes: x forward, y left, z up."""
    rows = np.arange(0, depth.shape[0], stride, dtype=np.int32)
    columns = np.arange(0, depth.shape[1], stride, dtype=np.int32)
    uu, vv = np.meshgrid(columns, rows)
    distance = depth[vv, uu]
    valid = (
        np.isfinite(distance) & (distance >= minimum) & (distance < maximum)
    )
    distance = distance[valid].astype(np.float32, copy=False)
    if distance.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    optical_x = (uu[valid].astype(np.float32) - cx) * distance / fx
    optical_y = (vv[valid].astype(np.float32) - cy) * distance / fy
    return np.column_stack((distance, -optical_x, -optical_y)).astype(
        np.float32, copy=False)


def transform_radar_to_world(points, body_position, body_rotation, camera_offset):
    """Apply T_world_body * T_body_radar to radar-frame row vectors."""
    radar_origin = (
        np.asarray(body_position, dtype=np.float64)
        + body_rotation @ np.asarray(camera_offset, dtype=np.float64))
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32), radar_origin
    world = points.astype(np.float64) @ body_rotation.T + radar_origin
    return world.astype(np.float32), radar_origin


def point_cloud(stamp, frame_id, points):
    """Create an XYZ float32 PointCloud2 without per-point Python objects."""
    xyz = np.ascontiguousarray(points, dtype="<f4").reshape((-1, 3))
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.height = 1
    message.width = xyz.shape[0]
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = bool(np.isfinite(xyz).all())
    message.data = xyz.tobytes()
    return message


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ObservedVoxelMap:
    """Persistent hit-confirmed voxel map with bounded publication sampling."""

    def __init__(self, resolution, minimum_hits):
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if minimum_hits < 1:
            raise ValueError("minimum hits must be at least one")
        self.resolution = float(resolution)
        self.minimum_hits = int(minimum_hits)
        self.voxel_hits = {}
        self.occupied_voxels = set()
        self.changed = True

    def integrate(self, world_points):
        """Count each voxel at most once per frame and retain confirmations."""
        if world_points.size == 0:
            return 0
        indices = np.floor(
            np.asarray(world_points, dtype=np.float64) / self.resolution
        ).astype(np.int64)
        frame_voxels = np.unique(indices, axis=0)
        newly_occupied = 0
        for index in frame_voxels:
            key = (int(index[0]), int(index[1]), int(index[2]))
            if key in self.occupied_voxels:
                continue
            hits = self.voxel_hits.get(key, 0) + 1
            if hits >= self.minimum_hits:
                self.voxel_hits.pop(key, None)
                self.occupied_voxels.add(key)
                newly_occupied += 1
            else:
                self.voxel_hits[key] = hits
        if newly_occupied:
            self.changed = True
        return newly_occupied

    def points(self, publication_resolution, maximum_points):
        """Return globally coarsened centers without changing the internal map."""
        if not self.occupied_voxels:
            return np.empty((0, 3), dtype=np.float32)
        indices = np.asarray(tuple(self.occupied_voxels), dtype=np.int64)
        centers = (indices.astype(np.float64) + 0.5) * self.resolution
        output_resolution = max(self.resolution, float(publication_resolution))
        while True:
            output_indices = np.unique(
                np.floor(centers / output_resolution).astype(np.int64), axis=0)
            if maximum_points <= 0 or output_indices.shape[0] <= maximum_points:
                break
            scale = max(
                2, int(math.ceil(
                    (output_indices.shape[0] / maximum_points) ** (1.0 / 3.0))))
            output_resolution *= scale
        return ((output_indices.astype(np.float64) + 0.5) * output_resolution).astype(
            np.float32)


class ObservedMapVisualizer(Node):
    """Read-only mapper; none of its outputs feed the navigation controller."""

    def __init__(self):
        super().__init__("observed_map_visualizer")

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.world_frame = str(p("world_frame", "world"))
        self.radar_frame = str(p("radar_frame", "radar"))
        self.depth_topic = str(p("depth_topic", "/sim/depth/image_raw"))
        self.camera_info_topic = str(
            p("camera_info_topic", "/sim/depth/camera_info"))
        self.odom_topic = str(p("odom_topic", "/sim/odom"))
        self.radar_topic = str(p("radar_topic", "/pc_gvf/radar_points"))
        self.map_topic = str(p("map_topic", "/pc_gvf/observed_map"))
        self.depth_stride = max(1, int(p("depth_stride", 4)))
        self.minimum_depth = max(0.0, float(p("minimum_depth", 0.30)))
        self.maximum_depth = max(
            self.minimum_depth + 0.01, float(p("maximum_depth", 15.0)))
        self.map_resolution = float(p("map_resolution", 0.10))
        self.minimum_hits = max(1, int(p("minimum_hits", 2)))
        self.map_publish_rate = max(0.1, float(p("map_publish_rate", 2.0)))
        self.publication_resolution = max(
            self.map_resolution, float(p("publication_resolution", 0.30)))
        self.maximum_publish_points = max(
            1000, int(p("maximum_publish_points", 120000)))
        self.maximum_pose_age = max(0.01, float(p("maximum_pose_age", 0.20)))
        self.camera_offset = np.asarray(
            p("camera_offset", [0.22, 0.0, 0.02]), dtype=np.float64)
        self.camera_info = None
        self.pose_history = deque(maxlen=120)
        self.observed_map = ObservedVoxelMap(
            self.map_resolution, self.minimum_hits)
        self.cached_map_points = np.empty((0, 3), dtype=np.float32)
        self.integrated_frames = 0
        self.last_log_time_ns = None

        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.radar_publisher = self.create_publisher(
            PointCloud2, self.radar_topic, sensor_qos)
        self.map_publisher = self.create_publisher(
            PointCloud2, self.map_topic, map_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            sensor_qos)
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, sensor_qos)
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, sensor_qos)
        self.map_timer = self.create_timer(
            1.0 / self.map_publish_rate, self.publish_observed_map)
        self.get_logger().info(
            f"online observed map resolution={self.map_resolution:.3f} m, "
            f"minimum_hits={self.minimum_hits}, rate={self.map_publish_rate:.1f} Hz, "
            f"publication_resolution={self.publication_resolution:.3f} m")

    def camera_info_callback(self, message):
        self.camera_info = message

    def odom_callback(self, message):
        pose = message.pose.pose
        rotation = quaternion_matrix(
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w)
        if rotation is None:
            return
        position = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64)
        stamp_ns = stamp_nanoseconds(message.header.stamp)
        self.pose_history.append((stamp_ns, position, rotation))

        radar_origin = position + rotation @ self.camera_offset
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.radar_frame
        transform.transform.translation.x = float(radar_origin[0])
        transform.transform.translation.y = float(radar_origin[1])
        transform.transform.translation.z = float(radar_origin[2])
        transform.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def decode_depth(message):
        if message.encoding == "32FC1":
            dtype, scale = np.dtype(">f4" if message.is_bigendian else "<f4"), 1.0
        elif message.encoding == "16UC1":
            dtype, scale = np.dtype(">u2" if message.is_bigendian else "<u2"), 0.001
        else:
            raise ValueError("depth encoding must be 32FC1 or 16UC1")
        item_size = dtype.itemsize
        if message.step < message.width * item_size:
            raise ValueError("depth image row step is too small")
        depth = np.ndarray(
            (message.height, message.width), dtype=dtype, buffer=message.data,
            strides=(message.step, item_size))
        return depth.astype(np.float32) * scale

    def closest_pose(self, stamp):
        if not self.pose_history:
            return None
        target_ns = stamp_nanoseconds(stamp)
        closest = min(self.pose_history, key=lambda pose: abs(pose[0] - target_ns))
        if abs(closest[0] - target_ns) > self.maximum_pose_age * 1.0e9:
            return None
        return closest[1], closest[2]

    def depth_callback(self, message):
        info = self.camera_info
        if info is None or info.width != message.width or info.height != message.height:
            return
        try:
            depth = self.decode_depth(message)
            radar_points = backproject_to_radar(
                depth, float(info.k[0]), float(info.k[4]), float(info.k[2]),
                float(info.k[5]), self.depth_stride, self.minimum_depth,
                self.maximum_depth)
            self.radar_publisher.publish(
                point_cloud(message.header.stamp, self.radar_frame, radar_points))

            pose = self.closest_pose(message.header.stamp)
            if pose is None:
                return
            world_points, _ = transform_radar_to_world(
                radar_points, pose[0], pose[1], self.camera_offset)
            self.observed_map.integrate(world_points)
            self.integrated_frames += 1
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def publish_observed_map(self):
        if self.observed_map.changed:
            self.cached_map_points = self.observed_map.points(
                self.publication_resolution, self.maximum_publish_points)
            self.observed_map.changed = False
        message = point_cloud(
            self.get_clock().now().to_msg(), self.world_frame,
            self.cached_map_points)
        self.map_publisher.publish(message)

        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_log_time_ns is None
            or now_ns - self.last_log_time_ns >= 10_000_000_000
        ):
            self.get_logger().info(
                f"observed map: occupied={len(self.observed_map.occupied_voxels)}, "
                f"candidates={len(self.observed_map.voxel_hits)}, "
                f"published={self.cached_map_points.shape[0]}, "
                f"integrated_frames={self.integrated_frames}")
            self.last_log_time_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = ObservedMapVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
