#!/usr/bin/env python3
"""Runtime probe for the persistent observed-map visualization contract."""

import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class ObservedMapProbe(Node):
    def __init__(self, resolution):
        super().__init__("observed_map_probe")
        self.resolution = resolution
        self.map_widths = []
        self.map_stamps = []
        self.map_frames = set()
        self.first_voxels = None
        self.last_voxels = set()
        self.last_points = np.empty((0, 3), dtype=np.float32)
        self.radar_widths = []
        self.radar_frames = set()
        self.odom_positions = []
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, "/pc_gvf/observed_map", self.map_callback, map_qos)
        self.create_subscription(
            PointCloud2, "/pc_gvf/radar_points", self.radar_callback,
            sensor_qos)
        self.create_subscription(
            Odometry, "/sim/odom", self.odom_callback, sensor_qos)

    def map_callback(self, message):
        self.map_widths.append(int(message.width))
        self.map_frames.add(message.header.frame_id)
        stamp = message.header.stamp
        self.map_stamps.append(stamp.sec + 1.0e-9 * stamp.nanosec)
        if not message.width:
            return
        floats_per_point = message.point_step // 4
        points = np.frombuffer(message.data, dtype="<f4").reshape(
            (-1, floats_per_point))[:, :3]
        self.last_points = points.copy()
        indices = np.floor(points / self.resolution).astype(np.int64)
        voxels = {tuple(int(value) for value in row) for row in indices}
        if self.first_voxels is None:
            self.first_voxels = voxels
        self.last_voxels = voxels

    def radar_callback(self, message):
        self.radar_widths.append(int(message.width))
        self.radar_frames.add(message.header.frame_id)

    def odom_callback(self, message):
        point = message.pose.pose.position
        self.odom_positions.append((point.x, point.y, point.z))

    def truth_alignment(self, occupancy_path, shape, origin, resolution):
        if not occupancy_path or self.last_points.size == 0:
            return {}
        occupancy = np.fromfile(occupancy_path, dtype=np.uint8)
        if occupancy.size != math.prod(shape):
            raise ValueError("truth occupancy shape does not match its file")
        occupancy = occupancy.reshape(shape).astype(bool)
        # Ground is a separate USD mesh and is intentionally absent from
        # occupancy.bin, so exclude its near-zero-height returns here.
        points = self.last_points[self.last_points[:, 2] >= 0.5 * resolution]
        indices = np.rint(
            (points - np.asarray(origin, dtype=np.float32)) / resolution
        ).astype(np.int64)
        valid = np.all(indices >= 0, axis=1) & np.all(
            indices < np.asarray(shape), axis=1)
        indices = indices[valid]
        if not indices.size:
            return {"truth_eligible_points": 0, "truth_neighbor_match_ratio": 0.0}
        matched = np.zeros(indices.shape[0], dtype=bool)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    shifted = indices + np.asarray((dx, dy, dz))
                    inside = np.all(shifted >= 0, axis=1) & np.all(
                        shifted < np.asarray(shape), axis=1)
                    selected = shifted[inside]
                    matched[inside] |= occupancy[
                        selected[:, 0], selected[:, 1], selected[:, 2]]
        return {
            "truth_eligible_points": int(indices.shape[0]),
            "truth_neighbor_match_ratio": float(np.mean(matched)),
        }

    def result(self, occupancy_path="", truth_shape=(400, 300, 50),
               truth_origin=(-80.0, -60.0, 0.0), truth_resolution=0.4):
        positive_deltas = [
            second - first
            for first, second in zip(self.map_stamps, self.map_stamps[1:])
            if second > first
        ]
        first_voxels = self.first_voxels or set()
        missing = first_voxels - self.last_voxels
        nondecreasing = all(
            second >= first
            for first, second in zip(self.map_widths, self.map_widths[1:]))
        positions = [
            np.asarray(position, dtype=np.float64)
            for position in self.odom_positions
            if np.linalg.norm(position) > 0.1
        ]
        motion = (
            float(np.linalg.norm(positions[-1] - positions[0]))
            if len(positions) >= 2 else 0.0)
        result = {
            "map_samples": len(self.map_widths),
            "map_first_nonempty_points": len(first_voxels),
            "map_last_points": len(self.last_voxels),
            "map_width_nondecreasing": nondecreasing,
            "first_map_voxels_missing_at_end": len(missing),
            "map_frames": sorted(self.map_frames),
            "map_rate_ros_hz": (
                1.0 / (sum(positive_deltas) / len(positive_deltas))
                if positive_deltas else 0.0),
            "radar_samples": len(self.radar_widths),
            "radar_mean_points": (
                sum(self.radar_widths) / len(self.radar_widths)
                if self.radar_widths else 0.0),
            "radar_frames": sorted(self.radar_frames),
            "odom_motion_m": motion,
        }
        result.update(self.truth_alignment(
            occupancy_path, truth_shape, truth_origin, truth_resolution))
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--resolution", type=float, default=0.15)
    parser.add_argument("--truth-occupancy", default="")
    arguments, ros_arguments = parser.parse_known_args()
    if not math.isfinite(arguments.duration) or arguments.duration <= 0.0:
        raise SystemExit("duration must be positive")
    rclpy.init(args=ros_arguments)
    node = ObservedMapProbe(arguments.resolution)
    deadline = time.monotonic() + arguments.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.result(arguments.truth_occupancy), sort_keys=True),
              flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
