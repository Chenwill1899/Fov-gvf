#!/usr/bin/env python3
"""Read-only ROS 2 camera projection audit; never stores image pixels."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


VIEWS = {
    "front": "/sim/depth",
    "left": "/sim/depth_left",
    "back": "/sim/depth_back",
    "right": "/sim/depth_right",
}


def header_metadata(message):
    stamp = message.header.stamp
    return {"frame_id": message.header.frame_id,
            "stamp_sec": int(stamp.sec), "stamp_nanosec": int(stamp.nanosec),
            "stamp_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)}


def info_metadata(message):
    return {**header_metadata(message), "width": int(message.width),
            "height": int(message.height), "distortion_model": message.distortion_model,
            "k": list(message.k), "d": list(message.d),
            "r": list(message.r), "p": list(message.p)}


def image_metadata(message):
    # Deliberately do not inspect, copy, retain, or serialize message.data.
    return {**header_metadata(message), "width": int(message.width),
            "height": int(message.height), "encoding": message.encoding}


def validate_view(view, expected_resolution, expected_hfov, expected_vfov, tolerance):
    errors = []
    info, image = view.get("info"), view.get("image")
    if info is None or image is None:
        return ["missing " + ", ".join(k for k in ("info", "image") if view.get(k) is None)]
    width, height = info["width"], info["height"]
    if width <= 0 or height <= 0:
        errors.append("CameraInfo resolution must be positive")
    if [width, height] != [image["width"], image["height"]]:
        errors.append("CameraInfo/Image resolutions differ")
    if expected_resolution is not None and [width, height] != list(expected_resolution):
        errors.append(f"resolution {[width, height]} != expected {list(expected_resolution)}")
    if not info["frame_id"] or info["frame_id"] != image["frame_id"]:
        errors.append("CameraInfo/Image frame_id is empty or differs")
    if image["encoding"] not in ("32FC1", "16UC1"):
        errors.append("unexpected depth encoding: " + image["encoding"])
    view["info_image_stamp_difference_s"] = (info["stamp_ns"] - image["stamp_ns"]) / 1e9
    arrays = {name: info[name] for name in ("k", "d", "r", "p")}
    for name, length in (("k", 9), ("r", 9), ("p", 12)):
        if len(arrays[name]) != length:
            errors.append(f"{name} has wrong length")
    if any(not math.isfinite(value) for values in arrays.values() for value in values):
        errors.append("non-finite camera matrix/distortion coefficient")
    if errors and any("length" in error or "non-finite" in error for error in errors):
        return errors
    k, p, r, d = info["k"], info["p"], info["r"], info["d"]
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    close = lambda a, b: math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-7)
    if fx <= 0 or fy <= 0:
        errors.append("fx/fy must be positive")
    if not close(fx, fy):
        errors.append("fx != fy; this renderer/CameraInfo helper requires square pixels")
    if not (0 <= cx <= width and 0 <= cy <= height):
        errors.append("principal point lies outside image")
    if not all(close(k[i], value) for i, value in ((1, 0), (3, 0), (6, 0), (7, 0), (8, 1))):
        errors.append("K is not a canonical zero-skew pinhole matrix")
    expected_r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    expected_p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    if not all(close(a, b) for a, b in zip(r, expected_r)):
        errors.append("R is not identity for this independent monocular camera")
    if not all(close(a, b) for a, b in zip(p, expected_p)):
        errors.append("P is inconsistent with K or contains a stereo baseline")
    if info["distortion_model"] != "plumb_bob" or len(d) != 5 or any(abs(value) > 1e-12 for value in d):
        errors.append("expected undistorted pinhole: plumb_bob with five zero coefficients")
    if fx > 0 and fy > 0 and width > 0 and height > 0:
        hfov = math.degrees(math.atan(cx / fx) + math.atan((width - cx) / fx))
        vfov = math.degrees(math.atan(cy / fy) + math.atan((height - cy) / fy))
        view["projection"] = {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                              "hfov_deg": hfov, "vfov_deg": vfov,
                              "aspect_ratio": width / height,
                              "principal_point_centered": close(cx, width / 2) and close(cy, height / 2)}
        for label, actual, expected in (("HFOV", hfov, expected_hfov), ("VFOV", vfov, expected_vfov)):
            if expected is not None and abs(actual - expected) > tolerance:
                errors.append(f"{label} {actual:.9f} differs from expected {expected:.9f} by more than {tolerance} deg")
    return errors


def emit(result, output):
    payload = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if output:
        destination = Path(output)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(payload, encoding="utf-8")
    sys.stdout.write(payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10.0, help="wall timeout in seconds (default 10)")
    parser.add_argument("--expected-resolution", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"))
    parser.add_argument("--expected-hfov", type=float)
    parser.add_argument("--expected-vfov", type=float)
    parser.add_argument("--fov-tolerance", type=float, default=0.1, help="degrees (default 0.1)")
    parser.add_argument("--output", help="also write JSON to this path")
    parser.add_argument("--run-id", default="", help="benchmark run identifier for evidence binding")
    parser.add_argument("--result-path", help="associated benchmark result path")
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if not math.isfinite(args.fov_tolerance) or args.fov_tolerance < 0:
        parser.error("--fov-tolerance must be finite and nonnegative")
    for value in (args.expected_hfov, args.expected_vfov):
        if value is not None and (not math.isfinite(value) or not 0 < value < 180):
            parser.error("expected FOV must lie between 0 and 180 degrees")
    if args.expected_resolution and any(value <= 0 for value in args.expected_resolution):
        parser.error("expected resolution must be positive")
    result = {"version": "ego2p1_camera_info_v1", "run_id": args.run_id,
              "result_path": str(Path(args.result_path).resolve()) if args.result_path else None,
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
              "ok": False, "read_only": True, "image_data_stored": False,
              "timeout_s": args.timeout,
              "expected": {"resolution": args.expected_resolution,
                           "hfov_deg": args.expected_hfov, "vfov_deg": args.expected_vfov,
                           "fov_tolerance_deg": args.fov_tolerance},
              "views": {name: {"info_topic": root + "/camera_info", "image_topic": root + "/image_raw"}
                        for name, root in VIEWS.items()}, "errors": []}
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as exc:
        result["errors"].append(f"ROS 2 imports unavailable; source the system ROS environment: {exc}")
        emit(result, args.output)
        return 3
    node = None
    started = time.monotonic()
    try:
        rclpy.init(args=ros_args)
        node = rclpy.create_node("ego2p1_read_camera_projection")
        subscriptions = []
        def accept(name, kind, message):
            # Keep only the first complete metadata sample from each topic.
            target = result["views"][name]
            if kind not in target:
                target[kind] = info_metadata(message) if kind == "info" else image_metadata(message)
        for name, root in VIEWS.items():
            subscriptions.append(node.create_subscription(
                CameraInfo, root + "/camera_info", lambda msg, n=name: accept(n, "info", msg), qos_profile_sensor_data))
            subscriptions.append(node.create_subscription(
                Image, root + "/image_raw", lambda msg, n=name: accept(n, "image", msg), qos_profile_sensor_data))
        while rclpy.ok() and time.monotonic() - started < args.timeout:
            if all("info" in view and "image" in view for view in result["views"].values()):
                break
            remaining = args.timeout - (time.monotonic() - started)
            rclpy.spin_once(node, timeout_sec=max(0., min(.1, remaining)))
        for name, view in result["views"].items():
            view["errors"] = validate_view(view, args.expected_resolution, args.expected_hfov,
                                          args.expected_vfov, args.fov_tolerance)
            result["errors"].extend(f"{name}: {error}" for error in view["errors"])
        first = result["views"]["front"].get("info")
        if first:
            for name, view in result["views"].items():
                info = view.get("info")
                if info and (info["width"] != first["width"] or info["height"] != first["height"] or
                             len(info["k"]) != len(first["k"]) or
                             any(not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-7)
                                 for a, b in zip(info["k"], first["k"]))):
                    result["errors"].append(f"{name}: projection/resolution differs from front camera")
        result["ok"] = not result["errors"]
    except (Exception, KeyboardInterrupt) as exc:
        result["errors"].append(f"reader failed: {type(exc).__name__}: {exc}")
    finally:
        result["wall_elapsed_s"] = time.monotonic() - started
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # Non-finite malformed ROS values must still yield valid JSON evidence.
    def finite_json(value):
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        if isinstance(value, dict):
            return {key: finite_json(item) for key, item in value.items()}
        if isinstance(value, list):
            return [finite_json(item) for item in value]
        return value
    emit(finite_json(result), args.output)
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
