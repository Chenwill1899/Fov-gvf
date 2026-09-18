#!/usr/bin/env python3
"""Add a kinematic quadrotor visual and four horizontal depth cameras."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("input", type=Path)
parser.add_argument("output", type=Path)
parser.add_argument("--start", type=str, help="optional start position x,y,z")
parser.add_argument("--goal", type=str, help="optional goal position x,y,z")
parser.add_argument(
    "--manual-spawn", type=str,
    help="manual-control spawn x,y,z; removes navigation start/goal markers")
parser.add_argument(
    "--environment-scale", type=float, default=1.0,
    help="uniformly scale source environment meshes before adding the UAV")
args = parser.parse_args()
source, target = args.input.resolve(), args.output.resolve()
stage = Usd.Stage.Open(str(source))
if stage is None:
    raise RuntimeError(f"could not open {source}")
UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
if not math.isfinite(args.environment_scale) or args.environment_scale <= 0.0:
    raise SystemExit("--environment-scale must be a finite positive number")
source_meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
if args.environment_scale != 1.0:
    scale = Gf.Vec3f(args.environment_scale)
    for prim in source_meshes:
        UsdGeom.XformCommonAPI(prim).SetScale(scale)

def position_argument(raw, name):
    if raw is None:
        return None
    try:
        values = tuple(float(value.strip()) for value in raw.split(","))
    except ValueError as exc:
        raise SystemExit(f"{name} must contain three comma-separated numbers") from exc
    if len(values) != 3:
        raise SystemExit(f"{name} must contain three comma-separated numbers")
    return Gf.Vec3d(*values)


def navigation_marker(path, requested, color):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        if requested is None:
            raise RuntimeError(f"required marker missing: {path}")
        marker = UsdGeom.Xform.Define(stage, path)
        UsdGeom.XformCommonAPI(marker).SetTranslate(requested)
        sphere = UsdGeom.Sphere.Define(stage, f"{path}/Marker")
        sphere.CreateRadiusAttr(0.22)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        prim = marker.GetPrim()
    elif requested is not None:
        UsdGeom.XformCommonAPI(prim).SetTranslate(requested)
    return prim


manual_spawn = position_argument(args.manual_spawn, "--manual-spawn")
if manual_spawn is not None:
    if args.start is not None or args.goal is not None:
        raise SystemExit("--manual-spawn cannot be combined with --start or --goal")
    for marker_path in ("/World/UAV_Start", "/World/UAV_Goal"):
        if stage.GetPrimAtPath(marker_path).IsValid():
            stage.RemovePrim(marker_path)
    spawn_position = manual_spawn
    goal_position = None
else:
    start = navigation_marker(
        "/World/UAV_Start", position_argument(args.start, "--start"),
        (0.05, 0.8, 0.12))
    goal = navigation_marker(
        "/World/UAV_Goal", position_argument(args.goal, "--goal"),
        (0.92, 0.05, 0.04))
    spawn_position = UsdGeom.Xformable(start).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    goal_position = UsdGeom.Xformable(goal).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()

robot = UsdGeom.Xform.Define(stage, "/World/Robot")
xform = UsdGeom.XformCommonAPI(robot)
xform.SetTranslate(spawn_position)
xform.SetRotate(Gf.Vec3f(0.0, 0.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
robot.GetPrim().CreateAttribute("fov:vehicleType", Sdf.ValueTypeNames.String).Set("kinematic_quadrotor")
robot.GetPrim().CreateAttribute("fov:bodyRadiusM", Sdf.ValueTypeNames.Double).Set(0.25)

def cube(path, scale, translate, color):
    value = UsdGeom.Cube.Define(stage, path)
    value.CreateSizeAttr(1.0)
    api = UsdGeom.XformCommonAPI(value)
    api.SetTranslate(Gf.Vec3d(*translate)); api.SetScale(Gf.Vec3f(*scale))
    value.CreateDisplayColorAttr([Gf.Vec3f(*color)])

def cylinder(path, radius, height, translate, color):
    value = UsdGeom.Cylinder.Define(stage, path)
    value.CreateRadiusAttr(radius); value.CreateHeightAttr(height); value.CreateAxisAttr("Z")
    UsdGeom.XformCommonAPI(value).SetTranslate(Gf.Vec3d(*translate))
    value.CreateDisplayColorAttr([Gf.Vec3f(*color)])

blue = (0.04, 0.25, 0.85)
dark = (0.015, 0.02, 0.035)
cube("/World/Robot/Body", (0.30, 0.18, 0.10), (0.0, 0.0, 0.0), blue)
cube("/World/Robot/ArmForward", (0.50, 0.045, 0.035), (0.0, 0.0, 0.0), dark)
cube("/World/Robot/ArmSide", (0.045, 0.50, 0.035), (0.0, 0.0, 0.0), dark)
for index, (x, y) in enumerate(((0.23, 0.23), (0.23, -0.23), (-0.23, 0.23), (-0.23, -0.23)), 1):
    cylinder(f"/World/Robot/Motor_{index}", 0.07, 0.07, (x, y, 0.02), dark)
    cylinder(f"/World/Robot/Propeller_{index}", 0.145, 0.012, (x, y, 0.07), (0.18, 0.20, 0.23))

# USD cameras look along local -Z with local +Y up.  Keep the original
# USER/MISSANDKEYBOARD 90-degree camera optics and tile four cardinal views
# over the horizontal 360 degrees.  A wider 120-degree view can see the UAV's
# own motors/propellers and incorrectly classify them as near-field obstacles.
camera_horizontal_fov_deg = 90.0
camera_max_depth = 10.0 * args.environment_scale
camera_views = (
    ("front", "CameraMount", 0.0),
    ("left", "CameraMountLeft", 90.0),
    ("back", "CameraMountBack", 180.0),
    ("right", "CameraMountRight", -90.0),
)
for camera_name, mount_name, yaw_deg in camera_views:
    yaw = math.radians(yaw_deg)
    half_cos, half_sin = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    camera_mount = UsdGeom.Xform.Define(stage, f"/World/Robot/{mount_name}")
    mount_api = UsdGeom.Xformable(camera_mount)
    mount_api.AddTranslateOp().Set(Gf.Vec3d(
        0.22 * math.cos(yaw), 0.22 * math.sin(yaw), 0.02))
    # q_yaw * q_front, with q_front=(0.5, 0.5, -0.5, -0.5).
    mount_api.AddOrientOp().Set(Gf.Quatf(
        0.5 * (half_cos + half_sin),
        Gf.Vec3f(
            0.5 * (half_cos + half_sin),
            0.5 * (-half_cos + half_sin),
            0.5 * (-half_cos + half_sin),
        ),
    ))
    camera = UsdGeom.Camera.Define(
        stage, f"/World/Robot/{mount_name}/DepthCamera")
    camera.CreateProjectionAttr("perspective")
    camera.CreateFocalLengthAttr(
        12.0 / math.tan(math.radians(camera_horizontal_fov_deg / 2.0)))
    camera.CreateHorizontalApertureAttr(24.0)
    camera.CreateVerticalApertureAttr(18.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.10, camera_max_depth))
    camera.GetPrim().CreateAttribute(
        "fov:rosFrame", Sdf.ValueTypeNames.String).Set(
            f"camera_{camera_name}_optical")

# This visible chase camera is a Robot child, so it follows the vehicle's
# translation and yaw while looking forward from behind and above.
third_person_camera = UsdGeom.Camera.Define(stage, "/World/Robot/ThirdPersonCamera")
third_person_camera.CreateProjectionAttr("perspective")
third_person_camera.CreateFocalLengthAttr(22.0)
third_person_camera.CreateHorizontalApertureAttr(36.0)
third_person_camera.CreateVerticalApertureAttr(20.25)
third_person_camera.CreateClippingRangeAttr(Gf.Vec2f(0.10, 1000.0))
third_person_view = Gf.Matrix4d(1.0)
third_person_view.SetLookAt(
    Gf.Vec3d(-4.0, 0.0, 1.8),
    Gf.Vec3d(1.5, 0.0, 0.2),
    Gf.Vec3d(0.0, 0.0, 1.0),
)
UsdGeom.Xformable(third_person_camera).AddTransformOp().Set(third_person_view.GetInverse())
third_person_camera.GetPrim().CreateAttribute(
    "fov:viewRole", Sdf.ValueTypeNames.String).Set("third_person_follow")

metadata = {
    "bodyRadiusM": 0.25,
    # Keep every horizontal view at the original USER/MISSANDKEYBOARD depth
    # resolution.  RViz consumes the unchanged front topic, so reducing each
    # camera to 160x120 made its live point cloud four times sparser.
    "cameraResolution": [320, 240],
    "cameraHorizontalFovDeg": camera_horizontal_fov_deg,
    "horizontalCameraViews": [
        {"name": name, "yawDeg": yaw_deg,
         "path": f"/World/Robot/{mount}/DepthCamera"}
        for name, mount, yaw_deg in camera_views
    ],
    "environmentScale": args.environment_scale,
    "cameraMaxDepthM": camera_max_depth,
}
if manual_spawn is not None:
    metadata.update({
        "manualControl": True,
        "spawn": [float(v) for v in spawn_position],
    })
else:
    metadata.update({
        "start": [float(v) for v in spawn_position],
        "goal": [float(v) for v in goal_position],
    })
stage.GetRootLayer().customLayerData = {
    **stage.GetRootLayer().customLayerData,
    "fovNavigationJson": json.dumps(metadata, separators=(",", ":")),
}
target.parent.mkdir(parents=True, exist_ok=True)
stage.GetRootLayer().Export(str(target))
print(f"[UAV SCENE] Wrote {target}")
if manual_spawn is not None:
    print(f"[UAV SCENE] Manual spawn "
          f"{tuple(round(float(v), 3) for v in spawn_position)}; no start/goal markers")
else:
    print(f"[UAV SCENE] Start {tuple(round(float(v), 3) for v in spawn_position)}")
    print(f"[UAV SCENE] Goal  {tuple(round(float(v), 3) for v in goal_position)}")
print("[UAV SCENE] Robot /World/Robot; four 90-degree horizontal depth cameras "
      "front/left/back/right; third-person camera /World/Robot/ThirdPersonCamera")
