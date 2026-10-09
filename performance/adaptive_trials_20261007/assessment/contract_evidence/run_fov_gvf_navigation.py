#!/usr/bin/env python3
"""Run a prepared manual-control USD with an Isaac Sim ROS 2 bridge.

The FOV-GVF controller remains an external ROS 2 node.  Isaac Sim supplies the
depth image, camera calibration and odometry through OmniGraph, receives the
holonomic velocity adapter command, and integrates the kinematic quadrotor at
fixed simulation time.  This keeps the vehicle interface deterministic while
the imported USD remains a normal editable environment.
"""

from __future__ import annotations

import json
import csv
import math
import os
import sys
import time
from pathlib import Path

from manual_control_math import (
    vector_safe_command,
    velocity_response,
    desired_velocity_from_direction,
    TimedVelocitySample,
)

from isaacsim import SimulationApp

if len(sys.argv) < 2:
    raise SystemExit("usage: run_fov_gvf_navigation.py SCENE.usd")
scene_path = Path(sys.argv[1]).expanduser().resolve()
if not scene_path.is_file():
    raise SystemExit(f"scene does not exist: {scene_path}")


def vector_from_env(name: str, default: tuple[float, float, float]) -> list[float]:
    raw = os.environ.get(name)
    if raw is None:
        return list(default)
    try:
        values = [float(value.strip()) for value in raw.split(",")]
    except ValueError as exc:
        raise SystemExit(f"{name} must contain three comma-separated numbers") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise SystemExit(f"{name} must contain three finite comma-separated numbers")
    return values


def scalar_from_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be a finite number") from exc
    if not math.isfinite(value) or value < 0.0:
        raise SystemExit(f"{name} must be a finite nonnegative number")
    return value


view_eye = vector_from_env("ISAAC_VIEW_EYE", (0.0, -180.0, 145.0))
view_target = vector_from_env("ISAAC_VIEW_TARGET", (0.0, 0.0, 6.0))
sun_intensity = scalar_from_env("ISAAC_SUN_INTENSITY", 800.0)
dome_intensity = scalar_from_env("ISAAC_DOME_INTENSITY", 0.0)
minimal_constant_color = vector_from_env(
    "ISAAC_MINIMAL_CONSTANT_COLOR", (0.60, 0.65, 0.70))
if not all(0.0 <= channel <= 1.0 for channel in minimal_constant_color):
    raise SystemExit("ISAAC_MINIMAL_CONSTANT_COLOR channels must be between 0 and 1")
ambient_light_intensity = scalar_from_env("ISAAC_AMBIENT_LIGHT_INTENSITY", 0.05)
manual_timeout = scalar_from_env("ISAAC_MANUAL_TIMEOUT", 0.0)
keyboard_speed = scalar_from_env("ISAAC_KEYBOARD_SPEED", 2.0)
keyboard_vertical_speed = scalar_from_env("ISAAC_KEYBOARD_VERTICAL_SPEED", 1.0)
manual_input_mode = os.environ.get("ISAAC_MANUAL_INPUT_MODE", "joystick").strip().lower()
if manual_input_mode not in {"keyboard", "joystick", "trace", "goal"}:
    raise SystemExit("ISAAC_MANUAL_INPUT_MODE must be keyboard, joystick, trace or goal")
twist_sample_source = os.environ.get("ISAAC_TWIST_SAMPLE_SOURCE", "callback").strip().lower()
if twist_sample_source not in {"omnigraph", "callback"}:
    raise SystemExit("ISAAC_TWIST_SAMPLE_SOURCE must be omnigraph or callback")
goal_trial = None
if manual_input_mode == "goal":
    from navigation_benchmark import GoalTrial
    goal_trial = GoalTrial(json.loads(Path(os.environ["ISAAC_GOAL_PROTOCOL"]).read_text()),
        float(os.environ.get("ISAAC_EXPLORATION_STALL_S", "0")))
intent_trace = None
trace_manual_yaw = False
trace_disturbances = []
if manual_input_mode == "trace":
    intent_trace = json.loads(Path(os.environ["ISAAC_INTENT_TRACE"]).read_text())
    events = intent_trace.get("events", [])
    if not events or any(len(e.get("velocity", [])) != 3 or
            not all(math.isfinite(float(v)) for v in [e["at"], *e["velocity"], e.get("yaw_rate", 0.0)]) for e in events):
        raise ValueError("trace events require finite at and three-dimensional velocity")
    if any(events[i]["at"] >= events[i+1]["at"] for i in range(len(events)-1)):
        raise ValueError("trace times must increase")
    trace_manual_yaw = any("yaw_rate" in event for event in events)
    trace_disturbances = list(intent_trace.get("disturbances", []))
plant_tau = scalar_from_env("ISAAC_PLANT_TAU", 0.22)
plant_accel = scalar_from_env("ISAAC_PLANT_ACCEL", 1.2)

if goal_trial is not None:
    plant_tau = goal_trial.protocol["plant_tau_s"]
    plant_accel = goal_trial.protocol["plant_accel_mps2"]

joystick_input = None
if manual_input_mode == "joystick":
    from beitong_joystick import BeitongMode2

    joystick_input = BeitongMode2()
viewport_capture_path = Path(
    os.environ.get("ISAAC_VIEW_CAPTURE", "/tmp/fov_gvf_isaac_viewport.png")
).expanduser()
performance_log_path = Path(os.environ.get(
    "FOV_GVF_PERFORMANCE_LOG", "performance/PERFORMANCE_METRICS.md"
)).expanduser()
performance_run_id = os.environ.get("FOV_GVF_RUN_ID", "unspecified")
viewport_antialiasing = os.environ.get("ISAAC_VIEW_ANTIALIASING", "TAA").upper()
viewport_aa_modes = {"OFF": 0, "TAA": 1, "FXAA": 2}
if viewport_antialiasing not in viewport_aa_modes:
    raise SystemExit(
        "ISAAC_VIEW_ANTIALIASING must be OFF, TAA, or FXAA; "
        "DLSS/DLAA are incompatible with the metric depth annotator"
    )
viewport_renderer = os.environ.get(
    "ISAAC_VIEW_RENDERER", "MinimalRendering").strip()
if viewport_renderer.lower() not in {"minimal", "minimalrendering", "raytracedlighting"}:
    raise SystemExit(
        "ISAAC_VIEW_RENDERER must be MinimalRendering or RaytracedLighting")
if viewport_renderer.lower() in {"minimal", "minimalrendering"}:
    viewport_renderer = "MinimalRendering"
else:
    viewport_renderer = "RaytracedLighting"
minimal_shading_mode = int(scalar_from_env("ISAAC_MINIMAL_SHADING_MODE", 2.0))
if minimal_shading_mode not in {0, 1, 2, 3, 4}:
    raise SystemExit("ISAAC_MINIMAL_SHADING_MODE must be 0, 1, 2, 3, or 4")

sys.argv = [sys.argv[0]]
headless = os.environ.get("ISAAC_HEADLESS", "0") == "1"
simulation_app = SimulationApp({
    "headless": headless,
    "renderer": viewport_renderer,
    "minimal_shading_mode": minimal_shading_mode,
    "active_gpu": 0,
    "multi_gpu": False,
    # Keep a depth-compatible AA mode.  The clean engineering viewport itself
    # uses MinimalRendering by default to avoid dense-voxel RTX speckle.
    "anti_aliasing": viewport_aa_modes[viewport_antialiasing],
})

import carb
import carb.input
import omni.graph.core as og
import omni.appwindow
import omni.kit.viewport.utility as viewport_utility
import omni.replicator.core as rep
import omni.usd
import numpy as np
import usdrt.Sdf
from scipy import ndimage
from isaacsim.core.simulation_manager import SimulationManager
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from pxr import Gf, Usd, UsdGeom, UsdLux

app_utils.enable_extension("isaacsim.ros2.bridge")
for _ in range(5):
    simulation_app.update()
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

if not rclpy.ok():
    rclpy.init(args=None)
manual_ros_node = rclpy.create_node(f"isaac_{manual_input_mode}_intent")
manual_intent_publisher = manual_ros_node.create_publisher(
    TwistStamped, "/human_intent", 10)
last_twist_wall = None
twist_sample = TimedVelocitySample()
def record_twist_receipt(message):
    global last_twist_wall
    last_twist_wall = time.monotonic()
    twist_sample.receive((message.linear.x, message.linear.y, message.linear.z), last_twist_wall)
manual_ros_node.create_subscription(Twist, "/sim/cmd_vel", record_twist_receipt, 1)
controller_status = "WAITING"
controller_diagnostics = {}
def record_controller_status(message):
    global controller_status
    controller_status = message.data
def record_controller_diagnostics(message):
    global controller_diagnostics
    try:
        controller_diagnostics = json.loads(message.data)
    except (ValueError, TypeError):
        controller_diagnostics = {}
# Validation-only subscriptions avoid adding callback load during normal flight.
if os.environ.get("ISAAC_ACCEPTANCE_TRACE"):
    manual_ros_node.create_subscription(String, "/depth_angular_controller/status", record_controller_status, 1)
    manual_ros_node.create_subscription(String, "/depth_angular_controller/paper_diagnostics", record_controller_diagnostics, 1)
context = omni.usd.get_context()
if not context.open_stage(scene_path.as_posix()):
    raise RuntimeError(f"failed to open {scene_path}")
simulation_app.reset_render_settings()
# Replicator's set_render_rtx_realtime() switches the whole application to
# RealTimePathTracing.  With FXAA that mode exposes its one-sample stochastic
# lighting as dense black/white speckle in the UI viewport.  Keep the renderer
# selected above (MinimalRendering by default) and apply only the
# depth-compatible anti-aliasing operation globally.
render_settings = carb.settings.get_settings()
render_settings.set("/rtx/rendermode", viewport_renderer)
render_settings.set("/rtx/minimal/mode", minimal_shading_mode)
render_settings.set("/rtx/minimal/constantColor", tuple(minimal_constant_color))
render_settings.set("/rtx/sceneDb/ambientLightColor", (1.0, 1.0, 1.0))
render_settings.set("/rtx/sceneDb/ambientLightIntensity", ambient_light_intensity)
render_settings.set(
    "/rtx/post/aa/op", viewport_aa_modes[viewport_antialiasing])
render_settings.set("/rtx-transient/post/aa/limitedOps", False)
render_settings.set("/rtx/post/tvNoise/enabled", False)
render_settings.set("/rtx/post/tvNoise/enableFilmGrain", False)
render_settings.set("/rtx/post/tvNoise/enableRandomSplotches", False)
render_settings.set("/rtx/post/tonemap/dither", 0.0)
# The imported voxel cloud contains more than 1.5 million triangles.  RTX's
# stochastic sampled-lighting passes leave individual pixels unresolved while
# the chase camera moves every frame, producing persistent salt-and-pepper
# noise.  Use deterministic direct lighting for this engineering viewport;
# metric depth depends only on geometry and is unaffected by these switches.
render_settings.set("/rtx/directLighting/sampledLighting/enabled", False)
render_settings.set("/rtx/reflections/enabled", False)
render_settings.set("/rtx/reflections/sampledLighting/enabled", False)
render_settings.set("/rtx/ambientOcclusion/enabled", False)
render_settings.set("/rtx/indirectDiffuse/enabled", False)
stage = context.get_stage()
sun_count = 0
dome_count = 0
for prim in stage.Traverse():
    if prim.IsA(UsdLux.DistantLight):
        UsdLux.DistantLight(prim).GetIntensityAttr().Set(sun_intensity)
        sun_count += 1
    elif prim.IsA(UsdLux.DomeLight):
        UsdLux.DomeLight(prim).GetIntensityAttr().Set(dome_intensity)
        dome_count += 1
if sun_count == 0:
    sun = UsdLux.DistantLight.Define(stage, "/World/FovGVF_SunLight")
    sun.CreateIntensityAttr(sun_intensity)
    sun.CreateAngleAttr(0.53)
    UsdGeom.XformCommonAPI(sun).SetRotate(
        Gf.Vec3f(-45.0, 0.0, 35.0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    sun_count = 1
if dome_count == 0:
    dome = UsdLux.DomeLight.Define(stage, "/World/FovGVF_DomeLight")
    dome.CreateIntensityAttr(dome_intensity)
    dome_count = 1
print(
    f"[LIGHTING] sun={sun_intensity:.1f} ({sun_count}) "
    f"dome={dome_intensity:.1f} ({dome_count}) "
    f"ambient={ambient_light_intensity:.2f} "
    f"constant_color={tuple(round(value, 2) for value in minimal_constant_color)}",
    flush=True,
)
robot = stage.GetPrimAtPath("/World/Robot")
depth_camera_views = (
    ("Front", "/World/Robot/CameraMount/DepthCamera",
     "/sim/depth/image_raw", "/sim/depth/camera_info", "camera_front_optical"),
    ("Left", "/World/Robot/CameraMountLeft/DepthCamera",
     "/sim/depth_left/image_raw", "/sim/depth_left/camera_info", "camera_left_optical"),
    ("Back", "/World/Robot/CameraMountBack/DepthCamera",
     "/sim/depth_back/image_raw", "/sim/depth_back/camera_info", "camera_back_optical"),
    ("Right", "/World/Robot/CameraMountRight/DepthCamera",
     "/sim/depth_right/image_raw", "/sim/depth_right/camera_info", "camera_right_optical"),
)
missing_cameras = [path for _, path, *_ in depth_camera_views
                   if not stage.GetPrimAtPath(path).IsValid()]
if not robot.IsValid() or missing_cameras:
    raise RuntimeError(
        "navigation USD is missing /World/Robot or horizontal depth cameras: "
        + ", ".join(missing_cameras))

navigation_data = stage.GetRootLayer().customLayerData.get("fovNavigationJson", "{}")
try:
    navigation_metadata = json.loads(navigation_data)
except (TypeError, json.JSONDecodeError):
    navigation_metadata = {}
if navigation_metadata.get("manualControl") is not True:
    raise RuntimeError("scene metadata must enable manualControl")
spawn = navigation_metadata.get("spawn")
if (
    not isinstance(spawn, list) or len(spawn) != 3
    or not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in spawn)
):
    raise RuntimeError("fovNavigationJson.spawn must contain three finite numbers")
for removed_marker in ("/World/UAV_Start", "/World/UAV_Goal"):
    if stage.GetPrimAtPath(removed_marker).IsValid():
        raise RuntimeError(f"manual-control scene still contains {removed_marker}")
if goal_trial is not None:
    spawn = goal_trial.protocol["start"]
    # Session-only coordinate markers do not alter the rendered geometry or USD hash.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for label, point in (("Start", spawn), ("Goal", goal_trial.goal)):
            marker = UsdGeom.Xform.Define(stage, f"/World/UAV_{label}")
            UsdGeom.XformCommonAPI(marker).SetTranslate(Gf.Vec3d(*point))
position = Gf.Vec3d(*(float(value) for value in spawn))
velocity = Gf.Vec3d(0.0, 0.0, 0.0)
yaw = 0.0
yaw_rate = 0.0
body_radius = (goal_trial.protocol["body_radius_m"] if goal_trial is not None
               else scalar_from_env("FOV_GVF_BODY_RADIUS", 0.48))
max_yaw_rate = math.radians(75.0)

robot_api = UsdGeom.XformCommonAPI(robot)
robot_api.SetTranslate(position)

# A monolithic voxel-cloud mesh cannot use its overall AABB for collision
# queries.  When occupancy.bin accompanies the USD, construct a signed
# Euclidean distance field and sample it trilinearly at each candidate pose.
esdf = None
environment_scale = float(navigation_metadata.get("environmentScale", 1.0))
camera_resolution = navigation_metadata.get("cameraResolution", [320, 240])
if (
    not isinstance(camera_resolution, list) or len(camera_resolution) != 2
    or any(not isinstance(value, int) or value < 8 for value in camera_resolution)
):
    raise RuntimeError("fovNavigationJson.cameraResolution must contain two positive integers")
camera_width, camera_height = camera_resolution
print(
    f"[RENDER] mode={viewport_renderer} minimal_shading_mode={minimal_shading_mode} "
    f"anti_aliasing={viewport_antialiasing}",
    flush=True,
)
print(
    f"[DEPTH CAMERAS] count={len(depth_camera_views)} "
    f"resolution_each={camera_width}x{camera_height} horizontal_coverage=360deg",
    flush=True,
)
esdf_resolution = 0.1 * environment_scale
esdf_origin = environment_scale * np.array((-20.0, -15.0, 0.0), dtype=np.float64)
occupancy_path = Path(os.environ.get(
    "FOV_GVF_ESDF_OCCUPANCY", scene_path.with_name("occupancy.bin")))
if occupancy_path.is_file():
    occupancy = np.fromfile(occupancy_path, dtype=np.uint8)
    expected_shape = (400, 300, 50)
    if occupancy.size != math.prod(expected_shape):
        raise RuntimeError(
            f"ESDF occupancy has {occupancy.size} cells, expected {math.prod(expected_shape)}")
    occupancy = occupancy.reshape(expected_shape).astype(bool)
    outside = ndimage.distance_transform_edt(~occupancy) * esdf_resolution
    inside = ndimage.distance_transform_edt(occupancy) * esdf_resolution
    half_cell = 0.5 * esdf_resolution
    esdf = np.where(
        occupancy, -(inside - half_cell), outside - half_cell).astype(np.float32)
    print(
        f"[ESDF] path={occupancy_path} shape={expected_shape} "
        f"resolution={esdf_resolution:.2f}m",
        flush=True,
    )

# Other scenes retain a conservative three-dimensional AABB guard.
obstacle_bounds = []
ground_top_z = None
if esdf is None:
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.IsA(UsdGeom.Mesh) or path.startswith("/World/Robot"):
            continue
        box = UsdGeom.Boundable(prim).ComputeWorldBound(
            Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedBox()
        minimum, maximum = box.GetMin(), box.GetMax()
        is_ground = path.lower().endswith("/ground") or "static_ground" == str(
            prim.GetAttribute("fov:collisionRole").Get() or "")
        if is_ground:
            ground_top_z = (
                float(maximum[2]) if ground_top_z is None
                else max(ground_top_z, float(maximum[2])))
            continue
        obstacle_bounds.append(tuple(float(value) for value in (
            minimum[0], minimum[1], minimum[2],
            maximum[0], maximum[1], maximum[2])))
    print(
        f"[SCENE] AABB guard obstacles={len(obstacle_bounds)} "
        f"ground_top_z={ground_top_z}",
        flush=True,
    )

first_person_camera_path = "/World/Robot/CameraMount/DepthCamera"
third_person_camera_path = "/World/Robot/ThirdPersonCamera"
if not headless:
    third_person_camera = UsdGeom.Camera.Define(stage, third_person_camera_path)
    third_person_camera.CreateProjectionAttr("perspective")
    third_person_camera.CreateFocalLengthAttr(22.0)
    third_person_camera.CreateHorizontalApertureAttr(36.0)
    third_person_camera.CreateVerticalApertureAttr(20.25)
    third_person_camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000.0))
    third_person_xform = UsdGeom.Xformable(third_person_camera)
    if not third_person_xform.GetOrderedXformOps():
        third_person_view = Gf.Matrix4d(1.0)
        third_person_view.SetLookAt(
            Gf.Vec3d(-4.0, 0.0, 1.8),
            Gf.Vec3d(1.5, 0.0, 0.2),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
        third_person_xform.AddTransformOp().Set(third_person_view.GetInverse())

def sample_esdf(candidate: Gf.Vec3d) -> float:
    grid = (np.asarray(tuple(candidate), dtype=np.float64) - esdf_origin) / esdf_resolution
    base = np.floor(grid).astype(np.int64)
    fraction = grid - base
    if np.any(base < 0) or np.any(base + 1 >= np.asarray(esdf.shape)):
        return float("-inf")
    value = 0.0
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                weight = (
                    (fraction[0] if dx else 1.0 - fraction[0]) *
                    (fraction[1] if dy else 1.0 - fraction[1]) *
                    (fraction[2] if dz else 1.0 - fraction[2]))
                value += weight * float(esdf[
                    base[0] + dx, base[1] + dy, base[2] + dz])
    return value


def collides(candidate: Gf.Vec3d) -> bool:
    if esdf is not None:
        return sample_esdf(candidate) <= body_radius
    if ground_top_z is not None and float(candidate[2]) <= ground_top_z + body_radius:
        return True
    for xmin, ymin, zmin, xmax, ymax, zmax in obstacle_bounds:
        dx = max(xmin - float(candidate[0]), 0.0, float(candidate[0]) - xmax)
        dy = max(ymin - float(candidate[1]), 0.0, float(candidate[1]) - ymax)
        dz = max(zmin - float(candidate[2]), 0.0, float(candidate[2]) - zmax)
        if math.sqrt(dx * dx + dy * dy + dz * dz) <= body_radius:
            return True
    return False

# Build one execution graph for odometry, four depth/info views, and Twist input.
keys = og.Controller.Keys
create_nodes = [
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
]
set_values = [
    ("SubscribeTwist.inputs:topicName", "/sim/cmd_vel"),
    ("SubscribeTwist.inputs:queueSize", 2),
    ("PublishClock.inputs:topicName", "/clock"),
    ("PublishOdometry.inputs:topicName", "/sim/odom"),
    ("PublishOdometry.inputs:odomFrameId", "world"),
    ("PublishOdometry.inputs:chassisFrameId", "base_link"),
    ("PublishOdometry.inputs:publishRawVelocities", True),
]
connections = [
    ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
    ("Context.outputs:context", "SubscribeTwist.inputs:context"),
    ("Context.outputs:context", "PublishClock.inputs:context"),
    ("Context.outputs:context", "PublishOdometry.inputs:context"),
    ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
]
for suffix, camera_path, depth_topic, info_topic, frame_id in depth_camera_views:
    render = f"CreateRenderProduct{suffix}"
    depth_helper = f"DepthHelper{suffix}"
    info_helper = f"CameraInfoHelper{suffix}"
    create_nodes.extend([
        (render, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        (depth_helper, "isaacsim.ros2.bridge.ROS2CameraHelper"),
        (info_helper, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ])
    set_values.extend([
        (f"{render}.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
        (f"{render}.inputs:width", camera_width),
        (f"{render}.inputs:height", camera_height),
        (f"{depth_helper}.inputs:topicName", depth_topic),
        (f"{depth_helper}.inputs:type", "depth"),
        (f"{depth_helper}.inputs:frameId", frame_id),
        (f"{depth_helper}.inputs:resetSimulationTimeOnStop", True),
        (f"{info_helper}.inputs:topicName", info_topic),
        (f"{info_helper}.inputs:frameId", frame_id),
    ])
    connections.extend([
        ("OnPlaybackTick.outputs:tick", f"{render}.inputs:execIn"),
        (f"{render}.outputs:execOut", f"{depth_helper}.inputs:execIn"),
        (f"{render}.outputs:execOut", f"{info_helper}.inputs:execIn"),
        (f"{render}.outputs:renderProductPath", f"{depth_helper}.inputs:renderProductPath"),
        (f"{render}.outputs:renderProductPath", f"{info_helper}.inputs:renderProductPath"),
        ("Context.outputs:context", f"{depth_helper}.inputs:context"),
        ("Context.outputs:context", f"{info_helper}.inputs:context"),
    ])
graph, _, _, _ = og.Controller.edit(
    {"graph_path": "/World/FovGVFGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: create_nodes,
        keys.SET_VALUES: set_values,
        keys.CONNECT: connections,
    },
)
og.Controller.evaluate_sync(graph)
viewport_rgb = None
keyboard_subscription = None
input_interface = None
keyboard = None
if not headless:
    ready, waited_frames = ViewportManager.wait_for_viewport(
        max_frames=120, sleep_time=0.02)
    viewport = viewport_utility.get_active_viewport()
    if not ready or viewport is None:
        raise RuntimeError(
            f"Isaac main viewport did not become ready after {waited_frames} frames")
    # Bind the visible viewport explicitly.  This is intentionally done after
    # the ROS depth render product is created so it cannot steal the UI view.
    active_view_mode = ["third"]

    def select_view(mode: str) -> None:
        camera_path = (first_person_camera_path
                       if mode == "first" else third_person_camera_path)
        ViewportManager.set_camera(
            camera_path, render_product_or_viewport=viewport)
        active_view_mode[0] = mode
        print(f"[CAMERA] {mode.upper()}_PERSON path={camera_path}", flush=True)

    def on_keyboard_event(event, *_args, **_kwargs):
        if event.type not in {
                carb.input.KeyboardEventType.KEY_PRESS,
                carb.input.KeyboardEventType.KEY_REPEAT}:
            return True
        key_name = str(getattr(event.input, "name", event.input)).upper().rsplit(".", 1)[-1]
        if key_name in {"1", "KEY_1", "NUMPAD_1"}:
            select_view("first")
        elif key_name in {"3", "KEY_3", "NUMPAD_3"}:
            select_view("third")
        elif key_name == "V":
            select_view("first" if active_view_mode[0] == "third" else "third")
        return True

    select_view("third")
    app_window = omni.appwindow.get_default_app_window()
    keyboard = app_window.get_keyboard()
    input_interface = carb.input.acquire_input_interface()
    keyboard_subscription = input_interface.subscribe_to_keyboard_events(
        keyboard, on_keyboard_event)
    for _ in range(10):
        simulation_app.update()
    active_camera = ViewportManager.get_camera(viewport)
    viewport_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    viewport_rgb.attach([viewport.render_product_path])
    print(
        f"[VIEWPORT] ready={ready} camera={active_camera.GetPath()} "
        f"resolution={tuple(viewport.resolution)} "
        f"input={manual_input_mode} "
        f"controls={'Mode-2 sticks' if joystick_input is not None else 'W/S A/D R/F SPACE'} "
        f"1/3/V:view",
        flush=True,
    )
elif manual_input_mode not in {"trace", "goal"}:
    raise RuntimeError("manual control requires ISAAC_HEADLESS=0")


def keyboard_value(*keys) -> float:
    """Read physical key state directly; no sticky press/release state is kept."""
    return max(float(input_interface.get_keyboard_value(keyboard, key)) for key in keys)


def sample_keyboard_intent() -> tuple[tuple[float, float, float], bool]:
    if keyboard_value(carb.input.KeyboardInput.SPACE) > 0.5:
        return (0.0, 0.0, 0.0), False
    direction = desired_velocity_from_direction(
        keyboard_value(carb.input.KeyboardInput.W, carb.input.KeyboardInput.UP)
        - keyboard_value(carb.input.KeyboardInput.S, carb.input.KeyboardInput.DOWN),
        keyboard_value(carb.input.KeyboardInput.A, carb.input.KeyboardInput.LEFT)
        - keyboard_value(carb.input.KeyboardInput.D, carb.input.KeyboardInput.RIGHT),
        keyboard_value(carb.input.KeyboardInput.R, carb.input.KeyboardInput.PAGE_UP)
        - keyboard_value(carb.input.KeyboardInput.F, carb.input.KeyboardInput.PAGE_DOWN),
        keyboard_speed,
        keyboard_vertical_speed,
    )
    if not any(abs(value) > 1.0e-6 for value in direction):
        return (0.0, 0.0, 0.0), False
    return direction, True


last_stick_log = None
last_stick_log_time = -1.0


def sample_manual_intent(current_yaw: float, now: float):
    """Return world translation, independent yaw rate, and physical activity."""
    global last_stick_log, last_stick_log_time
    if goal_trial is not None:
        direction = goal_trial.intent(position, now)
        return direction, 0.0, any(abs(v) > 1e-6 for v in direction)
    if intent_trace is not None:
        direction = (0.0, 0.0, 0.0)
        requested_yaw_rate = 0.0
        for event in intent_trace["events"]:
            if now < event["at"]:
                break
            direction = tuple(float(v) for v in event["velocity"])
            requested_yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, float(event.get("yaw_rate", 0.0))))
        return direction, requested_yaw_rate, any(abs(v) > 1e-6 for v in (*direction, requested_yaw_rate))
    if joystick_input is None:
        direction, active = sample_keyboard_intent()
        return direction, 0.0, active
    direction, requested_yaw_rate, active, channels = joystick_input.sample(
        current_yaw)
    changed = (last_stick_log is None or
               max(abs(channels[name] - last_stick_log[name])
                   for name in channels) >= 0.05)
    # A one-second heartbeat makes a neutral-gate or device-stream problem
    # visible even when no axis-change event reaches the process.
    if ((changed and now - last_stick_log_time >= 0.10) or
            now - last_stick_log_time >= 1.0):
        print(
            f"[STICKS] roll={channels['roll']:+.2f} "
            f"pitch={channels['pitch']:+.2f} "
            f"throttle={channels['throttle']:+.2f} "
            f"yaw={channels['yaw']:+.2f} "
            f"enabled={'yes' if joystick_input.enabled else 'no'} "
            f"connected={'yes' if joystick_input.device.connected else 'no'}",
            flush=True,
        )
        last_stick_log = channels.copy()
        last_stick_log_time = now
    return direction, requested_yaw_rate, active
twist_x = og.Controller.attribute("/World/FovGVFGraph/SubscribeTwist.outputs:linearVelocity").get()
twist_w = og.Controller.attribute("/World/FovGVFGraph/SubscribeTwist.outputs:angularVelocity").get()
odom_inputs = {
    "position": og.Controller.attribute("/World/FovGVFGraph/PublishOdometry.inputs:position"),
    "orientation": og.Controller.attribute("/World/FovGVFGraph/PublishOdometry.inputs:orientation"),
    "linearVelocity": og.Controller.attribute("/World/FovGVFGraph/PublishOdometry.inputs:linearVelocity"),
    "angularVelocity": og.Controller.attribute("/World/FovGVFGraph/PublishOdometry.inputs:angularVelocity"),
}

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()
start_time = simulation_app.get_time() if hasattr(simulation_app, "get_time") else 0.0
last_time = 0.0
now = 0.0
frame = 0
viewport_captured = False
performance_wall_start = time.perf_counter()
last_frame_wall = None
last_command_change_wall = None
last_applied_command = None
frame_interval_ms = []
command_change_interval_ms = []
command_change_count = 0
navigation_result = "RUNNING"
collision_block_count = 0
last_collision_report_frame = -60
acceptance_stream = None
if os.environ.get("ISAAC_ACCEPTANCE_TRACE"):
    acceptance_path = Path(os.environ["ISAAC_ACCEPTANCE_TRACE"])
    acceptance_path.parent.mkdir(parents=True, exist_ok=True)
    acceptance_stream = acceptance_path.open("w", newline="")
    acceptance_writer = csv.writer(acceptance_stream)
    acceptance_writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "status",
        "esdf_clearance_r048", "collision_blocks", "recorded_balls", "revoked_balls", "expired_views", "evicted_views", "build_reason", "required_prefix", "best_prefix", "free_directions", "recorded_tubes", "grid_refined", "command_changed", "intent_change_angle", "refreshed", "continued", "reset_reason", "diagnostic_stamp", "input_x", "input_y", "input_z", "target_x", "target_y", "target_z", "reference_intent_x", "reference_intent_y", "reference_intent_z", "command_x", "command_y", "command_z", "applied_target_x", "applied_target_y", "applied_target_z", "twist_source", "received_twist_sequence", "applied_twist_sequence", "twist_age_s", "received_body_x", "received_body_y", "received_body_z", "applied_body_x", "applied_body_y", "applied_body_z"])
while simulation_app.is_running() and app_utils.is_playing():
    simulation_app.update()
    frame_wall = time.perf_counter()
    if last_frame_wall is not None:
        frame_interval_ms.append((frame_wall - last_frame_wall) * 1000.0)
    last_frame_wall = frame_wall
    now = float(frame) / 60.0
    dt = max(1.0 / 240.0, min(0.1, now - last_time if frame else 1.0 / 60.0))
    last_time = now
    manual_direction, manual_yaw_rate, manual_active = sample_manual_intent(yaw, now)
    translation_active = any(abs(value) > 1.0e-6 for value in manual_direction)
    manual_message = TwistStamped()
    manual_message.header.stamp = manual_ros_node.get_clock().now().to_msg()
    manual_message.header.frame_id = "world"
    manual_message.twist.linear.x = manual_direction[0]
    manual_message.twist.linear.y = manual_direction[1]
    manual_message.twist.linear.z = manual_direction[2]
    manual_message.twist.angular.z = manual_yaw_rate
    manual_intent_publisher.publish(manual_message)
    # Drain a bounded batch: status/diagnostics must not starve the independent
    # Twist freshness callback (one spin services only one ready callback).
    for _ in range(8):
        rclpy.spin_once(manual_ros_node, timeout_sec=0.0)
    raw = og.Controller.attribute("/World/FovGVFGraph/SubscribeTwist.outputs:linearVelocity").get()
    # Vector and receipt time belong to the SAME DDS sample in callback mode.
    # Keep the legacy source selectable for controlled before/after experiments.
    sample_wall = time.monotonic()
    received_snapshot = twist_sample.snapshot
    callback_body, callback_sequence, twist_age = twist_sample.select(sample_wall, translation_active)
    if twist_sample_source == "callback":
        command_body = callback_body
        applied_twist_sequence = callback_sequence
    elif (not translation_active or raw is None or len(raw) < 3 or
            last_twist_wall is None or sample_wall-last_twist_wall > .10):
        command_body = (0.0, 0.0, 0.0)
        applied_twist_sequence = 0
    else:
        command_body = tuple(float(v) for v in raw[:3])
        # OmniGraph has no matching receive sequence; never label it with the
        # independent Python subscriber's sequence.
        applied_twist_sequence = -1
    if last_applied_command is None or any(
            abs(command_body[index] - last_applied_command[index]) > 1.0e-9
            for index in range(3)):
        command_change_count += 1
        if last_command_change_wall is not None:
            command_change_interval_ms.append(
                (frame_wall - last_command_change_wall) * 1000.0)
        last_command_change_wall = frame_wall
        last_applied_command = command_body
    c, s = math.cos(yaw), math.sin(yaw)
    command = (
        c * command_body[0] - s * command_body[1],
        s * command_body[0] + c * command_body[1],
        command_body[2],
    )
    applied_target = command
    if plant_tau > 0.0:
        command = velocity_response(velocity, command, dt, plant_tau, plant_accel)
    while trace_disturbances and now >= trace_disturbances[0]["at"]:
        disturbance = trace_disturbances.pop(0)
        delta = disturbance["delta_velocity"]
        command = tuple(command[i] + float(delta[i]) for i in range(3))
        print(f"[VALIDATION DISTURBANCE] t={now:.3f} delta_velocity={delta}", flush=True)
    command, collision_blocked = vector_safe_command(
        tuple(position), command, dt,
        lambda candidate: collides(Gf.Vec3d(*candidate)))
    candidate = Gf.Vec3d(*(float(position[i]) + command[i] * dt for i in range(3)))
    if collision_blocked:
        collision_block_count += 1
        if frame - last_collision_report_frame >= 60:
            print(
                f"[COLLISION GUARD] blocked movement at "
                f"({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}); "
                f"stopped planned vector command={command}",
                flush=True,
            )
            last_collision_report_frame = frame
    position = candidate
    velocity = Gf.Vec3d(*command)
    horizontal_command = math.hypot(command[0], command[1])
    if joystick_input is not None or trace_manual_yaw:
        yaw_step = manual_yaw_rate * dt if manual_active else 0.0
        yaw = math.atan2(math.sin(yaw + yaw_step), math.cos(yaw + yaw_step))
        yaw_rate = yaw_step / dt
    elif goal_trial is not None:
        desired_yaw = math.atan2(goal_trial.goal[1]-position[1], goal_trial.goal[0]-position[0])
        yaw_error = math.atan2(math.sin(desired_yaw-yaw), math.cos(desired_yaw-yaw))
        yaw_step = max(-max_yaw_rate*dt, min(max_yaw_rate*dt, yaw_error))
        yaw += yaw_step
        yaw_rate = yaw_step / dt
    elif horizontal_command > 0.03:
        # Keep the depth camera aligned with the actual collision-free command.
        desired_yaw = math.atan2(command[1], command[0])
        yaw_error = math.atan2(
            math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))
        yaw_step = max(-max_yaw_rate * dt, min(max_yaw_rate * dt, yaw_error))
        yaw += yaw_step
        yaw_rate = yaw_step / dt
    else:
        yaw_rate = 0.0
    robot_api.SetTranslate(position)
    robot_api.SetRotate(
        Gf.Vec3f(0.0, 0.0, math.degrees(yaw)),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    odom_inputs["position"].set([float(position[0]), float(position[1]), float(position[2])])
    odom_inputs["orientation"].set([
        0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)
    ])
    odom_inputs["linearVelocity"].set([float(velocity[0]), float(velocity[1]), float(velocity[2])])
    odom_inputs["angularVelocity"].set([0.0, 0.0, yaw_rate])
    if acceptance_stream is not None:
        acceptance_writer.writerow([now, *position, *velocity, *manual_direction, controller_status,
            sample_esdf(position)-.48 if esdf is not None else float("nan"), collision_block_count,
            *[controller_diagnostics.get(k, "") for k in ["recorded_balls", "revoked_balls", "expired_views", "evicted_views", "build_reason", "required_prefix", "best_prefix", "free_directions", "recorded_tubes", "grid_refined", "command_changed", "intent_change_angle", "refreshed", "continued", "reset_reason", "diagnostic_stamp", "input_x", "input_y", "input_z", "target_x", "target_y", "target_z", "reference_intent_x", "reference_intent_y", "reference_intent_z", "command_x", "command_y", "command_z"]], *applied_target,
            twist_sample_source, received_snapshot[2] if received_snapshot else 0,
            applied_twist_sequence, twist_age,
            *(received_snapshot[0] if received_snapshot else (0.0, 0.0, 0.0)), *command_body])
    if frame % 300 == 0:
        print(
            f"[NAV] t={now:.1f}s position=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) "
            f"yaw={math.degrees(yaw):.1f}deg "
            f"command=({command[0]:.2f},{command[1]:.2f},{command[2]:.2f}) "
            f"manual_active={int(manual_active)} "
            f"collision_blocks={collision_block_count}",
            flush=True,
        )
    if viewport_rgb is not None and not viewport_captured and frame >= 60:
        viewport_data = viewport_rgb.get_data()
        if viewport_data is not None and getattr(viewport_data, "size", 0) > 0:
            from PIL import Image

            viewport_capture_path.parent.mkdir(parents=True, exist_ok=True)
            viewport_color = viewport_data[:, :, :3]
            Image.fromarray(viewport_color).save(viewport_capture_path)
            viewport_mean = float(viewport_color.astype("float32").mean())
            viewport_captured = True
            print(
                f"[VIEWPORT CAPTURE] path={viewport_capture_path} "
                f"mean_rgb={viewport_mean:.2f}",
                flush=True,
            )
    frame += 1
    if goal_trial is not None:
        navigation_result = goal_trial.update(position, velocity, now, time.perf_counter(), collision_blocked)
        if navigation_result != "RUNNING":
            print(f"[BENCHMARK RESULT] {navigation_result} sim={now-goal_trial.warmup:.3f}s", flush=True)
            break
    if manual_timeout > 0.0 and now > manual_timeout:
        navigation_result = "MANUAL_TIMEOUT"
        print(f"[MANUAL RESULT] TIMEOUT position=({position[0]:.3f},"
              f"{position[1]:.3f},{position[2]:.3f})", flush=True)
        break

if acceptance_stream is not None:
    acceptance_stream.close()

if navigation_result == "RUNNING":
    navigation_result = "USER_EXIT"

if goal_trial is not None:
    result = goal_trial.result(navigation_result, position, now, time.perf_counter(), collision_block_count)
    result.update(algorithm=os.environ["FOV_GVF_BENCHMARK_ALGORITHM"], run_id=performance_run_id)
    Path(os.environ["ISAAC_BENCHMARK_RESULT"]).write_text(json.dumps(result, indent=2))

performance_wall_elapsed = max(1.0e-9, time.perf_counter() - performance_wall_start)
applied_control_frames = frame + 1


def timing_summary(values):
    if not values:
        return 0.0, 0.0, 0.0, 0.0
    ordered = sorted(values)
    percentile = lambda fraction: ordered[round(fraction * (len(ordered) - 1))]
    return (
        sum(values) / len(values), percentile(0.50),
        percentile(0.95), ordered[-1])


frame_mean, frame_p50, frame_p95, frame_max = timing_summary(frame_interval_ms)
change_mean, change_p50, change_p95, change_max = timing_summary(
    command_change_interval_ms)
simulation_wall_fps = applied_control_frames / performance_wall_elapsed
simulation_time_fps = applied_control_frames / max(now, 1.0e-9)
print(
    f"[PERF ISAAC FINAL] run={performance_run_id} result={navigation_result} "
    f"wall={performance_wall_elapsed:.2f}s sim_frames={applied_control_frames} "
    f"fps(sim/wall)={simulation_time_fps:.2f}/{simulation_wall_fps:.2f} "
    f"command_changes={command_change_count} "
    f"collision_blocks={collision_block_count} "
    f"frame_interval_ms(mean/p50/p95/max)="
    f"{frame_mean:.3f}/{frame_p50:.3f}/{frame_p95:.3f}/{frame_max:.3f}",
    flush=True,
)
performance_record = (
    f"\n#### Isaac command application — {performance_run_id}\n\n"
    f"- Result: {navigation_result}\n"
    f"- Wall duration / simulation duration: {performance_wall_elapsed:.3f} / "
    f"{now:.3f} s\n"
    f"- Applied simulation control frames: {applied_control_frames}\n"
    f"- Runtime FPS (simulation time / wall time): {simulation_time_fps:.3f} / "
    f"{simulation_wall_fps:.3f} Hz\n"
    f"- Isaac frame interval mean / P50 / P95 / max: {frame_mean:.3f} / "
    f"{frame_p50:.3f} / {frame_p95:.3f} / {frame_max:.3f} ms\n"
    f"- Distinct applied command values: {command_change_count}\n"
    f"- ESDF-blocked simulation frames: {collision_block_count}\n"
    f"- Applied-command change interval mean / P50 / P95 / max: "
    f"{change_mean:.3f} / {change_p50:.3f} / {change_p95:.3f} / "
    f"{change_max:.3f} ms (sampled on the 60 Hz OmniGraph tick)\n"
)
try:
    performance_log_path.parent.mkdir(parents=True, exist_ok=True)
    performance_log_path.open("a", encoding="utf-8").write(performance_record)
except OSError as exc:
    print(f"[PERF WARNING] cannot append {performance_log_path}: {exc}", flush=True)

if joystick_input is not None:
    joystick_input.close()
if input_interface is not None and keyboard_subscription is not None:
    input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_subscription)
manual_ros_node.destroy_node()
if rclpy.ok():
    rclpy.shutdown()
app_utils.stop()
simulation_app.close()
