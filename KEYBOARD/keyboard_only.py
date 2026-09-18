#!/usr/bin/env python3
"""Standalone Mode-2 USB joystick test: Isaac Sim only, no ROS/avoidance."""

from __future__ import annotations

import array
import errno
import fcntl
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


if len(sys.argv) != 2:
    raise SystemExit("usage: keyboard_only.py FLAT_GROUND_SCENE.usd")
scene_path = Path(sys.argv[1]).expanduser().resolve()
if not scene_path.is_file():
    raise SystemExit(f"scene does not exist: {scene_path}")


def nonnegative_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be a finite nonnegative number") from exc
    if not math.isfinite(value) or value < 0.0:
        raise SystemExit(f"{name} must be a finite nonnegative number")
    return value


def integer_env(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be an integer") from exc


def sign_env(name: str, default: int) -> int:
    value = integer_env(name, default)
    if value not in (-1, 1):
        raise SystemExit(f"{name} must be -1 or 1")
    return value


class LinuxJoystick:
    """Small non-blocking reader for Linux's stable /dev/input/js API."""

    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02
    JSIOCGAXES = 0x80016A11
    JSIOCGBUTTONS = 0x80016A12
    JSIOCGNAME_128 = 0x80806A13

    def __init__(self, path: Path) -> None:
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            raise SystemExit(
                f"cannot open joystick {path}: {exc}; connect the controller "
                "or set JOYSTICK_DEVICE") from exc
        self.path = path
        axis_count = array.array("B", [0])
        button_count = array.array("B", [0])
        name = array.array("B", [0] * 128)
        fcntl.ioctl(self.fd, self.JSIOCGAXES, axis_count, True)
        fcntl.ioctl(self.fd, self.JSIOCGBUTTONS, button_count, True)
        fcntl.ioctl(self.fd, self.JSIOCGNAME_128, name, True)
        self.axis_count = int(axis_count[0])
        self.button_count = int(button_count[0])
        self.name = name.tobytes().split(b"\0", 1)[0].decode(
            errors="replace")
        self.axes = [0] * self.axis_count
        self.buttons = [0] * self.button_count
        self.connected = True
        self.poll()

    def poll(self) -> None:
        while self.connected:
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.ENODEV, errno.EIO, errno.EBADF):
                    self.connected = False
                    return
                raise
            if not data:
                self.connected = False
                return
            if len(data) != 8:
                continue
            _, event_value, event_type, number = struct.unpack("IhBB", data)
            event_type &= 0x7F  # Ignore the initialization flag.
            if event_type == self.JS_EVENT_AXIS and number < self.axis_count:
                self.axes[number] = event_value
            elif (event_type == self.JS_EVENT_BUTTON and
                  number < self.button_count):
                self.buttons[number] = event_value

    def axis(self, index: int) -> float:
        return max(-1.0, min(1.0, self.axes[index] / 32767.0))

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


joystick_path = Path(os.environ.get(
    "JOYSTICK_DEVICE",
    "/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick"))
horizontal_speed = nonnegative_env("JOYSTICK_HORIZONTAL_SPEED", 2.0)
vertical_speed = nonnegative_env("JOYSTICK_VERTICAL_SPEED", 1.5)
yaw_rate_max = math.radians(nonnegative_env("JOYSTICK_YAW_RATE_DEG", 75.0))
deadzone = nonnegative_env("JOYSTICK_DEADZONE", 0.08)
neutral_hold = nonnegative_env("JOYSTICK_NEUTRAL_HOLD", 0.5)
test_timeout = nonnegative_env(
    "JOYSTICK_TEST_TIMEOUT", nonnegative_env("KEYBOARD_TEST_TIMEOUT", 0.0))
floor_z = nonnegative_env(
    "JOYSTICK_FLOOR_Z", nonnegative_env("KEYBOARD_FLOOR_Z", 0.0))
body_radius = 0.25
if deadzone >= 1.0:
    raise SystemExit("JOYSTICK_DEADZONE must be less than 1.0")

axis_indices = {
    "roll": integer_env("JOYSTICK_AXIS_ROLL", 2),
    "pitch": integer_env("JOYSTICK_AXIS_PITCH", 3),
    "throttle": integer_env("JOYSTICK_AXIS_THROTTLE", 1),
    "yaw": integer_env("JOYSTICK_AXIS_YAW", 0),
}
axis_signs = {
    "roll": sign_env("JOYSTICK_SIGN_ROLL", 1),
    "pitch": sign_env("JOYSTICK_SIGN_PITCH", -1),
    "throttle": sign_env("JOYSTICK_SIGN_THROTTLE", -1),
    "yaw": sign_env("JOYSTICK_SIGN_YAW", -1),
}
joystick = LinuxJoystick(joystick_path)
if len(set(axis_indices.values())) != 4:
    joystick.close()
    raise SystemExit("the four JOYSTICK_AXIS_* values must be different")
for channel, axis_index in axis_indices.items():
    if not 0 <= axis_index < joystick.axis_count:
        joystick.close()
        raise SystemExit(
            f"{channel} axis {axis_index} is outside 0..{joystick.axis_count - 1}")
print(
    f"[JOYSTICK] device={joystick.path} name={joystick.name!r} "
    f"axes={joystick.axis_count} buttons={joystick.button_count}",
    flush=True,
)
print(
    "[MODE 2] left-Y=throttle left-X=yaw "
    "right-Y=pitch right-X=roll; waiting for neutral",
    flush=True,
)

# Keep Kit from interpreting the scene argument as one of its own options.
sys.argv = [sys.argv[0]]
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 1,
    "active_gpu": 0,
    "multi_gpu": False,
})

import carb
import carb.settings
import omni.kit.viewport.utility as viewport_utility
import omni.usd
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
import isaacsim.core.experimental.utils.app as app_utils
from pxr import Gf, UsdGeom, UsdLux


context = omni.usd.get_context()
if not context.open_stage(scene_path.as_posix()):
    simulation_app.close()
    raise RuntimeError(f"failed to open {scene_path}")
simulation_app.reset_render_settings()
render_settings = carb.settings.get_settings()
render_settings.set("/rtx/post/histogram/enabled", False)
render_settings.set("/rtx/sceneDb/ambientLightColor", (1.0, 1.0, 1.0))
render_settings.set("/rtx/sceneDb/ambientLightIntensity", 0.01)
stage = context.get_stage()

robot = stage.GetPrimAtPath("/World/Robot")
if not robot.IsValid():
    simulation_app.close()
    raise RuntimeError("keyboard test scene is missing /World/Robot")

try:
    metadata = json.loads(
        stage.GetRootLayer().customLayerData.get("fovNavigationJson", "{}"))
except (TypeError, json.JSONDecodeError):
    metadata = {}
spawn = metadata.get("spawn", [0.0, 0.0, 1.5])
if not isinstance(spawn, list) or len(spawn) != 3:
    simulation_app.close()
    raise RuntimeError("scene spawn metadata must contain three values")

position = Gf.Vec3d(*(float(value) for value in spawn))
position[2] = max(float(position[2]), floor_z + body_radius)
yaw = 0.0
robot_api = UsdGeom.XformCommonAPI(robot)
robot_api.SetTranslate(position)

sun_intensity = 2.5
sun_count = 0
for prim in stage.Traverse():
    if prim.IsA(UsdLux.DistantLight):
        light = UsdLux.DistantLight(prim)
        light.GetIntensityAttr().Set(sun_intensity)
        light.CreateAngleAttr(0.53)
        sun_count += 1
if sun_count == 0:
    sun = UsdLux.DistantLight.Define(stage, "/World/KeyboardTestSun")
    sun.CreateIntensityAttr(sun_intensity)
    sun.CreateAngleAttr(0.53)
    UsdGeom.XformCommonAPI(sun).SetRotate(
        Gf.Vec3f(-45.0, 0.0, 35.0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )


def reference_cube(
        name: str,
        translate: tuple[float, float, float],
        scale: tuple[float, float, float],
        color: tuple[float, float, float]) -> None:
    cube = UsdGeom.Cube.Define(stage, f"/World/KeyboardReferences/{name}")
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.CreateDisplayOpacityAttr([1.0])
    api = UsdGeom.XformCommonAPI(cube)
    api.SetTranslate(Gf.Vec3d(*translate))
    api.SetScale(Gf.Vec3f(*scale))


# Fixed, non-physical ground markers make translation and world-axis direction
# visible in the chase view without adding obstacles to the keyboard-only test.
UsdGeom.Xform.Define(stage, "/World/KeyboardReferences")
reference_cube("CenterPad", (0.0, 0.0, 0.025), (1.20, 1.20, 0.05),
               (0.08, 0.09, 0.11))
reference_cube("XAxis", (0.0, 0.0, 0.055), (5.80, 0.12, 0.06),
               (0.78, 0.16, 0.08))
reference_cube("YAxis", (0.0, 0.0, 0.060), (0.12, 5.80, 0.07),
               (0.08, 0.62, 0.20))
reference_cube("PositiveX", (3.20, 0.0, 0.16), (0.48, 0.48, 0.32),
               (1.00, 0.32, 0.05))
reference_cube("NegativeX", (-3.20, 0.0, 0.16), (0.48, 0.48, 0.32),
               (0.08, 0.32, 0.95))
reference_cube("PositiveY", (0.0, 3.20, 0.16), (0.48, 0.48, 0.32),
               (0.10, 0.85, 0.24))
reference_cube("NegativeY", (0.0, -3.20, 0.16), (0.48, 0.48, 0.32),
               (0.82, 0.10, 0.72))
print(
    "[SCENE] sun=2.5; ground references: +X=orange -X=blue "
    "+Y=green -Y=magenta",
    flush=True,
)

ready, waited_frames = ViewportManager.wait_for_viewport(
    max_frames=120, sleep_time=0.02)
viewport = viewport_utility.get_active_viewport()
if not ready or viewport is None:
    simulation_app.close()
    raise RuntimeError(
        f"Isaac viewport did not become ready after {waited_frames} frames")

third_person_camera = "/World/Robot/ThirdPersonCamera"
ViewportManager.set_camera(third_person_camera, render_product_or_viewport=viewport)
print(f"[VIEW] third camera={third_person_camera}", flush=True)
screenshot_path_raw = os.environ.get(
    "JOYSTICK_SCREENSHOT", os.environ.get("KEYBOARD_SCREENSHOT", "")).strip()
screenshot_path = (
    Path(screenshot_path_raw).expanduser().resolve()
    if screenshot_path_raw else None
)
if screenshot_path is not None:
    screenshot_path.parent.mkdir(parents=True, exist_ok=True)


def apply_deadzone(value: float) -> float:
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign((magnitude - deadzone) / (1.0 - deadzone), value)


def sample_channels() -> dict[str, float]:
    joystick.poll()
    if not joystick.connected:
        return {name: 0.0 for name in axis_indices}
    return {
        name: apply_deadzone(
            joystick.axis(axis_indices[name]) * axis_signs[name])
        for name in axis_indices
    }


print("[JOYSTICK ONLY] ROS=off avoidance=off ESDF=off depth-control=off", flush=True)
print("[SAFETY] center all four sticks for 0.5 s; USB disconnect stops motion", flush=True)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()

frame = 0
capture_requested = False
control_ready = False
neutral_since: float | None = None
last_channel_log: dict[str, float] | None = None
last_channel_log_time = -1.0
result = "USER_EXIT"
try:
    while simulation_app.is_running() and app_utils.is_playing():
        simulation_app.update()
        now = frame / 60.0
        dt = 1.0 / 60.0

        if screenshot_path is not None and frame == 90:
            viewport_utility.capture_viewport_to_file(
                viewport, file_path=screenshot_path.as_posix())
            capture_requested = True
            print(f"[SCREENSHOT] requested {screenshot_path}", flush=True)

        channels = sample_channels()
        if not joystick.connected:
            if result != "JOYSTICK_DISCONNECTED":
                print("[SAFETY] joystick disconnected; motion stopped", flush=True)
            result = "JOYSTICK_DISCONNECTED"
        elif not control_ready:
            if max(abs(value) for value in channels.values()) == 0.0:
                if neutral_since is None:
                    neutral_since = time.monotonic()
                elif time.monotonic() - neutral_since >= neutral_hold:
                    control_ready = True
                    print("[SAFETY] controls enabled after neutral hold", flush=True)
            else:
                neutral_since = None

        if (last_channel_log is None or
                max(abs(channels[name] - last_channel_log[name])
                    for name in channels) >= 0.05) and now - last_channel_log_time >= 0.10:
            print(
                f"[STICKS] roll={channels['roll']:+.2f} "
                f"pitch={channels['pitch']:+.2f} "
                f"throttle={channels['throttle']:+.2f} "
                f"yaw={channels['yaw']:+.2f} "
                f"enabled={'yes' if control_ready else 'no'}",
                flush=True,
            )
            last_channel_log = channels.copy()
            last_channel_log_time = now

        forward = channels["pitch"] if control_ready else 0.0
        left = -channels["roll"] if control_ready else 0.0
        planar_norm = math.hypot(forward, left)
        if planar_norm > 1.0:
            forward /= planar_norm
            left /= planar_norm
        body_forward_velocity = horizontal_speed * forward
        body_left_velocity = horizontal_speed * left
        world_x_velocity = (
            math.cos(yaw) * body_forward_velocity -
            math.sin(yaw) * body_left_velocity)
        world_y_velocity = (
            math.sin(yaw) * body_forward_velocity +
            math.cos(yaw) * body_left_velocity)
        vertical_velocity = (
            vertical_speed * channels["throttle"] if control_ready else 0.0)
        yaw_velocity = yaw_rate_max * channels["yaw"] if control_ready else 0.0

        candidate = Gf.Vec3d(
            float(position[0]) + world_x_velocity * dt,
            float(position[1]) + world_y_velocity * dt,
            max(floor_z + body_radius,
                float(position[2]) + vertical_velocity * dt),
        )
        position = candidate
        yaw = math.atan2(math.sin(yaw + yaw_velocity * dt),
                         math.cos(yaw + yaw_velocity * dt))

        robot_api.SetTranslate(position)
        robot_api.SetRotate(
            Gf.Vec3f(0.0, 0.0, math.degrees(yaw)),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )

        if frame % 120 == 0:
            print(
                f"[JOYSTICK] t={now:.1f}s "
                f"position=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) "
                f"yaw={math.degrees(yaw):.1f}deg "
                f"enabled={'yes' if control_ready else 'no'}",
                flush=True,
            )

        frame += 1
        if test_timeout > 0.0 and now >= test_timeout:
            result = "TEST_TIMEOUT"
            break
finally:
    if capture_requested:
        for _ in range(30):
            if screenshot_path.is_file():
                break
            simulation_app.update()
        print(
            f"[SCREENSHOT] {'saved' if screenshot_path.is_file() else 'failed'} "
            f"{screenshot_path}",
            flush=True,
        )
    print(
        f"[JOYSTICK RESULT] {result} frames={frame} "
        f"position=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})",
        flush=True,
    )
    joystick.close()
    app_utils.stop()
    simulation_app.close()
