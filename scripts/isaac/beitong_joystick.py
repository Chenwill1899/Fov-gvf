#!/usr/bin/env python3
"""Linux joystick input used by the combined manual-avoidance runner."""

from __future__ import annotations

import array
import errno
import fcntl
import math
import os
import struct
import time
from pathlib import Path

from manual_control_math import desired_velocity_from_direction


def _nonnegative_env(name: str, default: float) -> float:
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


def _integer_env(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be an integer") from exc


def _sign_env(name: str, default: int) -> int:
    value = _integer_env(name, default)
    if value not in (-1, 1):
        raise SystemExit(f"{name} must be -1 or 1")
    return value


class LinuxJoystick:
    """Non-blocking reader for Linux's stable ``/dev/input/js`` API."""

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
                f"cannot open joystick {path}: {exc}; connect the BEITONG "
                "controller or set JOYSTICK_DEVICE") from exc
        self.path = path
        axis_count = array.array("B", [0])
        button_count = array.array("B", [0])
        name = array.array("B", [0] * 128)
        fcntl.ioctl(self.fd, self.JSIOCGAXES, axis_count, True)
        fcntl.ioctl(self.fd, self.JSIOCGBUTTONS, button_count, True)
        fcntl.ioctl(self.fd, self.JSIOCGNAME_128, name, True)
        self.axis_count = int(axis_count[0])
        self.button_count = int(button_count[0])
        self.name = name.tobytes().split(b"\0", 1)[0].decode(errors="replace")
        self.axes = [0] * self.axis_count
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
            _, value, event_type, number = struct.unpack("IhBB", data)
            event_type &= 0x7F
            if event_type == self.JS_EVENT_AXIS and number < self.axis_count:
                self.axes[number] = value

    def axis(self, index: int) -> float:
        return max(-1.0, min(1.0, self.axes[index] / 32767.0))

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


class BeitongMode2:
    """Validated BEITONG A2P3A Mode-2 mapping with a neutral-start gate."""

    BFM_DEVICE = Path(
        "/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick")
    XINPUT_DEVICE = Path(
        "/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_XINPUT_DONGLE-joystick")
    PROFILE_LAYOUTS = {
        "bfm": {"axes": 8, "buttons": 16, "roll": 2, "pitch": 3},
        "xinput": {"axes": 8, "buttons": 11, "roll": 3, "pitch": 4},
    }

    def __init__(self) -> None:
        configured_path = os.environ.get("JOYSTICK_DEVICE")
        if configured_path:
            self.path = Path(configured_path)
        elif self.XINPUT_DEVICE.exists():
            self.path = self.XINPUT_DEVICE
        else:
            self.path = self.BFM_DEVICE
        self.horizontal_speed = _nonnegative_env("JOYSTICK_HORIZONTAL_SPEED", 2.0)
        self.vertical_speed = _nonnegative_env("JOYSTICK_VERTICAL_SPEED", 1.5)
        self.max_yaw_rate = math.radians(
            _nonnegative_env("JOYSTICK_YAW_RATE_DEG", 75.0))
        self.deadzone = _nonnegative_env("JOYSTICK_DEADZONE", 0.08)
        self.neutral_hold = _nonnegative_env("JOYSTICK_NEUTRAL_HOLD", 0.5)
        if self.deadzone >= 1.0:
            raise SystemExit("JOYSTICK_DEADZONE must be less than 1.0")
        self.device = LinuxJoystick(self.path)
        requested_profile = os.environ.get("JOYSTICK_PROFILE", "").strip().lower()
        if requested_profile:
            self.profile = requested_profile
        elif (self.device.axis_count, self.device.button_count) == (8, 11):
            self.profile = "xinput"
        elif (self.device.axis_count, self.device.button_count) == (8, 16):
            self.profile = "bfm"
        else:
            self.close()
            raise SystemExit(
                "cannot infer BEITONG profile from joystick dimensions "
                f"{self.device.axis_count} axes/{self.device.button_count} buttons")
        if self.profile not in self.PROFILE_LAYOUTS:
            self.close()
            raise SystemExit("JOYSTICK_PROFILE must be bfm or xinput")
        layout = self.PROFILE_LAYOUTS[self.profile]
        if (self.device.axis_count, self.device.button_count) != (
                layout["axes"], layout["buttons"]):
            self.close()
            raise SystemExit(
                f"profile {self.profile} expects {layout['axes']} axes/"
                f"{layout['buttons']} buttons, got {self.device.axis_count}/"
                f"{self.device.button_count}")
        self.axis_indices = {
            "roll": _integer_env("JOYSTICK_AXIS_ROLL", layout["roll"]),
            "pitch": _integer_env("JOYSTICK_AXIS_PITCH", layout["pitch"]),
            "throttle": _integer_env("JOYSTICK_AXIS_THROTTLE", 1),
            "yaw": _integer_env("JOYSTICK_AXIS_YAW", 0),
        }
        self.axis_signs = {
            "roll": _sign_env("JOYSTICK_SIGN_ROLL", 1),
            "pitch": _sign_env("JOYSTICK_SIGN_PITCH", -1),
            "throttle": _sign_env("JOYSTICK_SIGN_THROTTLE", -1),
            "yaw": _sign_env("JOYSTICK_SIGN_YAW", -1),
        }
        if len(set(self.axis_indices.values())) != 4:
            self.close()
            raise SystemExit("the four JOYSTICK_AXIS_* values must be different")
        for channel, index in self.axis_indices.items():
            if not 0 <= index < self.device.axis_count:
                self.close()
                raise SystemExit(
                    f"{channel} axis {index} is outside "
                    f"0..{self.device.axis_count - 1}")
        self.enabled = False
        self.neutral_since: float | None = None
        self.disconnect_reported = False
        print(
            f"[JOYSTICK] device={self.path} name={self.device.name!r} "
            f"profile={self.profile} axes={self.device.axis_count} "
            f"buttons={self.device.button_count}",
            flush=True,
        )
        print(
            "[MODE 2 + AVOIDANCE] left-Y=throttle left-X=yaw "
            "right-Y=pitch right-X=roll; waiting for neutral",
            flush=True,
        )

    def _deadzone(self, value: float) -> float:
        magnitude = abs(value)
        if magnitude <= self.deadzone:
            return 0.0
        return math.copysign(
            (magnitude - self.deadzone) / (1.0 - self.deadzone), value)

    def sample(self, yaw: float) -> tuple[tuple[float, float, float], float, bool, dict[str, float]]:
        """Return world-frame translation, yaw rate, active flag, and raw channels."""
        self.device.poll()
        if not self.device.connected:
            if not self.disconnect_reported:
                print("[SAFETY] joystick disconnected; all motion stopped", flush=True)
                self.disconnect_reported = True
            channels = {name: 0.0 for name in self.axis_indices}
            self.enabled = False
        else:
            channels = {
                name: self._deadzone(
                    self.device.axis(self.axis_indices[name]) * self.axis_signs[name])
                for name in self.axis_indices
            }
            if not self.enabled:
                if max(abs(value) for value in channels.values()) == 0.0:
                    if self.neutral_since is None:
                        self.neutral_since = time.monotonic()
                    elif time.monotonic() - self.neutral_since >= self.neutral_hold:
                        self.enabled = True
                        print("[SAFETY] controls enabled after neutral hold", flush=True)
                else:
                    self.neutral_since = None

        if not self.enabled:
            return (0.0, 0.0, 0.0), 0.0, False, channels
        forward = channels["pitch"]
        left = -channels["roll"]
        body_forward, body_left, body_vertical = desired_velocity_from_direction(
            forward, left, channels["throttle"],
            self.horizontal_speed, self.vertical_speed)
        c, s = math.cos(yaw), math.sin(yaw)
        translation = (
            c * body_forward - s * body_left,
            s * body_forward + c * body_left,
            body_vertical,
        )
        yaw_rate = self.max_yaw_rate * channels["yaw"]
        active = any(abs(value) > 1.0e-6 for value in channels.values())
        return translation, yaw_rate, active, channels

    def close(self) -> None:
        self.device.close()
