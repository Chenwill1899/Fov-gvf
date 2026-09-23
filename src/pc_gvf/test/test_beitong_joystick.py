#!/usr/bin/env python3

import os
import unittest
from unittest import mock

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "isaac"))
import beitong_joystick


class FakeJoystick:
    def __init__(self, axis_count: int, button_count: int) -> None:
        self.axis_count = axis_count
        self.button_count = button_count
        self.name = "fake BEITONG"
        self.connected = True
        self.closed = False
        self.axis_values = [0.0] * axis_count

    def poll(self) -> None:
        pass

    def axis(self, index: int) -> float:
        return self.axis_values[index]

    def close(self) -> None:
        self.closed = True


class BeitongProfileTests(unittest.TestCase):
    def make_controller(self, axis_count, button_count, profile=None):
        fake = FakeJoystick(axis_count, button_count)
        environment = {"JOYSTICK_DEVICE": "/dev/input/js-test"}
        if profile is not None:
            environment["JOYSTICK_PROFILE"] = profile
        with mock.patch.dict(os.environ, environment, clear=True), mock.patch.object(
                beitong_joystick, "LinuxJoystick", return_value=fake):
            controller = beitong_joystick.BeitongMode2()
        return controller, fake

    def test_xinput_profile_uses_right_stick_axes_not_trigger(self):
        controller, fake = self.make_controller(8, 11, "xinput")
        self.addCleanup(controller.close)
        self.assertEqual(controller.profile, "xinput")
        self.assertEqual(
            controller.axis_indices,
            {"roll": 3, "pitch": 4, "throttle": 1, "yaw": 0},
        )
        fake.axis_values[2] = 1.0
        with mock.patch.object(
                beitong_joystick.time, "monotonic", side_effect=[10.0, 10.6]):
            first = controller.sample(yaw=0.0)
            second = controller.sample(yaw=0.0)
        self.assertFalse(first[2])
        self.assertTrue(controller.enabled)
        self.assertFalse(second[2])
        self.assertEqual(second[3], {
            "roll": 0.0, "pitch": 0.0, "throttle": 0.0, "yaw": 0.0,
        })

    def test_bfm_profile_keeps_validated_legacy_layout(self):
        controller, _ = self.make_controller(8, 16, "bfm")
        self.addCleanup(controller.close)
        self.assertEqual(controller.profile, "bfm")
        self.assertEqual(
            controller.axis_indices,
            {"roll": 2, "pitch": 3, "throttle": 1, "yaw": 0},
        )

    def test_profile_is_inferred_from_known_dimensions(self):
        controller, _ = self.make_controller(8, 11)
        self.addCleanup(controller.close)
        self.assertEqual(controller.profile, "xinput")

    def test_mismatched_profile_dimensions_fail_closed(self):
        fake = FakeJoystick(8, 16)
        with mock.patch.dict(
                os.environ,
                {
                    "JOYSTICK_DEVICE": "/dev/input/js-test",
                    "JOYSTICK_PROFILE": "xinput",
                    "JOYSTICK_EXPECTED_BUTTONS": "16",
                },
                clear=True,
        ), mock.patch.object(beitong_joystick, "LinuxJoystick", return_value=fake):
            with self.assertRaisesRegex(SystemExit, "expects 8 axes/11 buttons"):
                beitong_joystick.BeitongMode2()
        self.assertTrue(fake.closed)


if __name__ == "__main__":
    unittest.main()
