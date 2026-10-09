import math

from pc_gvf_platforms.command_bridge import convert, command_stamp_is_fresh
from pc_gvf_platforms.intent_trace_replay import load_trace
from pc_gvf_platforms.joy_to_intent import map_axes


def test_diff_drive_turns_toward_world_command():
    twist = convert(0.0, 1.0, 0.0, "diff_drive", 1.0, 1.0, 2.0, 1.0, 0.01)
    assert twist.linear.x < 1e-9
    assert math.isclose(twist.angular.z, math.pi / 2.0)


def test_holonomic_command_rotates_into_body():
    twist = convert(1.0, 0.0, math.pi / 2.0, "holonomic", 2.0, 2.0, 2.0, 1.0, 0.01)
    assert abs(twist.linear.x) < 1e-9
    assert math.isclose(twist.linear.y, -1.0)


def test_holonomic_preserves_clamped_vertical_velocity():
    twist = convert(
        0.0, 0.0, 0.0, "holonomic", 2.0, 2.0, 2.0, 1.0, 0.01,
        vz=1.4, max_vz=1.0)
    assert math.isclose(twist.linear.z, 1.0)


def test_joy_deadzone_and_scaling():
    assert map_axes([0.0, 0.1], 1, 0, 1.0, 1.0, 0.15, 1.0) == (0.0, 0.0)
    assert map_axes([0.0, 1.0], 1, 0, 1.0, 1.0, 0.15, 2.0) == (2.0, 0.0)


def test_trace_validation(tmp_path):
    path = tmp_path / "trace.yaml"
    path.write_text("events:\n  - {at: 0, forward: 1, lateral: 0}\n  - {at: 1, forward: 0, lateral: 0}\n")
    events, _ = load_trace(path)
    assert events[-1] == (1.0, 0.0, 0.0)


def test_holonomic_limits_scale_entire_vector():
    twist = convert(3.0, 1.5, 0.0, "holonomic", 2.0, 2.0, 0.0, 1.0, 0.01,
                    vz=2.0, max_vz=1.0)
    assert (twist.linear.x, twist.linear.y, twist.linear.z) == (1.5, 0.75, 1.0)


def test_bridge_rejects_delayed_and_future_commands():
    assert command_stamp_is_fresh(1_050_000_000, 1_000_000_000, .1)
    assert not command_stamp_is_fresh(1_110_000_000, 1_000_000_000, .1)
    assert not command_stamp_is_fresh(900_000_000, 1_000_000_000, .1)
