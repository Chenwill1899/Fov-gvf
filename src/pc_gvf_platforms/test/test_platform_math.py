import math

from pc_gvf_platforms.command_bridge import convert
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


def test_joy_deadzone_and_scaling():
    assert map_axes([0.0, 0.1], 1, 0, 1.0, 1.0, 0.15, 1.0) == (0.0, 0.0)
    assert map_axes([0.0, 1.0], 1, 0, 1.0, 1.0, 0.15, 2.0) == (2.0, 0.0)


def test_trace_validation(tmp_path):
    path = tmp_path / "trace.yaml"
    path.write_text("events:\n  - {at: 0, forward: 1, lateral: 0}\n  - {at: 1, forward: 0, lateral: 0}\n")
    events, _ = load_trace(path)
    assert events[-1] == (1.0, 0.0, 0.0)
