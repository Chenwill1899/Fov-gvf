import numpy as np

from pc_gvf.depth_angular_controller import DepthAngularController, quaternion_matrix
from pc_gvf.depth_angular_core import SimConfig, disk_mask, make_scenes, solve_angular_harmonic


def test_quaternion_matrix_identity():
    np.testing.assert_allclose(quaternion_matrix(0.0, 0.0, 0.0, 1.0), np.eye(3))


def test_depth_decoder_rejects_unknown_encoding():
    class Image:
        encoding = "rgb8"

    try:
        DepthAngularController.decode_depth(Image())
    except ValueError as exc:
        assert "32FC1" in str(exc)
    else:
        raise AssertionError("invalid encoding accepted")


def test_harmonic_field_respects_mask():
    mask = np.zeros((12, 16), dtype=bool)
    mask[3:9, 8] = True
    cfg = SimConfig()
    source, goal = np.array([2.0, 6.0]), np.array([13.0, 6.0])
    phi, valid = solve_angular_harmonic(mask, source, goal, cfg)
    assert valid
    assert np.all(np.isnan(phi[mask]))
    assert np.any(np.isfinite(phi[disk_mask(mask.shape, source, cfg.source_radius_cells)]))


def test_migrated_scenarios_are_available():
    assert {"empty", "single_pillar", "offset_box", "center_sphere", "overhead_bar",
            "diagonal_gap", "narrow_gate"} <= make_scenes().keys()
