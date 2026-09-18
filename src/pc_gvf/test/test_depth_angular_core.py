import numpy as np

from pc_gvf.depth_angular_core import SimConfig, disk_mask, make_scenes, solve_angular_harmonic


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
