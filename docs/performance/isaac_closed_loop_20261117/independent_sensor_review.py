#!/usr/bin/env python3
"""Additional offline sensor/physics evidence checks; never launches Isaac."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def review(folder, visual_note=None):
    simulation = folder / "simulation"
    trajectory = list(csv.DictReader((simulation / "trajectory.csv").open()))
    frames = list(csv.DictReader((simulation / "sensor_frames.csv").open()))
    calibration = json.loads((simulation / "simulation.sensors.json").read_text())
    result = json.loads((simulation / "result.json").read_text())
    names = {"Front", "Left", "Back", "Right"}
    groups = {}
    for frame in frames:
        groups.setdefault(int(frame["sequence"]), []).append(frame)
    complete = (len(frames) == 4 * (len(trajectory)-1)
                and set(groups) == set(range(1, len(trajectory)))
                and all(len(group) == 4 and {r["camera"] for r in group} == names
                        for group in groups.values()))
    clock_error = reference_error = 0.0
    finite_clocks = request_stamps = True
    changed = {name: 0 for name in sorted(names)}
    moving_comparisons = 0
    for sequence, group in groups.items():
        if not 1 <= sequence < len(trajectory):
            finite_clocks = False
            continue
        expected = float(trajectory[sequence-1]["physics_t"])
        for frame in group:
            clocks = [float(frame[key]) for key in
                      ("kit_time_s", "physics_time_s", "rendered_simulation_s")]
            numerator, denominator = int(frame["reference_numerator"]), int(frame["reference_denominator"])
            finite_clocks &= all(math.isfinite(value) for value in clocks+[expected]) and denominator > 0
            if finite_clocks:
                clock_error = max(clock_error, *(abs(value-expected) for value in clocks))
                reference_error = max(reference_error, abs(numerator/denominator-expected))
            request_stamps &= int(frame["request_simulation_ns"]) == (sequence-1)*5000000
        if sequence > 1:
            current = [float(trajectory[sequence-1][key]) for key in ("x", "y", "z")]
            previous = [float(trajectory[sequence-2][key]) for key in ("x", "y", "z")]
            if math.dist(current, previous) > 1e-10:
                moving_comparisons += 1
                previous_hash = {r["camera"]: r["raw_sha256"] for r in groups.get(sequence-1, [])}
                for frame in group:
                    if frame["camera"] in changed:
                        changed[frame["camera"]] += previous_hash.get(frame["camera"]) != frame["raw_sha256"]
    physics_delta_error = max(abs(float(b["physics_t"])-float(a["physics_t"])-.005)
                              for a, b in zip(trajectory, trajectory[1:]))
    count_ok = all(int(b["physics_step"]) == int(a["physics_step"])+1
                   for a, b in zip(trajectory, trajectory[1:]))
    overlay_paths = [simulation / f"overlay_{name}_depth.bin" for name in ("off_before", "on", "off_after")]
    overlay = [path.read_bytes() for path in overlay_paths]
    expected_bytes = sum(camera["width"]*camera["height"]*4 for camera in calibration["cameras"])
    overlay_complete = all(len(raw) == expected_bytes for raw in overlay)
    cal_ok = len(calibration["cameras"]) == 4 and all(
        camera["cx"] == (camera["width"]-1)/2 and camera["cy"] == (camera["height"]-1)/2
        and camera["actual_sensor_tick_rate_hz"] == 200
        and camera["actual_sensor_frame_rate"] == [200, 1]
        for camera in calibration["cameras"])
    checks = dict(exact_four_camera_identities_each_request=complete,
                  all_logged_sensor_clocks_absolute_match_physics=finite_clocks and clock_error <= 1e-7,
                  reference_rational_absolute_matches_physics=finite_clocks and reference_error <= 1e-7,
                  request_stamps_match_5ms=request_stamps,
                  each_actual_physics_delta_is_5ms=physics_delta_error <= 1e-8,
                  each_actual_physics_step_count_increments_once=count_ok,
                  overlay_raw_buffers_complete=overlay_complete,
                  overlay_raw_buffers_bitwise_identical=overlay_complete and overlay[0] == overlay[1] == overlay[2],
                  actual_integer_pixel_calibration_and_200hz=cal_ok)
    evidence = dict(trajectory_rows=len(trajectory), sensor_rows=len(frames),
                    maximum_absolute_logged_clock_error_s=clock_error,
                    maximum_reference_rational_error_s=reference_error,
                    maximum_actual_physics_delta_error_s=physics_delta_error,
                    moving_frame_comparisons=moving_comparisons,
                    full_image_sha_changes_during_position_motion=changed,
                    hash_change_interpretation="Camera depths may legitimately remain unchanged while moving parallel to visible planes; counts are evidence, not a universal pass condition.",
                    actual_renderer=result["actual_render_settings"]["/rtx/rendermode"],
                    headless=result["headless"], time_scale=result["time_scale"],
                    time_scale_override=result["time_scale_override"],
                    achieved_simulation_wall_ratio=result["achieved_simulation_wall_ratio"],
                    overlay_bytes_per_buffer=expected_bytes)
    inputs = [simulation / name for name in ("trajectory.csv", "sensor_frames.csv", "simulation.sensors.json", "result.json")]+overlay_paths
    capture = simulation / "capture.png"
    if not result["headless"]:
        from PIL import Image
        image = Image.open(capture).convert("RGB")
        pixels = list(image.getdata())
        nonblack = sum(max(pixel) > 4 for pixel in pixels)/len(pixels)
        lo, hi = min(map(min, pixels)), max(map(max, pixels))
        checks["saved_gui_capture_has_visible_rgb"] = nonblack > .01 and hi-lo >= 8
        evidence["capture"] = dict(path=str(capture), dimensions=list(image.size), minimum=lo,
                                    maximum=hi, nonblack_fraction=nonblack,
                                    manual_visual_review_note=visual_note)
        inputs.append(capture)
    return dict(passed=all(checks.values()), checks=checks, failed=[key for key, value in checks.items() if not value],
                evidence=evidence, input_sha256={str(path.relative_to(folder)): sha(path) for path in inputs},
                review_script_sha256=sha(Path(__file__)),
                scope="Additional actual sensor-clock/physics-step/full-overlay-byte review. Per-frame world-depth truth remains the separate 100-pixel sample audit; this does not assert full-frame ground truth, rigid-body flight or real-time controller performance.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--visual-review-note")
    args = parser.parse_args()
    folder = args.run.resolve()
    report = review(folder, args.visual_review_note)
    (folder / "independent_sensor_review.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"passed": report["passed"], "failed": report["failed"], "run": str(folder)}))
    raise SystemExit(0 if report["passed"] else 1)
