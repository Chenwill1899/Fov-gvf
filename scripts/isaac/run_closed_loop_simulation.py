#!/usr/bin/env python3
"""Current P3 controller IPC client with shared GUI/headless Isaac simulation.

This process never starts a controller and never replays a recorded trajectory.
The plant is the existing first-order kinematic model, not rigid-body dynamics.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import time

from closed_loop_scene import VIEWS, build_scene, read_scenario, swept_clearance
from depth_camera_projection import configure_depth_projection, save_projection_evidence
from manual_control_math import velocity_response

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from base_config import load_config

MAGIC = 0x45474F3450330001
REQUEST = struct.Struct("<6Q15d")
REPLY = struct.Struct("<6Q12d")


class SimulationFailure(RuntimeError):
    def __init__(self, status, detail):
        super().__init__(detail)
        self.status = status


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def rotation_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)


def receive_exact(connection, size):
    result = bytearray()
    deadline = time.monotonic() + 5.0
    while len(result) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise SimulationFailure("CONTROLLER_TIMEOUT", "reply exceeded its complete-message deadline")
        connection.settimeout(remaining)
        data = connection.recv(size - len(result))
        if not data:
            raise SimulationFailure("CONTROLLER_EOF", "controller closed a reply before completion")
        result.extend(data)
    connection.settimeout(5.0)
    return bytes(result)


def parse_reply(data, sequence, stamp):
    values = REPLY.unpack(data)
    magic, version, seq, timestamp, decision, flags = values[:6]
    if (magic, version, seq, timestamp) != (MAGIC, 1, sequence, stamp):
        raise SimulationFailure("PROTOCOL_ERROR", "reply identity/sequence/time mismatch")
    if decision > 6 or flags & ~31 or not all(math.isfinite(v) for v in values[6:]):
        raise SimulationFailure("PROTOCOL_ERROR", "reply contains invalid decision/flags/nonfinite values")
    return decision, flags, values[6:]


def main():
    base, base_sha = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-overlay", action="store_true")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--time-scale", type=float, default=base["time_scale"])
    parser.add_argument("--wall-timeout", type=float, help="run-only wall timeout; initialization is bounded separately by launcher")
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    if not math.isfinite(args.time_scale) or not 0 < args.time_scale <= 1:
        parser.error("--time-scale must be finite and in (0,1]")
    scenario_path, out = args.scenario.resolve(), args.output.resolve()
    if not out.is_relative_to(ROOT) or out == ROOT:
        parser.error("all durable output must be inside EGO4P3")
    if not args.socket.is_absolute() or len(os.fsencode(args.socket)) >= 104:
        parser.error("expected a short absolute private UNIX socket path")
    scenario, scenario_sha = read_scenario(scenario_path)
    run_wall_timeout = scenario["wall_timeout_s"] if args.wall_timeout is None else args.wall_timeout
    if not math.isfinite(run_wall_timeout) or run_wall_timeout <= 0:
        parser.error("--wall-timeout must be finite and positive")
    for key, expected in (("physics_dt_s", base["physics_dt_s"]), ("body_radius_m", base["body_radius_m"]),
                          ("cruise_speed_mps", base["max_speed_mps"])):
        if scenario[key] != expected:
            parser.error(key + " does not match current base configuration")
    out.mkdir(parents=True, exist_ok=False)
    (out / "scenario.json").write_bytes(scenario_path.read_bytes())
    dt = scenario["physics_dt_s"]
    limit = min(args.steps or math.ceil(scenario["simulation_timeout_s"] / dt),
                math.ceil(scenario["simulation_timeout_s"] / dt))
    requested_bounded = args.steps is not None and args.steps <= math.ceil(scenario["simulation_timeout_s"] / dt)
    initial_clearance, nearest = swept_clearance(scenario["start"], scenario["start"], scenario["seed_radius_m"], scenario)
    result = dict(schema=1, scenario=str(scenario_path), scenario_sha256=scenario_sha,
                  base_config=base, base_config_sha256=base_sha, static_scene=True,
                  status="INITIALIZING", success=False, arrived=False, completed_steps=0,
                  headless=args.headless, overlay_enabled=not args.no_overlay,
                  physics_dt_s=dt, time_scale_requested=args.time_scale,
                  runtime_wall_timeout_s=run_wall_timeout, initialization_timeout_s=scenario["wall_timeout_s"],
                  time_scale_default=base["time_scale"], time_scale_override=args.time_scale != base["time_scale"],
                  pacing_policy="minimum wall interval dt/time_scale; no catch-up",
                  plant="existing first-order kinematic velocity_response",
                  pose_policy="yaw-only kinematic pose, yaw follows actual velocity at <=2rad/s",
                  body_radius_m=scenario["body_radius_m"], body_visual_radius_m=.085,
                  camera_model="four co-located pinhole cameras, actual USD calibration",
                  no_hit_conversion=False, truth_used_for="initial seed certificate and independent execution sweep only",
                  seed_certificate=dict(center=scenario["start"], radius_m=scenario["seed_radius_m"],
                                        clearance_m=initial_clearance, nearest=nearest),
                  collision_blocks=0, failures=[], controller_started_by_simulator=False,
                  simulation_motion_only=True, ros_publishers_created=0)
    app = connection = None
    sensors, products, reference_clocks, simulation_clocks = [], [], [], []
    position = list(map(float, scenario["start"]))
    velocity = [0.0, 0.0, 0.0]
    yaw = 0.0
    total_start = time.monotonic()
    motion_start = None
    initial_body_clearance, _ = swept_clearance(position, position, scenario["body_radius_m"], scenario)
    minimum_clearance = initial_body_clearance
    peak_speed = maximum_accel = distance = hold = 0.0
    code = 1
    capture = None
    try:
        shared_cache_value = os.environ.get("EGO4P3_SIM_SHARED_CACHE")
        if shared_cache_value:
            shared_cache = Path(shared_cache_value).resolve()
            if not shared_cache.is_relative_to(ROOT):
                raise SimulationFailure("CACHE_PATH_ERROR", "shared simulator cache must stay inside EGO4P3")
            target = shared_cache / "kit"
            target.mkdir(parents=True, exist_ok=True)
            (out / "kit").mkdir(exist_ok=True)
            link = out / "kit/cache"
            if link.is_symlink():
                if link.resolve() != target.resolve():
                    raise SimulationFailure("CACHE_PATH_ERROR", "existing Kit cache symlink points elsewhere")
            elif link.exists():
                raise SimulationFailure("CACHE_PATH_ERROR", "refusing to replace an existing Kit cache directory")
            else:
                link.symlink_to(target, target_is_directory=True)
            result["shared_kit_cache"] = str(link.resolve())
        else:
            result["shared_kit_cache"] = None
        sys.argv = [sys.argv[0]]
        from isaacsim import SimulationApp
        result["renderer"] = "RaytracedLighting"
        atomic_json(out / "phase.json", dict(phase="starting_kit", status=result["status"]))
        app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting",
                             "width": 1280, "height": 720, "window_width": 1280, "window_height": 720,
                             "minimal_shading_mode": 2, "anti_aliasing": 1, "active_gpu": 0,
                             "multi_gpu": False, "limit_cpu_threads": 8,
                             "extra_args": ["--portable-root", str(out / "kit"),
                                            "--/app/settings/persistent=false", "--/log/file=" + str(out / "kit.log")]})
        import numpy as np
        import carb
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
        from pxr import Gf, Usd, UsdGeom

        context = omni.usd.get_context()
        context.new_stage()
        # Opening a stage may restore stage/default renderer settings.
        app.reset_render_settings()
        stage = context.get_stage()
        robot = build_scene(stage, scenario)
        stage.GetRootLayer().Export(str(out / "scene.usda"))
        scene_sha = hashlib.sha256((out / "scene.usda").read_bytes()).hexdigest()
        result["scene_sha256"] = scene_sha
        result["visual_material"] = "opaque UsdPreviewSurface diffuse + 0.12*diffuse emissive; geometry unchanged"
        settings = carb.settings.get_settings()
        result["actual_render_settings"] = {key: settings.get(key) for key in (
            "/rtx/rendermode", "/rtx/minimal/mode", "/rtx/post/aa/op", "/rtx/post/tonemap/op",
            "/rtx/sceneDb/ambientLightIntensity")}
        stage.SetEditTarget(stage.GetSessionLayer())
        robot_api = UsdGeom.XformCommonAPI(robot)
        calibration = configure_depth_projection(stage, Usd, UsdGeom, VIEWS,
                                                 scenario["image_width"], scenario["image_height"], environment={},
                                                 run_id=out.name, scene_path=str(out / "scene.usda"))
        # RTX samples at raster x+.5/y+.5. Core ray tables use integer pixel
        # indices, so their equivalent principal point is (width-1)/2.
        # Keep this app-specific correction out of the inherited production helper.
        convention = "integer pixel indices; RTX raster centers x+0.5,y+0.5; cx=(width-1)/2,cy=(height-1)/2"
        for camera in calibration["cameras"]:
            camera["aperture_principal_point"] = [camera["cx"], camera["cy"]]
            camera["cx"], camera["cy"] = (camera["width"] - 1) * .5, (camera["height"] - 1) * .5
            camera["pixel_coordinate_convention"] = convention
            prim = stage.GetPrimAtPath(camera["prim_path"])
            camera["actual_sensor_tick_rate_hz"] = float(prim.GetAttribute("omni:sensor:tickRate").Get())
            camera["actual_sensor_frame_rate"] = list(prim.GetAttribute("omni:sensor:frameRate").Get())
            if abs(camera["actual_sensor_tick_rate_hz"] - 1.0/dt) > 1e-6 or camera["actual_sensor_frame_rate"] != [1.0/dt, 1.0]:
                raise SimulationFailure("SENSOR_RATE_ERROR", "camera rate readback does not match physical step")
        calibration.update(depth_type="optical_axis_z_m", body_frame="base_link", scene_sha256=scene_sha,
                           scenario_sha256=scenario_sha, simulation_only=True, no_hit_conversion=False,
                           camera_model="co-located center rig", base_config=base, base_config_sha256=base_sha,
                           pixel_coordinate_convention=convention, sensor_frame_rate_hz=1.0/dt)
        cal_path = save_projection_evidence(calibration, out / "simulation.json")
        SimulationManager.setup_simulation(dt=dt, device="cpu")
        RenderingManager.set_dt(dt)
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_time_codes_per_second(1.0 / dt)
        timeline.set_target_framerate(1.0 / dt)
        timeline.set_ticks_per_frame(1)
        timeline.set_end_time(scenario["simulation_timeout_s"] + 10)
        timeline.set_auto_update(False)
        carb.settings.get_settings().set("/app/runLoops/main/rateLimitEnabled", False)
        # Setup only configures dt/device. In this SDK the multi-tick renderer
        # timestamps images from real physics steps, not the timeline slider.
        # Drive steps explicitly while the timeline is paused. Do not disable
        # app/player/playSimulations globally: that also gates sensor graphs.
        timeline.pause()
        SimulationManager.initialize_physics()
        physics_origin = SimulationManager.get_simulation_time()
        physics_step_origin = SimulationManager.get_num_physics_steps()
        timeline.set_current_time(physics_origin)
        timeline.commit()
        result["engine_initialization_time_s"] = physics_origin
        result["engine_initialization_physics_steps"] = physics_step_origin
        result["physics_step_policy"] = "paused timeline; one explicit SimulationManager.step per accepted plant step, then zero-time render"
        rep.orchestrator.set_capture_on_play(False)
        products = [rep.create.render_product(view[1], (scenario["image_width"], scenario["image_height"])) for view in VIEWS]
        sensors = [rep.AnnotatorRegistry.get_annotator("distance_to_image_plane") for _ in VIEWS]
        for sensor, product in zip(sensors, products):
            sensor.attach(product)
        app_utils.enable_extension("isaacsim.core.nodes")
        reference_clocks = [rep.AnnotatorRegistry.get_annotator("ReferenceTime") for _ in VIEWS]
        simulation_clocks = [rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime") for _ in VIEWS]
        for reference_clock, simulation_clock, product in zip(reference_clocks, simulation_clocks, products):
            reference_clock.attach(product)
            simulation_clock.initialize(resetOnStop=False)
            simulation_clock.attach(product)
        def render_current():
            before_time = SimulationManager.get_simulation_time()
            before_step = SimulationManager.get_num_physics_steps()
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False, wait_for_render=True)
            if (abs(SimulationManager.get_simulation_time()-before_time) > 1e-7 or
                    SimulationManager.get_num_physics_steps() != before_step):
                raise SimulationFailure("PHYSICS_CLOCK_MISMATCH", "render advanced physical simulation")

        def advance_physics_and_render():
            before_time = SimulationManager.get_simulation_time()
            before_step = SimulationManager.get_num_physics_steps()
            SimulationManager.step(steps=1)
            after_time = SimulationManager.get_simulation_time()
            after_step = SimulationManager.get_num_physics_steps()
            if abs(after_time-before_time-dt) > 1e-7 or after_step != before_step+1:
                raise SimulationFailure("PHYSICS_CLOCK_MISMATCH", "explicit physics step did not advance exactly one dt")
            # The timeline is a display playhead in the official manual-step
            # workflow. Synchronize it to the measured physics time, never a
            # fabricated sensor timestamp. Rendered clocks are checked separately.
            timeline.set_current_time(after_time)
            timeline.commit()
            render_current()
            if abs(timeline.get_current_time()-after_time) > 1e-7:
                raise SimulationFailure("KIT_CLOCK_MISMATCH", "render changed the synchronized Kit timeline")

        for _ in range(3):
            advance_physics_and_render()

        viewport_rgb = draw = None
        def set_view():
            eye = [position[0] - 4.0, position[1] - 5.0, position[2] + 3.0]
            eye = [min(scenario["bounds_max"][i] - .4, max(scenario["bounds_min"][i] + .4, eye[i])) for i in range(3)]
            ViewportManager.set_camera_view("/World/ChaseCamera", eye=eye, target=position)
        if not args.headless:
            import omni.kit.viewport.utility as viewport_utility
            ready, waited = ViewportManager.wait_for_viewport(max_frames=120, sleep_time=.02)
            viewport = viewport_utility.get_active_viewport()
            if not ready or viewport is None:
                raise SimulationFailure("VIEWPORT_ERROR", "visible viewport did not become ready")
            set_view()
            ViewportManager.set_camera("/World/ChaseCamera", render_product_or_viewport=viewport)
            result["viewport_camera"] = str(ViewportManager.get_camera(viewport).GetPath())
            result["viewport_render_product"] = str(viewport.render_product_path)
            viewport_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            viewport_rgb.attach([viewport.render_product_path])
            result["viewport_ready_frames"] = waited
        # Load in both modes so the same-pose depth A/B can actually toggle
        # the overlay before restoring the requested display mode.
        app_utils.enable_extension("isaacsim.util.debug_draw")
        from isaacsim.util.debug_draw import _debug_draw
        draw = _debug_draw.acquire_debug_draw_interface()

        trace = [tuple(position)]
        def overlay(enabled=True):
            if draw is None:
                return
            draw.clear_lines()
            draw.clear_points()
            if not enabled:
                return
            starts, ends = list(trace[:-1]), list(trace[1:])
            colors, widths = [(0.05, 0.9, 1.0, 1.0)] * len(starts), [3.0] * len(starts)
            radius = scenario["body_radius_m"]
            for a, b in ((0, 1), (0, 2), (1, 2)):
                points = []
                for j in range(33):
                    point = position.copy()
                    point[a] += radius * math.cos(2 * math.pi * j / 32)
                    point[b] += radius * math.sin(2 * math.pi * j / 32)
                    points.append(tuple(point))
                starts += points[:-1]; ends += points[1:]
                colors += [(0.0, 1.0, .3, 1.0)] * 32; widths += [2.0] * 32
            if starts:
                draw.draw_lines(starts, ends, colors, widths)
            draw.draw_points([tuple(scenario["goal"])], [(1.0, .1, .2, 1.0)], [18.0])

        def save_capture(name):
            if viewport_rgb is None or not app.is_running():
                return False
            from PIL import Image
            for _ in range(3):
                render_current()
                pixels = np.asarray(viewport_rgb.get_data())
                if pixels.ndim == 3 and pixels.shape[2] >= 3 and pixels.size:
                    rgb = pixels[:, :, :3].astype(np.uint8)
                    evidence = dict(minimum=int(rgb.min()), maximum=int(rgb.max()),
                                    nonblack_fraction=float(np.mean(np.max(rgb, axis=2) > 4)))
                    evidence["valid"] = evidence["maximum"]-evidence["minimum"] >= 8 and evidence["nonblack_fraction"] > .01
                    result.setdefault("capture_diagnostics", {})[name] = evidence
                    Image.fromarray(rgb).save(out / name)
                    if evidence["valid"]:
                        return True
            return False
        capture = save_capture
        overlay_data = {}
        overlay_times = {}
        for label, enabled in (("off_before", False), ("on", True), ("off_after", False)):
            overlay(enabled)
            if enabled:
                draw.draw_lines([(position[0]+1, position[1]-1, position[2])],
                                [(position[0]+3, position[1]+1, position[2])], [(1,0,1,1)], [8.0])
            # Fixed-rate sensors require a fresh real physical tick. The body
            # stays at exactly the spawn pose throughout all these warm steps.
            advance_physics_and_render()
            overlay_times[label] = dict(physics_t=SimulationManager.get_simulation_time(),
                                       physics_step=SimulationManager.get_num_physics_steps(),
                                       kit_t=timeline.get_current_time())
            overlay_data[label] = [np.asarray(sensor.get_data(), dtype="<f4").copy() for sensor in sensors]
            if any(image.shape != (scenario["image_height"], scenario["image_width"])
                   for image in overlay_data[label]):
                raise SimulationFailure("DEPTH_FRAME_ERROR", "overlay A/B requires four complete camera images")
            with (out / ("overlay_" + label + "_depth.bin")).open("wb") as evidence:
                for image in overlay_data[label]:
                    evidence.write(memoryview(np.ascontiguousarray(image)).cast("B"))
        overlay_rows = []
        for index, view in enumerate(VIEWS):
            off = overlay_data["off_before"][index]
            row = dict(camera=view[0], hashes={key:hashlib.sha256(value[index].tobytes()).hexdigest()
                                             for key,value in overlay_data.items()})
            row["comparisons"] = []
            for label in ("on", "off_after"):
                other = overlay_data[label][index]
                same_invalid = (off.shape == other.shape and np.array_equal(np.isfinite(off), np.isfinite(other))
                                and np.array_equal(np.isnan(off), np.isnan(other))
                                and np.array_equal(np.isposinf(off), np.isposinf(other))
                                and np.array_equal(np.isneginf(off), np.isneginf(other)))
                common = np.isfinite(off) & np.isfinite(other) if off.shape == other.shape else np.array([], dtype=bool)
                maximum = float(np.max(np.abs(off[common].astype(float)-other[common].astype(float)))) if common.any() else 0.0
                row["comparisons"].append(dict(mode=label, invalid_mask_equal=bool(same_invalid),
                                               max_finite_absolute_difference_m=maximum,
                                               bitwise_equal=off.tobytes()==other.tobytes(),
                                               passed=bool(same_invalid and maximum <= 1e-5)))
            overlay_rows.append(row)
        overlay_report = dict(pose=position, yaw=yaw, kit_time_s=timeline.get_current_time(),
                              capture_times=overlay_times, body_stationary=True,
                              scene_sha256=scene_sha, tolerance_m=1e-5, source="actual raw renderer depth; no filtering",
                              checks=overlay_rows, passed=all(c["passed"] for row in overlay_rows for c in row["comparisons"]))
        atomic_json(out / "overlay_depth_check.json", overlay_report)
        result["overlay_depth_check_passed"] = overlay_report["passed"]
        if not overlay_report["passed"]:
            raise SimulationFailure("OVERLAY_DEPTH_CONTAMINATION", "same-pose overlay A/B changed raw depth")
        overlay(not args.no_overlay)
        render_current()
        if not args.headless:
            atomic_json(out / "phase.json", dict(phase="initial_capture", status=result["status"]))
            result["initial_capture_saved"] = save_capture("capture_initial.png")
            if not result["initial_capture_saved"]:
                raise SimulationFailure("VIEWPORT_ERROR", "visible viewport did not produce RGB")
        kit_origin = timeline.get_current_time()
        physics_origin = SimulationManager.get_simulation_time()
        physics_step_origin = SimulationManager.get_num_physics_steps()
        if abs(kit_origin-physics_origin) > 1e-7:
            raise SimulationFailure("PHYSICS_CLOCK_MISMATCH", "initialization desynchronized clocks")
        result["physics_time_origin_s"] = physics_origin
        result["initialization_physics_steps"] = physics_step_origin
        result["sensor_warmup_physics_steps"] = physics_step_origin-result["engine_initialization_physics_steps"]
        result["kit_time_origin_s"] = kit_origin
        result["actual_physics_dt_s"] = SimulationManager.get_physics_dt()
        result["actual_render_dt_s"] = RenderingManager.get_dt()
        atomic_json(out / "sensors_ready.json", dict(calibration=str(cal_path), scenario_sha256=scenario_sha,
                    scene_sha256=scene_sha, pid=os.getpid(), width=scenario["image_width"], height=scenario["image_height"],
                    physics_dt_s=dt, time_scale=args.time_scale, headless=args.headless,
                    seed_certificate=result["seed_certificate"]))
        atomic_json(out / "phase.json", dict(phase="sensors_ready", status=result["status"]))
        wait_start = time.monotonic()
        while True:
            if not app.is_running():
                raise SimulationFailure("USER_EXIT", "window closed before controller connection")
            if time.monotonic() - wait_start > min(120.0, scenario["wall_timeout_s"]):
                raise SimulationFailure("CONTROLLER_TIMEOUT", "controller socket did not become ready")
            candidate = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            candidate.settimeout(5.0)
            try:
                candidate.connect(str(args.socket))
                connection = candidate
                break
            except (FileNotFoundError, ConnectionRefusedError):
                candidate.close()
                app.update()
                time.sleep(.02)

        columns = ["step", "t", "kit_t", "physics_t", "physics_step", "x", "y", "z", "vx", "vy", "vz", "yaw",
                   "ux", "uy", "uz", "mode", "clearance", "wall_s", "sequence", "simulation_ns",
                   "reply_simulation_ns", "predicted_vx", "predicted_vy", "predicted_vz", "flags",
                   "requested_speed", "goal_distance", "cross_track_error", "reference_progress", "certified_ahead",
                   "controller_compute_ms", "render_ms", "depth_copy_ms", "ipc_ms", "plant_sweep_ms",
                   "step_wall_ms", "pacing_sleep_ms", "finite_depth_pixels", "invalid_depth_pixels"]
        motion_start = time.monotonic()
        frame_time_origins = None
        with (out / "trajectory.csv").open("w", newline="") as stream, (out / "sensor_frames.csv").open("w", newline="") as frame_stream, (out / "sensor_samples.csv").open("w", newline="") as sample_stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            frame_writer = csv.writer(frame_stream)
            frame_writer.writerow(["sequence", "camera", "request_simulation_ns", "kit_time_s", "physics_time_s", "reference_numerator", "reference_denominator", "rendered_simulation_s", "relative_rendered_simulation_s", "raw_sha256", "finite_positive_pixels"])
            sample_writer = csv.writer(sample_stream)
            sample_writer.writerow(["sequence", "camera_index", "u", "v", "depth_m"])
            # Match the independent 100-ray audit; keep invalid raw values.
            sample_pixels = [(x, y) for y in (30, 75, 120, 165, 210) for x in (40, 100, 160, 220, 280)
                             if x < scenario["image_width"] and y < scenario["image_height"]]
            initial_row = dict.fromkeys(columns, 0)
            initial_row.update(kit_t=kit_origin, physics_t=SimulationManager.get_simulation_time(),
                               physics_step=SimulationManager.get_num_physics_steps(), x=position[0], y=position[1], z=position[2],
                               clearance=initial_body_clearance,
                               goal_distance=math.sqrt(sum((a-b)**2 for a,b in zip(position, scenario["goal"]))))
            writer.writerow(initial_row)
            for sequence in range(1, limit + 1):
                if not app.is_running():
                    result["status"] = "USER_EXIT"; code = 0; break
                if time.monotonic() - motion_start > run_wall_timeout:
                    raise SimulationFailure("WALL_TIMEOUT", "simulation exceeded configured wall time")
                step_start = time.monotonic()
                stamp = round((sequence - 1) * dt * 1e9)
                kit_time = timeline.get_current_time()
                physics_time = SimulationManager.get_simulation_time()
                physics_step = SimulationManager.get_num_physics_steps()
                if abs(kit_time - kit_origin - stamp * 1e-9) > 1e-7:
                    raise SimulationFailure("KIT_CLOCK_MISMATCH", f"Kit {kit_time-kit_origin} != requested {stamp*1e-9}")
                if (abs(physics_time-physics_origin-stamp*1e-9) > 1e-7 or
                        abs(physics_time-kit_time) > 1e-7 or physics_step != physics_step_origin+sequence-1):
                    raise SimulationFailure("PHYSICS_CLOCK_MISMATCH", "actual physics time/step count does not match request")
                copy_start = time.monotonic()
                images = [np.ascontiguousarray(sensor.get_data(), dtype="<f4") for sensor in sensors]
                shape = (scenario["image_height"], scenario["image_width"])
                if any(image.shape != shape for image in images):
                    raise SimulationFailure("DEPTH_FRAME_ERROR", "missing or incorrectly sized camera image")
                frame_times = [float(clock.get_data()["simulationTime"]) for clock in simulation_clocks]
                if not all(math.isfinite(value) for value in frame_times) or max(frame_times)-min(frame_times) > 1e-7:
                    raise SimulationFailure("SENSOR_TIME_MISMATCH", "four rendered camera times are not coherent")
                if frame_time_origins is None:
                    frame_time_origins = frame_times.copy()
                    result["rendered_sensor_time_origins_s"] = frame_time_origins
                for index, (view, image, reference_clock, frame_time) in enumerate(zip(VIEWS, images, reference_clocks, frame_times)):
                    reference = reference_clock.get_data()
                    numerator, denominator = int(reference["referenceTimeNumerator"]), int(reference["referenceTimeDenominator"])
                    relative = frame_time-frame_time_origins[index]
                    frame_writer.writerow([sequence, view[0], stamp, kit_time, physics_time, numerator, denominator, frame_time,
                                           relative, hashlib.sha256(image.tobytes()).hexdigest(),
                                           int(np.count_nonzero(np.isfinite(image) & (image > 0)))])
                    for pixel_x, pixel_y in sample_pixels:
                        sample_writer.writerow([sequence, index, pixel_x, pixel_y, float(image[pixel_y, pixel_x])])
                    if denominator <= 0 or abs(relative-stamp*1e-9) > 1e-7 or abs(frame_time-physics_time) > 1e-7:
                        frame_stream.flush()
                        sample_stream.flush()
                        raise SimulationFailure("SENSOR_TIME_MISMATCH", f"{view[0]} rendered time {relative} does not match request {stamp*1e-9}")
                finite = sum(int(np.count_nonzero(np.isfinite(image) & (image > 0))) for image in images)
                invalid = sum(image.size for image in images) - finite
                if sequence == 1:
                    with (out / "first_depth.bin").open("wb") as depth:
                        for image in images:
                            depth.write(memoryview(image).cast("B"))
                    result["first_depth_sha256"] = hashlib.sha256((out / "first_depth.bin").read_bytes()).hexdigest()
                depth_ms = (time.monotonic() - copy_start) * 1000
                ipc_start = time.monotonic()
                state = (*position, *velocity, *rotation_yaw(yaw))
                if not all(math.isfinite(value) for value in state):
                    raise SimulationFailure("INVALID_STATE", "nonfinite state before IPC")
                connection.sendall(REQUEST.pack(MAGIC, 1, sequence, stamp, scenario["image_width"], scenario["image_height"], *state))
                for image in images:
                    connection.sendall(memoryview(image).cast("B"))
                decision, flags, reply = parse_reply(receive_exact(connection, REPLY.size), sequence, stamp)
                ipc_ms = (time.monotonic() - ipc_start) * 1000
                result["last_reply"] = dict(sequence=sequence, simulation_ns=stamp, decision=decision,
                                            flags=flags, plant_input_W=list(reply[:3]),
                                            predicted_next_velocity_W=list(reply[3:6]))
                if decision == 6:
                    raise SimulationFailure("UNCERTIFIED_STATE", "controller returned fatal uncertified state")
                plant_start = time.monotonic()
                command, predicted = reply[:3], reply[3:6]
                next_velocity = list(velocity_response(velocity, command, dt, base["plant_tau_s"], base["plant_acceleration_mps2"]))
                if max(abs(a - b) for a, b in zip(predicted, next_velocity)) > 1e-8:
                    raise SimulationFailure("PLANT_PREDICTION_MISMATCH", "C++ and Python plant predictions differ")
                speed = math.sqrt(sum(v * v for v in next_velocity))
                accel = math.sqrt(sum((a-b)**2 for a,b in zip(next_velocity, velocity))) / dt
                if speed > base["max_speed_mps"] + 1e-7 or accel > base["plant_acceleration_mps2"] + 1e-7:
                    raise SimulationFailure("EXECUTION_LIMIT", "predicted physical speed/acceleration exceeds base limits")
                next_position = [p + dt * v for p, v in zip(position, next_velocity)]
                clearance, nearest = swept_clearance(position, next_position, scenario["body_radius_m"], scenario)
                if clearance <= 1e-10:
                    result["collision_blocks"] += 1
                    result["blocked_sweep"] = dict(begin=position, end=next_position, clearance_m=clearance, nearest=nearest)
                    raise SimulationFailure("INDEPENDENT_SWEEP_BLOCK", "independent continuous sphere sweep intersects " + nearest)
                minimum_clearance = min(minimum_clearance, clearance)
                distance += math.sqrt(sum((a-b)**2 for a,b in zip(next_position, position)))
                old_arrival = (math.dist(position, scenario["goal"]) <= scenario["arrival_radius_m"]
                               and math.sqrt(sum(v*v for v in velocity)) <= scenario["arrival_speed_mps"])
                position, velocity = next_position, next_velocity
                peak_speed, maximum_accel = max(peak_speed, speed), max(maximum_accel, accel)
                if math.hypot(velocity[0], velocity[1]) > .1:
                    target_yaw = math.atan2(velocity[1], velocity[0])
                    change = math.atan2(math.sin(target_yaw-yaw), math.cos(target_yaw-yaw))
                    yaw += max(-2*dt, min(2*dt, change))
                robot_api.SetTranslate(Gf.Vec3d(*position))
                robot_api.SetRotate(Gf.Vec3f(0, 0, math.degrees(yaw)), UsdGeom.XformCommonAPI.RotationOrderXYZ)
                if not args.headless:
                    set_view()
                if sum((a-b)**2 for a,b in zip(position, trace[-1])) > .03**2:
                    trace.append(tuple(position))
                if sequence % 10 == 0:
                    overlay(not args.no_overlay)
                plant_ms = (time.monotonic() - plant_start) * 1000
                # Advance the actual Kit clock only after accepting this
                # command and updating the kinematic pose. These rendered
                # images are the next request's observation, at exactly t+dt.
                render_start = time.monotonic()
                advance_physics_and_render()
                kit_time = timeline.get_current_time()
                physics_time = SimulationManager.get_simulation_time()
                physics_step = SimulationManager.get_num_physics_steps()
                if (abs(physics_time-physics_origin-sequence*dt) > 1e-7 or physics_step != physics_step_origin+sequence or
                        abs(physics_time-kit_time) > 1e-7):
                    raise SimulationFailure("PHYSICS_CLOCK_MISMATCH", "render added a physics step or desynchronized clocks")
                if abs(kit_time - kit_origin - sequence * dt) > 1e-7:
                    raise SimulationFailure("KIT_CLOCK_MISMATCH", "Kit did not advance by the accepted physical step")
                render_ms = (time.monotonic() - render_start) * 1000
                goal_distance = math.sqrt(sum((a-b)**2 for a,b in zip(position, scenario["goal"])))
                hold = hold + dt if old_arrival and goal_distance <= scenario["arrival_radius_m"] and speed <= scenario["arrival_speed_mps"] else 0.0
                result["completed_steps"] = sequence
                sleep_for = max(0.0, step_start + dt / args.time_scale - time.monotonic())
                if sleep_for:
                    time.sleep(sleep_for)
                writer.writerow(dict(step=sequence, t=sequence*dt, kit_t=kit_time,
                                     physics_t=physics_time, physics_step=physics_step,
                                     x=position[0], y=position[1], z=position[2],
                                     vx=velocity[0], vy=velocity[1], vz=velocity[2], yaw=yaw,
                                     ux=command[0], uy=command[1], uz=command[2], mode=decision,
                                     clearance=clearance, wall_s=time.monotonic()-motion_start,
                                     sequence=sequence, simulation_ns=round(sequence*dt*1e9), reply_simulation_ns=stamp,
                                     predicted_vx=predicted[0], predicted_vy=predicted[1], predicted_vz=predicted[2],
                                     flags=flags, requested_speed=reply[6], goal_distance=goal_distance,
                                     cross_track_error=reply[8], reference_progress=reply[9], certified_ahead=reply[10],
                                     controller_compute_ms=reply[11], render_ms=render_ms, depth_copy_ms=depth_ms,
                                     ipc_ms=ipc_ms, plant_sweep_ms=plant_ms, step_wall_ms=(time.monotonic()-step_start)*1000,
                                     pacing_sleep_ms=sleep_for*1000, finite_depth_pixels=finite, invalid_depth_pixels=invalid))
                if sequence % 20 == 0 or sequence == 1:
                    stream.flush()
                    frame_stream.flush()
                    sample_stream.flush()
                if hold + 1e-12 >= scenario["arrival_hold_s"] and flags & 4:
                    result["status"] = "ARRIVED"; result["arrived"] = result["success"] = True; code = 0; break
                if (out / "stop_requested").exists():
                    result["status"] = "STOP_REQUESTED"; code = 0; break
            else:
                result["status"] = "STEP_LIMIT" if requested_bounded else "SIMULATION_TIMEOUT"
                code = 0 if requested_bounded else 1
                if not requested_bounded:
                    result["failures"].append("goal hold was not achieved before simulation timeout")
        overlay(not args.no_overlay)
        if not args.headless:
            result["capture_saved"] = save_capture("capture.png")
            if app.is_running() and not result["capture_saved"]:
                raise SimulationFailure("VIEWPORT_ERROR", "final viewport capture unavailable")
    except SimulationFailure as exc:
        result["status"] = exc.status
        result["success"] = False
        result["failures"].append(str(exc))
        code = 0 if exc.status == "USER_EXIT" else 1
    except (socket.timeout, TimeoutError) as exc:
        result["success"] = False
        result["status"] = "CONTROLLER_TIMEOUT"; result["failures"].append(str(exc)); code = 1
    except KeyboardInterrupt:
        result["status"] = "USER_EXIT"; code = 0
    except Exception as exc:
        result["success"] = False
        result["status"] = "ERROR"; result["failures"].append(type(exc).__name__ + ": " + str(exc)); code = 1
        import traceback
        traceback.print_exc()
    finally:
        if connection is not None:
            connection.close()
        result.update(final_position=position, final_velocity=velocity, final_yaw_rad=yaw,
                      final_speed_mps=math.sqrt(sum(v*v for v in velocity)), goal=scenario["goal"],
                      remaining_m=math.sqrt(sum((a-b)**2 for a,b in zip(position, scenario["goal"]))),
                      simulation_s=result["completed_steps"]*dt, arrival_hold_s=hold,
                      peak_speed_mps=peak_speed, maximum_acceleration_mps2=maximum_accel, distance_m=distance,
                      minimum_swept_clearance_m=minimum_clearance if math.isfinite(minimum_clearance) else None,
                      wall_elapsed_s=time.monotonic()-total_start,
                      motion_wall_s=None if motion_start is None else time.monotonic()-motion_start,
                      exit_code=code)
        result.update(start=scenario["start"], final=position, wall_s=result["wall_elapsed_s"],
                      body_radius=scenario["body_radius_m"], overlaps=0, minimum_clearance=result["minimum_swept_clearance_m"],
                      frames=result["completed_steps"], time_scale=args.time_scale,
                      scene_identity=dict(scenario_sha256=scenario_sha, scene_sha256=result.get("scene_sha256")))
        if result["motion_wall_s"]:
            result["achieved_simulation_wall_ratio"] = result["simulation_s"] / result["motion_wall_s"]
        atomic_json(out / "phase.json", dict(phase="cleanup", status=result["status"], failures=result["failures"]))
        # Preserve the primary result before any optional capture/Kit cleanup.
        atomic_json(out / "result.json", result)
        if capture is not None and not args.headless and app is not None and app.is_running() and not (out / "capture.png").exists():
            try:
                result["capture_saved"] = capture("capture.png")
            except Exception as exc:
                result["failures"].append("capture cleanup: " + str(exc))
            atomic_json(out / "result.json", result)
        if app is not None:
            for clock, product in zip(reference_clocks, products):
                try:
                    clock.detach(product)
                except Exception:
                    pass
            for clock, product in zip(simulation_clocks, products):
                try:
                    clock.detach(product)
                except Exception:
                    pass
            for sensor, product in zip(sensors, products):
                try:
                    sensor.detach(product); product.destroy()
                except Exception:
                    pass
            app.close()
    print(json.dumps(result, allow_nan=False), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
