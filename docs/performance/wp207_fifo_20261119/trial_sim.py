"""R05 model test with the current P3 core and original experimental guidance wrapper."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

p = argparse.ArgumentParser()
p.add_argument("assets", type=Path)
p.add_argument("controller", type=Path)
p.add_argument("output", type=Path)
p.add_argument("--seconds", type=float, default=75)
p.add_argument("--headless", action="store_true")
p.add_argument("--time-scale", type=float, default=.02)
p.add_argument("--guide", type=Path)
p.add_argument("--fifo", action="store_true")
p.add_argument("--direct-only", action="store_true")
p.add_argument("--isaac-no-hit", action="store_true")
a = p.parse_args()
project = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(project/"src/r05_model"))
sys.path.insert(0, str(project / "scripts/isaac"))
sys.path.insert(0, str(project / "scripts"))
from base_config import load_config
from depth_camera_projection import configure_depth_projection, save_projection_evidence
from manual_control_math import velocity_response
from presentation import WIDTH,HEIGHT,FPS,attach_arl,ReferenceCamera,configure_camera,set_camera,LiveOverlay
from isaacsim import SimulationApp

scene, occ = [a.assets / name for name in ("ego_swarm_cloud_navigation.usd", "occupancy.bin")]
scene_sha = "cfcc22694241eb2ca5e506060c136a2f9fe5b3b5e2efd76cf0d44a1da26ed1ca"
occupancy_sha = "e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8"
for path, sha in [(scene, scene_sha), (occ, occupancy_sha)]:
    if hashlib.sha256(path.read_bytes()).hexdigest() != sha:
        raise ValueError("R05 asset identity mismatch")
base, _ = load_config()
if not math.isfinite(a.seconds) or not 0 < a.seconds <= 150 or not math.isfinite(a.time_scale) or not 0 < a.time_scale <= 1:
    p.error("seconds must be (0,150], time-scale must be (0,1]")
dt = base["physics_dt_s"]
a.output.mkdir(exist_ok=False)
(a.output / "simulator_pid.json").write_text(json.dumps({"pid": os.getpid()}))
# Reuse P3's existing cache without copying historical experiment caches.
portable = a.output / "kit"
portable.mkdir()
shared = project / "docs/performance/isaac_closed_loop_20261117/shared_cache/kit"
if shared.is_dir():
    (portable / "cache").symlink_to(shared, target_is_directory=True)
sys.argv = [sys.argv[0]]
app = SimulationApp({
    "width":WIDTH,"height":HEIGHT,"window_width":WIDTH,"window_height":HEIGHT,
    "headless": a.headless, "renderer": "RaytracedLighting", "disable_viewport_updates": a.headless,
    "limit_cpu_threads": 8, "minimal_shading_mode": 2, "anti_aliasing": 1, "active_gpu": 0,
    "multi_gpu": False, "extra_args": ["--portable-root", str(portable),
        "--/app/settings/persistent=false", "--/log/file=" + str(a.output / "kit.log")],
})
import carb
import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from pxr import Gf, Sdf, Usd, UsdGeom

START = np.array([-46.4, -16.8, 10.0])
GOAL = np.array([246.4, 48.8, 10.0])
ORIGIN = np.array([-80., -60., 0.])
SHAPE = np.array([1200, 300, 50])
RES = .4

def grid_clearance(grid, first, last, radius):
    center = (first + last) * .5
    r = radius + np.linalg.norm(last - first) * .5
    index = np.floor((center - ORIGIN) / RES + .5).astype(int)
    reach = int(np.ceil((r + RES) / RES)) + 1
    lo = np.maximum(0, index - reach)
    hi = np.minimum(SHAPE, index + reach + 1)
    bounds = min(float(np.min(center - (ORIGIN - .2))), float(np.min(ORIGIN + RES*(SHAPE - 1) + .2 - center)), float(center[2]))
    if np.any(lo >= hi):
        return -math.inf
    indices = np.argwhere(grid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]) + lo
    if len(indices):
        distances = np.linalg.norm(np.maximum(np.abs(ORIGIN + RES * indices - center) - .2, 0), axis=1)
        bounds = min(bounds, float(distances.min()))
    return bounds - r

def guidance(path):
    if not path:
        return np.array([GOAL])
    with path.open(newline="") as stream:
        points = np.array([[float(r[k]) for k in "xyz"] for r in csv.DictReader(stream)])
    # Only route geometry. No velocity, actuator command, or measured pose is replayed.
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points,axis=0), axis=1))]
    spans = np.arange(0., lengths[-1] - .25, .50)
    return np.vstack([np.column_stack([np.interp(spans, lengths, points[:,i]) for i in range(3)]), GOAL])

class RouteIntent:
    def __init__(self, points):
        self.points = points
        self.offset = 0.
        self.index = 0
        if len(points)>1:
            self.delta = np.diff(points, axis=0)
            self.length = np.linalg.norm(self.delta, axis=1)
            self.arc = np.r_[0., np.cumsum(self.length)]
            self.tangent = self.delta / self.length[:, None]
            change = np.linalg.norm(np.diff(self.tangent, axis=0),axis=1)
            self.bend = np.r_[change/(.5*(self.length[:-1]+self.length[1:])), 0.]

    def update(self, pose, speed):
        if len(self.points)==1:
            return self.points[0], True, 12.
        indices = np.arange(max(0,self.index-2), min(len(self.length),self.index+10))
        rel = pose-self.points[indices]
        frac = np.clip(np.sum(rel*self.tangent[indices],axis=1)/self.length[indices],0.,1.)
        dist = np.linalg.norm(rel-self.delta[indices]*frac[:,None],axis=1)
        nearest = int(np.argmin(dist))
        self.offset = max(self.offset,float(self.arc[indices[nearest]]+frac[nearest]*self.length[indices[nearest]]))
        self.index = min(len(self.length)-1, int(np.searchsorted(self.arc,self.offset,side="right"))-1)
        if self.arc[-1]-self.offset<4.:
            return GOAL, True, 12.
        lookahead = min(self.arc[-1],self.offset+.55+.08*speed)
        target = np.array([np.interp(lookahead,self.arc,self.points[:,i]) for i in range(3)])
        last = max(self.index+1,int(np.searchsorted(self.arc,self.offset+3.)))
        bend = float(np.max(self.bend[self.index:last]))
        grade = float(np.max(np.abs(self.tangent[self.index:last,2])))
        limit = 12./(1.+6.*max(0.,bend-.07)+2.5*max(0.,grade-.12))
        if dist[nearest]>.20:
            limit = min(limit, 6.)
        return target, False, max(3.,limit)

child = None
try:
    ctx = omni.usd.get_context()
    if not ctx.open_stage(str(scene)):
        raise RuntimeError("failed to open R05 USD")
    for _ in range(8):
        app.update()
    app.reset_render_settings()
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    visual_provenance=attach_arl(stage)
    (a.output / "robot_visual.json").write_text(json.dumps(visual_provenance,indent=2))
    robot = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Robot"))
    robot.SetTranslate(Gf.Vec3d(*START))
    robot.SetRotate(Gf.Vec3f(0,0,0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    names = [("Front", ""), ("Left", "Left"), ("Back", "Back"), ("Right", "Right")]
    views = [(name, f"/World/Robot/CameraMount{suffix}/DepthCamera", f"/depth/{name}", "",
              f"camera_{name.lower()}_optical") for name,suffix in names]
    cal = configure_depth_projection(stage, Usd, UsdGeom, views, 320, 240, scene_path=str(scene))
    cal["depth_type"] = "optical_axis_z_m"
    for camera in cal["cameras"]:
        camera["aperture_principal_point"] = [camera["cx"],camera["cy"]]
        camera["cx"],camera["cy"] = 159.5,119.5
        camera["pixel_coordinate_convention"] = "integer indices for RTX pixel centers"
        prim=stage.GetPrimAtPath(camera["prim_path"])
        prim.CreateAttribute("omni:sensor:tickRate",Sdf.ValueTypeNames.Float).Set(1./dt)
        prim.CreateAttribute("omni:sensor:frameRate",Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(1./dt,1.))
    (a.output / "model_readback.json").write_text(json.dumps({"used_layers":[layer.identifier for layer in stage.GetUsedLayers()],
        "robot_path":"/World/Robot","scene_sha256":scene_sha,"camera_paths":[v[1] for v in views]},indent=2))
    no_hit_z = base["local_range_m"] + .5
    if a.isaac_no_hit:
        for view in views:
            near, far = UsdGeom.Camera(stage.GetPrimAtPath(view[1])).GetClippingRangeAttr().Get()
            if not 0. < near < no_hit_z < far:
                raise RuntimeError("camera clipping cannot support the Isaac no-hit adapter")
        cal["positive_infinity_is_clear_to_optical_z_m"] = no_hit_z
    save_projection_evidence(cal, a.output / "simulation.json")
    SimulationManager.setup_simulation(dt=dt, device="cpu")
    RenderingManager.set_dt(dt)
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_time_codes_per_second(1./dt)
    timeline.set_target_framerate(1./dt)
    timeline.set_end_time(10000.)
    timeline.set_ticks_per_frame(1)
    timeline.set_auto_update(False)
    timeline.pause()
    SimulationManager.initialize_physics()
    carb.settings.get_settings().set("/app/runLoops/main/rateLimitEnabled", False)
    rep.orchestrator.set_capture_on_play(False)
    products = [rep.create.render_product(view[1], (320, 240)) for view in views]
    sensors = [rep.AnnotatorRegistry.get_annotator("distance_to_image_plane") for _ in views]
    for sensor, product in zip(sensors, products):
        sensor.attach(product)
    import isaacsim.core.utils.extensions as extensions
    extensions.enable_extension("isaacsim.core.nodes")
    clocks=[rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime") for _ in views]
    for clock,product in zip(clocks,products):
        clock.initialize(resetOnStop=False)
        clock.attach(product)
    def advance_physics_and_render():
        before=SimulationManager.get_simulation_time()
        step=SimulationManager.get_num_physics_steps()
        SimulationManager.step(steps=1)
        after=SimulationManager.get_simulation_time()
        if abs(after-before-dt)>1e-7 or SimulationManager.get_num_physics_steps()!=step+1:
            raise RuntimeError("physical step is not exactly 5ms")
        timeline.set_current_time(after)
        timeline.commit()
        rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=False,wait_for_render=True)
        if abs(SimulationManager.get_simulation_time()-after)>1e-7:
            raise RuntimeError("render advanced physics unexpectedly")
    for _ in range(3):
        advance_physics_and_render()
    grid = np.fromfile(occ, dtype=np.uint8).reshape(SHAPE)
    viewport_rgb=None;live_hud=None;camera_rule=ReferenceCamera(grid);camera_frame=-1
    def follow_view(position,body_yaw=0.,simulation_time=0.):
        global camera_frame
        if not a.headless and simulation_time+.0025 >= (camera_frame+1)/FPS:
            eye,aim=camera_rule.sample(position,body_yaw)
            set_camera(chase,eye,aim)
            camera_frame+=1
            return True
        return False
    if not a.headless:
        chase=configure_camera(stage,"/World/R05Chase")
        import omni.kit.viewport.utility as viewport_utility
        ready,_=ViewportManager.wait_for_viewport(max_frames=120,sleep_time=.02)
        viewport=viewport_utility.get_active_viewport()
        if not ready or viewport is None:raise RuntimeError("GUI viewport unavailable")
        follow_view(START)
        ViewportManager.set_camera("/World/R05Chase",render_product_or_viewport=viewport)
        viewport_rgb=rep.AnnotatorRegistry.get_annotator("rgb")
        viewport_rgb.attach([viewport.render_product_path])
    route = guidance(a.guide)
    intent = RouteIntent(route)
    if not a.headless:
        live_hud=LiveOverlay(viewport_utility.get_active_viewport_window(),route,a.seconds)
        live_hud.update(dict(t=0,x=START[0],y=START[1],z=START[2],vx=0,vy=0,vz=0,mode=0),0,0)
    np.savetxt(a.output / "intent_waypoints.csv", route, delimiter=",", header="x,y,z", comments="")
    pose, velocity = START.copy(), np.zeros(3)
    route_index = 0
    yaw = distance = still = stall = 0.
    minimum = math.inf
    peak = maximum_acc = 0.
    blocks = overlaps = 0
    no_hit_pixels = 0
    terminal_hold_steps = 0
    status = "TIMEOUT"
    sensor_times=[]
    render_retries = 0
    wall = time.monotonic()
    with (a.output / "controller.log").open("w") as clog, (a.output / "trajectory.csv").open("w", newline="") as stream:
        # Raise only this child's soft limit to its inherited hard limit. Kit
        # workers retain a zero soft limit, including workers started later.
        controller_prefix = ["/usr/bin/prlimit", "--rtprio=5:5", "--"] if a.fifo else []
        child = subprocess.Popen([*controller_prefix, str(a.controller), str(a.output / "simulation.sensors.json"),
            str(scene), str(occ), str(a.output / "controller.csv"), "direct-only" if a.direct_only else "search",
            "fifo" if a.fifo else "normal"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=clog)
        csv_out = csv.writer(stream)
        csv_out.writerow(("step","t","kit_t","x","y","z","vx","vy","vz","ux","uy","uz","mode","waypoint","clearance","wall_s","guide_cap","yaw_rad"))
        kit_zero = timeline.get_current_time()
        csv_out.writerow((0,0,kit_zero,*pose,*velocity,0,0,0,0,0,grid_clearance(grid,pose,pose,.10),0,12.,yaw))
        (a.output / "controller_pid.json").write_text(json.dumps({"pid": child.pid}))
        for frame in range(1, int(a.seconds/dt)+1):
            step_wall = time.monotonic()
            advance_physics_and_render()
            kit_t = timeline.get_current_time()
            if abs(kit_t - kit_zero - frame*dt) > 1e-6:
                raise RuntimeError(f"unexpected Kit step {kit_t} at frame {frame}")
            aim, final_aim, guide_cap = intent.update(pose, float(np.linalg.norm(velocity)))
            route_index = intent.index
            actual_time=SimulationManager.get_simulation_time()
            times=[float(clock.get_data()["simulationTime"]) for clock in clocks]
            # A viewport update can return the preceding render fence. Retry
            # only at this same pose/time, before passing depth to the controller.
            physics_step = SimulationManager.get_num_physics_steps()
            for retry in range(3):
                if all(math.isfinite(t) and abs(t-actual_time)<=1e-7 for t in times):
                    break
                rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=False,wait_for_render=True)
                render_retries += 1
                if (SimulationManager.get_num_physics_steps()!=physics_step or
                        abs(SimulationManager.get_simulation_time()-actual_time)>1e-7 or
                        abs(timeline.get_current_time()-kit_t)>1e-7):
                    raise RuntimeError("sensor retry advanced simulation")
                times=[float(clock.get_data()["simulationTime"]) for clock in clocks]
            if any(not math.isfinite(t) or abs(t-actual_time)>1e-7 for t in times):
                raise RuntimeError("camera timestamp differs from real physics time: "+str(times)+" vs "+str(actual_time))
            sensor_times.append([frame,actual_time,*times])
            data = [np.ascontiguousarray(sensor.get_data(),dtype=np.float32) for sensor in sensors]
            if any(d.size != 320*240 for d in data):
                raise RuntimeError("invalid four-camera frame")
            if frame==1:
                with (a.output / "first_depth.bin").open("wb") as depth:
                    for d in data:
                        depth.write(memoryview(d).cast("B"))
            if a.isaac_no_hit:
                for i, image in enumerate(data):
                    mask = np.isposinf(image)
                    no_hit_pixels += int(np.count_nonzero(mask))
                    data[i] = np.where(mask, np.float32(no_hit_z), image)
            child.stdin.write(struct.pack("=12d", *pose, *velocity, yaw, *aim, float(final_aim), guide_cap))
            for d in data:
                child.stdin.write(memoryview(d).cast("B"))
            child.stdin.flush()
            reply = child.stdout.read(32)
            if len(reply)!=32:
                raise RuntimeError("controller returned an incomplete command")
            ux,uy,uz,mode = struct.unpack("=4d",reply)
            command = (ux,uy,uz)
            if int(mode)==4:
                status = "UNCERTIFIED_BRAKE"
                break
            new_v = np.array(velocity_response(velocity, command, dt, base["plant_tau_s"], base["plant_acceleration_mps2"]))
            nxt = pose + new_v * dt
            clearance = grid_clearance(grid,pose,nxt,base["body_radius_m"])
            if clearance <= 0.:
                blocks += 1
                status = "INDEPENDENT_SWEEP_BLOCK"
                break
            minimum = min(minimum, clearance)
            maximum_acc = max(maximum_acc,float(np.linalg.norm(new_v-velocity)/dt))
            segment = float(np.linalg.norm(nxt-pose))
            distance += segment
            pose, velocity = nxt, new_v
            speed = float(np.linalg.norm(velocity))
            peak = max(peak, speed)
            if math.hypot(*velocity[:2])>.1:
                desired_yaw = math.atan2(velocity[1],velocity[0])
            else:
                delta = aim - pose
                desired_yaw = math.atan2(delta[1],delta[0])
                if stall > .25:
                    desired_yaw += .60*math.sin(2.*stall)
            error = math.atan2(math.sin(desired_yaw-yaw), math.cos(desired_yaw-yaw))
            yaw += max(-dt*2.,min(dt*2.,error))
            robot.SetTranslate(Gf.Vec3d(*pose))
            robot.SetRotate(Gf.Vec3f(0.,0.,math.degrees(yaw)),UsdGeom.XformCommonAPI.RotationOrderXYZ)
            elapsed = time.monotonic()-wall
            csv_out.writerow((frame,frame*dt,kit_t,*pose,*velocity,*command,int(mode),route_index,clearance,elapsed,guide_cap,yaw))
            if not a.headless:
                if follow_view(pose,yaw,frame*dt):
                    live_hud.update(dict(t=frame*dt,x=pose[0],y=pose[1],z=pose[2],vx=velocity[0],vy=velocity[1],vz=velocity[2],mode=int(mode)),route_index,distance)
            pause = dt / a.time_scale - (time.monotonic() - step_wall)
            if pause > 0:
                time.sleep(pause)
            remaining=float(np.linalg.norm(pose-GOAL))
            if frame%200==0 or frame in (1,10,50,100):
                print(f"step={frame} sim={frame*dt:.3f}s wall={elapsed:.1f}s position={pose} v={speed:.3f} goal={remaining:.2f} mode={mode:.0f} waypoint={route_index}",flush=True)
                stream.flush()
            if remaining<=.30 and speed<.1 and int(mode) in (1,2):
                # The accepted controller step has already certified its
                # straight emergency stop. Below .1 m/s, zero input takes
                # exactly one kinematic step to rest. Hold the resulting
                # state for .5 simulated seconds without requesting depth.
                for sensor, product in zip(sensors, products):
                    sensor.detach(product)
                for clock, product in zip(clocks, products):
                    clock.detach(product)
                for product in products:
                    product.destroy()
                timeline.pause()
                timeline.set_auto_update(False)
                for hold in range(1,round(.5/dt)+1):
                    next_v = np.array(velocity_response(velocity,(0.,0.,0.),dt,base["plant_tau_s"],base["plant_acceleration_mps2"]))
                    stopped = pose+next_v*dt
                    clearance = grid_clearance(grid,pose,stopped,base["body_radius_m"])
                    if clearance<=0. or np.linalg.norm(stopped-GOAL)>1. or np.linalg.norm(next_v)>=.1:
                        raise RuntimeError("terminal stopping step did not verify")
                    minimum=min(minimum,clearance)
                    maximum_acc=max(maximum_acc,float(np.linalg.norm(next_v-velocity)/dt))
                    distance+=float(np.linalg.norm(stopped-pose))
                    pose,velocity=stopped,next_v
                    robot.SetTranslate(Gf.Vec3d(*pose))
                    advance_physics_and_render()
                    kit_t=timeline.get_current_time()
                    if abs(kit_t-kit_zero-(frame+hold)*dt)>1e-6:
                        raise RuntimeError("terminal hold clock mismatch")
                    csv_out.writerow((frame+hold,(frame+hold)*dt,kit_t,*pose,*velocity,0,0,0,6,route_index,
                                      clearance,time.monotonic()-wall,0.,yaw))
                    terminal_hold_steps+=1
                frame+=terminal_hold_steps
                status="ARRIVED"
                break
            stall = stall+dt if speed<.05 else 0.
            if stall>5.:
                status="STALLED"
                break
            if (a.output / "stop_requested").is_file():
                status="STOP_REQUESTED"
                break
        child.stdin.close()
        if child.wait(timeout=15)!=0:
            raise RuntimeError("controller did not finish cleanly")
    result = dict(status=status, experiment="R05 original guidance wrapper with current P3 core",
        time_scale=a.time_scale, guided=bool(a.guide), fifo_requested=a.fifo, frame=frame, simulation_s=frame*dt,
        isaac_no_hit=a.isaac_no_hit, mapped_positive_infinity_pixels=no_hit_pixels,
        terminal_hold_steps=terminal_hold_steps, sensor_render_retries=render_retries,
        kit_time_s=kit_t, wall_s=time.monotonic()-wall, start=START.tolist(), goal=GOAL.tolist(), final=pose.tolist(),
        remaining_m=float(np.linalg.norm(pose-GOAL)), final_speed_mps=float(np.linalg.norm(velocity)), peak_speed_mps=peak,
        acceleration_max_mps2=maximum_acc, distance_m=distance, independent_collision_blocks=blocks,
        swept_overlap_segments=overlaps, minimum_swept_clearance_m=minimum if math.isfinite(minimum) else None,
        scene_sha256=scene_sha, occupancy_sha256=occupancy_sha, plant="fixed-step Isaac kinematic integration")
    if viewport_rgb is not None:
        from PIL import Image
        for _ in range(3):
            rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=False,wait_for_render=True)
            pixels=np.asarray(viewport_rgb.get_data())
            if pixels.ndim==3 and pixels.shape[2]>=3 and int(pixels[:,:,:3].max())>4:
                image=Image.fromarray(pixels[:,:,:3].astype(np.uint8)).convert("RGBA")
                if live_hud is not None:
                    overlay=live_hud.last_image.resize(image.size)
                    image=Image.alpha_composite(image,overlay)
                image.convert("RGB").save(a.output / "preview.png")
                break
        result["preview_saved"]=(a.output / "preview.png").is_file()
        result["viewport_camera"]=str(ViewportManager.get_camera(viewport).GetPath())
        result["viewport_camera_transform"]=[list(row) for row in UsdGeom.Xformable(chase).ComputeLocalToWorldTransform(Usd.TimeCode.Default())]
    np.savetxt(a.output / "sensor_times.csv",np.asarray(sensor_times),delimiter=",",header="step,physics_s,front_s,left_s,back_s,right_s",comments="")
    result["robot_visual"]=visual_provenance
    result["presentation"]=dict(reference="original R05 video",width=WIDTH,height=HEIGHT,fps=FPS,
        live_camera="causal 168-candidate occlusion chase; no future control states",replay_camera="global DP and Gaussian reference path")
    result["physics_clock_policy"]="one explicit 5ms physical step, synchronized playhead, zero-time render"
    result["sensor_timestamp_checks"]=len(sensor_times)*4
    (a.output / "result.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result),flush=True)
except BaseException as error:
    import traceback
    (a.output / "failure.json").write_text(json.dumps({"error": repr(error), "traceback": traceback.format_exc()}, indent=2))
    traceback.print_exc()
    sys.stderr.flush()
    raise
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        child.wait(timeout=10)
    app.close()
