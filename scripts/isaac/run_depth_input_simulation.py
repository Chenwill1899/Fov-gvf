#!/usr/bin/env python3
"""Read-only stationary Isaac sensor test using the existing four USD cameras.

No command/intent publisher, motion executor, controller, or scene export.
The wrapper reader ends after 0.2 seconds of observed ROS simulation clock.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from base_config import load_config
BASE, BASE_SHA = load_config()

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-session', help='fresh run UUID (32 lowercase hex digits) shared with all input publishers')
parser.add_argument('--source-generation',type=int,default=0)
parser.add_argument('--scene', required=True)
parser.add_argument('--output', required=True)
parser.add_argument('--run-id', required=True)
parser.add_argument('--camera-hz', type=int, default=BASE["camera_hz"])
parser.add_argument('--pace', choices=('realtime','unlimited'), default='realtime')
parser.add_argument('--time-scale', type=float, default=BASE["time_scale"])
parser.add_argument('--sensor-mode', choices=('rate','auto'), default='rate')
parser.add_argument('--disable-viewport', action='store_true')
parser.add_argument('--render-threads', type=int, default=12)
parser.add_argument('--profile-stages', action='store_true')
parser.add_argument('--record-physics', action='store_true')
args = parser.parse_args()
if args.source_session=="new":
    import uuid
    args.source_session=uuid.uuid4().hex
if args.source_session and (len(args.source_session)!=32 or any(c not in '0123456789abcdef' for c in args.source_session)):
    parser.error("invalid source-session UUID")
if not 0<=args.source_generation<=2**32-1 or (args.source_generation and not args.source_session):
    parser.error("source generation requires a session and a nonnegative index")
topic_prefix=f"/ego4p3/s_{args.source_session}/e{args.source_generation}" if args.source_session else ""
if args.camera_hz != BASE["camera_hz"] or args.time_scale != BASE["time_scale"] or args.pace != "realtime" or args.sensor_mode != "rate":
    parser.error("step-1 sensor configuration must match base_step1.json")
if not math.isfinite(args.time_scale) or not 0 < args.time_scale <= 1:
    parser.error('time-scale must be finite and within (0,1]')
if args.pace == 'unlimited' and args.time_scale != 1:
    parser.error('unlimited pacing requires --time-scale 1')
if not 1 <= args.camera_hz <= 1000:
    parser.error('camera-hz must be within [1,1000]')
scene, out = Path(args.scene).resolve(), Path(args.output).resolve()
sys.argv = [sys.argv[0]]
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'MinimalRendering',
    'disable_viewport_updates': args.disable_viewport,
    'limit_cpu_threads': args.render_threads,
    'minimal_shading_mode': 2, 'anti_aliasing': 1, 'active_gpu': 0, 'multi_gpu': False,
    'extra_args': ['--portable-root', os.environ['EGO4P0_RUNTIME']+'/kit',
                   '--/app/settings/persistent=false', '--/log/file='+str(out/'kit.log')]})
import omni.graph.core as og
import omni.usd
import omni.timeline
import carb
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.rendering_manager import RenderingManager
import isaacsim.core.experimental.utils.app as app_utils
from pxr import Gf, Usd, UsdGeom, Sdf
from depth_camera_projection import configure_depth_projection, save_projection_evidence

app_utils.enable_extension('isaacsim.ros2.bridge')
for _ in range(5):
    app.update()
context = omni.usd.get_context()
if not context.open_stage(str(scene)):
    raise RuntimeError('failed to open scene')
app.update()
stage = context.get_stage()
metadata = json.loads(stage.GetRootLayer().customLayerData['fovNavigationJson'])
robot = stage.GetPrimAtPath('/World/Robot')
views = (
    ('Front','/World/Robot/CameraMount/DepthCamera','/sim/depth/image_raw','/sim/depth/camera_info','camera_front_optical'),
    ('Left','/World/Robot/CameraMountLeft/DepthCamera','/sim/depth_left/image_raw','/sim/depth_left/camera_info','camera_left_optical'),
    ('Back','/World/Robot/CameraMountBack/DepthCamera','/sim/depth_back/image_raw','/sim/depth_back/camera_info','camera_back_optical'),
    ('Right','/World/Robot/CameraMountRight/DepthCamera','/sim/depth_right/image_raw','/sim/depth_right/camera_info','camera_right_optical'))
spawn = metadata['spawn']
# Runtime SessionLayer only. Keep the test body fixed at the inherited spawn.
with Usd.EditContext(stage, stage.GetSessionLayer()):
    api = UsdGeom.XformCommonAPI(robot)
    api.SetTranslate(Gf.Vec3d(*spawn))
    api.SetRotate(Gf.Vec3f(0,0,0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
width, height = metadata.get('cameraResolution', [320,240])
# Actual camera sensor cadence as well as physics and timeline are configured.
# Apply only to the SessionLayer; the saved scene and pixel dimensions stay fixed.
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for _, path, *_ in views:
        sensor = stage.GetPrimAtPath(path)
        sensor.ApplyAPI('OmniSensorAPI')
        attr = sensor.GetAttribute('omni:sensor:tickRate')
        if not attr:
            attr = sensor.CreateAttribute('omni:sensor:tickRate', Sdf.ValueTypeNames.Float)
        attr.Set(float(args.camera_hz) if args.sensor_mode == 'rate' else 0.)
calibration = configure_depth_projection(stage, Usd, UsdGeom, views, width, height,
    run_id=args.run_id, scene_path=str(scene))
if args.source_session:
    calibration['input_session']=args.source_session
    calibration['source_generation']=args.source_generation
calibration['scene_sha256'] = hashlib.sha256(scene.read_bytes()).hexdigest()
calibration['depth_type'] = 'optical_axis_z_m'
calibration['body_frame'] = 'base_link'
calibration['requested_camera_hz'] = args.camera_hz
calibration['base_config'] = BASE
calibration['base_config_sha256'] = BASE_SHA
calibration['sensor_mode'] = args.sensor_mode
calibration['actual_sensor_tick_rate'] = [stage.GetPrimAtPath(v[1]).GetAttribute('omni:sensor:tickRate').Get() for v in views]
save_projection_evidence(calibration, out/'simulation.json')
dt = BASE["physics_dt_s"]
SimulationManager.setup_simulation(dt=dt, device='cuda:0')
RenderingManager.set_dt(dt)
settings = carb.settings.get_settings()
settings.set('/app/player/useFixedTimeStepping', True)
settings.set('/app/player/CompensatePlayDelayInSecs', 0.)
settings.set('/persistent/simulation/minFrameRate', float(args.camera_hz))
timeline = omni.timeline.get_timeline_interface()
timeline.set_time_codes_per_second(float(args.camera_hz))
timeline.set_target_framerate(float(args.camera_hz))
timeline.set_ticks_per_frame(1)
timeline.set_play_every_frame(True)
timeline.set_end_time(10000.)

nodes = [('Tick','omni.graph.action.OnPlaybackTick'),
    ('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),
    ('Context','isaacsim.ros2.bridge.ROS2Context'),
    ('Clock','isaacsim.ros2.bridge.ROS2PublishClock'),
    ('Odom','isaacsim.ros2.bridge.ROS2PublishOdometry')]
values = [('Clock.inputs:topicName',topic_prefix+'/clock'),('Odom.inputs:topicName',topic_prefix+'/sim/odom'),
    ('Odom.inputs:odomFrameId','world'),('Odom.inputs:chassisFrameId','base_link'),
    ('Odom.inputs:publishRawVelocities',True),('Odom.inputs:position',spawn),
    ('Odom.inputs:orientation',[0.,0.,0.,1.]),('Odom.inputs:linearVelocity',[0.,0.,0.]),
    ('Odom.inputs:angularVelocity',[0.,0.,0.])]
connections = [('Tick.outputs:tick','Clock.inputs:execIn'),('Tick.outputs:tick','Odom.inputs:execIn'),
    ('Time.outputs:simulationTime','Clock.inputs:timeStamp'),
    ('Time.outputs:simulationTime','Odom.inputs:timeStamp'),
    ('Context.outputs:context','Clock.inputs:context'),('Context.outputs:context','Odom.inputs:context')]
for suffix, path, depth_topic, info_topic, frame in views:
    render, depth, info = 'Render'+suffix, 'Depth'+suffix, 'Info'+suffix
    nodes += [(render,'isaacsim.core.nodes.IsaacCreateRenderProduct'),
              (depth,'isaacsim.ros2.bridge.ROS2CameraHelper'),
              (info,'isaacsim.ros2.bridge.ROS2CameraInfoHelper')]
    values += [(render+'.inputs:cameraPrim',[usdrt.Sdf.Path(path)]),
               (render+'.inputs:width',width),(render+'.inputs:height',height),
               (depth+'.inputs:type','depth'),(depth+'.inputs:topicName',topic_prefix+depth_topic),
               (depth+'.inputs:frameId',frame),(depth+'.inputs:resetSimulationTimeOnStop',True),
               (info+'.inputs:topicName',topic_prefix+info_topic),(info+'.inputs:frameId',frame)]
    connections += [('Tick.outputs:tick',render+'.inputs:execIn')]
    for helper in (depth, info):
        connections += [(render+'.outputs:execOut',helper+'.inputs:execIn'),
                        (render+'.outputs:renderProductPath',helper+'.inputs:renderProductPath'),
                        ('Context.outputs:context',helper+'.inputs:context')]
keys = og.Controller.Keys
graph, _, _, _ = og.Controller.edit({'graph_path':'/World/DepthInputTestGraph','evaluator_name':'execution'},
    {keys.CREATE_NODES:nodes,keys.SET_VALUES:values,keys.CONNECT:connections})
og.Controller.evaluate_sync(graph)
(out/'simulator_ready').write_text(args.run_id+'\n')
started = time.monotonic()
while not (out/'reader_ready').exists():
    if time.monotonic()-started > 120:
        raise RuntimeError('reader did not become ready')
    app.update()
    time.sleep(.02)
# Optional diagnostic capture; no control code or extra image copies in Python.
physics_rows = []
physics_subscriptions = []
profiler = None
if args.profile_stages or args.record_physics:
    import omni.physics.core
    physics = omni.physics.core.get_physics_simulation_interface()
    for pre in (True, False):
        def physics_event(step_dt, event_context, pre=pre):
            physics_rows.append((time.monotonic_ns(), 'pre' if pre else 'post', step_dt))
        physics_subscriptions.append(physics.subscribe_physics_on_step_events(
            pre_step=pre, order=0, on_update=physics_event))
if args.profile_stages:
    import carb.profiler
    profiler = carb.profiler.acquire_profiler_interface(plugin_name='carb.profiler-cpu.plugin')
    settings.set('/plugins/carb.profiler-cpu.plugin/filePath', str(out/'cpu_trace.json.gz'))
    settings.set('/plugins/carb.profiler-cpu.plugin/compressProfile', True)
    profiler.set_capture_mask(0xFFFFFFFFFFFFFFFF)
    settings.set('/plugins/carb.profiler-cpu.plugin/saveProfile', True)
app_utils.play()
# Fixed simulation time per accepted render tick; wall throughput is measured.
settings.set('/app/runLoops/main/rateLimitEnabled', False)
start = time.monotonic()
frames = 0
update_rows = []
wall_step_s = dt / args.time_scale
while app.is_running() and app_utils.is_playing() and not (out/'reader_done').exists():
    frame_wall_start = time.monotonic()
    sim_start_s = timeline.get_current_time()
    if profiler:
        carb.profiler.begin(1, 'EGO4P0_APP_UPDATE')
    app.update()
    if profiler:
        carb.profiler.end(1)
    update_end = time.monotonic()
    sim_end_s = timeline.get_current_time()
    delay = max(0., frame_wall_start + wall_step_s - time.monotonic())
    if args.pace == 'realtime' and delay > 0:
        time.sleep(delay)
    update_rows.append((frames,frame_wall_start,update_end,sim_start_s,sim_end_s,
                        time.monotonic(),delay if args.pace == 'realtime' else 0.,
                        max(0.,update_end-frame_wall_start-wall_step_s)))
    frames += 1
    if time.monotonic()-start > 330:
        raise RuntimeError('reader did not complete within simulator wall timeout')
if profiler:
    settings.set('/plugins/carb.profiler-cpu.plugin/saveProfile', False)
    profiler.set_capture_mask(0)
for subscription in physics_subscriptions:
    subscription.unsubscribe()
if args.profile_stages or args.record_physics:
    with (out/'physics_events.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['wall_monotonic_ns','event','step_dt_s'])
        writer.writerows(physics_rows)
app_utils.stop()
final = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
report = dict(base_config=BASE, base_config_sha256=BASE_SHA, profile_stages=args.profile_stages, record_physics=args.record_physics, run_id=args.run_id, read_only=True, frames_updated=frames,
    wall_elapsed_s=time.monotonic()-start, nominal_step_s=dt,
    requested_camera_hz=args.camera_hz, pacing=args.pace, target_time_scale=args.time_scale,
    target_wall_step_s=wall_step_s, camera_time_domain="simulation", reader_time_domain="wall",
    pacing_policy="per-step minimum wall interval; no catch-up",
    sensor_mode=args.sensor_mode, disable_viewport_updates=args.disable_viewport,
    render_threads=args.render_threads,
    physics_device=str(SimulationManager.get_device()), renderer_active_gpu=0,
    run_loop_rate_limit_enabled=settings.get('/app/runLoops/main/rateLimitEnabled'),
    run_loop_rate_limit_frequency=settings.get('/app/runLoops/main/rateLimitFrequency'),
    timeline_time_codes_per_second=timeline.get_time_codes_per_seconds(),
    actual_physics_dt=SimulationManager.get_physics_dt(),
    actual_render_dt=RenderingManager.get_dt(),
    initial_position=spawn, final_position=list(final.ExtractTranslation()),
    command_publishers_created=0, reader_done=(out/'reader_done').exists(),
    scene_sha256_after=hashlib.sha256(scene.read_bytes()).hexdigest())
(out/'simulation.json').write_text(json.dumps(report,indent=2)+'\n')
with (out/'simulator_updates.csv').open('w',newline='') as stream:
    writer=csv.writer(stream);writer.writerow(['frame','start_wall_s','end_wall_s','sim_start_s','sim_end_s','pacing_end_wall_s','requested_sleep_s','step_budget_overrun_s']);writer.writerows(update_rows)
print('[DEPTH INPUT SIMULATION] '+json.dumps(report),flush=True)
app.close()
