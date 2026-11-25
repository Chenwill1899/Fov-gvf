# P3 simulation-only integration contract

Current sources stay authoritative. New app under `src/isaac_closed_loop/` includes current candidate headers and compiles the same sparse backend; no historical controller overlay. Core production motion flag remains false; this executable is explicitly simulation-only.

## Artifacts and arguments

- C++: `src/isaac_closed_loop/controller.cpp`, optional `flight_supervisor.hpp`, `supervisor_check.cpp`.
- Isaac: `scripts/isaac/run_closed_loop_simulation.py --scenario FILE --output DIR --socket PATH [--headless] [--no-overlay] [--steps N]`.
- Build: `scripts/build_isaac_closed_loop.py --tests` produces isolated executable and manifest under this experiment.
- Launcher: `scripts/run_isaac_closed_loop.py --scenario {straight,climb,offset_obstacle,blocking_obstacle} [--headless] [--check] [--correctness-only] [--steps N]`; GUI is default. Formal mode requires sudo, exactly five C++ FIFO5/CPU0..4 threads, normal simulator. Ordinary mode is explicitly correctness-only, never a timing result.
- C++ args: `--calibration FILE --scenario FILE --output DIR --socket PATH [--correctness-only]`.
- Sim writes actual `simulation.sensors.json`, then `sensors_ready.json`, and waits for the C++ UNIX socket. Supervisor starts C++ only after calibration is ready. Socket resides in a short private /tmp directory; all durable output goes into P3.

## Binary protocol (little endian, version 1)

Magic is uint64 `0x45474F3450330001`. Request is Python struct `<6Q15d` (168 bytes), then four contiguous float32 images, Front/Left/Back/Right, optical-axis Z in metres. Six integers: magic, version=1, sequence starting at 1, simulation_ns, width, height. Doubles: world position xyz, world velocity xyz, row-major R_WB (9). Sequence/time/size/non-finite state checks are mandatory. Exact payload reads with bounded timeout; no indefinite waits or partial-frame use.

Reply is `<6Q12d` (144 bytes). Integers: magic, version, echoed sequence, echoed simulation_ns, decision code, flags. Doubles: plant_input_W xyz, predicted_next_velocity_W xyz, requested_speed, goal_distance, cross_track_error, reference_progress, certified_ahead, controller_compute_ms. Flags: bit0 plan geometry ready, bit1 tracking reference, bit2 goal reached, bit3 obstacle veto, bit4 seed evidence used. Decisions: 0 protection, 1 startup/reference tracking, 2 cruise/reference tracking, 3 goal braking, 4 terminal hold, 5 certified emergency braking, 6 fatal uncertified state. Simulator must not advance on malformed reply or code6; records explicit failure. Predicted velocity must agree with Python velocity_response before execution.

## Scene and evidence

Scenario JSON under src/new_controller/config/isaac_closed_loop is authoritative: static_scene, room bounds, axis-aligned box list, start, goal, body radius .10, four co-located cameras (explicit simulation model; not old ARL geometry), actual near/far .10/50m, dt .005, tau .10, plant acceleration150, max speed12. Finite room gives real depth rather than converting infinity to free pixels. No historical guide or waypoint route. Ground truth only verifies initial seed sphere and performs independent swept collision checks, never normal path selection. Current raw PotentialObstacle and range/input rejection precede all historical/seed clear evidence. Historical raw images are allowed only under the fixed static-scene identity; uncertified Unknown stays rejected. No-hit pixels retain their original invalid status.

Motion directions must come from the current accepted NavigationPipeline/SpatialReference. Add explicit zero-speed startup and goal braking. At rest only, a measured-Available angular start may be selected if the original desired start is blocked; document this new restart behavior, and do not use direct goal/multi-axis motion as a fallback. All nonzero commands and every stopping envelope require current/retained raw-depth or certified-spawn evidence, plus independent simulation sweep. Missing frame or timeout halts safely with explicit status.

The displayed trajectory is the new closed-loop trajectory. Rendered overlays must not become depth obstacles. Scene motion uses the existing first-order kinematic plant, not full quadrotor rigid-body dynamics. Keep all early failures and distinguish ordinary correctness runs from formal FIFO5 results.
