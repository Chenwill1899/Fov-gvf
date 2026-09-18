# Python-to-C++ staged migration plan

> Started: 2026-09-07  
> Branch: `feature/local-modifications`  
> Baseline commit: `804f26e2e631d0571f5e1cc5088100481df1b16d`

## Migration invariant

The current Python runtime is the behavior oracle until the C++ implementation
passes stage-level and closed-loop parity checks. Language migration and behavior
changes are separate work. Known Python safety defects are frozen for comparison
and are corrected only after C++ cutover, with new tests and explicit log entries.

## Runtime scope

The first migration target is the default depth-angular runtime:

- `pc_gvf.depth_angular_core`;
- `pc_gvf.depth_angular_controller`;
- the public topics, messages, parameters, status strings, coordinate conventions,
  QoS choices, and launch behavior used by `depth_angular_demo.launch.py`.

The synthetic demo, visualization publisher, and `pc_gvf_platforms` Python nodes
may remain as migration test infrastructure until the C++ controller is accepted.
Whether all runtime Python must ultimately be removed is a later cutover decision.

The existing `pc_gvf_core` C++ library is not called by the default Python launch.
It must remain untouched until C++ parity is achieved and its independent 2D/3D
API retention decision is made. Removing it early would reduce repository
functionality even though the default demo would still run.

## Stage checkpoints

### Stage 1 — Freeze Python behavior

Status: implemented; validation results are recorded in `WORK_LOG.md`.

- Store deterministic inputs and intermediate/final outputs in a language-neutral
  `key=value` fixture format.
- Cover all seven built-in scenes and important edge conditions.
- Add a Python regression that proves the fixtures still describe the oracle.
- Exclude nondeterministic wall-clock timing.

Exit gate:

- fixture generator completes under the supported Humble Python environment;
- fixture regression and existing `pc_gvf` tests pass;
- fixture manifest identifies the exact Python core source hash.

### Stage 2 — Port the ROS-independent core

Status: implemented; validation results are recorded in `WORK_LOG.md`.

Progress:

- Stage 2.1 completed on 2026-09-07: the package now supports mixed
  `ament_cmake`/Python installation; C++ math helpers, the pinhole camera model,
  quaternion/fixed-camera transforms, and a standard-library fixture reader are
  built and tested while all existing Python executable names remain available.
- Stage 2.2 completed on 2026-09-07: C++ depth backprojection preserves the
  Python finite/range/stride filtering and row-major point ordering; C++
  collision-cone free distance preserves the sensor-range cap, strict cone
  membership test, sphere-contact calculation, and chunk-independent output.
  All 16 Python fixtures now expose obstacle points as a separately checked
  intermediate result.
- Stage 2.3 completed on 2026-09-07: C++ safe-goal selection preserves the
  Python row-major tie breaking, eight-neighbor components, Euclidean-clearance
  reward, previous-goal hysteresis, and deterministic image-left bias. The C++
  harmonic solver preserves source/goal disks, four-neighbor Laplacian assembly,
  obstacle/FOV zero-flux boundaries, and convergence validity. Fixtures now
  expose raw nullable selection results, component labels, and direct harmonic
  solutions independently of final command composition.
- Stage 2.4 completed on 2026-09-07: C++ now preserves the Python discrete
  harmonic descent path and tie order, angular distance/rate limiting,
  reference-line convergence gate, bilinear free-distance sampling, braking
  law, swept-point rollout, FOV rejection, and fixed ten-step binary speed
  limit. Fixtures expose each direction and speed intermediate independently.
- Stage 2.5 completed on 2026-09-07: the migrated slices are composed into a
  single C++ `computeGuidance()` operating on immutable depth and state inputs.
  Its full diagnostic result and final world-frame command pass all 16 Python
  fixtures, including unavailable-field fallback, goal behind the camera,
  alternate intrinsics, history, lateral velocity, and reference convergence.
- Stage 3 completed on 2026-09-07: a non-actuating C++ ROS node now mirrors
  Python message decoding, state handling, core invocation, acceleration limiting,
  status transitions, and `PositionCommand` construction. Its command, status,
  and JSON diagnostics are confined to `/depth_angular_shadow/*`. A deterministic
  ROS probe drives both implementations, checks the C++ diagnostics against a
  sequential Python oracle, compares settled commands and statuses, and verifies
  that the shadow node never publishes `/position_cmd`.
- Stage 4.1 completed on 2026-09-07: C++ now reproduces the seven-marker angular
  field visualization on an isolated shadow topic. An online comparison checks
  marker identity, type, point/color counts, frames, three-dimensional geometry,
  and RGB data against the Python controller; the accepted static snapshot has
  zero geometric error.
- Stage 4.2 completed on 2026-09-07: the public `depth_angular_controller`
  executable is now C++ and owns the selected command/status/field interfaces.
  The original Python controller remains installed as
  `depth_angular_controller_py`, and launch accepts `controller:=python` for an
  explicit rollback while defaulting to `controller:=cpp`.
- Stage 4.3 completed on 2026-09-07: all seven ROS closed-loop scenarios passed
  without body collision, command publication remained at approximately 50 Hz,
  five scenarios reached the goal, and the two geometrically unavailable cases
  settled to zero velocity. Dedicated input sequencing also verified every
  frozen status, sustained zero command in protection states, both supported
  depth encodings, and unchanged `PositionCommand` defaults.
- Stage 5.1 completed on 2026-09-07: the superseded Python ROS controller,
  executable wrappers, launch fallback, console entry, and controller-only
  tests have been removed. The Python numerical core and synthetic simulator
  remain non-production acceptance infrastructure; the independent legacy C++
  package remains until its 2-D/3-D APIs are reproduced or explicitly retired.
- Stage 5.2 completed on 2026-09-07: the temporary C++ shadow executable,
  launch switch, JSON parity diagnostics, and online dual-implementation probe
  have been retired. The node source now has its permanent controller name and
  is built once. Stage 5 is complete; Stage 6 is the next implementation phase.

Port in independently tested slices:

1. math helpers, camera projection, and coordinate transforms;
2. depth backprojection and collision-cone free distance;
3. safe-goal selection and connected components;
4. angular harmonic solve;
5. discrete path and angular-rate limit;
6. reference-line convergence;
7. braking limit, rollout, and binary speed limit;
8. `computeGuidance()` composition and result diagnostics.

The first C++ version preserves current Python conventions, including four- versus
eight-neighbor choices, sampling stride, and current invalid-depth behavior. These
are migrated defects, not approved final safety semantics.

### Stage 3 — Shadow-mode ROS integration

Status: implemented; validation results are recorded in `WORK_LOG.md`.

- Build Python and C++ controllers together temporarily.
- Feed identical immutable sensor snapshots to both.
- Publish C++ comparison commands on a non-actuating topic.
- Compare statuses, intermediate diagnostics, command direction, and speed.
- Only one controller may command the simulator or vehicle.

The demo launch exposes `shadow:=true`, defaulting to `false`. The selected
primary controller owns `/position_cmd`; the shadow process remains non-actuating
and publishes comparison data only under `/depth_angular_shadow/command`,
`/status`, `/diagnostics`, and `/angular_field`.

### Stage 4 — C++ cutover

Status: implemented; validation results are recorded in `WORK_LOG.md`.

Progress:

- Stage 4.1 completed on 2026-09-07: the C++ shadow node publishes the same
  unsafe samples, angular quiver, camera frustum, reference/goal/command rays,
  and depth-hit points as Python. Online comparison keeps both visualization
  topics isolated and validates the latest converged frame.
- Stage 4.2 completed on 2026-09-07: one C++ node source is compiled into an
  active public executable and an isolated shadow executable. The active build
  declares `cmd_topic`, publishes `/pc_gvf/angular_field`, and keeps the original
  node name and private status path. Launch defaults to C++ and retains a named
  Python fallback. The controller timer uses the node ROS clock, preserving
  simulated-time behavior.
- Stage 4.3 completed on 2026-09-07: a Release build sustained 50 Hz in all
  seven built-in ROS scenarios. `empty`, `single_pillar`, `offset_box`,
  `center_sphere`, and `diagonal_gap` reached the goal without collision;
  `overhead_bar` and `narrow_gate` produced stable zero-velocity safe stops.
  A separate state probe covered `WAITING_ODOMETRY`, `STALE_INTENT`,
  `ZERO_INTENT`, `WAITING_DEPTH`, `NAVIGATING`, `STALE_DEPTH`,
  `INVALID_ODOMETRY`, `INVALID_GUIDANCE`, and `GOAL_REACHED`.

- Make the C++ node own the existing executable and public ROS interface.
- Run clean-build, unit, launch, frequency, visualization, and seven-scene
  closed-loop acceptance checks.
- Preserve parameter defaults, topic names, message fields, status strings, and
  launch commands unless a separately logged behavior change is approved.

### Stage 5 — Remove superseded implementations

Status: implemented; validation results are recorded in `WORK_LOG.md`.

Progress:

- Stage 5.1 completed on 2026-09-07: removed the Python controller module,
  both legacy controller wrapper scripts, the Python rollback launch branch,
  the stale setuptools console entry, and controller-only Python visualization
  tests. A clean install exposes only the C++ public controller, C++ shadow,
  Python numerical oracle, and Python synthetic simulator; it contains neither
  `depth_angular_controller_py` nor a Python controller module.
- The retained `depth_angular_core.py` is not a selectable runtime controller.
  It still supplies deterministic fixtures, scene geometry, the synthetic demo,
  and acceptance probes, so deleting it in the same step would discard the
  migration oracle rather than merely remove a superseded implementation.
- A fresh Stage 5.1 build passed all 18 remaining tests. A clean-install `empty`
  ROS closed loop still reached the goal without collision at approximately
  50 Hz, with the public command and field topics owned only by the C++ node.
- Stage 5.2 completed on 2026-09-07: removed the second shadow build target,
  `shadow` launch argument and process, migration-only JSON diagnostics, and
  `ros_shadow_parity_probe.py`. Renamed `depth_angular_shadow_node.cpp` to
  `depth_angular_controller_node.cpp` and eliminated the compile-time mode
  branch, leaving one permanent controller implementation and executable.
- The Stage 5.2 clean install exposes exactly three `pc_gvf` executables:
  the C++ controller plus the retained Python numerical-oracle and synthetic
  simulator commands. All 18 tests, the full safety-state probe, and a 50 Hz
  clean-install closed loop passed after the cleanup.
- Retention scope is now explicit: Python scene/oracle infrastructure remains
  for regression and simulation, while `pc_gvf_core` remains an independent
  C++ library because its 2-D/3-D APIs are not superseded by the angular runtime.

- Remove Python algorithm/controller only after Stage 4 acceptance.
- Decide whether the Python simulator and platform utilities are also in scope.
- Remove legacy `pc_gvf_core` only if its independent 2D/3D features are explicitly
  retired or reproduced in the new C++ architecture.
- Verify from a clean build directory so stale install artifacts cannot hide a
  missing dependency or executable.

### Stage 6 — Implement new functionality on C++

Status: pending; this is the next phase.

After parity, address the recorded safety and CPU roadmap: conservative full-depth
coverage, explicit unknown semantics, consistent connectivity, final-command
continuous-motion and braking certification, real data-age handling, fixed memory,
parallel kernels, and end-to-end latency acceptance on the deployment CPU.

## Frozen public contract

The current controller consumes odometry, depth `Image`, `CameraInfo`, and
`TwistStamped` intent, and publishes `pc_gvf_msgs/PositionCommand`, status, and
angular-field visualization. It accepts `32FC1` metres and `16UC1` millimetres.

Default controller topics:

| Interface | Default |
|---|---|
| odometry | `/sim/odom` |
| depth | `/sim/depth/image_raw` |
| camera info | `/sim/depth/camera_info` |
| intent | `/human_intent` |
| command | `/position_cmd` |
| status | `~/status` |
| field visualization | `/pc_gvf/angular_field` |

Status strings to preserve include `WAITING_ODOMETRY`, `INVALID_ODOMETRY`,
`GOAL_REACHED`, `STALE_INTENT`, `ZERO_INTENT`, `WAITING_DEPTH`, `STALE_DEPTH`,
`INVALID_GUIDANCE`, `NAVIGATING`, and `DEGRADED`.

The controller timer is currently 20 ms. The core `SimConfig.control_dt` remains
50 ms; this mismatch is deliberately frozen for parity and corrected later.

## Numerical comparison policy

| Output | Stage-2 comparison |
|---|---|
| depth dimensions and mask | exact |
| connected component and chosen pixels | exact |
| free distance | `atol=1e-9`, `rtol=1e-9` |
| harmonic potential | `atol=1e-6`, `rtol=1e-6`, equal NaN layout |
| command pixel/direction/speed | `atol=1e-6`, `rtol=1e-6` |
| field validity and controller status | exact |

CG iteration order can differ between SciPy and Eigen. Acceptance therefore checks
the residual and behavior outputs rather than requiring bit-identical potentials.
