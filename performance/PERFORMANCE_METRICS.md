# USER-EGO performance metrics

This append-only document is populated by
`scripts/run_isaac_fov_gvf_navigation.sh`. Each run has one shared run ID and
up to three component sections:

- **Controller** measures the C++ avoidance solve, command period, depth-frame
  age at command generation, full callback-to-publish time, and DDS publish
  call time.
- **Command bridge** measures `/position_cmd` reception, conversion, and
  `/sim/cmd_vel` publication. The controller-stamp-to-bridge value is the
  measurable algorithm-output-to-UAV-command-interface latency.
- **Isaac command application** measures the 60 Hz OmniGraph command sampling
  loop, applied control-frame count, runtime frame rate, and command-change
  intervals.

ROS-time metrics use simulation time and steady-clock metrics use host wall
time. Isaac's `ROS2SubscribeTwist` output has no message timestamp, so the final
bridge-to-OmniGraph transport delay cannot be measured exactly without changing
the UAV command interface; the Isaac section instead reports its 60 Hz sampling
interval and actual applied-command changes.


### Run metrics_smoke_20260910

- Started: 2026-09-10 17:24:24 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Controller — metrics_smoke_20260910

- Wall duration: 8.400 s
- Timer callbacks / command frames / guidance frames: 0 / 0 / 0
- Command FPS / guidance FPS: 0.000 / 0.000 Hz
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- DDS publish call mean / P95 / max: 0.000 / 0.000 / 0.000 ms (steady clock)

#### Command bridge — metrics_smoke_20260910

- Wall duration: 8.293 s
- Received / published control frames: 0 / 0
- Published control FPS: 0.000 Hz
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)

### Run metrics_smoke_20260910_privileged

- Started: 2026-09-10 17:24:53 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — metrics_smoke_20260910_privileged

- Result: TIMEOUT
- Wall duration / simulation duration: 4.301 / 8.017 s
- Applied simulation control frames: 483
- Runtime simulation FPS: 112.293 Hz (wall clock)
- Isaac frame interval mean / P50 / P95 / max: 8.454 / 7.181 / 9.319 / 220.155 ms
- Distinct applied command values: 393
- Applied-command change interval mean / P50 / P95 / max: 10.373 / 7.396 / 15.773 / 226.730 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — metrics_smoke_20260910_privileged

- Wall duration: 19.735 s
- Timer callbacks / command frames / guidance frames: 401 / 401 / 398
- Command FPS / guidance FPS: 20.319 / 20.167 Hz
- Avoidance compute mean / P50 / P95 / max: 1.152 / 1.073 / 1.632 / 3.102 ms
- Command interval mean / P50 / P95 / max: 10.145 / 7.521 / 15.262 / 221.891 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 16.750 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.846 / 1.771 / 2.566 / 3.770 ms (steady clock)
- DDS publish call mean / P95 / max: 0.070 / 0.421 / 1.234 ms (steady clock)

#### Command bridge — metrics_smoke_20260910_privileged

- Wall duration: 19.624 s
- Received / published control frames: 401 / 401
- Published control FPS: 20.434 Hz
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.042 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 10.144 / 7.536 / 15.298 / 221.930 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.074 / 0.069 / 0.108 / 0.174 ms (steady clock)

### Run metrics_smoke_corrected_20260910

- Started: 2026-09-10 17:28:06 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — metrics_smoke_corrected_20260910

- Result: TIMEOUT
- Wall duration / simulation duration: 3.091 / 5.017 s
- Applied simulation control frames: 303
- Runtime FPS (simulation time / wall time): 60.399 / 98.039 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.603 / 7.315 / 9.448 / 222.239 ms
- Distinct applied command values: 247
- Applied-command change interval mean / P50 / P95 / max: 11.750 / 7.607 / 16.819 / 229.671 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — metrics_smoke_corrected_20260910

- Wall duration: 16.654 s
- Timer callbacks / command frames / guidance frames: 252 / 251 / 248
- Command FPS (ROS simulation time / active wall time): 50.000 / 86.702 Hz
- Guidance FPS (active wall time): 86.926 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.025 / 0.932 / 1.406 / 3.181 ms
- Command interval mean / P50 / P95 / max: 11.534 / 7.563 / 17.898 / 223.168 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 16.801 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.703 / 1.645 / 2.509 / 3.987 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.414 / 1.301 ms (steady clock)

#### Command bridge — metrics_smoke_corrected_20260910

- Wall duration: 16.543 s
- Received / published control frames: 251 / 251
- Published control FPS (ROS simulation time / active wall time): 50.000 / 86.707 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.066 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.533 / 7.605 / 17.889 / 223.302 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.078 / 0.073 / 0.111 / 0.189 ms (steady clock)

### Run 20260910_203943

- Started: 2026-09-10 20:39:43 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_203943

- Result: GOAL_REACHED
- Wall duration / simulation duration: 44.656 / 70.367 s
- Applied simulation control frames: 4223
- Runtime FPS (simulation time / wall time): 60.014 / 94.568 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.551 / 7.833 / 11.272 / 208.245 ms
- Distinct applied command values: 3451
- Applied-command change interval mean / P50 / P95 / max: 12.910 / 8.212 / 67.205 / 208.245 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_203943

- Wall duration: 60.933 s
- Timer callbacks / command frames / guidance frames: 3519 / 3518 / 3516
- Command FPS (ROS simulation time / active wall time): 49.993 / 78.972 Hz
- Guidance FPS (active wall time): 79.007 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.003 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.048 / 0.958 / 1.547 / 3.317 ms
- Command interval mean / P50 / P95 / max: 12.663 / 8.238 / 22.974 / 277.401 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.468 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.789 / 1.712 / 2.593 / 5.199 ms (steady clock)
- DDS publish call mean / P95 / max: 0.055 / 0.386 / 1.862 ms (steady clock)

#### Command bridge — 20260910_203943

- Wall duration: 60.821 s
- Received / published control frames: 3518 / 3518
- Published control FPS (ROS simulation time / active wall time): 49.993 / 78.972 Hz
- Command stamp interval mean / P50 / P95 / max: 20.003 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 12.663 / 8.265 / 23.007 / 276.998 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.071 / 0.067 / 0.094 / 2.141 ms (steady clock)

### Run 20260910_205409

- Started: 2026-09-10 20:54:09 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_205409

- Result: GOAL_REACHED
- Wall duration / simulation duration: 40.108 / 67.967 s
- Applied simulation control frames: 4079
- Runtime FPS (simulation time / wall time): 60.015 / 101.702 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.817 / 7.736 / 10.338 / 219.293 ms
- Distinct applied command values: 3272
- Applied-command change interval mean / P50 / P95 / max: 12.234 / 8.140 / 19.114 / 219.293 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_205409

- Wall duration: 54.104 s
- Timer callbacks / command frames / guidance frames: 3399 / 3398 / 3397
- Command FPS (ROS simulation time / active wall time): 49.993 / 84.871 Hz
- Guidance FPS (active wall time): 84.901 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.003 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.060 / 0.983 / 1.542 / 4.020 ms
- Command interval mean / P50 / P95 / max: 11.783 / 8.191 / 17.861 / 288.416 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.785 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.329 / 1.178 / 2.239 / 5.988 ms (steady clock)
- DDS publish call mean / P95 / max: 0.070 / 0.388 / 1.548 ms (steady clock)

#### Command bridge — 20260910_205409

- Wall duration: 53.715 s
- Received / published control frames: 3398 / 3398
- Published control FPS (ROS simulation time / active wall time): 49.993 / 84.874 Hz
- Command stamp interval mean / P50 / P95 / max: 20.003 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.782 / 8.188 / 17.845 / 288.417 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.076 / 0.071 / 0.103 / 1.071 ms (steady clock)

### Run 20260910_205520

- Started: 2026-09-10 20:55:20 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_205520

- Result: GOAL_REACHED
- Wall duration / simulation duration: 42.704 / 72.983 s
- Applied simulation control frames: 4380
- Runtime FPS (simulation time / wall time): 60.014 / 102.567 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.735 / 7.711 / 10.250 / 213.821 ms
- Distinct applied command values: 3517
- Applied-command change interval mean / P50 / P95 / max: 12.125 / 8.100 / 18.714 / 213.821 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_205520

- Wall duration: 56.355 s
- Timer callbacks / command frames / guidance frames: 3650 / 3649 / 3647
- Command FPS (ROS simulation time / active wall time): 49.995 / 85.590 Hz
- Guidance FPS (active wall time): 85.763 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.052 / 0.970 / 1.561 / 4.170 ms
- Command interval mean / P50 / P95 / max: 11.684 / 8.171 / 17.805 / 224.231 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 18.211 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.304 / 1.157 / 2.159 / 4.710 ms (steady clock)
- DDS publish call mean / P95 / max: 0.070 / 0.387 / 2.375 ms (steady clock)

#### Command bridge — 20260910_205520

- Wall duration: 56.345 s
- Received / published control frames: 3649 / 3649
- Published control FPS (ROS simulation time / active wall time): 49.995 / 85.591 Hz
- Command stamp interval mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.684 / 8.172 / 17.680 / 223.445 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.072 / 0.069 / 0.096 / 0.858 ms (steady clock)

### Run 20260910_205622

- Started: 2026-09-10 20:56:22 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_205622

- Result: GOAL_REACHED
- Wall duration / simulation duration: 45.070 / 75.083 s
- Applied simulation control frames: 4506
- Runtime FPS (simulation time / wall time): 60.013 / 99.979 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.988 / 7.774 / 10.331 / 218.639 ms
- Distinct applied command values: 3512
- Applied-command change interval mean / P50 / P95 / max: 12.816 / 8.187 / 21.311 / 276.248 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_205622

- Wall duration: 58.696 s
- Timer callbacks / command frames / guidance frames: 3755 / 3754 / 3752
- Command FPS (ROS simulation time / active wall time): 49.996 / 83.426 Hz
- Guidance FPS (active wall time): 83.465 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.051 / 0.981 / 1.516 / 3.905 ms
- Command interval mean / P50 / P95 / max: 11.987 / 8.165 / 18.230 / 230.576 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.626 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.289 / 1.150 / 2.130 / 5.465 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.376 / 2.931 ms (steady clock)

#### Command bridge — 20260910_205622

- Wall duration: 58.304 s
- Received / published control frames: 3754 / 3754
- Published control FPS (ROS simulation time / active wall time): 49.996 / 83.427 Hz
- Command stamp interval mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.004 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.987 / 8.191 / 18.112 / 230.611 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.077 / 0.071 / 0.103 / 1.672 ms (steady clock)

### Run 20260910_205723

- Started: 2026-09-10 20:57:23 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_205723

- Result: GOAL_REACHED
- Wall duration / simulation duration: 47.966 / 73.233 s
- Applied simulation control frames: 4395
- Runtime FPS (simulation time / wall time): 60.014 / 91.627 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.899 / 8.320 / 11.271 / 224.256 ms
- Distinct applied command values: 3475
- Applied-command change interval mean / P50 / P95 / max: 13.783 / 8.717 / 67.769 / 348.750 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_205723

- Wall duration: 61.602 s
- Timer callbacks / command frames / guidance frames: 3663 / 3662 / 3660
- Command FPS (ROS simulation time / active wall time): 50.002 / 76.462 Hz
- Guidance FPS (active wall time): 76.495 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.049 / 0.972 / 1.533 / 3.550 ms
- Command interval mean / P50 / P95 / max: 13.078 / 8.763 / 21.332 / 234.451 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.436 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.292 / 1.150 / 2.136 / 4.043 ms (steady clock)
- DDS publish call mean / P95 / max: 0.069 / 0.398 / 2.672 ms (steady clock)

#### Command bridge — 20260910_205723

- Wall duration: 61.484 s
- Received / published control frames: 3662 / 3662
- Published control FPS (ROS simulation time / active wall time): 50.002 / 76.462 Hz
- Command stamp interval mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 13.078 / 8.743 / 21.368 / 234.199 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.076 / 0.073 / 0.097 / 1.530 ms (steady clock)

### Run 20260910_205827

- Started: 2026-09-10 20:58:27 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_205827

- Result: GOAL_REACHED
- Wall duration / simulation duration: 45.265 / 67.333 s
- Applied simulation control frames: 4041
- Runtime FPS (simulation time / wall time): 60.015 / 89.275 Hz
- Isaac frame interval mean / P50 / P95 / max: 11.185 / 8.343 / 11.793 / 230.694 ms
- Distinct applied command values: 3255
- Applied-command change interval mean / P50 / P95 / max: 13.887 / 8.759 / 68.220 / 240.791 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_205827

- Wall duration: 59.190 s
- Timer callbacks / command frames / guidance frames: 3368 / 3368 / 3365
- Command FPS (ROS simulation time / active wall time): 50.005 / 74.399 Hz
- Guidance FPS (active wall time): 74.542 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.998 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.015 / 0.944 / 1.450 / 3.657 ms
- Command interval mean / P50 / P95 / max: 13.441 / 8.772 / 68.875 / 241.230 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.053 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.278 / 1.117 / 2.079 / 4.114 ms (steady clock)
- DDS publish call mean / P95 / max: 0.075 / 0.409 / 1.922 ms (steady clock)

#### Command bridge — 20260910_205827

- Wall duration: 59.079 s
- Received / published control frames: 3368 / 3368
- Published control FPS (ROS simulation time / active wall time): 50.005 / 74.400 Hz
- Command stamp interval mean / P50 / P95 / max: 19.998 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 13.441 / 8.761 / 68.602 / 241.000 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.078 / 0.074 / 0.099 / 1.985 ms (steady clock)

### Run 20260910_205929

- Started: 2026-09-10 20:59:29 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_205929

- Result: GOAL_REACHED
- Wall duration / simulation duration: 43.056 / 68.617 s
- Applied simulation control frames: 4118
- Runtime FPS (simulation time / wall time): 60.015 / 95.643 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.440 / 8.238 / 10.869 / 223.420 ms
- Distinct applied command values: 3286
- Applied-command change interval mean / P50 / P95 / max: 13.082 / 8.622 / 23.003 / 223.420 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_205929

- Wall duration: 56.995 s
- Timer callbacks / command frames / guidance frames: 3432 / 3431 / 3429
- Command FPS (ROS simulation time / active wall time): 50.000 / 79.818 Hz
- Guidance FPS (active wall time): 79.863 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.037 / 0.947 / 1.486 / 3.980 ms
- Command interval mean / P50 / P95 / max: 12.529 / 8.699 / 19.274 / 233.237 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.896 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.306 / 1.151 / 2.158 / 6.710 ms (steady clock)
- DDS publish call mean / P95 / max: 0.076 / 0.401 / 2.148 ms (steady clock)

#### Command bridge — 20260910_205929

- Wall duration: 56.884 s
- Received / published control frames: 3431 / 3431
- Published control FPS (ROS simulation time / active wall time): 50.000 / 79.819 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 12.528 / 8.697 / 19.370 / 233.242 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.074 / 0.071 / 0.096 / 0.596 ms (steady clock)

### Run 20260910_210029

- Started: 2026-09-10 21:00:29 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_210029

- Result: GOAL_REACHED
- Wall duration / simulation duration: 39.751 / 67.433 s
- Applied simulation control frames: 4047
- Runtime FPS (simulation time / wall time): 60.015 / 101.809 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.807 / 7.882 / 10.291 / 230.709 ms
- Distinct applied command values: 3243
- Applied-command change interval mean / P50 / P95 / max: 12.239 / 8.276 / 19.398 / 230.709 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_210029

- Wall duration: 53.539 s
- Timer callbacks / command frames / guidance frames: 3373 / 3373 / 3370
- Command FPS (ROS simulation time / active wall time): 50.005 / 84.845 Hz
- Guidance FPS (active wall time): 85.158 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.998 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.021 / 0.948 / 1.418 / 4.002 ms
- Command interval mean / P50 / P95 / max: 11.786 / 8.299 / 17.957 / 241.563 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.784 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.294 / 1.136 / 2.122 / 4.046 ms (steady clock)
- DDS publish call mean / P95 / max: 0.082 / 0.417 / 2.288 ms (steady clock)

#### Command bridge — 20260910_210029

- Wall duration: 53.528 s
- Received / published control frames: 3373 / 3373
- Published control FPS (ROS simulation time / active wall time): 50.005 / 84.846 Hz
- Command stamp interval mean / P50 / P95 / max: 19.998 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.786 / 8.308 / 18.027 / 241.587 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.075 / 0.071 / 0.096 / 0.997 ms (steady clock)

### Run 20260910_211427

- Started: 2026-09-10 21:14:27 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_211427

- Result: GOAL_REACHED
- Wall duration / simulation duration: 42.569 / 70.483 s
- Applied simulation control frames: 4230
- Runtime FPS (simulation time / wall time): 60.014 / 99.368 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.049 / 7.908 / 10.429 / 224.029 ms
- Distinct applied command values: 3375
- Applied-command change interval mean / P50 / P95 / max: 12.595 / 8.322 / 21.391 / 224.029 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_211427

- Wall duration: 56.515 s
- Timer callbacks / command frames / guidance frames: 3525 / 3524 / 3522
- Command FPS (ROS simulation time / active wall time): 49.995 / 82.921 Hz
- Guidance FPS (active wall time): 82.969 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.038 / 0.945 / 1.599 / 4.160 ms
- Command interval mean / P50 / P95 / max: 12.060 / 8.334 / 18.339 / 233.939 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.717 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.317 / 1.122 / 2.252 / 4.437 ms (steady clock)
- DDS publish call mean / P95 / max: 0.079 / 0.418 / 2.457 ms (steady clock)

#### Command bridge — 20260910_211427

- Wall duration: 56.402 s
- Received / published control frames: 3524 / 3524
- Published control FPS (ROS simulation time / active wall time): 49.995 / 82.922 Hz
- Command stamp interval mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 12.059 / 8.345 / 18.242 / 233.974 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.074 / 0.070 / 0.102 / 0.700 ms (steady clock)

### Run 20260910_211527

- Started: 2026-09-10 21:15:27 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_211527

- Result: GOAL_REACHED
- Wall duration / simulation duration: 39.324 / 68.100 s
- Applied simulation control frames: 4087
- Runtime FPS (simulation time / wall time): 60.015 / 103.932 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.606 / 7.719 / 10.111 / 226.603 ms
- Distinct applied command values: 3267
- Applied-command change interval mean / P50 / P95 / max: 12.018 / 8.103 / 18.756 / 235.552 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_211527

- Wall duration: 53.308 s
- Timer callbacks / command frames / guidance frames: 3406 / 3405 / 3403
- Command FPS (ROS simulation time / active wall time): 49.998 / 86.747 Hz
- Guidance FPS (active wall time): 86.802 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.065 / 0.952 / 1.739 / 4.637 ms
- Command interval mean / P50 / P95 / max: 11.528 / 8.217 / 17.580 / 236.946 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 18.792 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.347 / 1.160 / 2.329 / 5.713 ms (steady clock)
- DDS publish call mean / P95 / max: 0.081 / 0.420 / 3.117 ms (steady clock)

#### Command bridge — 20260910_211527

- Wall duration: 52.918 s
- Received / published control frames: 3405 / 3405
- Published control FPS (ROS simulation time / active wall time): 49.998 / 86.748 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.528 / 8.220 / 17.566 / 236.937 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.073 / 0.070 / 0.099 / 0.754 ms (steady clock)

### Run 20260910_211702

- Started: 2026-09-10 21:17:02 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260910_211702

- Result: RUNNING
- Wall duration / simulation duration: 53.159 / 4.783 s
- Applied simulation control frames: 289
- Runtime FPS (simulation time / wall time): 60.418 / 5.437 Hz
- Isaac frame interval mean / P50 / P95 / max: 184.963 / 7.836 / 67.605 / 10015.123 ms
- Distinct applied command values: 231
- Applied-command change interval mean / P50 / P95 / max: 230.801 / 8.189 / 68.906 / 10015.123 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260910_211702

- Wall duration: 81.342 s
- Timer callbacks / command frames / guidance frames: 240 / 239 / 237
- Command FPS (ROS simulation time / active wall time): 49.930 / 5.526 Hz
- Guidance FPS (active wall time): 5.486 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.028 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.004 / 0.945 / 1.352 / 2.327 ms
- Command interval mean / P50 / P95 / max: 180.959 / 8.282 / 68.534 / 20020.326 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 18.987 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.260 / 1.050 / 2.020 / 3.856 ms (steady clock)
- DDS publish call mean / P95 / max: 0.081 / 0.453 / 0.918 ms (steady clock)

#### Command bridge — 20260910_211702

- Wall duration: 81.228 s
- Received / published control frames: 239 / 239
- Published control FPS (ROS simulation time / active wall time): 49.930 / 5.526 Hz
- Command stamp interval mean / P50 / P95 / max: 20.028 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 180.957 / 8.311 / 68.716 / 20019.982 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.078 / 0.075 / 0.111 / 0.237 ms (steady clock)

### Run 20260911_104615

- Started: 2026-09-11 10:46:15 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260911_104615

- Result: GOAL_REACHED
- Wall duration / simulation duration: 42.214 / 70.983 s
- Applied simulation control frames: 4260
- Runtime FPS (simulation time / wall time): 60.014 / 100.913 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.886 / 7.681 / 11.119 / 223.147 ms
- Distinct applied command values: 3395
- Applied-command change interval mean / P50 / P95 / max: 12.406 / 8.138 / 21.464 / 223.147 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260911_104615

- Wall duration: 59.518 s
- Timer callbacks / command frames / guidance frames: 3550 / 3549 / 3547
- Command FPS (ROS simulation time / active wall time): 49.995 / 84.284 Hz
- Guidance FPS (active wall time): 84.337 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.029 / 0.955 / 1.448 / 3.271 ms
- Command interval mean / P50 / P95 / max: 11.865 / 8.280 / 18.827 / 232.117 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 18.570 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.294 / 1.138 / 2.182 / 4.660 ms (steady clock)
- DDS publish call mean / P95 / max: 0.077 / 0.409 / 2.332 ms (steady clock)

#### Command bridge — 20260911_104615

- Wall duration: 59.397 s
- Received / published control frames: 3549 / 3549
- Published control FPS (ROS simulation time / active wall time): 49.995 / 84.286 Hz
- Command stamp interval mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.005 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.864 / 8.261 / 18.764 / 232.046 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.074 / 0.070 / 0.099 / 0.851 ms (steady clock)

### Run 20260911_184415

- Started: 2026-09-11 18:44:15 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260911_184415

- Result: COLLISION_GUARD
- Wall duration / simulation duration: 0.110 / 0.000 s
- Applied simulation control frames: 1
- Runtime FPS (simulation time / wall time): 1000000000.000 / 9.054 Hz
- Isaac frame interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Distinct applied command values: 1
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260911_184415

- Wall duration: 16.264 s
- Timer callbacks / command frames / guidance frames: 1 / 0 / 0
- Command FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- DDS publish call mean / P95 / max: 0.000 / 0.000 / 0.000 ms (steady clock)

#### Command bridge — 20260911_184415

- Wall duration: 16.152 s
- Received / published control frames: 0 / 0
- Published control FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Command stamp interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)

### Run 20260911_184441

- Started: 2026-09-11 18:44:41 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260911_184441

- Result: COLLISION_GUARD
- Wall duration / simulation duration: 0.073 / 0.000 s
- Applied simulation control frames: 1
- Runtime FPS (simulation time / wall time): 1000000000.000 / 13.613 Hz
- Isaac frame interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Distinct applied command values: 1
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260911_184441

- Wall duration: 14.043 s
- Timer callbacks / command frames / guidance frames: 1 / 0 / 0
- Command FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- DDS publish call mean / P95 / max: 0.000 / 0.000 / 0.000 ms (steady clock)

#### Command bridge — 20260911_184441

- Wall duration: 14.031 s
- Received / published control frames: 0 / 0
- Published control FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Command stamp interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)

### Run 20260911_185133

- Started: 2026-09-11 18:51:33 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260911_185133

- Result: TIMEOUT
- Wall duration / simulation duration: 123.951 / 240.017 s
- Applied simulation control frames: 14403
- Runtime FPS (simulation time / wall time): 60.008 / 116.199 Hz
- Isaac frame interval mean / P50 / P95 / max: 8.602 / 7.141 / 9.465 / 222.677 ms
- Distinct applied command values: 5884
- Applied-command change interval mean / P50 / P95 / max: 21.029 / 8.044 / 68.978 / 734.629 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260911_185133

- Wall duration: 137.462 s
- Timer callbacks / command frames / guidance frames: 12002 / 12001 / 11999
- Command FPS (ROS simulation time / active wall time): 50.000 / 96.877 Hz
- Guidance FPS (active wall time): 96.899 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.117 / 1.082 / 1.385 / 3.933 ms
- Command interval mean / P50 / P95 / max: 10.322 / 7.525 / 16.367 / 232.102 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 22.798 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.235 / 1.159 / 1.778 / 3.964 ms (steady clock)
- DDS publish call mean / P95 / max: 0.082 / 0.422 / 2.114 ms (steady clock)

#### Command bridge — 20260911_185133

- Wall duration: 137.352 s
- Received / published control frames: 12001 / 12001
- Published control FPS (ROS simulation time / active wall time): 50.000 / 96.878 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.001 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 10.322 / 7.537 / 16.399 / 232.113 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.071 / 0.066 / 0.095 / 1.760 ms (steady clock)

### Run 20260911_185403

- Started: 2026-09-11 18:54:03 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260911_185403

- Result: TIMEOUT
- Wall duration / simulation duration: 137.984 / 240.017 s
- Applied simulation control frames: 14403
- Runtime FPS (simulation time / wall time): 60.008 / 104.381 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.576 / 7.669 / 11.167 / 222.197 ms
- Distinct applied command values: 5514
- Applied-command change interval mean / P50 / P95 / max: 24.969 / 8.617 / 75.783 / 1082.340 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260911_185403

- Wall duration: 151.668 s
- Timer callbacks / command frames / guidance frames: 12002 / 12001 / 11999
- Command FPS (ROS simulation time / active wall time): 50.000 / 87.021 Hz
- Guidance FPS (active wall time): 87.034 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.947 / 0.977 / 1.319 / 3.676 ms
- Command interval mean / P50 / P95 / max: 11.492 / 8.162 / 19.091 / 232.038 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 23.535 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.046 / 1.053 / 1.629 / 4.212 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.332 / 2.552 ms (steady clock)

#### Command bridge — 20260911_185403

- Wall duration: 151.555 s
- Received / published control frames: 12001 / 12001
- Published control FPS (ROS simulation time / active wall time): 50.000 / 87.021 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.001 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.491 / 8.150 / 19.055 / 232.082 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.071 / 0.067 / 0.093 / 1.717 ms (steady clock)

### Run 20260911_185648

- Started: 2026-09-11 18:56:48 CST
- Scene: `/home/starry/isaac-data/user_ego/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260911_185648

- Result: RUNNING
- Wall duration / simulation duration: 94.031 / 165.367 s
- Applied simulation control frames: 9924
- Runtime FPS (simulation time / wall time): 60.012 / 105.539 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.470 / 7.512 / 10.048 / 214.050 ms
- Distinct applied command values: 3889
- Applied-command change interval mean / P50 / P95 / max: 24.126 / 8.432 / 76.420 / 545.197 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260911_185648

- Wall duration: 108.018 s
- Timer callbacks / command frames / guidance frames: 8269 / 8269 / 8266
- Command FPS (ROS simulation time / active wall time): 49.998 / 87.938 Hz
- Guidance FPS (active wall time): 88.077 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.174 / 1.149 / 1.423 / 3.516 ms
- Command interval mean / P50 / P95 / max: 11.372 / 7.903 / 17.492 / 224.483 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 22.500 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 1.293 / 1.232 / 1.797 / 3.716 ms (steady clock)
- DDS publish call mean / P95 / max: 0.084 / 0.428 / 2.265 ms (steady clock)

#### Command bridge — 20260911_185648

- Wall duration: 107.907 s
- Received / published control frames: 8269 / 8269
- Published control FPS (ROS simulation time / active wall time): 49.998 / 87.938 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.002 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.372 / 7.902 / 17.501 / 224.376 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.071 / 0.068 / 0.094 / 2.053 ms (steady clock)

### Run 20260912_094202

- Started: 2026-09-12 09:42:02 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260912_094202

- Result: USER_EXIT
- Wall duration / simulation duration: 71.893 / 114.050 s
- Applied simulation control frames: 6845
- Runtime FPS (simulation time / wall time): 60.018 / 95.210 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.495 / 8.877 / 12.111 / 95.143 ms
- Distinct applied command values: 1145
- ESDF-blocked simulation frames: 431
- Applied-command change interval mean / P50 / P95 / max: 61.546 / 11.393 / 70.067 / 18532.723 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260912_094202

- Wall duration: 85.991 s
- Timer callbacks / command frames / guidance frames: 5703 / 5703 / 1310
- Command FPS (ROS simulation time / active wall time): 50.003 / 79.336 Hz
- Guidance FPS (active wall time): 20.023 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.121 / 1.103 / 1.491 / 2.988 ms
- Command interval mean / P50 / P95 / max: 12.605 / 9.339 / 20.908 / 142.013 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 20.712 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.328 / 0.030 / 1.417 / 3.530 ms (steady clock)
- DDS publish call mean / P95 / max: 0.045 / 0.072 / 1.574 ms (steady clock)

#### Command bridge — 20260912_094202

- Wall duration: 85.878 s
- Received / published control frames: 5703 / 5703
- Published control FPS (ROS simulation time / active wall time): 50.003 / 79.337 Hz
- Command stamp interval mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 12.605 / 9.343 / 21.029 / 142.271 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.071 / 0.066 / 0.091 / 3.035 ms (steady clock)

### Run 20260912_094337

- Started: 2026-09-12 09:43:37 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`

#### Controller — 20260912_094337

- Wall duration: 75.407 s
- Timer callbacks / command frames / guidance frames: 6585 / 6584 / 1103
- Command FPS (ROS simulation time / active wall time): 49.997 / 107.308 Hz
- Guidance FPS (active wall time): 42.795 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.057 / 0.941 / 1.534 / 3.473 ms
- Command interval mean / P50 / P95 / max: 9.319 / 7.081 / 15.095 / 82.056 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 17.845 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.248 / 0.030 / 1.416 / 3.548 ms (steady clock)
- DDS publish call mean / P95 / max: 0.045 / 0.079 / 1.765 ms (steady clock)

#### Command bridge — 20260912_094337

- Wall duration: 75.294 s
- Received / published control frames: 6584 / 6584
- Published control FPS (ROS simulation time / active wall time): 49.997 / 107.309 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.003 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 9.319 / 7.103 / 15.024 / 81.034 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.072 / 0.066 / 0.100 / 1.873 ms (steady clock)

### Run flat_ground_smoke_20260912

- Started: 2026-09-12 09:52:58 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/flat_ground/flat_ground_navigation.usd`
- ROS domain: `42`

#### Controller — flat_ground_smoke_20260912

- Wall duration: 12.229 s
- Timer callbacks / command frames / guidance frames: 0 / 0 / 0
- Command FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- DDS publish call mean / P95 / max: 0.000 / 0.000 / 0.000 ms (steady clock)

#### Command bridge — flat_ground_smoke_20260912

- Wall duration: 12.119 s
- Received / published control frames: 0 / 0
- Published control FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Command stamp interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)

### Run flat_ground_gui_smoke_20260912

- Started: 2026-09-12 09:53:21 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/flat_ground/flat_ground_navigation.usd`
- ROS domain: `42`

#### Isaac command application — flat_ground_gui_smoke_20260912

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 1.422 / 3.017 s
- Applied simulation control frames: 183
- Runtime FPS (simulation time / wall time): 60.663 / 128.657 Hz
- Isaac frame interval mean / P50 / P95 / max: 7.451 / 6.087 / 8.376 / 72.647 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — flat_ground_gui_smoke_20260912

- Wall duration: 15.372 s
- Timer callbacks / command frames / guidance frames: 152 / 151 / 0
- Command FPS (ROS simulation time / active wall time): 50.000 / 117.468 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 8.513 / 6.244 / 13.730 / 70.534 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.032 / 0.030 / 0.049 / 0.077 ms (steady clock)
- DDS publish call mean / P95 / max: 0.029 / 0.045 / 0.070 ms (steady clock)

#### Command bridge — flat_ground_gui_smoke_20260912

- Wall duration: 15.264 s
- Received / published control frames: 151 / 151
- Published control FPS (ROS simulation time / active wall time): 50.000 / 117.507 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 8.510 / 6.254 / 13.934 / 70.650 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.072 / 0.068 / 0.098 / 0.142 ms (steady clock)

### Run 20260912_095547

- Started: 2026-09-12 09:55:47 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/flat_ground/flat_ground_navigation.usd`
- ROS domain: `42`

#### Isaac command application — 20260912_095547

- Result: USER_EXIT
- Wall duration / simulation duration: 57.121 / 106.183 s
- Applied simulation control frames: 6373
- Runtime FPS (simulation time / wall time): 60.019 / 111.571 Hz
- Isaac frame interval mean / P50 / P95 / max: 8.954 / 7.588 / 9.982 / 247.605 ms
- Distinct applied command values: 3158
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 17.359 / 8.643 / 22.874 / 4129.772 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260912_095547

- Wall duration: 70.317 s
- Timer callbacks / command frames / guidance frames: 5310 / 5309 / 3515
- Command FPS (ROS simulation time / active wall time): 49.997 / 93.067 Hz
- Guidance FPS (active wall time): 69.338 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.496 / 0.405 / 1.007 / 2.793 ms
- Command interval mean / P50 / P95 / max: 10.745 / 7.997 / 17.426 / 320.936 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 16.951 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.521 / 0.431 / 1.464 / 3.456 ms (steady clock)
- DDS publish call mean / P95 / max: 0.046 / 0.082 / 2.207 ms (steady clock)

#### Command bridge — 20260912_095547

- Wall duration: 70.203 s
- Received / published control frames: 5309 / 5309
- Published control FPS (ROS simulation time / active wall time): 49.997 / 93.069 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 10.745 / 8.021 / 17.475 / 321.029 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.070 / 0.066 / 0.096 / 1.736 ms (steady clock)

### Run 20260912_120842

- Started: 2026-09-12 12:08:42 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260912_120842

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 2.241 / 5.017 s
- Applied simulation control frames: 303
- Runtime FPS (simulation time / wall time): 60.399 / 135.209 Hz
- Isaac frame interval mean / P50 / P95 / max: 6.991 / 6.130 / 9.398 / 67.041 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260912_120842

- Wall duration: 17.107 s
- Timer callbacks / command frames / guidance frames: 252 / 252 / 0
- Command FPS (ROS simulation time / active wall time): 50.033 / 112.519 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.987 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 8.887 / 6.404 / 15.404 / 139.546 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.039 / 0.028 / 0.053 / 1.629 ms (steady clock)
- DDS publish call mean / P95 / max: 0.037 / 0.047 / 1.626 ms (steady clock)

#### Command bridge — 20260912_120842

- Wall duration: 16.985 s
- Received / published control frames: 252 / 252
- Published control FPS (ROS simulation time / active wall time): 50.033 / 112.529 Hz
- Command stamp interval mean / P50 / P95 / max: 19.987 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 8.887 / 6.308 / 15.569 / 139.859 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.078 / 0.068 / 0.106 / 1.257 ms (steady clock)

### Run 20260912_121136

- Started: 2026-09-12 12:11:36 CST
- Scene: `/home/starry/isaac-data/user/Fov-gvf/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Controller — 20260912_121136

- Wall duration: 279.440 s
- Timer callbacks / command frames / guidance frames: 19912 / 19912 / 11881
- Command FPS (ROS simulation time / active wall time): 50.000 / 75.006 Hz
- Guidance FPS (active wall time): 46.285 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.146 / 1.080 / 1.848 / 4.416 ms
- Command interval mean / P50 / P95 / max: 13.332 / 10.034 / 22.376 / 99.920 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 16.756 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.852 / 0.941 / 2.146 / 4.553 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.334 / 2.418 ms (steady clock)

#### Command bridge — 20260912_121136

- Wall duration: 279.328 s
- Received / published control frames: 19912 / 19912
- Published control FPS (ROS simulation time / active wall time): 50.000 / 75.006 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 13.332 / 10.052 / 22.345 / 100.133 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.073 / 0.069 / 0.094 / 1.490 ms (steady clock)

### Run 20260917_130131

- Started: 2026-09-17 13:01:31 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/flat_ground/flat_ground_navigation.usd`
- ROS domain: `42`
- Manual input: `keyboard`

#### Isaac command application — 20260917_130131

- Result: USER_EXIT
- Wall duration / simulation duration: 15.448 / 26.517 s
- Applied simulation control frames: 1593
- Runtime FPS (simulation time / wall time): 60.075 / 103.119 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.555 / 8.433 / 11.443 / 71.746 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_130131

- Wall duration: 28.563 s
- Timer callbacks / command frames / guidance frames: 1327 / 1326 / 0
- Command FPS (ROS simulation time / active wall time): 50.000 / 87.304 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 11.454 / 8.827 / 18.006 / 80.966 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.045 / 0.032 / 0.067 / 1.650 ms (steady clock)
- DDS publish call mean / P95 / max: 0.039 / 0.061 / 1.645 ms (steady clock)

#### Command bridge — 20260917_130131

- Wall duration: 28.455 s
- Received / published control frames: 1326 / 1326
- Published control FPS (ROS simulation time / active wall time): 50.000 / 87.305 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.454 / 8.819 / 18.026 / 81.131 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.070 / 0.066 / 0.086 / 0.600 ms (steady clock)

### Run 20260917_130251

- Started: 2026-09-17 13:02:51 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/flat_ground/flat_ground_navigation.usd`
- ROS domain: `42`
- Manual input: `keyboard`

#### Isaac command application — 20260917_130251

- Result: USER_EXIT
- Wall duration / simulation duration: 6.156 / 10.933 s
- Applied simulation control frames: 658
- Runtime FPS (simulation time / wall time): 60.183 / 106.888 Hz
- Isaac frame interval mean / P50 / P95 / max: 9.000 / 8.179 / 9.529 / 70.680 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_130251

- Wall duration: 19.355 s
- Timer callbacks / command frames / guidance frames: 548 / 548 / 0
- Command FPS (ROS simulation time / active wall time): 50.030 / 89.157 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.988 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 11.216 / 8.433 / 17.404 / 256.364 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.040 / 0.030 / 0.053 / 1.467 ms (steady clock)
- DDS publish call mean / P95 / max: 0.035 / 0.047 / 1.462 ms (steady clock)

#### Command bridge — 20260917_130251

- Wall duration: 19.245 s
- Received / published control frames: 548 / 548
- Published control FPS (ROS simulation time / active wall time): 50.030 / 89.153 Hz
- Command stamp interval mean / P50 / P95 / max: 19.988 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 11.217 / 8.474 / 17.259 / 256.820 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.066 / 0.062 / 0.093 / 0.394 ms (steady clock)

### Run 20260917_131817

- Started: 2026-09-17 13:18:17 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `keyboard`

#### Isaac command application — 20260917_131817

- Result: USER_EXIT
- Wall duration / simulation duration: 66.491 / 94.183 s
- Applied simulation control frames: 5653
- Runtime FPS (simulation time / wall time): 60.021 / 85.018 Hz
- Isaac frame interval mean / P50 / P95 / max: 11.722 / 10.882 / 13.164 / 98.368 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_131817

- Wall duration: 80.633 s
- Timer callbacks / command frames / guidance frames: 4710 / 4709 / 0
- Command FPS (ROS simulation time / active wall time): 49.996 / 71.103 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 14.064 / 11.558 / 23.421 / 97.596 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.043 / 0.035 / 0.066 / 0.842 ms (steady clock)
- DDS publish call mean / P95 / max: 0.037 / 0.059 / 0.837 ms (steady clock)

#### Command bridge — 20260917_131817

- Wall duration: 80.519 s
- Received / published control frames: 4709 / 4709
- Published control FPS (ROS simulation time / active wall time): 49.996 / 71.103 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 14.064 / 11.565 / 23.384 / 97.933 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.070 / 0.068 / 0.083 / 1.287 ms (steady clock)

### Run 20260917_135221

- Started: 2026-09-17 13:52:21 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `keyboard`

#### Isaac command application — 20260917_135221

- Result: USER_EXIT
- Wall duration / simulation duration: 34.630 / 52.317 s
- Applied simulation control frames: 3141
- Runtime FPS (simulation time / wall time): 60.038 / 90.701 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.954 / 10.105 / 12.628 / 96.227 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_135221

- Wall duration: 48.795 s
- Timer callbacks / command frames / guidance frames: 2617 / 2616 / 0
- Command FPS (ROS simulation time / active wall time): 50.000 / 76.104 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 13.140 / 10.674 / 22.671 / 97.078 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.044 / 0.033 / 0.064 / 2.081 ms (steady clock)
- DDS publish call mean / P95 / max: 0.039 / 0.058 / 2.076 ms (steady clock)

#### Command bridge — 20260917_135221

- Wall duration: 48.687 s
- Received / published control frames: 2616 / 2616
- Published control FPS (ROS simulation time / active wall time): 50.000 / 76.105 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 13.140 / 10.689 / 22.764 / 97.330 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.070 / 0.067 / 0.087 / 0.890 ms (steady clock)

### Run 20260917_141522

- Started: 2026-09-17 14:15:22 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260917_141522

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 3.272 / 5.017 s
- Applied simulation control frames: 303
- Runtime FPS (simulation time / wall time): 60.399 / 92.607 Hz
- Isaac frame interval mean / P50 / P95 / max: 10.038 / 8.333 / 10.711 / 71.345 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_141522

- Wall duration: 18.385 s
- Timer callbacks / command frames / guidance frames: 252 / 251 / 0
- Command FPS (ROS simulation time / active wall time): 50.000 / 83.215 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 12.017 / 8.735 / 18.883 / 137.254 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.038 / 0.035 / 0.059 / 0.274 ms (steady clock)
- DDS publish call mean / P95 / max: 0.033 / 0.054 / 0.268 ms (steady clock)

#### Command bridge — 20260917_141522

- Wall duration: 18.279 s
- Received / published control frames: 251 / 251
- Published control FPS (ROS simulation time / active wall time): 50.000 / 83.217 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.066 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 12.017 / 8.725 / 18.774 / 137.321 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.068 / 0.065 / 0.082 / 0.221 ms (steady clock)

### Run 20260917_142003

- Started: 2026-09-17 14:20:03 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260917_142003

- Result: USER_EXIT
- Wall duration / simulation duration: 24.989 / 36.717 s
- Applied simulation control frames: 2205
- Runtime FPS (simulation time / wall time): 60.054 / 88.238 Hz
- Isaac frame interval mean / P50 / P95 / max: 11.228 / 10.181 / 12.585 / 95.684 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_142003

- Wall duration: 39.483 s
- Timer callbacks / command frames / guidance frames: 1837 / 1837 / 0
- Command FPS (ROS simulation time / active wall time): 50.005 / 73.530 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.998 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 13.600 / 10.821 / 22.600 / 259.112 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.042 / 0.033 / 0.059 / 1.136 ms (steady clock)
- DDS publish call mean / P95 / max: 0.036 / 0.054 / 1.130 ms (steady clock)

#### Command bridge — 20260917_142003

- Wall duration: 39.372 s
- Received / published control frames: 1837 / 1837
- Published control FPS (ROS simulation time / active wall time): 50.005 / 73.529 Hz
- Command stamp interval mean / P50 / P95 / max: 19.998 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 13.600 / 10.800 / 22.676 / 259.440 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.067 / 0.065 / 0.081 / 0.686 ms (steady clock)

### Run 20260917_142545

- Started: 2026-09-17 14:25:45 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260917_142545

- Result: USER_EXIT
- Wall duration / simulation duration: 42.560 / 50.950 s
- Applied simulation control frames: 3059
- Runtime FPS (simulation time / wall time): 60.039 / 71.874 Hz
- Isaac frame interval mean / P50 / P95 / max: 13.838 / 12.462 / 16.174 / 79.488 ms
- Distinct applied command values: 26
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 1603.521 / 52.216 / 5898.346 / 24162.934 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_142545

- Wall duration: 57.237 s
- Timer callbacks / command frames / guidance frames: 2548 / 2547 / 611
- Command FPS (ROS simulation time / active wall time): 50.003 / 60.332 Hz
- Guidance FPS (active wall time): 29.300 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.898 / 0.894 / 0.999 / 1.893 ms
- Command interval mean / P50 / P95 / max: 16.575 / 13.055 / 28.514 / 91.493 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.926 / 50.000 / 50.000 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.273 / 0.041 / 1.017 / 2.296 ms (steady clock)
- DDS publish call mean / P95 / max: 0.045 / 0.064 / 1.086 ms (steady clock)

#### Command bridge — 20260917_142545

- Wall duration: 57.122 s
- Received / published control frames: 2547 / 2547
- Published control FPS (ROS simulation time / active wall time): 50.003 / 60.331 Hz
- Command stamp interval mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.575 / 13.039 / 28.367 / 91.471 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.078 / 0.076 / 0.095 / 0.540 ms (steady clock)

### Run 20260917_143252

- Started: 2026-09-17 14:32:52 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260917_143252

- Result: USER_EXIT
- Wall duration / simulation duration: 50.581 / 67.933 s
- Applied simulation control frames: 4078
- Runtime FPS (simulation time / wall time): 60.029 / 80.623 Hz
- Isaac frame interval mean / P50 / P95 / max: 12.348 / 10.606 / 13.620 / 80.286 ms
- Distinct applied command values: 146
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 324.229 / 23.945 / 638.539 / 13842.434 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_143252

- Wall duration: 65.130 s
- Timer callbacks / command frames / guidance frames: 3398 / 3397 / 1499
- Command FPS (ROS simulation time / active wall time): 50.002 / 67.590 Hz
- Guidance FPS (active wall time): 38.817 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.928 / 0.917 / 1.083 / 2.644 ms
- Command interval mean / P50 / P95 / max: 14.795 / 11.139 / 24.743 / 89.040 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 33.267 / 33.333 / 50.000 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.476 / 0.056 / 1.101 / 2.724 ms (steady clock)
- DDS publish call mean / P95 / max: 0.049 / 0.076 / 2.127 ms (steady clock)

#### Command bridge — 20260917_143252

- Wall duration: 65.014 s
- Received / published control frames: 3397 / 3397
- Published control FPS (ROS simulation time / active wall time): 50.002 / 67.590 Hz
- Command stamp interval mean / P50 / P95 / max: 19.999 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 14.795 / 11.108 / 24.722 / 89.069 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.080 / 0.077 / 0.099 / 0.923 ms (steady clock)

### Run 20260917_143846

- Started: 2026-09-17 14:38:46 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260917_143846

- Result: USER_EXIT
- Wall duration / simulation duration: 156.374 / 199.567 s
- Applied simulation control frames: 11976
- Runtime FPS (simulation time / wall time): 60.010 / 76.585 Hz
- Isaac frame interval mean / P50 / P95 / max: 13.038 / 11.753 / 14.314 / 106.509 ms
- Distinct applied command values: 4933
- ESDF-blocked simulation frames: 121
- Applied-command change interval mean / P50 / P95 / max: 28.546 / 15.424 / 38.093 / 15076.442 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_143846

- Wall duration: 171.643 s
- Timer callbacks / command frames / guidance frames: 9979 / 9978 / 6465
- Command FPS (ROS simulation time / active wall time): 49.997 / 63.942 Hz
- Guidance FPS (active wall time): 49.928 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.852 / 0.838 / 1.126 / 3.504 ms
- Command interval mean / P50 / P95 / max: 15.639 / 12.251 / 26.002 / 148.554 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 31.433 / 33.333 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.634 / 0.825 / 1.229 / 3.668 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.097 / 2.693 ms (steady clock)

#### Command bridge — 20260917_143846

- Wall duration: 171.523 s
- Received / published control frames: 9978 / 9978
- Published control FPS (ROS simulation time / active wall time): 49.997 / 63.942 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.639 / 12.273 / 25.953 / 148.603 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.080 / 0.077 / 0.098 / 1.643 ms (steady clock)

### Run 20260917_144635

- Started: 2026-09-17 14:46:35 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260917_144635

- Result: USER_EXIT
- Wall duration / simulation duration: 170.054 / 204.583 s
- Applied simulation control frames: 12277
- Runtime FPS (simulation time / wall time): 60.010 / 72.195 Hz
- Isaac frame interval mean / P50 / P95 / max: 13.832 / 11.862 / 14.772 / 77.760 ms
- Distinct applied command values: 6029
- ESDF-blocked simulation frames: 119
- Applied-command change interval mean / P50 / P95 / max: 27.548 / 14.975 / 73.211 / 18208.817 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260917_144635

- Wall duration: 185.251 s
- Timer callbacks / command frames / guidance frames: 10223 / 10222 / 7845
- Command FPS (ROS simulation time / active wall time): 49.964 / 60.205 Hz
- Guidance FPS (active wall time): 53.580 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.014 / 16.667 / 33.333 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 0.857 / 0.842 / 1.219 / 4.007 ms
- Command interval mean / P50 / P95 / max: 16.610 / 12.293 / 27.000 / 148.158 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 27.372 / 33.333 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.741 / 0.867 / 1.363 / 4.431 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.146 / 1.880 ms (steady clock)

#### Command bridge — 20260917_144635

- Wall duration: 185.134 s
- Received / published control frames: 10222 / 10222
- Published control FPS (ROS simulation time / active wall time): 49.964 / 60.205 Hz
- Command stamp interval mean / P50 / P95 / max: 20.014 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.610 / 12.286 / 26.981 / 148.194 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.079 / 0.075 / 0.099 / 1.545 ms (steady clock)

### Run 20260918_191052

- Started: 2026-09-18 19:10:52 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260918_191052

- Result: USER_EXIT
- Wall duration / simulation duration: 67.528 / 75.483 s
- Applied simulation control frames: 4531
- Runtime FPS (simulation time / wall time): 60.026 / 67.098 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.846 / 13.548 / 16.513 / 96.390 ms
- Distinct applied command values: 1772
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 35.597 / 26.654 / 74.117 / 3618.836 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260918_191052

- Wall duration: 84.902 s
- Timer callbacks / command frames / guidance frames: 3775 / 3774 / 2449
- Command FPS (ROS simulation time / active wall time): 49.996 / 56.137 Hz
- Guidance FPS (active wall time): 42.489 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.819 / 0.825 / 1.015 / 1.938 ms
- Command interval mean / P50 / P95 / max: 17.814 / 14.047 / 29.961 / 96.168 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 29.468 / 33.333 / 50.000 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.605 / 0.811 / 1.096 / 2.680 ms (steady clock)
- DDS publish call mean / P95 / max: 0.047 / 0.069 / 1.407 ms (steady clock)

#### Command bridge — 20260918_191052

- Wall duration: 84.788 s
- Received / published control frames: 3774 / 3774
- Published control FPS (ROS simulation time / active wall time): 49.996 / 56.138 Hz
- Command stamp interval mean / P50 / P95 / max: 20.002 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.813 / 14.058 / 29.962 / 96.689 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.075 / 0.073 / 0.091 / 1.374 ms (steady clock)

### Run 20260923_190342

- Started: 2026-09-23 19:03:42 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Controller — 20260923_190342

- Wall duration: 144.057 s
- Timer callbacks / command frames / guidance frames: 0 / 0 / 0
- Command FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- DDS publish call mean / P95 / max: 0.000 / 0.000 / 0.000 ms (steady clock)

#### Command bridge — 20260923_190342

- Wall duration: 143.943 s
- Received / published control frames: 0 / 0
- Published control FPS (ROS simulation time / active wall time): 0.000 / 0.000 Hz
- Command stamp interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (steady clock)

### Run 20260923_190610

- Started: 2026-09-23 19:06:10 CST
- Scene: `/home/starry/isaac-data/EGO1P0/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260923_190610

- Result: USER_EXIT
- Wall duration / simulation duration: 114.326 / 128.100 s
- Applied simulation control frames: 7688
- Runtime FPS (simulation time / wall time): 60.016 / 67.246 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.837 / 13.515 / 16.635 / 82.451 ms
- Distinct applied command values: 3158
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 34.920 / 26.683 / 76.752 / 7837.142 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260923_190610

- Wall duration: 200.530 s
- Timer callbacks / command frames / guidance frames: 6406 / 6405 / 4498
- Command FPS (ROS simulation time / active wall time): 49.999 / 56.171 Hz
- Guidance FPS (active wall time): 43.889 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.922 / 0.900 / 1.151 / 2.722 ms
- Command interval mean / P50 / P95 / max: 17.803 / 14.014 / 30.110 / 149.664 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 34.023 / 33.333 / 50.000 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.715 / 0.921 / 1.213 / 3.434 ms (steady clock)
- DDS publish call mean / P95 / max: 0.041 / 0.056 / 2.348 ms (steady clock)

#### Command bridge — 20260923_190610

- Wall duration: 200.418 s
- Received / published control frames: 6405 / 6405
- Published control FPS (ROS simulation time / active wall time): 49.999 / 56.171 Hz
- Command stamp interval mean / P50 / P95 / max: 20.001 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.803 / 14.036 / 30.175 / 149.518 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.074 / 0.072 / 0.092 / 1.011 ms (steady clock)

### Run ego1p1_cleanup_smoke_20260923

- Started: 2026-09-23 19:49:48 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `keyboard`

#### Isaac command application — ego1p1_cleanup_smoke_20260923

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 2.490 / 3.017 s
- Applied simulation control frames: 183
- Runtime FPS (simulation time / wall time): 60.663 / 73.504 Hz
- Isaac frame interval mean / P50 / P95 / max: 12.331 / 10.016 / 15.086 / 72.748 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — ego1p1_cleanup_smoke_20260923

- Wall duration: 16.800 s
- Timer callbacks / command frames / guidance frames: 152 / 151 / 0
- Command FPS (ROS simulation time / active wall time): 50.000 / 69.704 Hz
- Guidance FPS (active wall time): 0.000 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms
- Command interval mean / P50 / P95 / max: 14.346 / 10.259 / 28.616 / 86.334 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.046 / 0.038 / 0.075 / 0.598 ms (steady clock)
- DDS publish call mean / P95 / max: 0.040 / 0.069 / 0.592 ms (steady clock)

#### Command bridge — ego1p1_cleanup_smoke_20260923

- Wall duration: 16.683 s
- Received / published control frames: 151 / 151
- Published control FPS (ROS simulation time / active wall time): 50.000 / 69.708 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 14.346 / 10.210 / 28.706 / 86.303 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.075 / 0.072 / 0.095 / 0.165 ms (steady clock)

### Run 20260923_201829

- Started: 2026-09-23 20:18:29 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260923_201829

- Result: USER_EXIT
- Wall duration / simulation duration: 112.990 / 153.317 s
- Applied simulation control frames: 9201
- Runtime FPS (simulation time / wall time): 60.013 / 81.432 Hz
- Isaac frame interval mean / P50 / P95 / max: 12.256 / 11.068 / 13.545 / 74.940 ms
- Distinct applied command values: 2125
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 50.525 / 12.835 / 139.965 / 18251.578 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260923_201829

- Wall duration: 127.076 s
- Timer callbacks / command frames / guidance frames: 7667 / 7666 / 5014
- Command FPS (ROS simulation time / active wall time): 50.000 / 68.003 Hz
- Guidance FPS (active wall time): 56.288 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.056 / 1.050 / 1.453 / 3.937 ms
- Command interval mean / P50 / P95 / max: 14.705 / 11.559 / 24.624 / 149.581 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 31.196 / 33.333 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.860 / 0.973 / 1.946 / 4.338 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.359 / 2.644 ms (steady clock)

#### Command bridge — 20260923_201829

- Wall duration: 126.963 s
- Received / published control frames: 7666 / 7666
- Published control FPS (ROS simulation time / active wall time): 50.000 / 68.003 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 14.705 / 11.571 / 24.615 / 149.599 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.081 / 0.106 / 1.555 ms (steady clock)

### Run 20260924_153932

- Started: 2026-09-24 15:39:32 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260924_153932

- Result: USER_EXIT
- Wall duration / simulation duration: 34.750 / 45.567 s
- Applied simulation control frames: 2736
- Runtime FPS (simulation time / wall time): 60.044 / 78.733 Hz
- Isaac frame interval mean / P50 / P95 / max: 12.608 / 11.128 / 14.917 / 96.133 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260924_153932

- Wall duration: 52.120 s
- Timer callbacks / command frames / guidance frames: 2279 / 2278 / 846
- Command FPS (ROS simulation time / active wall time): 49.989 / 66.110 Hz
- Guidance FPS (active wall time): 35.254 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.004 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.002 / 0.002 / 0.003 / 0.014 ms
- Command interval mean / P50 / P95 / max: 15.126 / 11.824 / 25.963 / 95.598 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.519 / 50.000 / 50.000 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.057 / 0.041 / 0.090 / 1.592 ms (steady clock)
- DDS publish call mean / P95 / max: 0.041 / 0.062 / 1.582 ms (steady clock)

#### Command bridge — 20260924_153932

- Wall duration: 52.008 s
- Received / published control frames: 2278 / 2278
- Published control FPS (ROS simulation time / active wall time): 49.989 / 66.110 Hz
- Command stamp interval mean / P50 / P95 / max: 20.004 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.126 / 11.826 / 25.896 / 95.915 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.073 / 0.071 / 0.090 / 0.664 ms (steady clock)

### Run 20260928_104603

- Started: 2026-09-28 10:46:03 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260928_104603

- Result: USER_EXIT
- Wall duration / simulation duration: 140.779 / 167.033 s
- Applied simulation control frames: 10024
- Runtime FPS (simulation time / wall time): 60.012 / 71.204 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.019 / 12.101 / 15.303 / 78.308 ms
- Distinct applied command values: 2791
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 39.926 / 14.119 / 87.418 / 4344.379 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260928_104603

- Wall duration: 157.653 s
- Timer callbacks / command frames / guidance frames: 6300 / 6300 / 5256
- Command FPS (ROS simulation time / active wall time): 37.711 / 44.764 Hz
- Guidance FPS (active wall time): 38.875 Hz
- Command interval in ROS time mean / P50 / P95 / max: 26.517 / 16.667 / 66.667 / 150.000 ms
- Avoidance compute mean / P50 / P95 / max: 8.174 / 0.543 / 41.724 / 95.122 ms
- Command interval mean / P50 / P95 / max: 22.340 / 14.929 / 62.820 / 300.630 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 22.847 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 18.215 / 13.562 / 49.638 / 107.935 ms (steady clock)
- DDS publish call mean / P95 / max: 0.076 / 0.413 / 3.461 ms (steady clock)

#### Command bridge — 20260928_104603

- Wall duration: 157.539 s
- Received / published control frames: 6300 / 6274
- Published control FPS (ROS simulation time / active wall time): 37.711 / 44.579 Hz
- Command stamp interval mean / P50 / P95 / max: 26.517 / 16.667 / 66.667 / 150.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 21.093 / 16.667 / 66.667 / 150.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 22.339 / 14.980 / 62.892 / 300.358 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.103 / 0.093 / 0.138 / 3.567 ms (steady clock)

### Run 20260928_112831

- Started: 2026-09-28 11:28:31 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260928_112831

- Result: USER_EXIT
- Wall duration / simulation duration: 68.887 / 87.217 s
- Applied simulation control frames: 5235
- Runtime FPS (simulation time / wall time): 60.023 / 75.994 Hz
- Isaac frame interval mean / P50 / P95 / max: 13.116 / 11.618 / 14.626 / 104.752 ms
- Distinct applied command values: 227
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 80.370 / 13.786 / 379.944 / 3147.566 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260928_112831

- Wall duration: 83.205 s
- Timer callbacks / command frames / guidance frames: 4326 / 4326 / 2231
- Command FPS (ROS simulation time / active wall time): 49.589 / 62.792 Hz
- Guidance FPS (active wall time): 37.384 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.166 / 16.667 / 33.333 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 0.738 / 0.296 / 2.435 / 23.181 ms
- Command interval mean / P50 / P95 / max: 15.926 / 13.064 / 27.535 / 253.382 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 25.049 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 10.866 / 11.768 / 15.036 / 36.954 ms (steady clock)
- DDS publish call mean / P95 / max: 0.081 / 0.432 / 1.871 ms (steady clock)

#### Command bridge — 20260928_112831

- Wall duration: 83.095 s
- Received / published control frames: 4326 / 4326
- Published control FPS (ROS simulation time / active wall time): 49.589 / 62.792 Hz
- Command stamp interval mean / P50 / P95 / max: 20.166 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 10.680 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.926 / 13.064 / 27.681 / 254.017 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.096 / 0.091 / 0.120 / 2.378 ms (steady clock)

### Run 20260928_113114

- Started: 2026-09-28 11:31:14 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260928_113114

- Result: USER_EXIT
- Wall duration / simulation duration: 71.816 / 88.867 s
- Applied simulation control frames: 5334
- Runtime FPS (simulation time / wall time): 60.023 / 74.273 Hz
- Isaac frame interval mean / P50 / P95 / max: 13.422 / 11.839 / 14.702 / 75.525 ms
- Distinct applied command values: 1014
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 38.514 / 13.932 / 86.385 / 1776.725 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260928_113114

- Wall duration: 86.179 s
- Timer callbacks / command frames / guidance frames: 3918 / 3917 / 3237
- Command FPS (ROS simulation time / active wall time): 44.074 / 54.727 Hz
- Guidance FPS (active wall time): 45.831 Hz
- Command interval in ROS time mean / P50 / P95 / max: 22.689 / 16.667 / 50.000 / 116.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.611 / 0.028 / 25.899 / 64.445 ms
- Command interval mean / P50 / P95 / max: 18.272 / 13.294 / 49.385 / 146.673 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 24.812 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 13.827 / 11.989 / 35.877 / 76.506 ms (steady clock)
- DDS publish call mean / P95 / max: 0.078 / 0.423 / 1.527 ms (steady clock)

#### Command bridge — 20260928_113114

- Wall duration: 86.069 s
- Received / published control frames: 3917 / 3916
- Published control FPS (ROS simulation time / active wall time): 44.074 / 54.714 Hz
- Command stamp interval mean / P50 / P95 / max: 22.689 / 16.667 / 50.000 / 116.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 15.097 / 16.667 / 50.000 / 116.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.272 / 13.307 / 49.530 / 146.716 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.099 / 0.091 / 0.130 / 2.821 ms (steady clock)

### Run 20260928_161515

- Started: 2026-09-28 16:15:15 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Controller — 20260928_161515

- Wall duration: 136.640 s
- Timer callbacks / command frames / guidance frames: 6783 / 6782 / 4895
- Command FPS (ROS simulation time / active wall time): 47.025 / 55.280 Hz
- Guidance FPS (active wall time): 46.723 Hz
- Command interval in ROS time mean / P50 / P95 / max: 21.265 / 16.667 / 33.333 / 166.667 ms
- Avoidance compute mean / P50 / P95 / max: 4.732 / 0.909 / 26.663 / 118.974 ms
- Command interval mean / P50 / P95 / max: 18.090 / 12.985 / 49.388 / 144.188 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 22.993 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 11.075 / 8.845 / 29.931 / 126.238 ms (steady clock)
- DDS publish call mean / P95 / max: 0.083 / 0.450 / 2.356 ms (steady clock)

#### Command bridge — 20260928_161515

- Wall duration: 136.529 s
- Received / published control frames: 6781 / 6775
- Published control FPS (ROS simulation time / active wall time): 47.023 / 55.228 Hz
- Command stamp interval mean / P50 / P95 / max: 21.266 / 16.667 / 33.333 / 166.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 7.042 / 0.000 / 33.333 / 166.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.091 / 13.005 / 49.614 / 144.090 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.101 / 0.094 / 0.127 / 5.070 ms (steady clock)

### Run 20260928_210259

- Started: 2026-09-28 21:02:59 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260928_210259

- Result: USER_EXIT
- Wall duration / simulation duration: 96.353 / 109.450 s
- Applied simulation control frames: 6569
- Runtime FPS (simulation time / wall time): 60.018 / 68.176 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.629 / 12.819 / 16.000 / 77.573 ms
- Distinct applied command values: 2066
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 42.517 / 14.140 / 99.304 / 4216.307 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260928_210259

- Wall duration: 113.494 s
- Timer callbacks / command frames / guidance frames: 4828 / 4827 / 4085
- Command FPS (ROS simulation time / active wall time): 44.107 / 50.248 Hz
- Guidance FPS (active wall time): 44.767 Hz
- Command interval in ROS time mean / P50 / P95 / max: 22.672 / 16.667 / 33.333 / 133.333 ms
- Avoidance compute mean / P50 / P95 / max: 6.036 / 0.359 / 30.284 / 85.181 ms
- Command interval mean / P50 / P95 / max: 19.901 / 13.623 / 58.041 / 144.605 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 21.518 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 12.639 / 8.598 / 37.015 / 96.319 ms (steady clock)
- DDS publish call mean / P95 / max: 0.073 / 0.419 / 2.821 ms (steady clock)

#### Command bridge — 20260928_210259

- Wall duration: 113.381 s
- Received / published control frames: 4827 / 4822
- Published control FPS (ROS simulation time / active wall time): 44.107 / 50.196 Hz
- Command stamp interval mean / P50 / P95 / max: 22.672 / 16.667 / 33.333 / 133.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 9.809 / 0.000 / 33.333 / 133.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 19.901 / 13.616 / 58.232 / 144.610 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.102 / 0.094 / 0.129 / 3.299 ms (steady clock)

### Run 20260929_183510

- Started: 2026-09-29 18:35:10 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20260929_183510

- Result: USER_EXIT
- Wall duration / simulation duration: 68.800 / 67.083 s
- Applied simulation control frames: 4027
- Runtime FPS (simulation time / wall time): 60.030 / 58.532 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.030 / 12.952 / 72.619 / 78.814 ms
- Distinct applied command values: 1341
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 46.765 / 25.020 / 117.176 / 2301.298 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20260929_183510

- Wall duration: 83.542 s
- Timer callbacks / command frames / guidance frames: 2314 / 2313 / 1940
- Command FPS (ROS simulation time / active wall time): 34.473 / 33.733 Hz
- Guidance FPS (active wall time): 31.230 Hz
- Command interval in ROS time mean / P50 / P95 / max: 29.008 / 16.667 / 83.333 / 366.667 ms
- Avoidance compute mean / P50 / P95 / max: 10.984 / 1.230 / 49.585 / 304.821 ms
- Command interval mean / P50 / P95 / max: 29.645 / 18.344 / 79.058 / 326.096 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 25.146 / 33.333 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 22.152 / 15.845 / 64.975 / 325.752 ms (steady clock)
- DDS publish call mean / P95 / max: 0.073 / 0.399 / 2.129 ms (steady clock)

#### Command bridge — 20260929_183510

- Wall duration: 83.430 s
- Received / published control frames: 2313 / 2295
- Published control FPS (ROS simulation time / active wall time): 34.473 / 33.470 Hz
- Command stamp interval mean / P50 / P95 / max: 29.008 / 16.667 / 83.333 / 366.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 21.797 / 16.667 / 83.333 / 366.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 29.645 / 18.503 / 78.903 / 326.263 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.104 / 0.097 / 0.137 / 2.780 ms (steady clock)

### Run 20261005_192504

- Started: 2026-10-05 19:25:04 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261005_192504

- Result: USER_EXIT
- Wall duration / simulation duration: 118.159 / 135.517 s
- Applied simulation control frames: 8133
- Runtime FPS (simulation time / wall time): 60.015 / 68.831 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.497 / 13.037 / 15.606 / 77.175 ms
- Distinct applied command values: 5215
- ESDF-blocked simulation frames: 1
- Applied-command change interval mean / P50 / P95 / max: 22.181 / 13.472 / 28.133 / 5494.280 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261005_192504

- Wall duration: 135.476 s
- Timer callbacks / command frames / guidance frames: 7605 / 7604 / 6036
- Command FPS (ROS simulation time / active wall time): 56.111 / 64.526 Hz
- Guidance FPS (active wall time): 54.782 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.822 / 16.667 / 16.667 / 1116.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.530 / 1.020 / 4.616 / 830.986 ms
- Command interval mean / P50 / P95 / max: 15.498 / 12.992 / 20.890 / 839.917 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 72.300 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.207 / 2.723 / 8.578 / 839.485 ms (steady clock)
- DDS publish call mean / P95 / max: 0.085 / 0.448 / 2.422 ms (steady clock)

#### Command bridge — 20261005_192504

- Wall duration: 135.354 s
- Received / published control frames: 7604 / 7576
- Published control FPS (ROS simulation time / active wall time): 56.111 / 64.288 Hz
- Command stamp interval mean / P50 / P95 / max: 17.822 / 16.667 / 16.667 / 1116.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.493 / 0.000 / 0.000 / 1116.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.498 / 13.000 / 20.938 / 839.959 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.094 / 0.090 / 0.121 / 2.621 ms (steady clock)

### Run 20261006_205620

- Started: 2026-10-06 20:56:20 CST
- Scene: `/home/starry/isaac-data/EGO1P2/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261006_205620

- Result: USER_EXIT
- Wall duration / simulation duration: 41.162 / 52.817 s
- Applied simulation control frames: 3171
- Runtime FPS (simulation time / wall time): 60.038 / 77.036 Hz
- Isaac frame interval mean / P50 / P95 / max: 12.911 / 11.586 / 14.189 / 74.430 ms
- Distinct applied command values: 1226
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 26.018 / 12.741 / 65.121 / 2146.889 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261006_205620

- Wall duration: 55.202 s
- Timer callbacks / command frames / guidance frames: 3049 / 3048 / 1981
- Command FPS (ROS simulation time / active wall time): 57.708 / 74.514 Hz
- Guidance FPS (active wall time): 63.391 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.329 / 16.667 / 16.667 / 166.667 ms
- Avoidance compute mean / P50 / P95 / max: 4.418 / 1.703 / 12.397 / 111.790 ms
- Command interval mean / P50 / P95 / max: 13.420 / 11.627 / 19.507 / 120.810 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 62.334 / 66.667 / 100.000 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.623 / 2.424 / 13.409 / 112.106 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.353 / 1.311 ms (steady clock)

#### Command bridge — 20261006_205620

- Wall duration: 55.092 s
- Received / published control frames: 3048 / 3044
- Published control FPS (ROS simulation time / active wall time): 57.708 / 74.417 Hz
- Command stamp interval mean / P50 / P95 / max: 17.329 / 16.667 / 16.667 / 166.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 2.280 / 0.000 / 16.667 / 166.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 13.420 / 11.635 / 19.663 / 121.022 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.083 / 0.080 / 0.108 / 0.948 ms (steady clock)

### Run 20261006_213850

- Started: 2026-10-06 21:38:50 CST
- Scene: `/home/starry/isaac-data/EGO1P2/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261006_213850

- Result: USER_EXIT
- Wall duration / simulation duration: 198.146 / 217.500 s
- Applied simulation control frames: 13052
- Runtime FPS (simulation time / wall time): 60.009 / 65.871 Hz
- Isaac frame interval mean / P50 / P95 / max: 15.165 / 12.989 / 16.012 / 80.214 ms
- Distinct applied command values: 8405
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.548 / 13.556 / 72.406 / 11555.128 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261006_213850

- Wall duration: 212.353 s
- Timer callbacks / command frames / guidance frames: 12112 / 12112 / 10378
- Command FPS (ROS simulation time / active wall time): 55.683 / 61.146 Hz
- Guidance FPS (active wall time): 55.703 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.959 / 16.667 / 16.667 / 683.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.345 / 0.573 / 4.152 / 601.651 ms
- Command interval mean / P50 / P95 / max: 16.354 / 12.974 / 31.448 / 667.889 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 64.707 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.356 / 2.227 / 10.532 / 602.031 ms (steady clock)
- DDS publish call mean / P95 / max: 0.086 / 0.441 / 2.041 ms (steady clock)

#### Command bridge — 20261006_213850

- Wall duration: 212.242 s
- Received / published control frames: 12112 / 12056
- Published control FPS (ROS simulation time / active wall time): 55.683 / 60.863 Hz
- Command stamp interval mean / P50 / P95 / max: 17.959 / 16.667 / 16.667 / 683.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.807 / 0.000 / 0.000 / 683.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.354 / 12.972 / 31.302 / 667.827 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.082 / 0.112 / 1.516 ms (steady clock)

### Run 20261007_160119

- Started: 2026-10-07 16:01:19 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261007_160119

- Result: USER_EXIT
- Wall duration / simulation duration: 154.500 / 147.500 s
- Applied simulation control frames: 8852
- Runtime FPS (simulation time / wall time): 60.014 / 57.294 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.429 / 15.525 / 18.543 / 96.031 ms
- Distinct applied command values: 5389
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 28.087 / 16.458 / 54.211 / 9627.833 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261007_160119

- Wall duration: 168.834 s
- Timer callbacks / command frames / guidance frames: 8386 / 8385 / 6690
- Command FPS (ROS simulation time / active wall time): 56.847 / 54.362 Hz
- Guidance FPS (active wall time): 45.709 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.591 / 16.667 / 16.667 / 633.333 ms
- Avoidance compute mean / P50 / P95 / max: 4.144 / 0.759 / 14.848 / 633.340 ms
- Command interval mean / P50 / P95 / max: 18.395 / 15.553 / 33.965 / 645.668 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 33.012 / 33.333 / 66.667 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.615 / 3.314 / 17.278 / 645.596 ms (steady clock)
- DDS publish call mean / P95 / max: 0.082 / 0.439 / 2.104 ms (steady clock)

#### Command bridge — 20261007_160119

- Wall duration: 168.720 s
- Received / published control frames: 8385 / 8374
- Published control FPS (ROS simulation time / active wall time): 56.847 / 54.291 Hz
- Command stamp interval mean / P50 / P95 / max: 17.591 / 16.667 / 16.667 / 633.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.954 / 0.000 / 16.667 / 633.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.395 / 15.536 / 34.011 / 645.679 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.115 / 1.330 ms (steady clock)

### Run 20261007_210752

- Started: 2026-10-07 21:07:52 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261007_210752

- Result: USER_EXIT
- Wall duration / simulation duration: 220.093 / 253.817 s
- Applied simulation control frames: 15231
- Runtime FPS (simulation time / wall time): 60.008 / 69.202 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.436 / 12.750 / 15.205 / 82.276 ms
- Distinct applied command values: 10998
- ESDF-blocked simulation frames: 16
- Applied-command change interval mean / P50 / P95 / max: 19.714 / 13.213 / 36.567 / 6343.797 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261007_210752

- Wall duration: 234.385 s
- Timer callbacks / command frames / guidance frames: 13954 / 13953 / 12956
- Command FPS (ROS simulation time / active wall time): 54.972 / 63.470 Hz
- Guidance FPS (active wall time): 60.369 Hz
- Command interval in ROS time mean / P50 / P95 / max: 18.191 / 16.667 / 16.667 / 900.000 ms
- Avoidance compute mean / P50 / P95 / max: 3.530 / 0.580 / 11.467 / 724.041 ms
- Command interval mean / P50 / P95 / max: 15.755 / 13.046 / 30.336 / 739.187 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.061 / 33.333 / 66.667 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.611 / 3.707 / 15.330 / 739.076 ms (steady clock)
- DDS publish call mean / P95 / max: 0.072 / 0.409 / 3.565 ms (steady clock)

#### Command bridge — 20261007_210752

- Wall duration: 234.269 s
- Received / published control frames: 13953 / 13918
- Published control FPS (ROS simulation time / active wall time): 54.972 / 63.311 Hz
- Command stamp interval mean / P50 / P95 / max: 18.191 / 16.667 / 16.667 / 900.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 2.685 / 0.000 / 16.667 / 900.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.755 / 13.062 / 30.197 / 739.074 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.114 / 1.793 ms (steady clock)

### Run 20261008_125647

- Started: 2026-10-08 12:56:47 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261008_125647

- Result: USER_EXIT
- Wall duration / simulation duration: 67.289 / 78.217 s
- Applied simulation control frames: 4695
- Runtime FPS (simulation time / wall time): 60.026 / 69.773 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.285 / 12.974 / 15.289 / 76.208 ms
- Distinct applied command values: 3466
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.817 / 13.505 / 27.822 / 4839.752 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — 20261008_125647

- Wall duration: 81.474 s
- Received / published control frames: 4691 / 4691
- Published control FPS (ROS simulation time / active wall time): 59.974 / 69.986 Hz
- Command stamp interval mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.480 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 14.289 / 12.970 / 23.559 / 85.649 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.087 / 0.114 / 1.236 ms (steady clock)

#### Controller — 20261008_125647

- Wall duration: 81.589 s
- Timer callbacks / command frames / guidance frames: 4692 / 4691 / 4068
- Command FPS (ROS simulation time / active wall time): 59.974 / 69.985 Hz
- Guidance FPS (active wall time): 67.372 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.668 / 0.540 / 4.906 / 9.704 ms
- Command interval mean / P50 / P95 / max: 14.289 / 13.009 / 23.681 / 85.684 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.938 / 33.333 / 66.667 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.828 / 4.870 / 11.156 / 22.546 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.321 / 1.316 ms (steady clock)

### Run 20261008_141320

- Started: 2026-10-08 14:13:20 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261008_141320

- Result: USER_EXIT
- Wall duration / simulation duration: 90.470 / 90.250 s
- Applied simulation control frames: 5417
- Runtime FPS (simulation time / wall time): 60.022 / 59.876 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.585 / 14.479 / 20.365 / 94.200 ms
- Distinct applied command values: 3250
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.845 / 15.193 / 47.919 / 11024.147 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261008_141320

- Wall duration: 112.242 s
- Timer callbacks / command frames / guidance frames: 5345 / 5344 / 4015
- Command FPS (ROS simulation time / active wall time): 59.213 / 59.518 Hz
- Guidance FPS (active wall time): 54.660 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.888 / 16.667 / 16.667 / 100.000 ms
- Avoidance compute mean / P50 / P95 / max: 2.193 / 0.535 / 6.631 / 79.918 ms
- Command interval mean / P50 / P95 / max: 16.802 / 14.340 / 33.362 / 116.792 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 48.377 / 50.000 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.218 / 3.737 / 15.998 / 83.340 ms (steady clock)
- DDS publish call mean / P95 / max: 0.075 / 0.396 / 4.044 ms (steady clock)

#### Command bridge — 20261008_141320

- Wall duration: 112.048 s
- Received / published control frames: 5344 / 5344
- Published control FPS (ROS simulation time / active wall time): 59.213 / 59.518 Hz
- Command stamp interval mean / P50 / P95 / max: 16.888 / 16.667 / 16.667 / 100.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.400 / 0.000 / 16.667 / 100.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.802 / 14.327 / 33.325 / 116.920 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.102 / 0.097 / 0.138 / 2.825 ms (steady clock)

### Run 20261008_204506

- Started: 2026-10-08 20:45:06 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261008_204506

- Result: USER_EXIT
- Wall duration / simulation duration: 76.848 / 85.250 s
- Applied simulation control frames: 5117
- Runtime FPS (simulation time / wall time): 60.023 / 66.586 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.974 / 13.452 / 15.803 / 76.907 ms
- Distinct applied command values: 4079
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.178 / 13.781 / 29.273 / 931.551 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261008_204506

- Wall duration: 91.073 s
- Timer callbacks / command frames / guidance frames: 5003 / 5002 / 4785
- Command FPS (ROS simulation time / active wall time): 58.674 / 65.315 Hz
- Guidance FPS (active wall time): 65.351 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.043 / 16.667 / 16.667 / 116.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.662 / 0.590 / 7.514 / 82.054 ms
- Command interval mean / P50 / P95 / max: 15.310 / 13.347 / 26.250 / 92.172 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.762 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.912 / 5.126 / 14.860 / 92.091 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.377 / 1.764 ms (steady clock)

#### Command bridge — 20261008_204506

- Wall duration: 90.960 s
- Received / published control frames: 5002 / 5001
- Published control FPS (ROS simulation time / active wall time): 58.674 / 65.302 Hz
- Command stamp interval mean / P50 / P95 / max: 17.043 / 16.667 / 16.667 / 116.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.696 / 0.000 / 16.667 / 116.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.310 / 13.348 / 26.191 / 92.150 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.088 / 0.117 / 1.375 ms (steady clock)

### Run 20261008_204641

- Started: 2026-10-08 20:46:41 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261008_204641

- Result: USER_EXIT
- Wall duration / simulation duration: 70.579 / 61.433 s
- Applied simulation control frames: 3688
- Runtime FPS (simulation time / wall time): 60.033 / 52.253 Hz
- Isaac frame interval mean / P50 / P95 / max: 19.077 / 13.826 / 49.696 / 145.747 ms
- Distinct applied command values: 599
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 106.060 / 30.716 / 78.769 / 41102.221 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261008_204641

- Wall duration: 85.189 s
- Timer callbacks / command frames / guidance frames: 3632 / 3631 / 731
- Command FPS (ROS simulation time / active wall time): 59.104 / 51.640 Hz
- Guidance FPS (active wall time): 32.655 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.919 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 5.818 / 2.971 / 23.744 / 87.949 ms
- Command interval mean / P50 / P95 / max: 19.365 / 14.559 / 54.529 / 160.074 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 31.783 / 33.333 / 50.000 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.761 / 2.971 / 17.577 / 101.376 ms (steady clock)
- DDS publish call mean / P95 / max: 0.109 / 0.465 / 5.307 ms (steady clock)

#### Command bridge — 20261008_204641

- Wall duration: 85.075 s
- Received / published control frames: 3631 / 3631
- Published control FPS (ROS simulation time / active wall time): 59.104 / 51.640 Hz
- Command stamp interval mean / P50 / P95 / max: 16.919 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.129 / 0.000 / 16.667 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 19.365 / 14.521 / 54.049 / 160.364 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.097 / 0.083 / 0.137 / 3.609 ms (steady clock)

### Run 20261008_204811

- Started: 2026-10-08 20:48:11 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `joystick`

#### Isaac command application — 20261008_204811

- Result: USER_EXIT
- Wall duration / simulation duration: 193.467 / 133.133 s
- Applied simulation control frames: 7990
- Runtime FPS (simulation time / wall time): 60.015 / 41.299 Hz
- Isaac frame interval mean / P50 / P95 / max: 24.188 / 18.872 / 57.853 / 142.732 ms
- Distinct applied command values: 4102
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 45.470 / 30.004 / 76.859 / 33096.150 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — 20261008_204811

- Wall duration: 208.479 s
- Timer callbacks / command frames / guidance frames: 7770 / 7769 / 5015
- Command FPS (ROS simulation time / active wall time): 58.355 / 40.207 Hz
- Guidance FPS (active wall time): 32.692 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.137 / 16.667 / 16.667 / 983.333 ms
- Avoidance compute mean / P50 / P95 / max: 4.159 / 0.593 / 14.423 / 1733.236 ms
- Command interval mean / P50 / P95 / max: 24.871 / 18.789 / 64.020 / 1752.795 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 35.723 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 7.650 / 4.071 / 25.880 / 1752.661 ms (steady clock)
- DDS publish call mean / P95 / max: 0.218 / 1.104 / 9.366 ms (steady clock)

#### Command bridge — 20261008_204811

- Wall duration: 208.469 s
- Received / published control frames: 7769 / 7767
- Published control FPS (ROS simulation time / active wall time): 58.355 / 40.196 Hz
- Command stamp interval mean / P50 / P95 / max: 17.137 / 16.667 / 16.667 / 983.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.714 / 0.000 / 16.667 / 983.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 24.871 / 18.704 / 64.005 / 1753.160 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.118 / 0.091 / 0.178 / 3.994 ms (steady clock)
