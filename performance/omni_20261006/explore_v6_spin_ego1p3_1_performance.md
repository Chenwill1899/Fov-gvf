
### Run explore_v6_spin_ego1p3_1

- Started: 2026-10-06 22:24:23 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v6_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 69.278 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 55.472 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.886 / 16.276 / 19.338 / 83.731 ms
- Distinct applied command values: 3021
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.492 / 16.521 / 33.873 / 4382.073 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v6_spin_ego1p3_1

- Wall duration: 83.412 s
- Timer callbacks / command frames / guidance frames: 3771 / 3770 / 3230
- Command FPS (ROS simulation time / active wall time): 58.891 / 54.924 Hz
- Guidance FPS (active wall time): 50.503 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.981 / 16.667 / 16.667 / 216.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.377 / 2.952 / 8.743 / 216.417 ms
- Command interval mean / P50 / P95 / max: 18.207 / 16.394 / 24.934 / 229.069 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.142 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.238 / 4.522 / 11.608 / 218.601 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.374 / 2.042 ms (steady clock)

#### Command bridge — explore_v6_spin_ego1p3_1

- Wall duration: 83.302 s
- Received / published control frames: 3770 / 3769
- Published control FPS (ROS simulation time / active wall time): 58.891 / 54.910 Hz
- Command stamp interval mean / P50 / P95 / max: 16.981 / 16.667 / 16.667 / 216.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.654 / 0.000 / 0.000 / 216.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.207 / 16.398 / 24.903 / 229.174 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.086 / 0.115 / 1.153 ms (steady clock)
