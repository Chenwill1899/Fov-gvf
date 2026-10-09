
### Run p5_retain_20261008_manual_default_spin_baseline_1

- Started: 2026-10-08 11:13:42 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_manual_default_spin_baseline_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.036 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.327 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.301 / 15.833 / 17.961 / 78.999 ms
- Distinct applied command values: 2850
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.013 / 16.068 / 34.081 / 4068.476 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_manual_default_spin_baseline_1

- Wall duration: 80.743 s
- Timer callbacks / command frames / guidance frames: 3781 / 3780 / 3242
- Command FPS (ROS simulation time / active wall time): 59.047 / 56.886 Hz
- Guidance FPS (active wall time): 52.425 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.936 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.348 / 0.415 / 4.726 / 66.260 ms
- Command interval mean / P50 / P95 / max: 17.579 / 15.723 / 28.332 / 90.618 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 51.563 / 50.000 / 100.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.408 / 0.663 / 11.459 / 78.034 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.429 / 1.532 ms (steady clock)

#### Command bridge — p5_retain_20261008_manual_default_spin_baseline_1

- Wall duration: 80.630 s
- Received / published control frames: 3780 / 3780
- Published control FPS (ROS simulation time / active wall time): 59.047 / 56.886 Hz
- Command stamp interval mean / P50 / P95 / max: 16.936 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.556 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.579 / 15.707 / 28.128 / 90.593 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.112 / 1.167 ms (steady clock)
