
### Run p5_retain_20261008_spin_spin_baseline_3

- Started: 2026-10-08 10:48:38 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_baseline_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.387 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.887 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.138 / 15.815 / 17.864 / 78.950 ms
- Distinct applied command values: 3101
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.998 / 16.019 / 32.231 / 4110.602 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_spin_spin_baseline_3

- Wall duration: 80.030 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 58.350 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.052 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.138 / 15.492 / 23.225 / 91.200 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.108 / 1.404 ms (steady clock)

#### Controller — p5_retain_20261008_spin_spin_baseline_3

- Wall duration: 80.138 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 58.349 Hz
- Guidance FPS (active wall time): 54.090 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.012 / 0.419 / 4.312 / 21.380 ms
- Command interval mean / P50 / P95 / max: 17.138 / 15.494 / 23.298 / 91.125 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.449 / 33.333 / 50.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.239 / 3.107 / 8.275 / 24.438 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.154 / 1.863 ms (steady clock)
