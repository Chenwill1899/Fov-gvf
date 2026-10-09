
### Run p5_retain_20261008_spin_spin_v2_2

- Started: 2026-10-08 10:22:56 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_v2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.340 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.816 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.865 / 15.692 / 17.606 / 78.704 ms
- Distinct applied command values: 3070
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.909 / 15.913 / 32.123 / 3968.046 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_v2_2

- Wall duration: 79.126 s
- Timer callbacks / command frames / guidance frames: 3838 / 3837 / 3296
- Command FPS (ROS simulation time / active wall time): 59.938 / 59.234 Hz
- Guidance FPS (active wall time): 54.703 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.211 / 0.413 / 4.658 / 39.777 ms
- Command interval mean / P50 / P95 / max: 16.882 / 15.785 / 22.687 / 86.564 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.193 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.324 / 3.106 / 8.453 / 41.435 ms (steady clock)
- DDS publish call mean / P95 / max: 0.055 / 0.107 / 1.431 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_v2_2

- Wall duration: 79.017 s
- Received / published control frames: 3837 / 3837
- Published control FPS (ROS simulation time / active wall time): 59.938 / 59.234 Hz
- Command stamp interval mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.074 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.882 / 15.751 / 22.659 / 86.334 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.082 / 0.106 / 1.124 ms (steady clock)
