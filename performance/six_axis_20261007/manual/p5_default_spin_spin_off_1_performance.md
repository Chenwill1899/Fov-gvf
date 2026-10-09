
### Run p5_default_spin_spin_off_1

- Started: 2026-10-07 17:04:38 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_default_spin_spin_off_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.119 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.257 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.326 / 15.888 / 17.928 / 79.503 ms
- Distinct applied command values: 2798
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.427 / 16.402 / 33.245 / 4103.664 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_default_spin_spin_off_1

- Wall duration: 80.847 s
- Timer callbacks / command frames / guidance frames: 3799 / 3798 / 3257
- Command FPS (ROS simulation time / active wall time): 59.328 / 57.073 Hz
- Guidance FPS (active wall time): 52.656 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.855 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.522 / 0.413 / 6.921 / 60.698 ms
- Command interval mean / P50 / P95 / max: 17.521 / 15.761 / 24.743 / 92.261 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.404 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.747 / 2.850 / 10.250 / 66.987 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.376 / 1.293 ms (steady clock)

#### Command bridge — p5_default_spin_spin_off_1

- Wall duration: 80.736 s
- Received / published control frames: 3798 / 3798
- Published control FPS (ROS simulation time / active wall time): 59.328 / 57.073 Hz
- Command stamp interval mean / P50 / P95 / max: 16.855 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.610 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.521 / 15.743 / 24.784 / 92.273 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.110 / 1.216 ms (steady clock)
