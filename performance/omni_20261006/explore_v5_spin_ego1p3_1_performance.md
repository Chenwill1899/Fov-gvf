
### Run explore_v5_spin_ego1p3_1

- Started: 2026-10-06 22:21:51 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v5_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.197 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.190 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.349 / 16.033 / 18.312 / 80.156 ms
- Distinct applied command values: 3068
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.439 / 16.256 / 32.106 / 4249.365 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v5_spin_ego1p3_1

- Wall duration: 80.813 s
- Timer callbacks / command frames / guidance frames: 3839 / 3838 / 3297
- Command FPS (ROS simulation time / active wall time): 59.953 / 57.595 Hz
- Guidance FPS (active wall time): 53.338 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.333 / 2.754 / 5.394 / 12.567 ms
- Command interval mean / P50 / P95 / max: 17.363 / 16.142 / 22.405 / 84.145 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.751 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.083 / 3.823 / 8.950 / 22.065 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.309 / 1.160 ms (steady clock)

#### Command bridge — explore_v5_spin_ego1p3_1

- Wall duration: 80.709 s
- Received / published control frames: 3838 / 3838
- Published control FPS (ROS simulation time / active wall time): 59.953 / 57.595 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.039 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.362 / 16.141 / 22.356 / 84.202 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.085 / 0.109 / 0.948 ms (steady clock)
