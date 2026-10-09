
### Run final_v5_sweep_ego1p3_2

- Started: 2026-10-06 22:31:22 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_sweep_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.670 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 56.316 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.620 / 16.109 / 18.438 / 79.086 ms
- Distinct applied command values: 1880
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.527 / 16.653 / 33.773 / 990.749 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_sweep_ego1p3_2

- Wall duration: 56.598 s
- Timer callbacks / command frames / guidance frames: 2398 / 2397 / 2216
- Command FPS (ROS simulation time / active wall time): 59.900 / 56.658 Hz
- Guidance FPS (active wall time): 56.130 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.694 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.679 / 0.221 / 2.451 / 13.188 ms
- Command interval mean / P50 / P95 / max: 17.650 / 15.992 / 24.190 / 85.630 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 44.660 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.707 / 1.919 / 8.024 / 20.357 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.123 / 2.591 ms (steady clock)

#### Command bridge — final_v5_sweep_ego1p3_2

- Wall duration: 56.490 s
- Received / published control frames: 2397 / 2397
- Published control FPS (ROS simulation time / active wall time): 59.900 / 56.658 Hz
- Command stamp interval mean / P50 / P95 / max: 16.694 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.014 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.650 / 15.999 / 24.005 / 85.683 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.111 / 0.604 ms (steady clock)
