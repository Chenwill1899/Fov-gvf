
### Run final_v17_sweep_ego1p3_3

- Started: 2026-10-07 00:19:42 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_sweep_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.657 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 56.333 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.617 / 15.954 / 18.212 / 79.612 ms
- Distinct applied command values: 1891
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.351 / 16.472 / 33.630 / 890.200 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v17_sweep_ego1p3_3

- Wall duration: 56.853 s
- Received / published control frames: 2400 / 2400
- Published control FPS (ROS simulation time / active wall time): 59.975 / 56.744 Hz
- Command stamp interval mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.623 / 16.008 / 23.235 / 84.468 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.085 / 0.110 / 1.373 ms (steady clock)

#### Controller — final_v17_sweep_ego1p3_3

- Wall duration: 56.965 s
- Timer callbacks / command frames / guidance frames: 2401 / 2400 / 2219
- Command FPS (ROS simulation time / active wall time): 59.975 / 56.744 Hz
- Guidance FPS (active wall time): 56.186 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.321 / 0.222 / 1.094 / 4.453 ms
- Command interval mean / P50 / P95 / max: 17.623 / 15.970 / 23.280 / 84.551 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.951 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.346 / 2.046 / 6.938 / 13.178 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.093 / 1.009 ms (steady clock)
