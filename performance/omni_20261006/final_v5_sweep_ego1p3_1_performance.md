
### Run final_v5_sweep_ego1p3_1

- Started: 2026-10-06 22:30:23 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 43.568 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 55.155 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.971 / 16.132 / 18.623 / 80.290 ms
- Distinct applied command values: 1888
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.721 / 16.731 / 33.800 / 962.079 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v5_sweep_ego1p3_1

- Wall duration: 58.440 s
- Received / published control frames: 2399 / 2399
- Published control FPS (ROS simulation time / active wall time): 59.950 / 55.598 Hz
- Command stamp interval mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.986 / 15.819 / 23.284 / 89.969 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.092 / 0.087 / 0.114 / 1.309 ms (steady clock)

#### Controller — final_v5_sweep_ego1p3_1

- Wall duration: 58.551 s
- Timer callbacks / command frames / guidance frames: 2400 / 2399 / 2219
- Command FPS (ROS simulation time / active wall time): 59.950 / 55.598 Hz
- Guidance FPS (active wall time): 55.414 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.680 / 0.222 / 2.579 / 10.014 ms
- Command interval mean / P50 / P95 / max: 17.986 / 15.882 / 23.335 / 90.084 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.852 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.787 / 2.495 / 7.238 / 15.337 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.122 / 1.442 ms (steady clock)
