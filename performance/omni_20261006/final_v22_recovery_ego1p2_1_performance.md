
### Run final_v22_recovery_ego1p2_1

- Started: 2026-10-07 02:10:56 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_recovery_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 72.707 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 59.458 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.696 / 15.520 / 17.582 / 79.344 ms
- Distinct applied command values: 2521
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.127 / 16.099 / 32.380 / 3926.252 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_recovery_ego1p2_1

- Wall duration: 86.571 s
- Timer callbacks / command frames / guidance frames: 4280 / 4279 / 3198
- Command FPS (ROS simulation time / active wall time): 59.417 / 59.312 Hz
- Guidance FPS (active wall time): 47.412 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.830 / 16.667 / 16.667 / 633.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.875 / 0.487 / 1.267 / 595.795 ms
- Command interval mean / P50 / P95 / max: 16.860 / 15.481 / 23.002 / 604.873 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 64.832 / 66.667 / 116.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.944 / 2.149 / 8.088 / 604.800 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.270 / 2.388 ms (steady clock)

#### Command bridge — final_v22_recovery_ego1p2_1

- Wall duration: 86.465 s
- Received / published control frames: 4279 / 4278
- Published control FPS (ROS simulation time / active wall time): 59.417 / 59.299 Hz
- Command stamp interval mean / P50 / P95 / max: 16.830 / 16.667 / 16.667 / 633.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.261 / 0.000 / 0.000 / 633.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.860 / 15.490 / 22.766 / 604.977 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.082 / 0.110 / 2.903 ms (steady clock)
