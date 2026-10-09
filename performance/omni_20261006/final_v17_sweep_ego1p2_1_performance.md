
### Run final_v17_sweep_ego1p2_1

- Started: 2026-10-07 00:15:00 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_sweep_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.596 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.769 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.173 / 15.942 / 17.949 / 79.139 ms
- Distinct applied command values: 1639
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 24.034 / 16.503 / 33.708 / 1647.754 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_sweep_ego1p2_1

- Wall duration: 55.519 s
- Timer callbacks / command frames / guidance frames: 2280 / 2279 / 2098
- Command FPS (ROS simulation time / active wall time): 56.950 / 55.275 Hz
- Guidance FPS (active wall time): 54.594 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.559 / 16.667 / 16.667 / 266.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.654 / 0.361 / 1.241 / 259.303 ms
- Command interval mean / P50 / P95 / max: 18.092 / 16.020 / 22.626 / 269.918 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 67.183 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.634 / 2.363 / 6.496 / 269.855 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.384 / 1.278 ms (steady clock)

#### Command bridge — final_v17_sweep_ego1p2_1

- Wall duration: 55.410 s
- Received / published control frames: 2279 / 2270
- Published control FPS (ROS simulation time / active wall time): 56.950 / 55.056 Hz
- Command stamp interval mean / P50 / P95 / max: 17.559 / 16.667 / 16.667 / 266.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.177 / 0.000 / 0.000 / 266.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.092 / 15.950 / 22.633 / 269.997 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.083 / 0.113 / 1.356 ms (steady clock)
