
### Run final_v22_sweep_ego1p2_3

- Started: 2026-10-07 02:00:35 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_sweep_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.545 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.841 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.154 / 15.808 / 17.933 / 77.684 ms
- Distinct applied command values: 1781
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.977 / 16.338 / 33.570 / 890.811 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_sweep_ego1p2_3

- Wall duration: 55.443 s
- Received / published control frames: 2322 / 2319
- Published control FPS (ROS simulation time / active wall time): 58.025 / 56.307 Hz
- Command stamp interval mean / P50 / P95 / max: 17.234 / 16.667 / 16.667 / 200.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.847 / 0.000 / 0.000 / 200.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.737 / 15.838 / 23.913 / 189.439 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.113 / 0.999 ms (steady clock)

#### Controller — final_v22_sweep_ego1p2_3

- Wall duration: 55.551 s
- Timer callbacks / command frames / guidance frames: 2323 / 2322 / 2141
- Command FPS (ROS simulation time / active wall time): 58.025 / 56.379 Hz
- Guidance FPS (active wall time): 55.981 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.234 / 16.667 / 16.667 / 200.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.377 / 0.406 / 1.649 / 189.181 ms
- Command interval mean / P50 / P95 / max: 17.737 / 15.793 / 23.978 / 189.452 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 70.582 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.416 / 1.956 / 8.215 / 189.307 ms (steady clock)
- DDS publish call mean / P95 / max: 0.068 / 0.425 / 1.909 ms (steady clock)
