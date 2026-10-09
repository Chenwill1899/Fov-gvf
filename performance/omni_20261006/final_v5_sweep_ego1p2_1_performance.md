
### Run final_v5_sweep_ego1p2_1

- Started: 2026-10-06 22:29:22 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_sweep_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 44.516 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 53.980 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.235 / 16.474 / 19.874 / 82.402 ms
- Distinct applied command values: 1808
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.127 / 16.803 / 37.460 / 1297.492 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v5_sweep_ego1p2_1

- Wall duration: 59.998 s
- Received / published control frames: 2330 / 2326
- Published control FPS (ROS simulation time / active wall time): 58.225 / 53.134 Hz
- Command stamp interval mean / P50 / P95 / max: 17.175 / 16.667 / 16.667 / 183.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.737 / 0.000 / 0.000 / 183.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.788 / 16.661 / 25.035 / 181.575 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.093 / 0.086 / 0.120 / 2.036 ms (steady clock)

#### Controller — final_v5_sweep_ego1p2_1

- Wall duration: 60.103 s
- Timer callbacks / command frames / guidance frames: 2331 / 2330 / 2149
- Command FPS (ROS simulation time / active wall time): 58.225 / 53.225 Hz
- Guidance FPS (active wall time): 53.046 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.175 / 16.667 / 16.667 / 183.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.370 / 0.410 / 1.774 / 166.221 ms
- Command interval mean / P50 / P95 / max: 18.788 / 16.669 / 25.372 / 181.702 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 68.458 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.440 / 2.541 / 7.137 / 181.584 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.340 / 2.120 ms (steady clock)
