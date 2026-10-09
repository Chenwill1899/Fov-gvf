
### Run final_v16_sweep_ego1p2_1

- Started: 2026-10-07 00:03:55 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v16_sweep_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.145 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 58.403 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.987 / 15.809 / 17.817 / 78.846 ms
- Distinct applied command values: 1663
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.439 / 16.360 / 33.903 / 883.294 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v16_sweep_ego1p2_1

- Wall duration: 55.048 s
- Received / published control frames: 2306 / 2301
- Published control FPS (ROS simulation time / active wall time): 57.625 / 56.417 Hz
- Command stamp interval mean / P50 / P95 / max: 17.354 / 16.667 / 16.667 / 250.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.005 / 0.000 / 0.000 / 250.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.687 / 15.962 / 23.184 / 221.431 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.081 / 0.107 / 1.408 ms (steady clock)

#### Controller — final_v16_sweep_ego1p2_1

- Wall duration: 55.153 s
- Timer callbacks / command frames / guidance frames: 2307 / 2306 / 2125
- Command FPS (ROS simulation time / active wall time): 57.625 / 56.539 Hz
- Guidance FPS (active wall time): 55.784 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.354 / 16.667 / 16.667 / 250.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.502 / 0.367 / 1.435 / 206.549 ms
- Command interval mean / P50 / P95 / max: 17.687 / 15.988 / 23.247 / 221.451 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 74.878 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.469 / 1.696 / 7.922 / 221.378 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.413 / 2.053 ms (steady clock)
