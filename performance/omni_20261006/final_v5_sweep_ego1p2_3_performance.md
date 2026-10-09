
### Run final_v5_sweep_ego1p2_3

- Started: 2026-10-06 22:33:17 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_sweep_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.716 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 56.256 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.640 / 16.101 / 18.595 / 82.625 ms
- Distinct applied command values: 1764
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.840 / 16.526 / 35.178 / 897.525 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v5_sweep_ego1p2_3

- Wall duration: 56.479 s
- Received / published control frames: 2315 / 2311
- Published control FPS (ROS simulation time / active wall time): 57.850 / 54.566 Hz
- Command stamp interval mean / P50 / P95 / max: 17.286 / 16.667 / 16.667 / 250.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.871 / 0.000 / 0.000 / 250.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.295 / 16.197 / 25.367 / 277.845 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.113 / 1.025 ms (steady clock)

#### Controller — final_v5_sweep_ego1p2_3

- Wall duration: 56.586 s
- Timer callbacks / command frames / guidance frames: 2316 / 2315 / 2134
- Command FPS (ROS simulation time / active wall time): 57.850 / 54.661 Hz
- Guidance FPS (active wall time): 54.179 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.286 / 16.667 / 16.667 / 250.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.554 / 0.395 / 2.758 / 263.317 ms
- Command interval mean / P50 / P95 / max: 18.295 / 16.168 / 25.050 / 277.823 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 73.883 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.649 / 2.593 / 7.904 / 277.710 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.202 / 1.674 ms (steady clock)
