
### Run candidate_v12_spin_ego1p3_1

- Started: 2026-10-06 23:22:35 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v12_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.720 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.749 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.478 / 15.987 / 18.679 / 79.371 ms
- Distinct applied command values: 3046
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.788 / 16.304 / 32.823 / 3980.747 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v12_spin_ego1p3_1

- Wall duration: 81.466 s
- Timer callbacks / command frames / guidance frames: 3826 / 3825 / 3284
- Command FPS (ROS simulation time / active wall time): 59.750 / 56.977 Hz
- Guidance FPS (active wall time): 52.625 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.736 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 2.437 / 2.419 / 6.795 / 30.634 ms
- Command interval mean / P50 / P95 / max: 17.551 / 16.133 / 23.736 / 88.236 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.717 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.297 / 3.802 / 10.087 / 37.399 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.356 / 3.162 ms (steady clock)

#### Command bridge — candidate_v12_spin_ego1p3_1

- Wall duration: 81.362 s
- Received / published control frames: 3825 / 3825
- Published control FPS (ROS simulation time / active wall time): 59.750 / 56.977 Hz
- Command stamp interval mean / P50 / P95 / max: 16.736 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.318 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.551 / 16.165 / 23.730 / 88.241 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.111 / 3.260 ms (steady clock)
