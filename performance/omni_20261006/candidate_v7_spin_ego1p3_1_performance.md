
### Run candidate_v7_spin_ego1p3_1

- Started: 2026-10-06 22:47:51 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v7_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.801 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.681 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.503 / 15.780 / 18.033 / 80.356 ms
- Distinct applied command values: 2918
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.348 / 16.116 / 32.592 / 4225.044 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v7_spin_ego1p3_1

- Wall duration: 82.429 s
- Timer callbacks / command frames / guidance frames: 3714 / 3714 / 3179
- Command FPS (ROS simulation time / active wall time): 58.001 / 54.774 Hz
- Guidance FPS (active wall time): 50.776 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.241 / 16.667 / 16.667 / 133.333 ms
- Avoidance compute mean / P50 / P95 / max: 3.091 / 2.390 / 6.719 / 103.125 ms
- Command interval mean / P50 / P95 / max: 18.257 / 15.649 / 26.333 / 577.454 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.901 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.012 / 3.821 / 11.711 / 124.187 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.414 / 2.107 ms (steady clock)

#### Command bridge — candidate_v7_spin_ego1p3_1

- Wall duration: 82.320 s
- Received / published control frames: 3714 / 3703
- Published control FPS (ROS simulation time / active wall time): 58.001 / 54.612 Hz
- Command stamp interval mean / P50 / P95 / max: 17.241 / 16.667 / 16.667 / 133.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.983 / 0.000 / 0.000 / 133.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.257 / 15.624 / 26.412 / 577.615 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.109 / 1.155 ms (steady clock)
