
### Run explore_v1_spin_ego1p3_1

- Started: 2026-10-06 22:09:52 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v1_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.657 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.654 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.205 / 15.900 / 18.138 / 78.432 ms
- Distinct applied command values: 1684
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 35.866 / 16.301 / 75.695 / 5464.550 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v1_spin_ego1p3_1

- Wall duration: 80.320 s
- Timer callbacks / command frames / guidance frames: 2495 / 2494 / 1959
- Command FPS (ROS simulation time / active wall time): 38.953 / 37.736 Hz
- Guidance FPS (active wall time): 31.730 Hz
- Command interval in ROS time mean / P50 / P95 / max: 25.672 / 16.667 / 100.000 / 283.333 ms
- Avoidance compute mean / P50 / P95 / max: 16.177 / 4.459 / 121.348 / 256.250 ms
- Command interval mean / P50 / P95 / max: 26.500 / 16.609 / 120.301 / 268.368 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.779 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 16.636 / 5.258 / 119.002 / 267.698 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.360 / 1.081 ms (steady clock)

#### Command bridge — explore_v1_spin_ego1p3_1

- Wall duration: 80.213 s
- Received / published control frames: 2494 / 2369
- Published control FPS (ROS simulation time / active wall time): 38.953 / 35.844 Hz
- Command stamp interval mean / P50 / P95 / max: 25.672 / 16.667 / 100.000 / 283.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 12.477 / 0.000 / 100.000 / 283.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 26.500 / 16.603 / 120.446 / 268.399 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.086 / 0.120 / 1.152 ms (steady clock)
