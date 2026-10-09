
### Run candidate_v19_spin_ego1p3_1

- Started: 2026-10-07 01:02:19 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v19_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.083 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.287 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.318 / 15.994 / 18.048 / 81.088 ms
- Distinct applied command values: 2735
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.937 / 16.475 / 33.574 / 4212.359 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v19_spin_ego1p3_1

- Wall duration: 80.855 s
- Timer callbacks / command frames / guidance frames: 3687 / 3686 / 3146
- Command FPS (ROS simulation time / active wall time): 57.578 / 55.413 Hz
- Guidance FPS (active wall time): 50.910 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.368 / 16.667 / 16.667 / 216.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.202 / 0.413 / 8.631 / 179.797 ms
- Command interval mean / P50 / P95 / max: 18.046 / 16.019 / 29.132 / 197.539 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.532 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.513 / 3.019 / 13.134 / 197.457 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.304 / 1.559 ms (steady clock)

#### Command bridge — candidate_v19_spin_ego1p3_1

- Wall duration: 80.748 s
- Received / published control frames: 3686 / 3685
- Published control FPS (ROS simulation time / active wall time): 57.578 / 55.399 Hz
- Command stamp interval mean / P50 / P95 / max: 17.368 / 16.667 / 16.667 / 216.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.479 / 0.000 / 0.000 / 216.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.046 / 15.990 / 29.186 / 197.458 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.086 / 0.112 / 1.619 ms (steady clock)
