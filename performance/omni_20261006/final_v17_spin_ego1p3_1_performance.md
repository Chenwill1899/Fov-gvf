
### Run final_v17_spin_ego1p3_1

- Started: 2026-10-07 00:22:01 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 71.219 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 53.961 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.397 / 16.644 / 22.631 / 150.781 ms
- Distinct applied command values: 2889
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.109 / 17.028 / 35.137 / 4687.954 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v17_spin_ego1p3_1

- Wall duration: 84.862 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.359 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.013 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.396 / 16.806 / 26.060 / 155.071 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.097 / 0.091 / 0.133 / 1.363 ms (steady clock)

#### Controller — final_v17_spin_ego1p3_1

- Wall duration: 84.969 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.359 Hz
- Guidance FPS (active wall time): 50.089 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.730 / 0.411 / 3.311 / 17.757 ms
- Command interval mean / P50 / P95 / max: 18.396 / 16.811 / 26.076 / 155.404 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.768 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.864 / 2.628 / 7.627 / 27.720 ms (steady clock)
- DDS publish call mean / P95 / max: 0.068 / 0.320 / 4.710 ms (steady clock)
