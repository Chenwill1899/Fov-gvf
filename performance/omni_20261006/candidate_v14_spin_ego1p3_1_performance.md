
### Run candidate_v14_spin_ego1p3_1

- Started: 2026-10-06 23:40:49 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v14_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.717 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.751 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.478 / 16.137 / 18.272 / 80.287 ms
- Distinct applied command values: 2901
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.833 / 16.545 / 33.170 / 4301.806 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — candidate_v14_spin_ego1p3_1

- Wall duration: 81.437 s
- Received / published control frames: 3837 / 3837
- Published control FPS (ROS simulation time / active wall time): 59.938 / 57.157 Hz
- Command stamp interval mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.039 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.496 / 16.128 / 23.040 / 84.683 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.110 / 1.300 ms (steady clock)

#### Controller — candidate_v14_spin_ego1p3_1

- Wall duration: 81.852 s
- Timer callbacks / command frames / guidance frames: 3838 / 3837 / 3296
- Command FPS (ROS simulation time / active wall time): 59.938 / 57.156 Hz
- Guidance FPS (active wall time): 52.806 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.072 / 0.429 / 4.157 / 28.183 ms
- Command interval mean / P50 / P95 / max: 17.496 / 16.136 / 23.050 / 84.574 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.386 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.269 / 3.210 / 7.990 / 31.477 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.338 / 1.889 ms (steady clock)
