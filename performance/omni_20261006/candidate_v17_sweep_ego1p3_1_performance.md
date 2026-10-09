
### Run candidate_v17_sweep_ego1p3_1

- Started: 2026-10-07 00:11:26 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v17_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.245 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 56.883 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.448 / 16.027 / 18.851 / 80.538 ms
- Distinct applied command values: 1877
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.308 / 16.544 / 33.205 / 870.895 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v17_sweep_ego1p3_1

- Wall duration: 56.154 s
- Timer callbacks / command frames / guidance frames: 2401 / 2400 / 2219
- Command FPS (ROS simulation time / active wall time): 59.975 / 57.293 Hz
- Guidance FPS (active wall time): 56.718 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.332 / 0.223 / 1.088 / 9.917 ms
- Command interval mean / P50 / P95 / max: 17.454 / 16.011 / 23.792 / 87.454 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.493 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.423 / 1.663 / 6.810 / 16.129 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.169 / 2.255 ms (steady clock)

#### Command bridge — candidate_v17_sweep_ego1p3_1

- Wall duration: 56.049 s
- Received / published control frames: 2400 / 2400
- Published control FPS (ROS simulation time / active wall time): 59.975 / 57.293 Hz
- Command stamp interval mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.454 / 15.972 / 23.783 / 87.471 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.111 / 0.517 ms (steady clock)
