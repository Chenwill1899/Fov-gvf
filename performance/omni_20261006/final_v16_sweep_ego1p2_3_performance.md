
### Run final_v16_sweep_ego1p2_3

- Started: 2026-10-07 00:09:08 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v16_sweep_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.852 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.416 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.280 / 15.895 / 17.989 / 78.743 ms
- Distinct applied command values: 1718
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.063 / 16.384 / 33.782 / 986.800 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v16_sweep_ego1p2_3

- Wall duration: 56.155 s
- Timer callbacks / command frames / guidance frames: 2291 / 2290 / 2109
- Command FPS (ROS simulation time / active wall time): 57.225 / 55.194 Hz
- Guidance FPS (active wall time): 54.455 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.475 / 16.667 / 16.667 / 250.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.609 / 0.380 / 1.606 / 197.152 ms
- Command interval mean / P50 / P95 / max: 18.118 / 16.178 / 22.321 / 212.139 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 67.402 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.647 / 2.563 / 6.530 / 211.604 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.419 / 1.933 ms (steady clock)

#### Command bridge — final_v16_sweep_ego1p2_3

- Wall duration: 55.709 s
- Received / published control frames: 2290 / 2281
- Published control FPS (ROS simulation time / active wall time): 57.225 / 54.977 Hz
- Command stamp interval mean / P50 / P95 / max: 17.475 / 16.667 / 16.667 / 250.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.114 / 0.000 / 0.000 / 233.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.118 / 16.160 / 22.399 / 212.174 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.111 / 0.813 ms (steady clock)
