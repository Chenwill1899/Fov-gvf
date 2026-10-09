
### Run final_v5_spin_ego1p3_2

- Started: 2026-10-06 22:37:54 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_spin_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 68.058 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.467 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.556 / 16.046 / 18.717 / 80.870 ms
- Distinct applied command values: 3003
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.139 / 16.331 / 32.651 / 4424.716 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v5_spin_ego1p3_2

- Wall duration: 81.507 s
- Received / published control frames: 3838 / 3838
- Published control FPS (ROS simulation time / active wall time): 59.953 / 56.917 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.043 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.569 / 15.969 / 24.068 / 89.798 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.112 / 0.537 ms (steady clock)

#### Controller — final_v5_spin_ego1p3_2

- Wall duration: 81.620 s
- Timer callbacks / command frames / guidance frames: 3839 / 3838 / 3297
- Command FPS (ROS simulation time / active wall time): 59.953 / 56.917 Hz
- Guidance FPS (active wall time): 52.670 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.280 / 2.700 / 5.496 / 9.578 ms
- Command interval mean / P50 / P95 / max: 17.569 / 15.967 / 24.183 / 89.765 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 47.594 / 50.000 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.018 / 3.793 / 9.666 / 21.548 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.213 / 1.856 ms (steady clock)
