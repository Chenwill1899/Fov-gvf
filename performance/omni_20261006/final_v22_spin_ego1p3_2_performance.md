
### Run final_v22_spin_ego1p3_2

- Started: 2026-10-07 02:05:28 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_spin_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.731 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.590 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.222 / 15.999 / 17.970 / 79.534 ms
- Distinct applied command values: 2828
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.040 / 16.485 / 33.014 / 4024.077 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_spin_ego1p3_2

- Wall duration: 80.874 s
- Timer callbacks / command frames / guidance frames: 3813 / 3812 / 3271
- Command FPS (ROS simulation time / active wall time): 59.547 / 57.627 Hz
- Guidance FPS (active wall time): 53.233 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.793 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.483 / 0.427 / 6.626 / 70.848 ms
- Command interval mean / P50 / P95 / max: 17.353 / 16.051 / 24.670 / 90.360 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.686 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.640 / 2.816 / 10.248 / 74.577 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.295 / 1.807 ms (steady clock)

#### Command bridge — final_v22_spin_ego1p3_2

- Wall duration: 80.762 s
- Received / published control frames: 3812 / 3812
- Published control FPS (ROS simulation time / active wall time): 59.547 / 57.628 Hz
- Command stamp interval mean / P50 / P95 / max: 16.793 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.455 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.353 / 16.029 / 24.645 / 90.466 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.112 / 0.655 ms (steady clock)
