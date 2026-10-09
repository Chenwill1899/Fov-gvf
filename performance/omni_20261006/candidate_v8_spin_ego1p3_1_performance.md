
### Run candidate_v8_spin_ego1p3_1

- Started: 2026-10-06 22:59:33 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v8_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 71.035 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 54.100 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.345 / 16.660 / 22.156 / 88.591 ms
- Distinct applied command values: 3037
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.915 / 16.937 / 34.400 / 4574.275 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v8_spin_ego1p3_1

- Wall duration: 84.773 s
- Timer callbacks / command frames / guidance frames: 3842 / 3842 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.081 Hz
- Guidance FPS (active wall time): 50.260 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.354 / 2.794 / 5.528 / 9.763 ms
- Command interval mean / P50 / P95 / max: 18.491 / 16.679 / 25.691 / 576.476 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.293 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.087 / 3.849 / 9.858 / 17.048 ms (steady clock)
- DDS publish call mean / P95 / max: 0.076 / 0.424 / 3.458 ms (steady clock)

#### Command bridge — candidate_v8_spin_ego1p3_1

- Wall duration: 84.667 s
- Received / published control frames: 3842 / 3842
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.081 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.004 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.491 / 16.672 / 25.471 / 576.772 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.096 / 0.089 / 0.129 / 3.150 ms (steady clock)
