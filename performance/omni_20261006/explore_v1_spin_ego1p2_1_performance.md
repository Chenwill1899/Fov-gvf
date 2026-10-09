
### Run explore_v1_spin_ego1p2_1

- Started: 2026-10-06 22:08:30 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v1_spin_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.207 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.182 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.352 / 16.080 / 18.786 / 83.080 ms
- Distinct applied command values: 2795
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.437 / 16.643 / 34.077 / 4100.380 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v1_spin_ego1p2_1

- Wall duration: 80.820 s
- Timer callbacks / command frames / guidance frames: 3840 / 3839 / 3298
- Command FPS (ROS simulation time / active wall time): 59.969 / 57.603 Hz
- Guidance FPS (active wall time): 53.334 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.594 / 0.523 / 1.004 / 5.753 ms
- Command interval mean / P50 / P95 / max: 17.360 / 15.738 / 25.735 / 89.706 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.269 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.659 / 0.826 / 8.508 / 22.518 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.132 / 2.595 ms (steady clock)

#### Command bridge — explore_v1_spin_ego1p2_1

- Wall duration: 80.713 s
- Received / published control frames: 3839 / 3839
- Published control FPS (ROS simulation time / active wall time): 59.969 / 57.603 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.056 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.360 / 15.741 / 25.770 / 89.591 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.083 / 0.114 / 1.864 ms (steady clock)
