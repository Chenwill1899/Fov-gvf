
### Run candidate_v16_spin_ego1p3_1

- Started: 2026-10-06 23:59:41 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v16_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.150 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.230 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.333 / 15.974 / 18.810 / 79.882 ms
- Distinct applied command values: 3031
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.483 / 16.148 / 31.885 / 4074.079 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v16_spin_ego1p3_1

- Wall duration: 81.374 s
- Timer callbacks / command frames / guidance frames: 3815 / 3814 / 3275
- Command FPS (ROS simulation time / active wall time): 59.578 / 57.285 Hz
- Guidance FPS (active wall time): 53.567 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.785 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.102 / 1.075 / 6.442 / 35.486 ms
- Command interval mean / P50 / P95 / max: 17.457 / 16.341 / 23.566 / 84.828 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.654 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.894 / 3.136 / 9.943 / 41.313 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.377 / 1.846 ms (steady clock)

#### Command bridge — candidate_v16_spin_ego1p3_1

- Wall duration: 81.264 s
- Received / published control frames: 3814 / 3814
- Published control FPS (ROS simulation time / active wall time): 59.578 / 57.285 Hz
- Command stamp interval mean / P50 / P95 / max: 16.785 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.459 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.456 / 16.343 / 23.600 / 85.102 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.086 / 0.121 / 0.991 ms (steady clock)
