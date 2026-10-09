
### Run candidate_v10_spin_ego1p3_1

- Started: 2026-10-06 23:15:42 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v10_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 71.861 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 53.478 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.560 / 16.529 / 23.853 / 120.404 ms
- Distinct applied command values: 2933
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.976 / 16.829 / 35.814 / 4752.186 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — candidate_v10_spin_ego1p3_1

- Wall duration: 85.544 s
- Received / published control frames: 3768 / 3767
- Published control FPS (ROS simulation time / active wall time): 58.859 / 52.843 Hz
- Command stamp interval mean / P50 / P95 / max: 16.990 / 16.667 / 16.667 / 333.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.115 / 0.000 / 0.000 / 333.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.919 / 16.799 / 29.772 / 313.886 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.096 / 0.087 / 0.133 / 3.971 ms (steady clock)

#### Controller — candidate_v10_spin_ego1p3_1

- Wall duration: 85.651 s
- Timer callbacks / command frames / guidance frames: 3769 / 3768 / 3227
- Command FPS (ROS simulation time / active wall time): 58.859 / 52.856 Hz
- Guidance FPS (active wall time): 48.525 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.990 / 16.667 / 16.667 / 333.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.657 / 0.331 / 10.533 / 285.367 ms
- Command interval mean / P50 / P95 / max: 18.919 / 16.830 / 29.691 / 313.918 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.610 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.720 / 3.423 / 14.352 / 313.797 ms (steady clock)
- DDS publish call mean / P95 / max: 0.076 / 0.439 / 3.497 ms (steady clock)
