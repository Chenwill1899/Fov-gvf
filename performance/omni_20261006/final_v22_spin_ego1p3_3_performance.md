
### Run final_v22_spin_ego1p3_3

- Started: 2026-10-07 02:09:32 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_spin_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 68.510 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.094 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.688 / 15.978 / 18.269 / 86.370 ms
- Distinct applied command values: 2872
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.346 / 16.477 / 33.265 / 4352.569 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_spin_ego1p3_3

- Wall duration: 82.552 s
- Received / published control frames: 3829 / 3829
- Published control FPS (ROS simulation time / active wall time): 59.812 / 56.360 Hz
- Command stamp interval mean / P50 / P95 / max: 16.719 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.239 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.743 / 16.143 / 23.616 / 89.625 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.087 / 0.113 / 0.712 ms (steady clock)

#### Controller — final_v22_spin_ego1p3_3

- Wall duration: 82.661 s
- Timer callbacks / command frames / guidance frames: 3830 / 3829 / 3288
- Command FPS (ROS simulation time / active wall time): 59.813 / 56.359 Hz
- Guidance FPS (active wall time): 51.970 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.719 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.211 / 0.427 / 4.811 / 27.316 ms
- Command interval mean / P50 / P95 / max: 17.743 / 16.116 / 23.784 / 89.641 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.561 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.182 / 2.741 / 8.228 / 27.466 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.267 / 1.619 ms (steady clock)
