
### Run final_v9_spin_ego1p3_3

- Started: 2026-10-06 23:14:04 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v9_spin_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.536 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.903 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.435 / 15.931 / 18.246 / 80.848 ms
- Distinct applied command values: 3054
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.735 / 16.222 / 32.150 / 4301.426 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v9_spin_ego1p3_3

- Wall duration: 81.648 s
- Timer callbacks / command frames / guidance frames: 3812 / 3811 / 3270
- Command FPS (ROS simulation time / active wall time): 59.531 / 56.919 Hz
- Guidance FPS (active wall time): 52.422 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.798 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.080 / 2.958 / 7.915 / 57.508 ms
- Command interval mean / P50 / P95 / max: 17.569 / 16.050 / 23.624 / 84.988 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.764 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.830 / 4.121 / 11.543 / 61.238 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.381 / 1.479 ms (steady clock)

#### Command bridge — final_v9_spin_ego1p3_3

- Wall duration: 81.201 s
- Received / published control frames: 3811 / 3811
- Published control FPS (ROS simulation time / active wall time): 59.531 / 56.919 Hz
- Command stamp interval mean / P50 / P95 / max: 16.798 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.429 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.569 / 16.026 / 23.589 / 85.094 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.109 / 0.869 ms (steady clock)
