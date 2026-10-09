
### Run p5_refine_formal_operator_20261007_jitter_after_1

- Started: 2026-10-07 21:43:59 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_formal_operator_20261007_jitter_after_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.142 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.005 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.019 / 16.988 / 19.170 / 79.267 ms
- Distinct applied command values: 1273
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.296 / 17.811 / 36.022 / 2033.690 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_formal_operator_20261007_jitter_after_1

- Wall duration: 51.108 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 55.472 Hz
- Guidance FPS (active wall time): 47.825 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.305 / 0.209 / 0.685 / 3.115 ms
- Command interval mean / P50 / P95 / max: 18.027 / 16.953 / 26.989 / 88.219 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 47.282 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.636 / 0.395 / 9.575 / 13.582 ms (steady clock)
- DDS publish call mean / P95 / max: 0.055 / 0.074 / 1.194 ms (steady clock)

#### Command bridge — p5_refine_formal_operator_20261007_jitter_after_1

- Wall duration: 50.997 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 55.473 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.027 / 16.929 / 26.910 / 88.356 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.109 / 1.060 ms (steady clock)
