
### Run p5_retain_20261008_spin_spin_v4_2

- Started: 2026-10-08 10:26:35 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_v4_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.320 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.946 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.116 / 15.833 / 17.876 / 80.115 ms
- Distinct applied command values: 2962
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.910 / 16.044 / 33.000 / 4050.196 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_v4_2

- Wall duration: 80.040 s
- Timer callbacks / command frames / guidance frames: 3764 / 3763 / 3223
- Command FPS (ROS simulation time / active wall time): 58.781 / 57.242 Hz
- Guidance FPS (active wall time): 52.953 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.012 / 16.667 / 16.667 / 100.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.780 / 0.403 / 6.411 / 96.034 ms
- Command interval mean / P50 / P95 / max: 17.470 / 15.947 / 25.254 / 103.904 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.233 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.992 / 2.751 / 11.089 / 103.799 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.180 / 1.787 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_v4_2

- Wall duration: 79.931 s
- Received / published control frames: 3763 / 3763
- Published control FPS (ROS simulation time / active wall time): 58.781 / 57.242 Hz
- Command stamp interval mean / P50 / P95 / max: 17.012 / 16.667 / 16.667 / 100.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.846 / 0.000 / 0.000 / 100.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.470 / 15.939 / 25.150 / 103.926 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.111 / 1.475 ms (steady clock)
