
### Run p5_retain_20261008_spin_spin_v2_3

- Started: 2026-10-08 10:44:57 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_v2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.002 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 59.121 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.775 / 15.668 / 17.599 / 77.672 ms
- Distinct applied command values: 3043
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.940 / 15.829 / 32.154 / 4291.512 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_v2_3

- Wall duration: 78.722 s
- Timer callbacks / command frames / guidance frames: 3821 / 3820 / 3279
- Command FPS (ROS simulation time / active wall time): 59.672 / 59.299 Hz
- Guidance FPS (active wall time): 54.831 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.758 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.213 / 0.411 / 4.675 / 33.147 ms
- Command interval mean / P50 / P95 / max: 16.864 / 15.403 / 23.943 / 83.324 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.494 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.219 / 2.589 / 9.089 / 38.721 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.390 / 1.471 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_v2_3

- Wall duration: 78.612 s
- Received / published control frames: 3820 / 3820
- Published control FPS (ROS simulation time / active wall time): 59.672 / 59.299 Hz
- Command stamp interval mean / P50 / P95 / max: 16.758 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.340 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.864 / 15.448 / 23.968 / 83.321 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.083 / 0.081 / 0.105 / 1.092 ms (steady clock)
