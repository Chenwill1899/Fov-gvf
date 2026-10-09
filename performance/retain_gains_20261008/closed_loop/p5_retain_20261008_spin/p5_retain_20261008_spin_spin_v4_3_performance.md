
### Run p5_retain_20261008_spin_spin_v4_3

- Started: 2026-10-08 10:37:37 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_v4_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.945 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.276 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.020 / 15.632 / 17.720 / 78.873 ms
- Distinct applied command values: 2941
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.783 / 15.898 / 32.339 / 4172.013 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_v4_3

- Wall duration: 79.730 s
- Timer callbacks / command frames / guidance frames: 3628 / 3627 / 3090
- Command FPS (ROS simulation time / active wall time): 56.656 / 55.478 Hz
- Guidance FPS (active wall time): 50.661 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.650 / 16.667 / 16.667 / 650.000 ms
- Avoidance compute mean / P50 / P95 / max: 2.319 / 0.410 / 6.663 / 612.453 ms
- Command interval mean / P50 / P95 / max: 18.025 / 15.962 / 25.587 / 618.394 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.722 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.619 / 3.369 / 10.769 / 618.326 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.389 / 1.349 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_v4_3

- Wall duration: 79.621 s
- Received / published control frames: 3627 / 3611
- Published control FPS (ROS simulation time / active wall time): 56.656 / 55.233 Hz
- Command stamp interval mean / P50 / P95 / max: 17.650 / 16.667 / 16.667 / 650.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.402 / 0.000 / 0.000 / 650.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.025 / 15.962 / 25.478 / 618.374 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.111 / 0.887 ms (steady clock)
