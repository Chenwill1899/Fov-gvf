
### Run p5_retain_20261008_spin_spin_baseline_2

- Started: 2026-10-08 10:30:16 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_baseline_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.660 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.650 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.209 / 15.715 / 17.681 / 79.672 ms
- Distinct applied command values: 3040
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.537 / 15.976 / 32.420 / 4006.761 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_baseline_2

- Wall duration: 80.325 s
- Timer callbacks / command frames / guidance frames: 3803 / 3802 / 3261
- Command FPS (ROS simulation time / active wall time): 59.391 / 57.518 Hz
- Guidance FPS (active wall time): 52.983 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.838 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.473 / 0.409 / 6.410 / 43.350 ms
- Command interval mean / P50 / P95 / max: 17.386 / 15.662 / 24.349 / 91.283 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.708 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.686 / 2.792 / 10.648 / 56.957 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.393 / 1.405 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_baseline_2

- Wall duration: 80.217 s
- Received / published control frames: 3802 / 3802
- Published control FPS (ROS simulation time / active wall time): 59.391 / 57.518 Hz
- Command stamp interval mean / P50 / P95 / max: 16.838 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.728 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.386 / 15.641 / 24.318 / 91.396 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.081 / 0.108 / 1.738 ms (steady clock)
