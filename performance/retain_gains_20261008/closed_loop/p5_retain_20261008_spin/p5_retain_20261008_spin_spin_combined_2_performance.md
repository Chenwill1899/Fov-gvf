
### Run p5_retain_20261008_spin_spin_combined_2

- Started: 2026-10-08 10:33:57 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_combined_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.381 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.779 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.875 / 15.715 / 17.684 / 79.261 ms
- Distinct applied command values: 2949
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.757 / 15.968 / 32.913 / 3954.819 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_combined_2

- Wall duration: 79.123 s
- Timer callbacks / command frames / guidance frames: 3694 / 3693 / 3151
- Command FPS (ROS simulation time / active wall time): 57.688 / 56.979 Hz
- Guidance FPS (active wall time): 52.220 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.335 / 16.667 / 16.667 / 183.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.187 / 0.402 / 6.859 / 164.206 ms
- Command interval mean / P50 / P95 / max: 17.550 / 15.998 / 24.453 / 175.560 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.120 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.362 / 3.104 / 11.148 / 175.480 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.161 / 1.548 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_combined_2

- Wall duration: 79.017 s
- Received / published control frames: 3693 / 3691
- Published control FPS (ROS simulation time / active wall time): 57.688 / 56.948 Hz
- Command stamp interval mean / P50 / P95 / max: 17.335 / 16.667 / 16.667 / 183.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.250 / 0.000 / 0.000 / 183.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.550 / 16.009 / 24.510 / 175.565 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.083 / 0.107 / 1.018 ms (steady clock)
