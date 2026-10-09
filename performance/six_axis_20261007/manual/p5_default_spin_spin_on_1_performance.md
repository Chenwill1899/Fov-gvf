
### Run p5_default_spin_spin_on_1

- Started: 2026-10-07 17:06:00 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_default_spin_spin_on_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.842 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.367 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.996 / 15.761 / 17.907 / 79.093 ms
- Distinct applied command values: 2885
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.279 / 16.177 / 32.281 / 4155.718 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_default_spin_spin_on_1

- Wall duration: 79.555 s
- Timer callbacks / command frames / guidance frames: 3839 / 3838 / 3297
- Command FPS (ROS simulation time / active wall time): 59.953 / 58.792 Hz
- Guidance FPS (active wall time): 54.547 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.194 / 0.415 / 4.567 / 25.425 ms
- Command interval mean / P50 / P95 / max: 17.009 / 15.913 / 22.931 / 84.514 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.677 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.409 / 3.270 / 8.704 / 26.223 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.091 / 1.248 ms (steady clock)

#### Command bridge — p5_default_spin_spin_on_1

- Wall duration: 79.448 s
- Received / published control frames: 3838 / 3838
- Published control FPS (ROS simulation time / active wall time): 59.953 / 58.792 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.169 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.009 / 15.877 / 22.929 / 84.595 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.106 / 1.644 ms (steady clock)
