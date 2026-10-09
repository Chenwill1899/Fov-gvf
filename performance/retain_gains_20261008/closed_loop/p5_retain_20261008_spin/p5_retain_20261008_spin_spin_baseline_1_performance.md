
### Run p5_retain_20261008_spin_spin_baseline_1

- Started: 2026-10-08 10:09:42 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_baseline_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.498 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.673 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.904 / 15.703 / 17.756 / 78.943 ms
- Distinct applied command values: 3120
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.651 / 15.924 / 31.850 / 4259.098 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_baseline_1

- Wall duration: 79.237 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 59.159 Hz
- Guidance FPS (active wall time): 54.709 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.924 / 0.416 / 3.819 / 16.512 ms
- Command interval mean / P50 / P95 / max: 16.903 / 15.731 / 22.953 / 83.152 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.429 / 33.333 / 50.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.048 / 2.901 / 7.613 / 22.383 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.278 / 1.796 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_baseline_1

- Wall duration: 79.126 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 59.160 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.065 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.903 / 15.709 / 22.896 / 83.254 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.085 / 0.110 / 1.029 ms (steady clock)
