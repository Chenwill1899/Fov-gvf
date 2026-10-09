
### Run explore_v4_spin_ego1p3_1

- Started: 2026-10-06 22:17:19 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v4_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.612 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.572 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.936 / 15.755 / 17.761 / 78.398 ms
- Distinct applied command values: 3068
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.955 / 16.061 / 31.466 / 4209.064 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — explore_v4_spin_ego1p3_1

- Wall duration: 79.260 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 59.050 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.022 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.935 / 15.744 / 21.817 / 82.741 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.081 / 0.105 / 1.806 ms (steady clock)

#### Controller — explore_v4_spin_ego1p3_1

- Wall duration: 79.270 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 59.050 Hz
- Guidance FPS (active wall time): 54.683 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.792 / 3.747 / 5.959 / 10.935 ms
- Command interval mean / P50 / P95 / max: 16.935 / 15.761 / 21.910 / 82.652 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.601 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.399 / 4.516 / 9.725 / 17.255 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.082 / 1.323 ms (steady clock)
