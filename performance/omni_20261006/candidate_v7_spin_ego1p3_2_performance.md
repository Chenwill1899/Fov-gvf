
### Run candidate_v7_spin_ego1p3_2

- Started: 2026-10-06 22:49:14 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v7_spin_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.727 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.593 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.223 / 16.002 / 18.206 / 78.821 ms
- Distinct applied command values: 2955
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.148 / 16.316 / 33.089 / 4111.431 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v7_spin_ego1p3_2

- Wall duration: 80.439 s
- Timer callbacks / command frames / guidance frames: 3748 / 3747 / 3206
- Command FPS (ROS simulation time / active wall time): 58.531 / 56.642 Hz
- Guidance FPS (active wall time): 52.052 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.085 / 16.667 / 16.667 / 183.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.940 / 2.525 / 6.607 / 184.457 ms
- Command interval mean / P50 / P95 / max: 17.655 / 15.800 / 25.173 / 194.188 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.738 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.749 / 3.658 / 11.315 / 194.110 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.314 / 2.317 ms (steady clock)

#### Command bridge — candidate_v7_spin_ego1p3_2

- Wall duration: 80.333 s
- Received / published control frames: 3747 / 3744
- Published control FPS (ROS simulation time / active wall time): 58.531 / 56.597 Hz
- Command stamp interval mean / P50 / P95 / max: 17.085 / 16.667 / 16.667 / 183.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.823 / 0.000 / 0.000 / 183.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.655 / 15.793 / 25.035 / 194.266 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.109 / 2.046 ms (steady clock)
