
### Run final_v17_spin_ego1p3_2

- Started: 2026-10-07 00:23:27 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_spin_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.949 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.402 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.283 / 15.921 / 18.037 / 81.077 ms
- Distinct applied command values: 2941
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.256 / 16.340 / 32.506 / 4245.830 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v17_spin_ego1p3_2

- Wall duration: 80.831 s
- Received / published control frames: 3837 / 3837
- Published control FPS (ROS simulation time / active wall time): 59.938 / 57.800 Hz
- Command stamp interval mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.100 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.301 / 16.057 / 22.823 / 92.804 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.111 / 1.410 ms (steady clock)

#### Controller — final_v17_spin_ego1p3_2

- Wall duration: 80.840 s
- Timer callbacks / command frames / guidance frames: 3838 / 3837 / 3296
- Command FPS (ROS simulation time / active wall time): 59.938 / 57.799 Hz
- Guidance FPS (active wall time): 53.478 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.056 / 0.423 / 4.309 / 25.987 ms
- Command interval mean / P50 / P95 / max: 17.301 / 16.062 / 22.830 / 92.721 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.582 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.172 / 3.084 / 7.838 / 30.017 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.260 / 1.357 ms (steady clock)
