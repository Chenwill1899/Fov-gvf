
### Run p5_manual_continuous_explore_20261007_jitter_jitter_before_3

- Started: 2026-10-07 23:07:00 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_manual_continuous_explore_20261007_jitter_jitter_before_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.754 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.586 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.831 / 16.670 / 18.782 / 78.876 ms
- Distinct applied command values: 1514
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.671 / 16.846 / 34.656 / 2222.969 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_manual_continuous_explore_20261007_jitter_jitter_before_3

- Wall duration: 50.721 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.087 Hz
- Guidance FPS (active wall time): 48.436 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.315 / 0.211 / 1.009 / 3.777 ms
- Command interval mean / P50 / P95 / max: 17.829 / 16.336 / 26.416 / 87.747 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.016 / 33.333 / 66.667 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.734 / 2.134 / 8.905 / 14.844 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.077 / 1.230 ms (steady clock)

#### Command bridge — p5_manual_continuous_explore_20261007_jitter_jitter_before_3

- Wall duration: 50.611 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.087 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.829 / 16.357 / 26.227 / 87.521 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.111 / 1.063 ms (steady clock)
