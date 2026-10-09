
### Run p5_manual_continuous_explore_20261007_jitter_jitter_after_3

- Started: 2026-10-07 23:09:20 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_manual_continuous_explore_20261007_jitter_jitter_after_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.333 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.724 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.110 / 16.765 / 18.994 / 80.628 ms
- Distinct applied command values: 1498
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.328 / 16.945 / 35.166 / 2093.653 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_manual_continuous_explore_20261007_jitter_jitter_after_3

- Wall duration: 51.241 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.220 Hz
- Guidance FPS (active wall time): 47.740 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.317 / 0.209 / 1.916 / 4.231 ms
- Command interval mean / P50 / P95 / max: 18.109 / 16.853 / 26.586 / 88.510 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.669 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.764 / 1.885 / 8.901 / 13.653 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.071 / 1.593 ms (steady clock)

#### Command bridge — p5_manual_continuous_explore_20261007_jitter_jitter_after_3

- Wall duration: 51.128 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.221 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.109 / 16.817 / 26.486 / 88.230 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.107 / 0.969 ms (steady clock)
