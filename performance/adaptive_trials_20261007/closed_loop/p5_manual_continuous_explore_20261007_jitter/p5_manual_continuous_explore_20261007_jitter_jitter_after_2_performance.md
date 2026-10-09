
### Run p5_manual_continuous_explore_20261007_jitter_jitter_after_2

- Started: 2026-10-07 23:01:58 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_manual_continuous_explore_20261007_jitter_jitter_after_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.221 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.889 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.059 / 16.764 / 19.121 / 81.039 ms
- Distinct applied command values: 1456
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.901 / 16.940 / 35.654 / 2284.636 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_manual_continuous_explore_20261007_jitter_jitter_after_2

- Wall duration: 51.112 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1620
- Command FPS (ROS simulation time / active wall time): 59.971 / 55.342 Hz
- Guidance FPS (active wall time): 47.762 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.310 / 0.211 / 0.504 / 3.509 ms
- Command interval mean / P50 / P95 / max: 18.070 / 16.632 / 28.005 / 87.637 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 48.169 / 50.000 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.618 / 0.391 / 10.270 / 14.556 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.083 / 1.108 ms (steady clock)

#### Command bridge — p5_manual_continuous_explore_20261007_jitter_jitter_after_2

- Wall duration: 51.002 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 55.342 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.069 / 16.610 / 27.773 / 87.781 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.081 / 0.105 / 2.064 ms (steady clock)
