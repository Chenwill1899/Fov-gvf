
### Run explore_v3_spin_ego1p3_1

- Started: 2026-10-06 22:13:22 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v3_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.988 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.238 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.032 / 15.626 / 17.757 / 78.995 ms
- Distinct applied command values: 2487
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 24.852 / 16.149 / 49.265 / 4730.595 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v3_spin_ego1p3_1

- Wall duration: 79.655 s
- Timer callbacks / command frames / guidance frames: 3271 / 3270 / 2745
- Command FPS (ROS simulation time / active wall time): 51.078 / 49.986 Hz
- Guidance FPS (active wall time): 45.081 Hz
- Command interval in ROS time mean / P50 / P95 / max: 19.578 / 16.667 / 33.333 / 1150.000 ms
- Avoidance compute mean / P50 / P95 / max: 7.889 / 2.616 / 22.734 / 1135.383 ms
- Command interval mean / P50 / P95 / max: 20.006 / 16.423 / 40.885 / 1136.087 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.939 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 9.868 / 5.881 / 28.494 / 1136.032 ms (steady clock)
- DDS publish call mean / P95 / max: 0.067 / 0.429 / 2.123 ms (steady clock)

#### Command bridge — explore_v3_spin_ego1p3_1

- Wall duration: 79.551 s
- Received / published control frames: 3270 / 3249
- Published control FPS (ROS simulation time / active wall time): 51.078 / 49.665 Hz
- Command stamp interval mean / P50 / P95 / max: 19.578 / 16.667 / 33.333 / 1150.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 5.698 / 0.000 / 33.333 / 1150.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 20.006 / 16.443 / 40.707 / 1136.102 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.085 / 0.116 / 2.083 ms (steady clock)
