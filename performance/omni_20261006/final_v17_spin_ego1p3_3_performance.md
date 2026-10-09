
### Run final_v17_spin_ego1p3_3

- Started: 2026-10-07 00:27:38 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_spin_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 69.138 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 55.585 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.851 / 16.117 / 21.913 / 150.723 ms
- Distinct applied command values: 2862
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.572 / 16.599 / 33.957 / 4003.968 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_spin_ego1p3_3

- Wall duration: 82.984 s
- Timer callbacks / command frames / guidance frames: 3841 / 3840 / 3299
- Command FPS (ROS simulation time / active wall time): 59.984 / 56.004 Hz
- Guidance FPS (active wall time): 51.828 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.955 / 0.418 / 4.255 / 15.364 ms
- Command interval mean / P50 / P95 / max: 17.856 / 16.155 / 25.438 / 167.180 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 44.508 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.092 / 2.376 / 8.844 / 34.706 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.187 / 4.685 ms (steady clock)

#### Command bridge — final_v17_spin_ego1p3_3

- Wall duration: 82.878 s
- Received / published control frames: 3840 / 3840
- Published control FPS (ROS simulation time / active wall time): 59.984 / 56.005 Hz
- Command stamp interval mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.069 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.856 / 16.160 / 25.400 / 168.165 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.094 / 0.084 / 0.123 / 3.812 ms (steady clock)
