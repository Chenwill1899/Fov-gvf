
### Run final_v9_spin_ego1p3_2

- Started: 2026-10-06 23:09:53 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v9_spin_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 69.230 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 55.510 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.874 / 16.446 / 19.287 / 80.286 ms
- Distinct applied command values: 3003
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.503 / 16.766 / 33.818 / 4316.089 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v9_spin_ego1p3_2

- Wall duration: 83.057 s
- Timer callbacks / command frames / guidance frames: 3820 / 3819 / 3278
- Command FPS (ROS simulation time / active wall time): 59.656 / 55.639 Hz
- Guidance FPS (active wall time): 51.593 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.763 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.072 / 2.978 / 7.712 / 55.327 ms
- Command interval mean / P50 / P95 / max: 17.973 / 16.630 / 24.688 / 91.186 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 47.071 / 33.333 / 100.000 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.864 / 4.328 / 12.223 / 59.300 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.129 / 1.434 ms (steady clock)

#### Command bridge — final_v9_spin_ego1p3_2

- Wall duration: 82.952 s
- Received / published control frames: 3819 / 3819
- Published control FPS (ROS simulation time / active wall time): 59.656 / 55.639 Hz
- Command stamp interval mean / P50 / P95 / max: 16.763 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.388 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.973 / 16.640 / 24.735 / 91.271 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.086 / 0.117 / 1.493 ms (steady clock)
