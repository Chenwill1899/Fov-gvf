
### Run final_v17_sweep_ego1p2_3

- Started: 2026-10-07 00:18:46 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_sweep_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.646 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.700 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.197 / 16.018 / 18.084 / 79.522 ms
- Distinct applied command values: 1807
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.712 / 16.445 / 33.582 / 941.482 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_sweep_ego1p2_3

- Wall duration: 55.235 s
- Timer callbacks / command frames / guidance frames: 2339 / 2338 / 2157
- Command FPS (ROS simulation time / active wall time): 58.425 / 56.621 Hz
- Guidance FPS (active wall time): 56.337 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.116 / 16.667 / 16.667 / 133.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.240 / 0.402 / 1.826 / 133.535 ms
- Command interval mean / P50 / P95 / max: 17.661 / 16.151 / 22.522 / 135.896 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 74.618 / 66.667 / 116.667 / 150.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.304 / 2.510 / 6.751 / 135.795 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.398 / 2.102 ms (steady clock)

#### Command bridge — final_v17_sweep_ego1p2_3

- Wall duration: 55.128 s
- Received / published control frames: 2338 / 2335
- Published control FPS (ROS simulation time / active wall time): 58.425 / 56.549 Hz
- Command stamp interval mean / P50 / P95 / max: 17.116 / 16.667 / 16.667 / 133.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.741 / 0.000 / 0.000 / 150.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.661 / 16.144 / 22.455 / 135.993 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.087 / 0.114 / 1.043 ms (steady clock)
