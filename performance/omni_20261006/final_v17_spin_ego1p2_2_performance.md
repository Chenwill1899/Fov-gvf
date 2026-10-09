
### Run final_v17_spin_ego1p2_2

- Started: 2026-10-07 00:24:49 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_spin_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 71.304 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 53.896 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.414 / 16.530 / 24.700 / 88.730 ms
- Distinct applied command values: 2845
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.548 / 17.040 / 36.491 / 4063.500 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v17_spin_ego1p2_2

- Wall duration: 84.968 s
- Received / published control frames: 3840 / 3840
- Published control FPS (ROS simulation time / active wall time): 59.984 / 54.292 Hz
- Command stamp interval mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.004 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.419 / 16.422 / 26.764 / 99.413 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.101 / 0.089 / 0.134 / 3.368 ms (steady clock)

#### Controller — final_v17_spin_ego1p2_2

- Wall duration: 85.077 s
- Timer callbacks / command frames / guidance frames: 3841 / 3840 / 3299
- Command FPS (ROS simulation time / active wall time): 59.984 / 54.292 Hz
- Guidance FPS (active wall time): 49.910 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.604 / 0.528 / 1.039 / 7.707 ms
- Command interval mean / P50 / P95 / max: 18.419 / 16.415 / 26.764 / 99.931 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 68.753 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.883 / 2.601 / 7.491 / 33.333 ms (steady clock)
- DDS publish call mean / P95 / max: 0.076 / 0.424 / 4.061 ms (steady clock)
