
### Run ego1p1_paper_dynamic_final

- Started: 2026-09-24 14:42:45 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — ego1p1_paper_dynamic_final

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 7.433 / 10.017 s
- Applied simulation control frames: 603
- Runtime FPS (simulation time / wall time): 60.200 / 81.121 Hz
- Isaac frame interval mean / P50 / P95 / max: 11.955 / 10.594 / 12.838 / 78.387 ms
- Distinct applied command values: 192
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 34.270 / 11.918 / 43.373 / 2367.082 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — ego1p1_paper_dynamic_final

- Wall duration: 21.671 s
- Timer callbacks / command frames / guidance frames: 502 / 501 / 400
- Command FPS (ROS simulation time / active wall time): 50.000 / 69.776 Hz
- Guidance FPS (active wall time): 67.750 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 3.287 / 0.466 / 10.851 / 12.461 ms
- Command interval mean / P50 / P95 / max: 14.332 / 10.602 / 31.934 / 94.380 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 35.958 / 33.333 / 50.000 / 66.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.715 / 0.464 / 10.871 / 12.521 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.314 / 0.712 ms (steady clock)

#### Command bridge — ego1p1_paper_dynamic_final

- Wall duration: 21.564 s
- Received / published control frames: 501 / 501
- Published control FPS (ROS simulation time / active wall time): 50.000 / 69.779 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.597 / 0.000 / 16.667 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 14.331 / 10.647 / 32.078 / 94.041 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.078 / 0.076 / 0.110 / 0.169 ms (steady clock)
