
### Run intent_cloud9

- Started: 2026-09-29 18:11:19 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `84`
- Manual input: `trace`

#### Isaac command application — intent_cloud9

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 85.278 / 84.017 s
- Applied simulation control frames: 5043
- Runtime FPS (simulation time / wall time): 60.024 / 59.136 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.838 / 15.371 / 17.972 / 79.966 ms
- Distinct applied command values: 2109
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 38.486 / 16.988 / 90.420 / 2902.796 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — intent_cloud9

- Wall duration: 101.962 s
- Timer callbacks / command frames / guidance frames: 3315 / 3314 / 3062
- Command FPS (ROS simulation time / active wall time): 39.440 / 39.038 Hz
- Guidance FPS (active wall time): 38.147 Hz
- Command interval in ROS time mean / P50 / P95 / max: 25.355 / 16.667 / 50.000 / 283.333 ms
- Avoidance compute mean / P50 / P95 / max: 6.499 / 0.410 / 32.651 / 195.309 ms
- Command interval mean / P50 / P95 / max: 25.616 / 19.337 / 64.597 / 217.811 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 26.023 / 33.333 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 19.878 / 14.823 / 50.483 / 217.737 ms (steady clock)
- DDS publish call mean / P95 / max: 0.067 / 0.307 / 1.664 ms (steady clock)

#### Command bridge — intent_cloud9

- Wall duration: 101.850 s
- Received / published control frames: 3314 / 3286
- Published control FPS (ROS simulation time / active wall time): 39.440 / 38.708 Hz
- Command stamp interval mean / P50 / P95 / max: 25.355 / 16.667 / 50.000 / 283.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 16.194 / 16.667 / 50.000 / 283.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 25.616 / 19.385 / 64.905 / 217.767 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.109 / 0.096 / 0.134 / 3.386 ms (steady clock)
